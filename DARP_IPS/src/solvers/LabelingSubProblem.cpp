//
// Created by Ella on 1/28/2022.
//

#include "LabelingSubProblem.h"

#include <utility>



//-----------------------------------------------------------------------------
//  Labeling Sub problem class
//-----------------------------------------------------------------------------


LabelingSubProblem::LabelingSubProblem(PVehicle &vehicle, PSolverOption solverOptions) : SubproModeler(vehicle),
                                                                                            solverOptions_(std::move(solverOptions)) {
    nbDominated_ = 0;
    nbEliminated_ = 0;
    nbGenerated_ = 0;
    nbActivated_ = 0;
}
LabelingSubProblem::~LabelingSubProblem() {
    dominatedLabels_.clear();
    activeNodes_.clear();
    nodesOrder_.clear();
    solverOptions_.reset();
}

// this function sort the list of nodes based of their dual values
void LabelingSubProblem::sortNodes() {
    subGraph_->nodeIDToInt_.clear();
    subGraph_->intToNodeID_.clear();
    requestIDToInt_.clear();
    nodesOrder_.clear();
    int counter = 0;

    // sorting requests based on their dual values
    sort(subRequests_.begin(),subRequests_.end(),[](const PRequest &lhs, const PRequest &rhs){
        return lhs->dual_ > rhs->dual_;});

    // adding onboard nodes
    for (auto & nodeID : (*Vehicle_)->onboards_) {
        subGraph_->nodeIDToInt_[nodeID] = subGraph_->intToNodeID_.size();
        subGraph_->intToNodeID_.push_back(nodeID);
        requestIDToInt_[subGraph_->nodes_[nodeID]->related_Request_->getRequestId()] = counter;
        counter++;
    }

    // add pickup nodes
    for (auto &requestObj : subRequests_) {
        std::string pickID = Tools::createNodeID(requestObj->getRequestId(), PICKUP);
        subGraph_->nodeIDToInt_[pickID] = subGraph_->intToNodeID_.size();
        subGraph_->intToNodeID_.push_back(pickID);
        nodesOrder_.push_back(subGraph_->nodes_[pickID]);
        requestIDToInt_[requestObj->getRequestId()] = counter;
        counter++;
    }
    // add drop off points
    for (auto &requestObj : subRequests_) {
        if (requestObj->dual_ >= 0) {
            std::string dropID = Tools::createNodeID(requestObj->getRequestId(), DROPOFF);
            subGraph_->nodeIDToInt_[dropID] = subGraph_->intToNodeID_.size();
            subGraph_->intToNodeID_.push_back(dropID);
        }
    }

    // adding sink node
    subGraph_->nodeIDToInt_[(*Vehicle_)->sinkID_] = subGraph_->intToNodeID_.size();
    subGraph_->intToNodeID_.push_back((*Vehicle_)->sinkID_);
//    nodesOrder_.push_back(subGraph_->nodes_[(*Vehicle_)->sinkID_]);
}

void LabelingSubProblem::sortSuccessors() {
    for (auto & nodeObj: subGraph_->nodes_) {
        nodeObj.second->predecessor_.clear();
    }
    for (auto & nodeObj: subGraph_->nodes_){
        if (nodeObj.second->type_ != SINK){
            nodeObj.second->successors_.clear();
            for (auto &requestObj : subRequests_) {
                std::string pickID = Tools::createNodeID(requestObj->getRequestId(), PICKUP);
                if (nodeObj.second->nodeID_ != pickID) {
                    subGraph_->nodes_[pickID]->travelTimeFromNode_ = durationMatrix_[nodeObj.second->locationID_][subGraph_->nodes_[pickID]->locationID_];
                    /*if (!(*Vehicle_)->onboards_.empty()) {
                        float distanceToOnboard = 0;
                        for (auto &onboardNodes: (*Vehicle_)->onboards_)
                            distanceToOnboard += durationMatrix_[subGraph_->nodes_[pickID]->locationID_][subGraph_->nodes_[onboardNodes]->locationID_];
                        distanceToOnboard /= (*Vehicle_)->onboards_.size();
                        subGraph_->nodes_[pickID]->travelTimeFromNode_ += distanceToOnboard;
                    }*/
                    if (subGraph_->nodes_[pickID]->nodeStatus_ != COMMITTED)
                        nodeObj.second->successors_.push_back(subGraph_->nodes_[pickID]);
                }
            }
            sort(nodeObj.second->successors_.begin(),nodeObj.second->successors_.end(),[](const PNode &lhs, const PNode &rhs){
                return lhs->travelTimeFromNode_ < rhs->travelTimeFromNode_;});
            if (solverOptions_->isSuccessorsLimited_) {
                int location = (int)floor(1*nodeObj.second->successors_.size()/2) + 1;
                nodeObj.second->successors_.erase(nodeObj.second->successors_.begin()+location, nodeObj.second->successors_.end());
            }
            if (nodeObj.second->type_ == PICKUP) {
                for (auto &succNode: nodeObj.second->successors_)
                    succNode->predecessor_.insert(nodeObj.second);
            }
//            nodeObj.second->successors_.push_back(subGraph_->nodes_[(*Vehicle_)->sinkID_]);
        }
    }
}


// reset that active lists of the nodes, create the first label at the source, add onboards
void LabelingSubProblem::initialization() {

    nbActivated_ = 0;
    /*for (auto labelObj: dominatedLabels_)
        labelObj.reset();*/
    dominatedLabels_.clear();
    // clear active lists
    for (auto &nodeID: subGraph_->intToNodeID_) {
        /*for (auto labelObj: subGraph_->nodes_[nodeID]->activeLabels_)
            labelObj.reset();*/
        subGraph_->nodes_[nodeID]->activeLabels_.clear();
        subGraph_->nodes_[nodeID]->bestLabelReduceCost_ = INFINITY;
        subGraph_->nodes_[nodeID]->nbActiveLabels_ = 0;
        subGraph_->nodes_[nodeID]->generatedLabels_.clear();
    }

    sortNodes();
    sortSuccessors();

    // create the initial label at the source and add the source to the list active nodes
    PLabel initialLabel = std::make_shared<Label>(Vehicle_, subGraph_->nodes_[(*Vehicle_)->departID_]);
    initialLabel->completedRequests_.resize(requestIDToInt_.size());

    initialLabel->requestIDToInt_ = requestIDToInt_;
    // update travel resource for the initial label based on the onboards
    for (auto &nodeID: (*Vehicle_)->onboards_) {
        initialLabel->openNodes_.insert(subGraph_->nodes_[nodeID]);
        initialLabel->completedRequests_[requestIDToInt_[subGraph_->nodes_[nodeID]->related_Request_->getRequestId()]] = 1;
        float remainedTime = subGraph_->nodes_[nodeID]->related_Request_->maxTravelTime_ -
                             (*Vehicle_)->departTime_ + subGraph_->nodes_[nodeID]->related_Request_->pickTime_ +
                             subGraph_->nodes_[nodeID]->related_Request_->deltaTime_;
        initialLabel->travelResource_.insert(std::pair<std::string, float>(nodeID, remainedTime));
    }
    for (int i = 0; i < requestIDToInt_.size(); ++i){
        initialLabel->openRequests_.push_back(initialLabel->completedRequests_[i]);
    }

    if ((*Vehicle_)->currentRoute_->routeSize_ > 1) {
        int i = 1;
        while ((*Vehicle_)->currentRoute_->routeNodes_[i]->nodeStatus_ == COMMITTED){
            initialLabel->extend((*Vehicle_)->currentRoute_->routeNodes_[i]);
            i++;
            if (i == (*Vehicle_)->currentRoute_->routeSize_ - 1)
                break;
        }
    }
    initialLabel->currentNode_->activeLabels_.push_back(initialLabel);
    initialLabel->currentNode_->generatedLabels_[initialLabel->nbPickUp_].push_back(initialLabel);
    initialLabel->currentNode_->nbActiveLabels_++;
    initialLabel->currentNode_->bestLabelReduceCost_ = initialLabel->reducedCost_;
    activeNodes_.clear();
    activeNodes_.push_back(initialLabel->currentNode_);
    nbActivated_ ++;
    initialLabel.reset();
}

void LabelingSubProblem::labelExtend(PLabel &parentLabel, PNode &outNode, std::vector<PNode> &activeNodeList) {
    PLabel newLabel = std::make_shared<Label>(*parentLabel);
    newLabel->parent_ = parentLabel;
    newLabel->extend(outNode);
    nbGenerated_++;
    if (!newLabel->isEliminated(subGraph_)) {
        if (isLabelAdded(newLabel, outNode)) {
            /*if ((newLabel->currentNode_->type_ != SINK)&&(outNode->nbActiveLabels_ == 1)) {
                activeNodeList.push_back(outNode);
            }*/
        }
        else
            nbDominated_++;
    }
    else
        nbEliminated_++;
}

void LabelingSubProblem::labelDrop(PLabel &parentLabel, vector<PNode> &activeNodeList) {
    std::vector<PLabel> openLabelList;
    openLabelList.push_back(parentLabel);
    while(!openLabelList.empty()) {
        PLabel selectedLabel = openLabelList.back();
        openLabelList.pop_back();
        std::vector<PNode> outNodes(selectedLabel->openNodes_.begin(),
                                    selectedLabel->openNodes_.end());
        for (auto &neighbourNode: outNodes) {
            PLabel newLabel = std::make_shared<Label>(*parentLabel);
            newLabel->parent_ = parentLabel;
            newLabel->extend(neighbourNode);
            nbGenerated_++;
            if (!newLabel->isEliminated(subGraph_)) {
                if (isLabelAdded(newLabel, neighbourNode)) {
                    if (!newLabel->openNodes_.empty())
                        // add to label list for pushing to other drop off points
                        openLabelList.push_back(newLabel);
                    else
                        // terminate to the sink
                        labelExtend(newLabel, subGraph_->nodes_[(*Vehicle_)->sinkID_], activeNodes_);
                    if ((newLabel->currentNode_->type_ != SINK)&&(neighbourNode->nbActiveLabels_ == 1)) {
                        activeNodeList.push_back(neighbourNode);
                    }
                }
                else
                    nbDominated_++;
            }
            else
                nbEliminated_++;
        }
        selectedLabel->isDropped_ = true;
        if (selectedLabel->extendCheck_.size() == nodesOrder_.size() ||
            selectedLabel->nbPickUp_ == solverOptions_->maxPickup_) {
            selectedLabel->status_ = INACTIVE;
            selectedLabel->currentNode_->nbActiveLabels_--;
            nbActivated_--;
        }
        selectedLabel.reset();
    }
}


bool LabelingSubProblem::isLabelAdded(PLabel &newLabel, PNode &outNode) {
    std::map<int, std::vector<PLabel>>::iterator it;
    for (it = outNode->generatedLabels_.begin(); it != outNode->generatedLabels_.end(); it++) {
//        if (it->first <= newLabel->completedRequests_.size()) {
        if (it->first <= newLabel->nbPickUp_) {
            for (auto labelObj: it->second) {
                if (newLabel->isDominated(labelObj, this->solverOptions_)){
                    newLabel.reset();
                    return false;
                }
            }
        }

        // remove previous dominated labels
 //       if (it->first >= newLabel->completedRequests_.size()) {
        if (it->first >= newLabel->nbPickUp_) {
            for (int i = it->second.size()-1; i >= 0; --i){
                if (it->second[i]->isDominated(newLabel, this->solverOptions_)){
                    if (it->second[i]->status_ == ACTIVE){
                        outNode->nbActiveLabels_--;
                        this->nbActivated_--;
                    }
                    it->second[i]->status_ = DOMINATED;
                    dominatedLabels_.push_back(it->second[i]);
                    it->second.erase(it->second.begin() + i);
                    this->nbDominated_++;
                }
            }
        }
    }
//    outNode->generatedLabels_[newLabel->completedRequests_.size()].push_back(newLabel);
    outNode->generatedLabels_[newLabel->nbPickUp_].push_back(newLabel);
    outNode->activeLabels_.push_back(newLabel);
    if (outNode->bestLabelReduceCost_ > newLabel->reducedCost_)
        outNode->bestLabelReduceCost_ = newLabel->reducedCost_;
    if (outNode->type_ == SINK)
        newLabel->status_ = INACTIVE;
    else {
        outNode->nbActiveLabels_++;
        this->nbActivated_++;
    }
    return true;
}

void LabelingSubProblem::solveDynamic_pulling1() {
    // create initial label
    int nbActive = 0;
    while(true) {
        // create initial label
        initialization();


        while (nbActivated_ > 0) {
            // select a node to pull other labels to it
            for (auto &currentNode: nodesOrder_) {
                sort(activeNodes_.begin(),activeNodes_.end(),[](const PNode &lhs, const PNode &rhs){
                    return lhs->nbActiveLabels_ < rhs->nbActiveLabels_;});
                for (int j = activeNodes_.size()-1; j >= 0; j--) {
                    if (activeNodes_[j]->nbActiveLabels_ == 0)
                        activeNodes_.erase(activeNodes_.begin() + j);
                    else {
                        for (int l = activeNodes_[j]->activeLabels_.size() - 1; l >= 0; l--) {
                            if (activeNodes_[j]->activeLabels_[l]->status_ != ACTIVE)
                                activeNodes_[j]->activeLabels_.erase(activeNodes_[j]->activeLabels_.begin() + l);
                            else {
                                if ((!solverOptions_->areHeuristicsDisabled()) ||
                                    (!activeNodes_[j]->activeLabels_[l]->haveDominatedParent())) {
                                    PLabel selectedLabel = activeNodes_[j]->activeLabels_[l];
                                    // push to drop onboards
                                    if (!selectedLabel->isDropped_) {
                                        if (!selectedLabel->openNodes_.empty()) {
                                            std::vector<PNode> outNodes(selectedLabel->openNodes_.begin(),
                                                                        selectedLabel->openNodes_.end());
                                            for (auto &onboardNode: outNodes) {
                                                nbActive = onboardNode->nbActiveLabels_;
                                                labelExtend(selectedLabel, onboardNode, activeNodes_);
                                                if ((onboardNode->nbActiveLabels_ == 1) && (nbActive == 0)) {
                                                    activeNodes_.push_back(onboardNode);
                                                }
                                            }
                                            selectedLabel->isDropped_ = true;
                                        }
                                            // push to the sink
                                        else if (selectedLabel->openNodes_.empty()) {
                                            labelExtend(selectedLabel, subGraph_->nodes_[(*Vehicle_)->sinkID_],
                                                        activeNodes_);
                                            selectedLabel->isDropped_ = true;
                                        }
                                    }
                                    if (selectedLabel->extendCheck_.size() == nodesOrder_.size() ||
                                        selectedLabel->nbPickUp_ == solverOptions_->maxPickup_) {
                                        selectedLabel->status_ = INACTIVE;
                                        activeNodes_[j]->nbActiveLabels_--;
                                        nbActivated_--;
                                        activeNodes_[j]->activeLabels_.erase(
                                                activeNodes_[j]->activeLabels_.begin() + l);
                                        if (activeNodes_[j]->nbActiveLabels_ == 0) {
                                            activeNodes_.erase(activeNodes_.begin() + j);
                                            break;
                                        }
                                    }
                                        // pull all labels to the current node
                                    else if ((activeNodes_[j]->nodeID_ != currentNode->nodeID_) &&
                                             (selectedLabel->isExtendFeasible(currentNode,
                                                                              solverOptions_->maxPickup_))) {
                                        nbActive = currentNode->nbActiveLabels_;
                                        labelExtend(selectedLabel, currentNode, activeNodes_);
                                        if ((currentNode->nbActiveLabels_ == 1) && (nbActive == 0)) {
                                            activeNodes_.push_back(currentNode);
                                        }
                                    }
                                    selectedLabel.reset();
                                } else {
                                    activeNodes_[j]->activeLabels_[l]->status_ = DOMINATED;
                                    activeNodes_[j]->nbActiveLabels_--;
                                    nbActivated_--;
                                    activeNodes_[j]->activeLabels_.erase(activeNodes_[j]->activeLabels_.begin() + l);
                                    if (activeNodes_[j]->nbActiveLabels_ == 0) {
                                        activeNodes_.erase(activeNodes_.begin() + j);
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }

                // decrease the number of active labels if truncated strategy is used
                if ((solverOptions_->isTruncated_) && (currentNode->nbActiveLabels_ > solverOptions_->MaxLabel_)){
                    nbActivated_ -= (currentNode->nbActiveLabels_ - solverOptions_->MaxLabel_);
                    truncateLabelList(currentNode, solverOptions_->MaxLabel_);
                }
            }
        }
        if (!solverOptions_->areHeuristicsDisabled()) {
            if (subGraph_->nodes_[(*Vehicle_)->sinkID_]->bestLabelReduceCost_ - (*Vehicle_)->dual_ >= -0.0001) {
                solverOptions_->disableHeuristics();
                std::cout << " Disable Heuristics!!!" << std::endl;
            }
            else
                break;
        } else
            break;
    }
    // sort final labels based on reduced cost
    subGraph_->nodes_[(*Vehicle_)->sinkID_]->activeLabels_.clear();
    std::map<int, std::vector<PLabel>>::reverse_iterator it;
    for (it = subGraph_->nodes_[(*Vehicle_)->sinkID_]->generatedLabels_.rbegin(); it != subGraph_->nodes_[(*Vehicle_)->sinkID_]->generatedLabels_.rend(); it++) {
        for (auto &labelObj: it->second) {
            if (!labelObj->haveDominatedParent())
                subGraph_->nodes_[(*Vehicle_)->sinkID_]->activeLabels_.push_back(labelObj);
        }
    }
}

//***************************************************************************************//
//                           P U S H I N G  S T R A T E G Y
//***************************************************************************************//
void LabelingSubProblem::solveDynamic_pushing() {
    // create initial label
    int nbActive = 0;
    while(true) {
        // create initial label
        initialization();


        while (nbActivated_ > 0) {
            sort(activeNodes_.begin(),activeNodes_.end(),[](const PNode &lhs, const PNode &rhs){
                return lhs->nbActiveLabels_ < rhs->nbActiveLabels_;});

            // select a node to extend active labels
            PNode currentNode = activeNodes_.back();
            activeNodes_.pop_back();

            // decrease the number of active labels if truncated strategy is used
            if ((solverOptions_->isTruncated_) && (currentNode->nbActiveLabels_ > solverOptions_->MaxLabel_)){
                nbActivated_ -= (currentNode->nbActiveLabels_ - solverOptions_->MaxLabel_);
                truncateLabelList(currentNode, solverOptions_->MaxLabel_);
            }
            while (!currentNode->activeLabels_.empty()){
                if (currentNode->activeLabels_.back()->status_ != ACTIVE)
                    currentNode->activeLabels_.pop_back();
                else {
                    // if the heuristics are node disabled, we are not allowed to dominate labels with dominated parents
                    if ((!solverOptions_->areHeuristicsDisabled())||(!currentNode->activeLabels_.back()->haveDominatedParent())) {
                        PLabel selectedLabel = currentNode->activeLabels_.back();
                        currentNode->activeLabels_.pop_back();
                        currentNode->nbActiveLabels_--;
                        nbActivated_--;
                        selectedLabel->status_ = INACTIVE;
                        // terminate to sink
                        if (selectedLabel->openNodes_.empty())
                            labelExtend(selectedLabel, subGraph_->nodes_[(*Vehicle_)->sinkID_], activeNodes_);
                        // drop onboards
                        else {
                            std::vector<PNode> outNodes(selectedLabel->openNodes_.begin(),
                                                        selectedLabel->openNodes_.end());
                            for (auto &neighbourNode: outNodes) {
                                nbActive = neighbourNode->nbActiveLabels_;
                                labelExtend(selectedLabel, neighbourNode, activeNodes_);
                                if ((neighbourNode->nbActiveLabels_ == 1)&&(nbActive == 0)) {
                                    activeNodes_.push_back(neighbourNode);
                                }
                            }
                        }
                        if (selectedLabel->nbPickUp_ != solverOptions_->maxPickup_) {
                            // push to pickup points
 //                           for (auto &neighbourNode: nodesOrder_) {
                            for (auto &neighbourNode: selectedLabel->currentNode_->successors_) {
                                if (selectedLabel->isExtendFeasible(neighbourNode, solverOptions_->maxPickup_)) {
                                    nbActive = neighbourNode->nbActiveLabels_;
                                    labelExtend(selectedLabel, neighbourNode, activeNodes_);
                                    if ((neighbourNode->nbActiveLabels_ == 1)&&(nbActive == 0)) {
                                        activeNodes_.push_back(neighbourNode);
                                    }
                                }
                            }
                        }
                    }
                    else {
                        currentNode->activeLabels_.back()->status_ = DOMINATED;
                        currentNode->nbActiveLabels_--;
                        nbActivated_--;
                        currentNode->activeLabels_.pop_back();
                    }
                }
            }
            currentNode.reset();
        }
        if (!solverOptions_->areHeuristicsDisabled()) {
            if (subGraph_->nodes_[(*Vehicle_)->sinkID_]->bestLabelReduceCost_ - (*Vehicle_)->dual_ >= -0.0001)
                solverOptions_->disableHeuristics();
            else
                break;
        } else
            break;
    }
    subGraph_->nodes_[(*Vehicle_)->sinkID_]->activeLabels_.clear();
    std::map<int, std::vector<PLabel>>::reverse_iterator it;
    for (it = subGraph_->nodes_[(*Vehicle_)->sinkID_]->generatedLabels_.rbegin(); it != subGraph_->nodes_[(*Vehicle_)->sinkID_]->generatedLabels_.rend(); it++) {
        for (auto &labelObj: it->second) {
            subGraph_->nodes_[(*Vehicle_)->sinkID_]->activeLabels_.push_back(labelObj);
        }
    }
}

void LabelingSubProblem::solveDynamic_pushingDrop() {
// create initial label
    int nbActive = 0;
    while(true) {
        // create initial label
        initialization();


        while (nbActivated_ > 0) {
            sort(activeNodes_.begin(),activeNodes_.end(),[](const PNode &lhs, const PNode &rhs){
                return lhs->nbActiveLabels_ < rhs->nbActiveLabels_;});

            // select a node to extend active labels
            PNode currentNode = activeNodes_.back();
            activeNodes_.pop_back();

            // decrease the number of active labels if truncated strategy is used
            if ((solverOptions_->isTruncated_) && (currentNode->nbActiveLabels_ > solverOptions_->MaxLabel_)){
                nbActivated_ -= (currentNode->nbActiveLabels_ - solverOptions_->MaxLabel_);
                truncateLabelList(currentNode, solverOptions_->MaxLabel_);
            }
            while (!currentNode->activeLabels_.empty()){
                if (currentNode->activeLabels_.back()->status_ != ACTIVE)
                    currentNode->activeLabels_.pop_back();
                else {
                    // if the heuristics are node disabled, we are not allowed to dominate labels with dominated parents
                    if ((!solverOptions_->areHeuristicsDisabled())||(!currentNode->activeLabels_.back()->haveDominatedParent())) {
                        PLabel selectedLabel = currentNode->activeLabels_.back();
                        currentNode->activeLabels_.pop_back();
                        currentNode->nbActiveLabels_--;
                        nbActivated_--;
                        selectedLabel->status_ = INACTIVE;
                        // terminate to sink
                        if (selectedLabel->openNodes_.empty())
                            labelExtend(selectedLabel, subGraph_->nodes_[(*Vehicle_)->sinkID_], activeNodes_);
                            // drop onboards
                        else {
                            std::vector<PNode> outNodes(selectedLabel->openNodes_.begin(),
                                                        selectedLabel->openNodes_.end());
                            for (auto &neighbourNode: outNodes) {
                                nbActive = neighbourNode->nbActiveLabels_;
                                labelExtend(selectedLabel, neighbourNode, activeNodes_);
                                if ((neighbourNode->nbActiveLabels_ == 1)&&(nbActive == 0)) {
                                    activeNodes_.push_back(neighbourNode);
                                }
                                if (!neighbourNode->activeLabels_.empty())
                                    neighbourNode->activeLabels_.back()->isDropped_ = true;
                            }
                        }
                        if ((selectedLabel->nbPickUp_ != solverOptions_->maxPickup_)&&(!selectedLabel->isDropped_)) {
                            // push to pickup points
                            for (auto &neighbourNode: selectedLabel->currentNode_->successors_) {
                                if (selectedLabel->isExtendFeasible(neighbourNode, solverOptions_->maxPickup_)) {
                                    nbActive = neighbourNode->nbActiveLabels_;
                                    labelExtend(selectedLabel, neighbourNode, activeNodes_);
                                    if ((neighbourNode->nbActiveLabels_ == 1)&&(nbActive == 0)) {
                                        activeNodes_.push_back(neighbourNode);
                                    }
                                }
                            }
                        }
                    }
                    else {
                        currentNode->activeLabels_.back()->status_ = DOMINATED;
                        currentNode->nbActiveLabels_--;
                        nbActivated_--;
                        currentNode->activeLabels_.pop_back();
                    }
                }
            }
            currentNode.reset();
        }
        if (!solverOptions_->areHeuristicsDisabled()) {
            if (subGraph_->nodes_[(*Vehicle_)->sinkID_]->bestLabelReduceCost_ - (*Vehicle_)->dual_ >= -0.0001)
                solverOptions_->disableHeuristics();
            else
                break;
        } else
            break;
    }
    subGraph_->nodes_[(*Vehicle_)->sinkID_]->activeLabels_.clear();
    std::map<int, std::vector<PLabel>>::reverse_iterator it;
    for (it = subGraph_->nodes_[(*Vehicle_)->sinkID_]->generatedLabels_.rbegin(); it != subGraph_->nodes_[(*Vehicle_)->sinkID_]->generatedLabels_.rend(); it++) {
        for (auto &labelObj: it->second) {
            subGraph_->nodes_[(*Vehicle_)->sinkID_]->activeLabels_.push_back(labelObj);
        }
    }
}

//***************************************************************************************//
//                           P U L L I N G  S T R A T E G Y
//***************************************************************************************//

void LabelingSubProblem::solveDynamic_pulling() {
    // create initial label
    while(true) {
        // create initial label
        initialization();


        while (nbActivated_ > 0) {
            // select a node to pull other labels to it
            for (auto &currentNode: nodesOrder_) {
                sort(activeNodes_.begin(),activeNodes_.end(),[](const PNode &lhs, const PNode &rhs){
                    return lhs->nbActiveLabels_ > rhs->nbActiveLabels_;});
                for (auto & activeNodeObj : activeNodes_){
                    for (int l = activeNodeObj->activeLabels_.size()-1; l >= 0; l--){
                        if (activeNodeObj->activeLabels_[l]->status_ != ACTIVE)
                            activeNodeObj->activeLabels_.erase(activeNodeObj->activeLabels_.begin() + l);
                        else {
                            if ((!solverOptions_->areHeuristicsDisabled())||(!activeNodeObj->activeLabels_[l]->haveDominatedParent())) {
                                PLabel selectedLabel = activeNodeObj->activeLabels_[l];
                                if (selectedLabel->extendCheck_.size() == nodesOrder_.size() ||
                                    selectedLabel->nbPickUp_ == solverOptions_->maxPickup_) {
                                    selectedLabel->status_ = INACTIVE;
                                    activeNodeObj->nbActiveLabels_--;
                                    nbActivated_--;
                                    activeNodeObj->activeLabels_.erase(
                                            activeNodeObj->activeLabels_.begin() + l);
                                }
                                    // pull all labels to the current node
                                else if (selectedLabel->isExtendFeasible(currentNode, solverOptions_->maxPickup_)) {
                                    labelExtend(selectedLabel, currentNode, activeNodes_);
                                }
                            }
                            else {
                                activeNodeObj->activeLabels_[l]->status_ = DOMINATED;
                                activeNodeObj->nbActiveLabels_--;
                                nbActivated_--;
                                activeNodeObj->activeLabels_.erase(activeNodeObj->activeLabels_.begin() + l);
                            }
                        }
                    }
                }

                // decrease the number of active labels if truncated strategy is used
                if ((solverOptions_->isTruncated_) && (currentNode->nbActiveLabels_ > solverOptions_->MaxLabel_)){
                    nbActivated_ -= (currentNode->nbActiveLabels_ - solverOptions_->MaxLabel_);
                    truncateLabelList(currentNode, solverOptions_->MaxLabel_);
                }

                // drop onboards
                for (auto &label : currentNode->activeLabels_){
                    if (!label->isDropped_)
                        labelDrop(label, activeNodes_);
                }
            }
        }
        if (!solverOptions_->areHeuristicsDisabled()) {
            if (subGraph_->nodes_[(*Vehicle_)->sinkID_]->bestLabelReduceCost_ - (*Vehicle_)->dual_ >= -0.0001)
                solverOptions_->disableHeuristics();
            else
                break;
        } else
            break;
    }
    // sort final labels based on reduced cost
    subGraph_->nodes_[(*Vehicle_)->sinkID_]->activeLabels_.clear();
    std::map<int, std::vector<PLabel>>::reverse_iterator it;
    for (it = subGraph_->nodes_[(*Vehicle_)->sinkID_]->generatedLabels_.rbegin(); it != subGraph_->nodes_[(*Vehicle_)->sinkID_]->generatedLabels_.rend(); it++) {
        for (auto & labelObj: it->second) {
            subGraph_->nodes_[(*Vehicle_)->sinkID_]->activeLabels_.push_back(labelObj);
        }
    }
}

void LabelingSubProblem::solveDynamic() {
    if ((solverOptions_->LabelingStrategy_ == PUSHING)||(subRequests_.empty())){
        if (solverOptions_->isDropPickPossible_)
            this->solveDynamic_pushing();
        else
            this->solveDynamic_pushingDrop();
    }

    else if (solverOptions_->LabelingStrategy_ == PULLING)
        this->solveDynamic_pulling1();

    if (!subGraph_->nodes_[(*Vehicle_)->sinkID_]->activeLabels_.empty()) {
        bestReducedCost_ = subGraph_->nodes_[(*Vehicle_)->sinkID_]->bestLabelReduceCost_ - (*Vehicle_)->dual_;
        (*Vehicle_)->bestReducedCost_ = bestReducedCost_;
    }
    nbNegativeColumns_ = 0;
    for (auto & labelObj : subGraph_->nodes_[(*Vehicle_)->sinkID_]->activeLabels_) {
        if (labelObj->reducedCost_ - (*Vehicle_)->dual_ < 0)
            nbNegativeColumns_ ++;
    }
//    std::cout << this->toString() << std::endl;
}

void LabelingSubProblem::SolutionToRoutes(PVehicle &vehicle, vector<PRoute> &availableRoutes, std::unordered_map<std::string, PRoute> &generatedRoutes) {
    for (auto & labelObj : subGraph_->nodes_[vehicle->sinkID_]->activeLabels_) {
  //      if (labelObj->reducedCost_ - vehicle->dual_ <= 0) {
            PRoute newRoute = labelObj->labelToRoute(vehicle);
            availableRoutes.push_back(newRoute);
            generatedRoutes.insert(std::pair <std::string , PRoute> (newRoute->name_ , newRoute));
 //       }
    }
    /*if (!availableRoutes.empty()) {
        for (auto &nodeObj: availableRoutes[0]->routeNodes_) {
            if (nodeObj->type_ == PICKUP) {
                nodeObj->related_Request_->selectStatus_ = SELECTED;
            }
        }
    }*/
}

void LabelingSubProblem::SolutionToRoutes(PVehicle &vehicle, vector<PRoute> &availableRoutes,
                                          std::unordered_map<std::string, PRoute> &generatedRoutes, PInstance &pInst) {
    for (auto & labelObj : subGraph_->nodes_[vehicle->sinkID_]->activeLabels_) {
        //      if (labelObj->reducedCost_ - vehicle->dual_ <= 0) {
        PRoute newRoute = labelObj->labelToRoute(vehicle, pInst);
        availableRoutes.push_back(newRoute);
        generatedRoutes.insert(std::pair <std::string , PRoute> (newRoute->name_ , newRoute));
        //       }
    }
}

std::string LabelingSubProblem::toString() const {
    std::stringstream repStr;
    repStr << std::endl;
    repStr << "# ------------------------------------------------------------" << std::endl;
    repStr << "# SUB PROBLEM SOLUTION RESULT FOR VEHICLE: " << (*Vehicle_)->vehicleID_ << std::endl;
    repStr << "#" << std::endl;
    repStr << "# Solution status = " << "Optimal (Label Setting Algorithm)" << std::endl;
    repStr << "# In total, " << nbGenerated_ << " labels were generated." << std::endl;
    repStr << "# " << nbDominated_ << " labels were removed via Domination " << std::endl;
    repStr << "# " << nbEliminated_ << " labels were removed via Elimination " << std::endl;
    if (!subGraph_->nodes_[(*Vehicle_)->sinkID_]->activeLabels_.empty())
        repStr << "# best objective value = " << subGraph_->nodes_[(*Vehicle_)->sinkID_]->bestLabelReduceCost_ - (*Vehicle_)->dual_ << std::endl;
    repStr << "# The solution pool contains = " << subGraph_->nodes_[(*Vehicle_)->sinkID_]->activeLabels_.size() << " solutions." << std::endl;

    /*int nbValidSolution = 0;
    for (auto & labelObj : subGraph_->nodes_[(*Vehicle_)->sinkID_]->activeLabels_) {
        if (labelObj->reducedCost_ - (*Vehicle_)->dual_ < 0)
            nbValidSolution ++;
    }*/
    repStr << "# The solution pool contains = " << nbNegativeColumns_ << " solutions with negative reduced cost." << std::endl;
    return repStr.str();
}




void truncateLabelList(PNode &node, int MaxLabel) {
    sort(node->activeLabels_.begin(),node->activeLabels_.end(),[](const PLabel &lhs, const PLabel &rhs){
        return lhs->reducedCost_ < rhs->reducedCost_;});
    while (node->nbActiveLabels_ > MaxLabel) {
        if (node->activeLabels_.back()->status_ == ACTIVE) {
            node->nbActiveLabels_--;
            node->activeLabels_.back()->status_ = OUTBOUND;
            node->activeLabels_.pop_back();
        }
        else
            node->activeLabels_.pop_back();
    }
}


