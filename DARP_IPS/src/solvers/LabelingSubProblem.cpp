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
    nbOutputs_ = 0;
    maxPickup_ = 3;
    subproTime_ = new myTools::Timer(); subproTime_->init();
    subproRouteTime_ = new myTools::Timer(); subproRouteTime_->init();
    sortTime_ = new myTools::Timer(); sortTime_->init();
}
LabelingSubProblem::~LabelingSubProblem() {

    delete subproTime_;
    delete subproRouteTime_;
    delete sortTime_;
}

// this function sort the list of nodes based of their dual values
void LabelingSubProblem::sortNodes() {
//    requestIDToInt_.clear();
//    nodesOrder_.clear();
//    nodesOrder_.reserve(subRequests_.size());
    int counter = nbTotalRequest_;

    // sorting requests based on their dual values
    /*sort(subRequests_.begin(),subRequests_.end(),[](const PRequest &lhs, const PRequest &rhs){
        return lhs->dual_ > rhs->dual_;});*/

    // add pickup nodes
    /*for (auto &requestObj : subRequests_) {
        std::string pickID = myTools::createNodeID(requestObj->getRequestId(), PICKUP);
        nodesOrder_.push_back(subGraph_->nodes_[pickID]);
        requestIDToInt_[requestObj->getRequestId()] = counter;
        counter++;
    }*/
    // add drop off points
    /*for (auto &requestObj : subRequests_) {
        if (requestObj->dual_ >= 0) {
            std::string dropID = myTools::createNodeID(requestObj->getRequestId(), DROPOFF);
//            subGraph_->intToNodeID_.push_back(dropID);
        }
    }*/
    // adding onboard nodes
    for (auto & nodeObj : onboards_) {
//        requestIDToInt_[subGraph_->nodes_[nodeID]->related_Request_->getRequestId()] = counter;
//        subGraph_->nodes_[nodeID]->related_Request_->taskIndex_ = counter;
        nodeObj->related_Request_->taskIndexLabel_ = counter;
        counter++;
    }

    // adding sink node
//    subGraph_->intToNodeID_.push_back((*Vehicle_)->sinkID_);
}

void LabelingSubProblem::sortSuccessors(std::vector<PNode> &nodeList) {
    /*for (auto & nodeObj: subGraph_->nodes_) {
        nodeObj.second->predecessor_.clear();
    }*/
    /*for (auto & nodeObj: nodes_){
        if (nodeObj.second->type_ != SINK){
            nodeObj.second->successors_.clear();
            for (auto &pickNodeObj : pickNodes_) {
                if (nodeObj.second->nodeID_ != pickNodeObj->nodeID_) {
                    pickNodeObj->travelTimeFromNode_ = durationMatrix_[nodeObj.second->locationID_][pickNodeObj->locationID_];
                    if (pickNodeObj->nodeStatus_ != COMMITTED)
                        nodeObj.second->successors_.push_back(&pickNodeObj);
                }
            }
            sort(nodeObj.second->successors_.begin(),nodeObj.second->successors_.end(),[](std::shared_ptr<Node> *lhs, std::shared_ptr<Node> *rhs){
                return (*lhs)->travelTimeFromNode_ < (*rhs)->travelTimeFromNode_;});
            if (solverOptions_->isSuccessorsLimited_) {
                int location = (int)floor(1*nodeObj.second->successors_.size()/2) + 1;
                nodeObj.second->successors_.erase(nodeObj.second->successors_.begin()+location, nodeObj.second->successors_.end());
            }
        }
    }*/
    for (auto & nodeObj: nodeList){
        if (nodeObj->type_ != SINK){
            nodeObj->successors_.clear();
            for (auto &pickNodeObj : subGraph_->pickNodes_) {
                if (nodeObj->nodeID_ != pickNodeObj->nodeID_) {
                    pickNodeObj->travelTimeFromNode_ = durationMatrix_[nodeObj->locationID_][pickNodeObj->locationID_];
                    if (pickNodeObj->nodeStatus_ != COMMITTED)
                        nodeObj->successors_.push_back(&pickNodeObj);
                }
            }
            sort(nodeObj->successors_.begin(),nodeObj->successors_.end(),[](std::shared_ptr<Node> *lhs, std::shared_ptr<Node> *rhs){
                return (*lhs)->travelTimeFromNode_ < (*rhs)->travelTimeFromNode_;});
            if (solverOptions_->isSuccessorsLimited_) {
                int location = (int)floor(1*nodeObj->successors_.size()/2) + 1;
                nodeObj->successors_.erase(nodeObj->successors_.begin()+location, nodeObj->successors_.end());
            }
        }
    }
}


// reset that active lists of the nodes, create the first label at the source, add onboards
void LabelingSubProblem::initialization() {

    activeNodes_.reserve(subGraph_->pickNodes_.size()*2+ onboards_.size()+2);
    nbActivated_ = 0;
    // clear active lists

    /*for (auto &nodeObj: nodes_) {
        nodeObj.second->activeLabels_.clear();
        nodeObj.second->bestLabelReduceCost_ = INFINITY;
        nodeObj.second->nbActiveLabels_ = 0;
 //       nodeObj.second->generatedLabel_.clear();
        // if the length of the route is not limited, this command should be changed
 //       nodeObj.second->generatedLabel_.resize(maxPickup_ + 1);
    }*/

    sortNodes();
    sortSuccessors(subGraph_->sourceNodes_);
    sortSuccessors(subGraph_->pickNodes_);
    sortSuccessors(subGraph_->dropNodes_);
    sortSuccessors(onboards_);

    // create the initial label at the source and add the source to the list active nodes
    PLabel initialLabel = std::make_shared<Label>(Vehicle_, subGraph_->sourceNodes_[0]);
    initialLabel->completedRequests_.resize(nbTotalRequest_ + (*Vehicle_)->onboards_.size());
    initialLabel->travelResources_.resize(nbTotalRequest_ + (*Vehicle_)->onboards_.size());
    // update travel resource for the initial label based on the onboards
    for (auto &nodeObj: onboards_) {
        initialLabel->openNode_.push_back(nodeObj);
        initialLabel->completedRequests_[nodeObj->related_Request_->taskIndexLabel_] = 1;
        float remainedTime = nodeObj->related_Request_->maxTravelTime_ - (*Vehicle_)->departTime_ +
                nodeObj->related_Request_->pickTime_ + nodeObj->related_Request_->deltaTime_;

        initialLabel->travelResources_[nodeObj->related_Request_->taskIndexLabel_] = remainedTime;
    }
    for (int i = 0; i < nbTotalRequest_ + (*Vehicle_)->onboards_.size(); ++i){
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

//    initialLabel->currentNode_->generatedLabel_.resize(maxPickup_ + 1);
//    initialLabel->currentNode_->generatedLabel_[initialLabel->nbPickUp_].push_back(initialLabel);
    initialLabel->currentNode_->nbActiveLabels_++;
    initialLabel->currentNode_->bestLabelReduceCost_ = initialLabel->reducedCost_;
    activeNodes_.clear();
    activeNodes_.push_back(initialLabel->currentNode_);
    initialLabel->currentNode_->activeLabels_.push_back(std::move(initialLabel));
    nbActivated_ ++;
}

void LabelingSubProblem::labelExtend2(PLabel &parentLabel, PNode &outNode) {
    PLabel newLabel = std::make_shared<Label>(*parentLabel);

//    newLabel->parent_ = parentLabel;
    newLabel->extend(outNode);
    nbGenerated_++;
    if (!newLabel->isEliminated()) {
        if (!isLabelAdded2(newLabel, outNode))
            nbDominated_++;
        else if(outNode->type_ == SINK)
            newLabel->createTime_ = subproTime_->dSinceInit().count();
    }
    else {
        nbEliminated_++;
        newLabel->status_ = DOMINATED;
        dominatedLabels_.push_back(std::move(newLabel));
    }
}

void LabelingSubProblem::labelExtend(PLabel &parentLabel, PNode &outNode) {
    PLabel newLabel;

    if (!dominatedLabels_.empty()){
        newLabel = dominatedLabels_.back();
        dominatedLabels_.pop_back();
        newLabel->copyLabel(*parentLabel);
    }
    else {
        newLabel = std::make_shared<Label>(*parentLabel);
    }

//    newLabel->parent_ = parentLabel;
    newLabel->extend(outNode);
    nbGenerated_++;
    if (!newLabel->isEliminated()) {
        if (!isLabelAdded2(newLabel, outNode))
            nbDominated_++;
    }
    else {
        nbEliminated_++;
        newLabel->status_ = DOMINATED;
        dominatedLabels_.push_back(std::move(newLabel));
    }
}

void LabelingSubProblem::labelDrop(PLabel &parentLabel, vector<PNode> &activeNodeList) {
    std::vector<PLabel> openLabelList;
    openLabelList.push_back(parentLabel);
    while(!openLabelList.empty()) {
        PLabel selectedLabel = openLabelList.back();
        openLabelList.pop_back();
        /*std::vector<PNode> outNodes(selectedLabel->openNodes_.begin(),
                                    selectedLabel->openNodes_.end());*/
        std::vector<PNode> outNodes;
        for (auto & nodeObj : selectedLabel->openNode_)
            outNodes.push_back(nodeObj);
        /*std::vector<PNode> outNodes(selectedLabel->openNodes_.begin(),
                                    selectedLabel->openNodes_.end());*/
        for (auto &neighbourNode: outNodes) {
            PLabel newLabel = std::make_shared<Label>(*parentLabel);
//            newLabel->parent_ = parentLabel;
            newLabel->extend(neighbourNode);
            nbGenerated_++;
            if (!newLabel->isEliminated()) {
                if (isLabelAdded2(newLabel, neighbourNode)) {
  //                  if (!newLabel->openNodes_.empty())
                    if (!newLabel->openNode_.empty())
                        // add to label list for pushing to other drop off points
                        openLabelList.push_back(newLabel);
                    else
                        // terminate to the sink
                        labelExtend(newLabel, subGraph_->sinkNodes_[0]);
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
        /*if (selectedLabel->extendCheck_.size() == nodesOrder_.size() ||
            selectedLabel->nbPickUp_ == solverOptions_->maxPickup_) {
            selectedLabel->status_ = INACTIVE;
            selectedLabel->currentNode_->nbActiveLabels_--;
            nbActivated_--;
        }*/
 //       selectedLabel.reset();
    }
}


/*bool LabelingSubProblem::isLabelAdded(PLabel &newLabel, PNode &outNode) {
    // check the dominance state of the new label
    for (int i = 0; i <= newLabel->nbPickUp_; i++){
        for (auto labelObj: outNode->generatedLabel_[i]) {
            if (newLabel->isDominated(labelObj, this->solverOptions_)){
                newLabel->status_ = DOMINATED;
                dominatedLabels_.push_back(newLabel);
                this->nbDominated_++;
//                newLabel.reset();
                return false;
            }
        }
    }
    // remove previous dominated labels
    for (int j = newLabel->nbPickUp_; j < outNode->generatedLabel_.size(); j++){
        for (int i = outNode->generatedLabel_[j].size()-1; i >= 0; --i){
            if (outNode->generatedLabel_[j][i]->isDominated(newLabel, this->solverOptions_)){
                if (outNode->generatedLabel_[j][i]->status_ == ACTIVE){
                    outNode->nbActiveLabels_--;
                    this->nbActivated_--;
                }
                outNode->generatedLabel_[j][i]->status_ = DOMINATED;
                dominatedLabels_.push_back(outNode->generatedLabel_[j][i]);
                outNode->generatedLabel_[j].erase(outNode->generatedLabel_[j].begin() + i);
                this->nbDominated_++;
            }
        }
    }

    outNode->generatedLabel_[newLabel->nbPickUp_].push_back(newLabel);
    outNode->activeLabels_.push_back(newLabel);
    if (outNode->bestLabelReduceCost_ > newLabel->reducedCost_)
        outNode->bestLabelReduceCost_ = newLabel->reducedCost_;
    if (outNode->type_ == SINK)
        newLabel->status_ = TERMINATED;
    else {
        outNode->nbActiveLabels_++;
        this->nbActivated_++;
    }
    return true;
}*/

bool LabelingSubProblem::isLabelAdded2(PLabel &newLabel, PNode &outNode) {
    // check the dominance state of the new label
    for (auto & labelObj: outNode->activeLabels_){
        if (newLabel->isDominated(labelObj, this->solverOptions_)){
            newLabel->status_ = DOMINATED;
            dominatedLabels_.push_back(std::move(newLabel));
            this->nbDominated_++;
            return false;
        }
    }
    // remove previous dominated labels
    for (int i = outNode->activeLabels_.size()-1; i >= 0; i--){
        if (outNode->activeLabels_[i]->isDominated(newLabel, this->solverOptions_)){
            if (outNode->activeLabels_[i]->status_ == ACTIVE){
                outNode->nbActiveLabels_--;
                this->nbActivated_--;
            }
            outNode->activeLabels_[i]->status_ = DOMINATED;
            dominatedLabels_.push_back(outNode->activeLabels_[i]);
            outNode->activeLabels_.erase(outNode->activeLabels_.begin() + i);
            this->nbDominated_++;
        }
    }
    if (outNode->bestLabelReduceCost_ > newLabel->reducedCost_)
        outNode->bestLabelReduceCost_ = newLabel->reducedCost_;
    if (outNode->maxLabelReducedCost_ < newLabel->reducedCost_)
        outNode->maxLabelReducedCost_ = newLabel->reducedCost_;

    if (outNode->type_ == SINK){
        newLabel->status_ = TERMINATED;
        newLabel->createTime_ = subproTime_->dSinceInit().count();
        outNode->activeLabels_.push_back(std::move(newLabel));
    }
    else{
        outNode->activeLabels_.push_back(std::move(newLabel));
        outNode->nbActiveLabels_++;
        this->nbActivated_++;
    }
    return true;
}

void LabelingSubProblem::solveDynamic_pulling1() {
    // create initial label
    int nbActive;
    while(true) {
        // create initial label
        initialization();

        while (!activeNodes_.empty()) {
            // select a node to pull other labels to it
            for (auto &currentNode: subGraph_->pickNodes_) {
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
  //                                      if (!selectedLabel->openNodes_.empty()) {
                                        if (!selectedLabel->openNode_.empty()) {
                                            /*std::vector<PNode> outNodes(selectedLabel->openNodes_.begin(),
                                                                        selectedLabel->openNodes_.end());*/
                                            std::vector<PNode> outNodes;
                                            for (auto & nodeObj: selectedLabel->openNode_)
                                                outNodes.push_back(nodeObj);
                                            for (auto &onboardNode: outNodes) {
                                                nbActive = onboardNode->nbActiveLabels_;
                                                labelExtend(selectedLabel, onboardNode);
                                                if ((onboardNode->nbActiveLabels_ == 1) && (nbActive == 0)) {
                                                    activeNodes_.push_back(onboardNode);
                                                }
                                            }
                                            selectedLabel->isDropped_ = true;
                                        }
                                        // push to the sink
               //                         else if (selectedLabel->openNodes_.empty()) {
                                        else if (selectedLabel->openNode_.empty()) {
                                            labelExtend(selectedLabel, subGraph_->sinkNodes_[0]);
                                            selectedLabel->isDropped_ = true;
                                        }
                                    }
                                    /*if (selectedLabel->extendCheck_.size() == nodesOrder_.size() ||
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
                                    }*/
                                        // pull all labels to the current node
                                    else if ((activeNodes_[j]->nodeID_ != currentNode->nodeID_) &&
                                             (selectedLabel->isExtendFeasible(currentNode,maxPickup_))) {
                                        nbActive = currentNode->nbActiveLabels_;
                                        labelExtend(selectedLabel, currentNode);
                                        if ((currentNode->nbActiveLabels_ == 1) && (nbActive == 0)) {
                                            activeNodes_.push_back(currentNode);
                                        }
                                    }
   //                                 selectedLabel.reset();
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
            if ((subGraph_->sinkNodes_[0])->bestLabelReduceCost_ - (*Vehicle_)->dual_ >= -0.0001) {
                solverOptions_->disableHeuristics();
                std::cout << " Disable Heuristics!!!" << std::endl;
            }
            else
                break;
        } else
            break;
    }
    // sort final labels based on reduced cost
    /*sinkNode_->activeLabels_.clear();
    for (int i = sinkNode_->generatedLabel_.size()-1; i >= 0; i--){
        for (auto &labelObj: sinkNode_->generatedLabel_[i]) {
            if (!labelObj->haveDominatedParent())
                sinkNode_->activeLabels_.push_back(labelObj);
        }
    }*/
}

//***************************************************************************************//
//                           P U S H I N G  S T R A T E G Y
//***************************************************************************************//
void LabelingSubProblem::solveDynamic_pushing() {
    // create initial label

    int nbActive;
    while(true) {
        // create initial label
        initialization();


        while (!activeNodes_.empty()) {
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
            for (int j = currentNode->activeLabels_.size()-1; j >=0; j--) {
                if (currentNode->activeLabels_[j]->status_ == ACTIVE) {
                    PLabel selectedLabel = currentNode->activeLabels_[j];
                    currentNode->nbActiveLabels_--;
                    nbActivated_--;
                    selectedLabel->status_ = INACTIVE;
                    // terminate to sink
                    if (selectedLabel->openNode_.empty())
                        labelExtend(selectedLabel, subGraph_->sinkNodes_[0]);
                        // drop onboards
                    else {
                        std::vector<PNode> outNodes;
                        for (auto &nodeObj: selectedLabel->openNode_)
                            outNodes.push_back(nodeObj);
                        for (auto &neighbourNode: outNodes) {
                            nbActive = neighbourNode->nbActiveLabels_;
                            labelExtend(selectedLabel, neighbourNode);
                            if ((neighbourNode->nbActiveLabels_ == 1) && (nbActive == 0)) {
                                activeNodes_.push_back(neighbourNode);
                            }
                        }
                    }
                    if (selectedLabel->nbPickUp_ != maxPickup_) {
                        // push to pickup points
                        for (auto &neighbourNode: selectedLabel->currentNode_->successors_) {
                            if (selectedLabel->isExtendFeasible(*neighbourNode, maxPickup_)) {
                                nbActive = (*neighbourNode)->nbActiveLabels_;
                                labelExtend(selectedLabel, (*neighbourNode));
                                if (((*neighbourNode)->nbActiveLabels_ == 1) && (nbActive == 0)) {
                                    activeNodes_.push_back(*neighbourNode);
                                }
                            }
                        }
                    }
                }
            }
        }
        break;
    }
}

void LabelingSubProblem::solveDynamic_pushingDrop() {
// create initial label
    int nbActive;
    while(true) {
        // create initial label
        initialization();


        while (!activeNodes_.empty()) {
            sort(activeNodes_.begin(),activeNodes_.end(),[](const PNode &lhs, const PNode &rhs){
                return lhs->travelTimeFromNode_ > rhs->travelTimeFromNode_;});
            // select a node to extend active labels
            PNode currentNode = activeNodes_.back();

            activeNodes_.pop_back();

            // remove de-active labels
            for (int i = currentNode->activeLabels_.size()-1; i >= 0; --i){
                if (currentNode->activeLabels_[i]->status_ != ACTIVE){
                    currentNode->activeLabels_.erase(currentNode->activeLabels_.begin() + i);
                }
            }

            // decrease the number of active labels if truncated strategy is used
            if ((solverOptions_->isTruncated_) && (currentNode->nbActiveLabels_ > solverOptions_->MaxLabel_)){
                nbActivated_ -= (currentNode->nbActiveLabels_ - solverOptions_->MaxLabel_);
                truncateLabelList(currentNode, solverOptions_->MaxLabel_, dominatedLabels_);
            }

            while (!currentNode->activeLabels_.empty()){
                if ((currentNode->activeLabels_.back()->status_ != ACTIVE)||
                    (currentNode->activeLabels_.back()->currentNode_->nodeID_ != currentNode->nodeID_ && currentNode->activeLabels_.back()->nbUsed_ == 1))
                    currentNode->activeLabels_.pop_back();
                else {
                    // if the heuristics are node disabled, we are not allowed to dominate labels with dominated parents
                    PLabel selectedLabel = currentNode->activeLabels_.back();
                    currentNode->activeLabels_.pop_back();
                    currentNode->nbActiveLabels_--;
                    nbActivated_--;
                    selectedLabel->status_ = INACTIVE;
                    // terminate to sink
                    if (selectedLabel->openNode_.empty())
                        labelExtend(selectedLabel, subGraph_->sinkNodes_[0]);
                        // drop onboards
                    else {
                        /*std::vector<PNode> outNodes(selectedLabel->openNodes_.begin(),
                                                    selectedLabel->openNodes_.end());*/
                        std::vector<PNode> outNodes;
                        for (auto & nodeObj : selectedLabel->openNode_)
                            outNodes.push_back(nodeObj);
                        for (auto &neighbourNode: outNodes) {
                            nbActive = neighbourNode->nbActiveLabels_;
                            labelExtend(selectedLabel, neighbourNode);
                            if ((neighbourNode->nbActiveLabels_ == 1)&&(nbActive == 0)) {
                                activeNodes_.push_back(neighbourNode);
                            }
                            if (!neighbourNode->activeLabels_.empty())
                                neighbourNode->activeLabels_.back()->isDropped_ = true;
                        }
                    }
                    if ((selectedLabel->nbPickUp_ != maxPickup_)&&(!selectedLabel->isDropped_)) {
                        // push to pickup points
                        for (auto &neighbourNode: selectedLabel->currentNode_->successors_) {
                            if (selectedLabel->isExtendFeasible(*neighbourNode, maxPickup_)) {
                                nbActive = (*neighbourNode)->nbActiveLabels_;
                                labelExtend(selectedLabel, *neighbourNode);
                                if (((*neighbourNode)->nbActiveLabels_ == 1)&&(nbActive == 0)) {
                                    activeNodes_.push_back(*neighbourNode);
                                }
                            }
                        }
                    }
                }
            }
            currentNode.reset();
        }
        break;
        if (!solverOptions_->areHeuristicsDisabled()) {
            if ((subGraph_->sinkNodes_[0])->bestLabelReduceCost_ - (*Vehicle_)->dual_ >= -0.0001)
                solverOptions_->disableHeuristics();
            else
                break;
        } else
            break;
    }
    /*sinkNode_->activeLabels_.clear();
    for (int i = sinkNode_->generatedLabel_.size()-1; i >= 0; i--){
        for (auto &labelObj: sinkNode_->generatedLabel_[i]) {
            if (labelObj->status_ == TERMINATED)
                sinkNode_->activeLabels_.push_back(labelObj);
        }
    }*/
}

void LabelingSubProblem::solveDynamic_pushingWave() {
// create initial label
    while(true) {
        // create initial label
        initialization();
        activeNodes_.clear();
        for (auto & nodeObj : subGraph_->nodes_){
            if (nodeObj.second->type_ != SINK)
                activeNodes_.push_back(nodeObj.second);
        }
        sort(activeNodes_.begin(),activeNodes_.end(),[](const PNode &lhs, const PNode &rhs){
            return lhs->travelTimeFromSource_ < rhs->travelTimeFromSource_;});
        while (nbActivated_ > 0){
            for (auto & currentNode : activeNodes_){
                if (currentNode->nbActiveLabels_ > 0){
                    for (auto &selectedLabel: currentNode->activeLabels_) {
                        if (selectedLabel->status_ == ACTIVE){
                            currentNode->nbActiveLabels_--;
                            nbActivated_--;
                            selectedLabel->status_ = INACTIVE;
                            // push to pickup points
                            for (auto &neighbourNode: selectedLabel->currentNode_->successors_) {
                                if (selectedLabel->isExtendFeasible(*neighbourNode, maxPickup_)) {
                                    labelExtend(selectedLabel, *neighbourNode);
                                }
                            }
                        }
                    }
                }
            }
        }
        for (auto & currentNode : activeNodes_){
            for (auto &selectedLabel: currentNode->activeLabels_) {
                selectedLabel->status_ = ACTIVE;
                currentNode->nbActiveLabels_++;
                nbActivated_++;
            }
        }
        while (nbActivated_ > 0){
            for (auto & currentNode : activeNodes_){
                if (currentNode->nbActiveLabels_ > 0){
                    for (auto &selectedLabel: currentNode->activeLabels_) {
                        if (selectedLabel->status_ == ACTIVE){
                            currentNode->nbActiveLabels_--;
                            nbActivated_--;
                            selectedLabel->status_ = INACTIVE;
                            // terminate to sink
                            if (selectedLabel->openNode_.empty())
                                labelExtend(selectedLabel, subGraph_->sinkNodes_[0]);
                                // drop onboards
                            else {
                                std::vector<PNode> outNodes;
                                for (auto & nodeObj : selectedLabel->openNode_)
                                    outNodes.push_back(nodeObj);
                                for (auto &neighbourNode: outNodes) {
                                    labelExtend(selectedLabel, neighbourNode);
                                }
                            }
                        }
                    }
                }
            }
        }
        break;
    }
    /*sinkNode_->activeLabels_.clear();
    for (int i = sinkNode_->generatedLabel_.size()-1; i >= 0; i--){
        for (auto &labelObj: sinkNode_->generatedLabel_[i]) {
            if (labelObj->status_ == TERMINATED)
                sinkNode_->activeLabels_.push_back(labelObj);
        }
    }*/
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
            for (auto &currentNode: subGraph_->pickNodes_) {
                sort(activeNodes_.begin(),activeNodes_.end(),[](const PNode &lhs, const PNode &rhs){
                    return lhs->nbActiveLabels_ > rhs->nbActiveLabels_;});
                for (auto & activeNodeObj : activeNodes_){
                    for (int l = activeNodeObj->activeLabels_.size()-1; l >= 0; l--){
                        if (activeNodeObj->activeLabels_[l]->status_ != ACTIVE)
                            activeNodeObj->activeLabels_.erase(activeNodeObj->activeLabels_.begin() + l);
                        else {
                            if ((!solverOptions_->areHeuristicsDisabled())||(!activeNodeObj->activeLabels_[l]->haveDominatedParent())) {
                                PLabel selectedLabel = activeNodeObj->activeLabels_[l];
                                /*if (selectedLabel->extendCheck_.size() == nodesOrder_.size() ||
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
                                }*/
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
            if ((subGraph_->sinkNodes_[0])->bestLabelReduceCost_ - (*Vehicle_)->dual_ >= -0.0001)
                solverOptions_->disableHeuristics();
            else
                break;
        } else
            break;
    }
    // sort final labels based on reduced cost
    /*sinkNode_->activeLabels_.clear();
    for (int i = sinkNode_->generatedLabel_.size()-1; i >= 0; i--){
        for (auto &labelObj: sinkNode_->generatedLabel_[i]) {
            sinkNode_->activeLabels_.push_back(labelObj);
        }
    }*/
}

void LabelingSubProblem::solveDynamic() {
    subproTime_->start();
    if ((solverOptions_->LabelingStrategy_ == PUSHING)||(subRequests_.empty())){
        if (solverOptions_->isDropPickPossible_)
            this->solveDynamic_pushing();
        else
//            this->solveDynamic_pushingWave();
            this->solveDynamic_pushingDrop();
    }

    else if (solverOptions_->LabelingStrategy_ == PULLING)
        this->solveDynamic_pulling1();

    if (!subGraph_->sinkNodes_[0]->activeLabels_.empty()) {
        bestReducedCost_ = subGraph_->sinkNodes_[0]->bestLabelReduceCost_ - (*Vehicle_)->dual_;
        (*Vehicle_)->bestReducedCost_ = bestReducedCost_;
    }
    nbNegativeColumns_ = 0;
    for (auto & labelObj : subGraph_->sinkNodes_[0]->activeLabels_) {
        if (labelObj->reducedCost_ - (*Vehicle_)->dual_ < 0)
            nbNegativeColumns_ ++;
    }
    subproTime_->stop();
    std::cout << this->toString() << std::endl;
}

void LabelingSubProblem::reconstructLabels(std::vector<PRoute> &availableRoutes) {
    subproTime_->start();
    nbActivated_ = 0;
    // clear active lists
    /*for (auto &nodeObj: nodes_) {
        nodeObj.second->activeLabels_.clear();
        nodeObj.second->bestLabelReduceCost_ = INFINITY;
        nodeObj.second->nbActiveLabels_ = 0;
//        nodeObj.second->generatedLabel_.clear();
        // if the length of the route is not limited, this command should be changed
//        nodeObj.second->generatedLabel_.resize(5);
    }*/
    // create the initial label at the source and add the source to the list active nodes
    PLabel initialLabel = std::make_shared<Label>(Vehicle_, subGraph_->sourceNodes_[0]);
    initialLabel->completedRequests_.resize(nbTotalRequest_ + (*Vehicle_)->onboards_.size());
    initialLabel->travelResources_.resize(nbTotalRequest_ + (*Vehicle_)->onboards_.size());
    // update travel resource for the initial label based on the onboards
    for (auto &nodeObj: onboards_) {
        initialLabel->openNode_.push_back(nodeObj);
        initialLabel->completedRequests_[nodeObj->related_Request_->taskIndexLabel_] = 1;
        float remainedTime = nodeObj->related_Request_->maxTravelTime_ - (*Vehicle_)->departTime_ +
                             nodeObj->related_Request_->pickTime_ + nodeObj->related_Request_->deltaTime_;

        initialLabel->travelResources_[nodeObj->related_Request_->taskIndexLabel_] = remainedTime;
    }

    for (int i = 0; i < nbTotalRequest_ + (*Vehicle_)->onboards_.size(); ++i){
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

    for (auto & routeObj: availableRoutes) {
        bool isRemoved = false;
        if (routeObj->routeSize_ > 1) {
            PLabel newLabel = std::make_shared<Label>(*initialLabel);
            for (int i = 1; i < routeObj->routeNodes_.size(); ++i) {
                if (subGraph_->nodes_.count(routeObj->routeNodes_[i]->nodeID_)>0 &&
                    routeObj->routeNodes_[i]->type_ != SOURCE){
                    if (newLabel->isExtendFeasible(routeObj->routeNodes_[i], solverOptions_->MaxLabel_)) {
                        newLabel->extend(routeObj->routeNodes_[i]);
                        if (newLabel->isEliminated()){
                            isRemoved = true;
                            break;
                        }
                    }
                }
            }
            if (!isRemoved){
                if (!newLabel->openNode_.empty()){
                    std::vector<PNode> outNodes;
                    for (auto & nodeObj : newLabel->openNode_)
                        outNodes.push_back(nodeObj);
                    for (auto &neighbourNode: outNodes) {
                        if (newLabel->isExtendFeasible(neighbourNode, solverOptions_->MaxLabel_)) {
                            newLabel->extend(neighbourNode);
                            if (newLabel->isEliminated()){
                                isRemoved = true;
                                break;
                            }
                        }
                        else {
                            isRemoved = true;
                            break;
                        }
                    }
                    if (!isRemoved){
                        newLabel->extend(subGraph_->sinkNodes_[0]);
                        subGraph_->sinkNodes_[0]->activeLabels_.push_back(newLabel);
                        if (subGraph_->sinkNodes_[0]->bestLabelReduceCost_ > newLabel->reducedCost_)
                            subGraph_->sinkNodes_[0]->bestLabelReduceCost_ = newLabel->reducedCost_;
                        newLabel->createTime_ = subproTime_->dSinceInit().count();
                    }
                }
            }
        }
    }
    subproTime_->stop();
}

/*void LabelingSubProblem::SolutionToRoutes(PVehicle &vehicle, vector<PRoute> &availableRoutes) {
//    availableRoutes.reserve(subGraph_->nodes_[vehicle->sinkID_]->activeLabels_.size());
    for (auto & labelObj : (*sinkNode_)->activeLabels_) {
  //      if (labelObj->reducedCost_ - vehicle->dual_ <= 0) {
            availableRoutes.emplace_back(labelObj->labelToRoute(vehicle));
//            generatedRoutes.insert(std::pair <std::string , PRoute> (availableRoutes.back()->name_ , availableRoutes.back()));
 //       }
    }
    *//*if (!availableRoutes.empty()) {
        for (auto &nodeObj: availableRoutes[0]->routeNodes_) {
            if (nodeObj->type_ == PICKUP) {
                nodeObj->related_Request_->selectStatus_ = SELECTED;
            }
        }
    }*//*
}*/

void LabelingSubProblem::SolutionToRoutes(PVehicle &vehicle, vector<PRoute> &availableRoutes, PInstance &pInst) {

    subproRouteTime_->start();
//    availableRoutes.reserve(subGraph_->nodes_[vehicle->sinkID_]->activeLabels_.size());
    for (auto & labelObj : subGraph_->sinkNodes_[0]->activeLabels_) {
        //      if (labelObj->reducedCost_ - vehicle->dual_ <= 0) {
        availableRoutes.emplace_back(labelObj->labelToRoute(vehicle, pInst));
        nbOutputs_++;
//        generatedRoutes.insert(std::pair <std::string , PRoute> (availableRoutes.back()->name_ , availableRoutes.back()));
        //       }
    }
    subproRouteTime_->stop();
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
    if (!subGraph_->sinkNodes_[0]->activeLabels_.empty())
        repStr << "# best objective value = " << subGraph_->sinkNodes_[0]->bestLabelReduceCost_ - (*Vehicle_)->dual_ << std::endl;
    repStr << "# The solution pool contains = " << subGraph_->sinkNodes_[0]->activeLabels_.size() << " solutions." << std::endl;

    /*int nbValidSolution = 0;
    for (auto & labelObj : subGraph_->nodes_[(*Vehicle_)->sinkID_]->activeLabels_) {
        if (labelObj->reducedCost_ - (*Vehicle_)->dual_ < 0)
            nbValidSolution ++;
    }*/
    repStr << "# The solution pool contains = " << nbNegativeColumns_ << " solutions with negative reduced cost." << std::endl;
    return repStr.str();
}

std::string LabelingSubProblem::toStringOut(int epoch) const {
    std::stringstream repStr;
    repStr << epoch << ",";
    repStr << (*Vehicle_)->vehicleID_ << ",";
    repStr << subRequests_.size() << ",";
    repStr << subGraph_->nbNodes_-2 << ",";
    repStr << maxPickup_ << ",";
    repStr << nbGenerated_ << ",";
    repStr << nbEliminated_ << ",";
    repStr << nbDominated_ << ",";
    repStr << nbOutputs_ << ",";
    repStr << subproTime_->dSinceInit().count() << ",";
    repStr << subproRouteTime_->dSinceInit().count() << ",";
    repStr << sortTime_->dSinceInit().count() << "\n";
    return repStr.str();
}


void truncateLabelList(PNode &node, int MaxLabel, std::vector<PLabel> & dominatedLabels) {
    sort(node->activeLabels_.begin(),node->activeLabels_.end(),[](const PLabel &lhs, const PLabel &rhs){
        return lhs->reducedCost_ < rhs->reducedCost_;});
    for (int i = node->activeLabels_.size()-1; i >=0; i--){
        if (node->nbActiveLabels_ <= MaxLabel)
            break;
        else {
            if (node->activeLabels_[i]->status_ == ACTIVE){
                node->nbActiveLabels_--;
                node->activeLabels_[i]->status_ = DOMINATED;
                dominatedLabels.push_back(node->activeLabels_[i]);
                node->activeLabels_.erase(node->activeLabels_.begin() + i);
            }
        }
    }
    /*while (node->nbActiveLabels_ > MaxLabel) {
        node->nbActiveLabels_--;
        node->activeLabels_.back()->status_ = DOMINATED;
        dominatedLabels.push_back(node->activeLabels_.back());
        node->activeLabels_.pop_back();
        *//*if (node->activeLabels_.back()->status_ == ACTIVE) {
            node->nbActiveLabels_--;
 //           node->activeLabels_.back()->status_ = OUTBOUND;
            node->activeLabels_.back()->status_ = DOMINATED;
            dominatedLabels.push_back(node->activeLabels_.back());
            node->activeLabels_.pop_back();
        }
        else
            node->activeLabels_.pop_back();*//*
    }*/
}

void truncateLabelList(PNode &node, int MaxLabel) {
    sort(node->activeLabels_.begin(),node->activeLabels_.end(),[](const PLabel &lhs, const PLabel &rhs){
        return lhs->reducedCost_ < rhs->reducedCost_;});
    for (int i = node->activeLabels_.size()-1; i >=0; i--){
        if (node->nbActiveLabels_ <= MaxLabel)
            break;
        else {
            if (node->activeLabels_[i]->status_ == ACTIVE){
                node->nbActiveLabels_--;
                node->activeLabels_[i]->status_ = OUTBOUND;
                node->activeLabels_.erase(node->activeLabels_.begin() + i);
            }
        }
    }
    /*while (node->nbActiveLabels_ > MaxLabel) {
        node->nbActiveLabels_--;
        node->activeLabels_.back()->status_ = OUTBOUND;
        node->activeLabels_.pop_back();
        *//*if (node->activeLabels_.back()->status_ == ACTIVE) {
            node->nbActiveLabels_--;
            node->activeLabels_.back()->status_ = OUTBOUND;
            node->activeLabels_.pop_back();
        }
        else
            node->activeLabels_.pop_back();*//*
    }*/
}

