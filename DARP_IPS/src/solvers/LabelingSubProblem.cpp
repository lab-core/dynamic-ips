//
// Created by Ella on 1/28/2022.
//

#include "LabelingSubProblem.h"

#include <utility>



//-----------------------------------------------------------------------------
//  Labeling Sub problem class
//-----------------------------------------------------------------------------


LabelingSubProblem::LabelingSubProblem(PVehicle &vehicle, PSolverOption &solverOptions) :
                                        SubproModeler(vehicle), solverOptions_(solverOptions) {
    nbGenerated_ = 0;
    nbDominated_ = 0;
    nbUnreachableDTrip_ = 0;
    nbUnreachableDelay_ = 0;
    nbOutputs_ = 0;
    maxPickup_ = solverOptions_->nbPick_;
 //   subproTime_ = new myTools::Timer(); subproTime_->init();
}
LabelingSubProblem::~LabelingSubProblem() {

 //   delete subproTime_;
}

void LabelingSubProblem::sortSuccessors(std::vector<PNode> &nodeList) {
    for (auto & nodeObj: nodeList){
        if (nodeObj->type_ != SINK){
            nodeObj->successors_.clear();
            for (auto &pickNodeObj : subGraph_->pickNodes_) {
                if (nodeObj->nodeID_ != pickNodeObj->nodeID_ && pickNodeObj->nodeStatus_ != COMMITTED) {
                    nodeObj->successors_.push_back(&(*pickNodeObj));
                }
            }
        }
    }
}


// reset that active lists of the nodes, create the first label at the source, add onboards
void LabelingSubProblem::initialization() {
    bool extendToOnboard = false;
    sortSuccessors(subGraph_->sourceNodes_);
    sortSuccessors(subGraph_->pickNodes_);
    if (solverOptions_->isDropPickPossible_) {
        sortSuccessors(subGraph_->dropNodes_);
        sortSuccessors(subGraph_->onboards_);
    }

    // create the initial label at the source and add the source to the list active nodes
    PLabel initialLabel = std::make_shared<Label>(Vehicle_, subGraph_->sourceNodes_[0]);
    initialNodeID_ = subGraph_->sourceNodes_[0]->nodeID_;
    initialLabel->openRequests_.resize(nbTotalRequest_ + (Vehicle_)->onboards_.size(), 0);
    initialLabel->travelResources_.resize(nbTotalRequest_ + (Vehicle_)->onboards_.size(),0);
    // update travel resource for the initial label based on the onboards
    for (auto &nodeObj: subGraph_->onboards_) {
        initialLabel->openNode_.push_back(&(*nodeObj));
        initialLabel->openRequests_[nodeObj->related_Request_->taskIndexLabel_] = 1;
        float remainedTime = nodeObj->related_Request_->maxTravelTime_ - (Vehicle_)->departTime_ +
                nodeObj->pairNode_->departTime_;

        initialLabel->travelResources_[nodeObj->related_Request_->taskIndexLabel_] = remainedTime;
    }

    if ((Vehicle_)->currentRoute_->routeSize_ > 1) {
        int i = 1;
        while ((Vehicle_)->currentRoute_->routeNodes_[i]->nodeStatus_ == COMMITTED){
            initialLabel->extend(&(*subGraph_->nodes_[(Vehicle_)->currentRoute_->routeNodes_[i]->nodeID_]), solverOptions_->isDropPickPossible_);
            initialNodeID_  = (Vehicle_)->currentRoute_->routeNodes_[i]->nodeID_;
            i++;
            if (i == (Vehicle_)->currentRoute_->routeSize_)
                break;
        }
    }

    initialLabel->pathNode_.back()->nbActiveLabels_++;
    initialLabel->pathNode_.back()->bestLabelReduceCost_ = initialLabel->reducedCost_;
    activeNodes_.clear();
    activeNodes_.push_back(initialLabel->pathNode_.back());
    if (!initialLabel->openNode_.empty())
        extendToOnboard = true;
    initialLabel->pathNode_.back()->activeLabels_.push_back(std::move(initialLabel));
    if (extendToOnboard && !solverOptions_->isDropPickPossible_) {
        vector<Node *> activeDropNodes = activeNodes_;
        int nbActive;
        while (!activeDropNodes.empty()) {
            // select a node to extend active labels
            Node *currentNode = activeDropNodes.back();
            activeDropNodes.pop_back();
            for (int j = currentNode->activeLabels_.size() - 1; j >= 0; j--) {
                if (currentNode->activeLabels_[j]->status_ == ACTIVE) {
                    PLabel selectedLabel = currentNode->activeLabels_[j];
                    currentNode->nbActiveLabels_--;
                    selectedLabel->status_ = INACTIVE;
                    // drop onboards
                    if (!selectedLabel->openNode_.empty()) {
                        for (auto &neighbourNode: selectedLabel->openNode_) {
                            if (selectedLabel->isTravelTimeFeasible(neighbourNode, nbUnreachableDTrip_)) {
                                nbActive = neighbourNode->nbActiveLabels_;
                                if (labelExtend(selectedLabel, neighbourNode, true)) {
                                    if ((neighbourNode->nbActiveLabels_ == 1) && (nbActive == 0)) {
                                        activeDropNodes.push_back(neighbourNode);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        PLabel dropLable = activeNodes_.back()->activeLabels_.back();
        dropLable->status_ = ACTIVE;
        activeNodes_.back()->nbActiveLabels_++;
        for (auto &dropNode: dropLable->openNode_){
            for (int j = dropNode->activeLabels_.size() - 1; j >= 0; j--) {
                dropNode->activeLabels_[j]->status_ = ACTIVE;
                dropNode->nbActiveLabels_++;
                activeNodes_.push_back(dropNode);
            }
        }
    }

}


bool LabelingSubProblem::labelExtend(PLabel &parentLabel, Node *outNode, bool Terminate) {
    PLabel newLabel;
    if (!labelPool_.empty()){
        newLabel = std::move(labelPool_.back());
        labelPool_.pop_back();
        newLabel->copyLabel(*parentLabel);
    }
    else {
        newLabel = std::make_shared<Label>(*parentLabel);
    }

    newLabel->extend(outNode, solverOptions_->isDropPickPossible_);
    nbGenerated_++;
    if (!newLabel->areDropsUnreachable()) {
        if (!isLabelAdded(newLabel, outNode, Terminate))
            nbDominated_++;
    }
    else {
//        nbUnreachableDTrip_++;
        return false;
    }
    return true;
}

bool LabelingSubProblem::isLabelAdded(PLabel &newLabel, Node *outNode, bool Terminate) {
    // check the dominance state of the new label
    for (auto &labelObj: outNode->activeLabels_) {
        if (newLabel->isDominated(labelObj, this->solverOptions_)) {
            newLabel->status_ = DOMINATED;
            labelPool_.push_back(std::move(newLabel));
            return false;
        }
    }
    // remove previous dominated labels
    for (int i = outNode->activeLabels_.size() - 1; i >= 0; i--) {
        if (outNode->activeLabels_[i]->isDominated(newLabel, this->solverOptions_)) {
            if (outNode->activeLabels_[i]->status_ == ACTIVE) {
                outNode->nbActiveLabels_--;
                //               this->nbActivated_--;
            }
            outNode->activeLabels_[i]->status_ = DOMINATED;
            labelPool_.push_back(std::move(outNode->activeLabels_[i]));
            outNode->activeLabels_.erase(outNode->activeLabels_.begin() + i);
            this->nbDominated_++;
        }
    }
    if (outNode->bestLabelReduceCost_ > newLabel->reducedCost_)
        outNode->bestLabelReduceCost_ = newLabel->reducedCost_;

    if (outNode->type_ == SINK){
        newLabel->status_ = TERMINATED;
//        newLabel->createTime_ = subproTime_->dSinceInit().count();
        if (Vehicle_->bestReducedCost_ > newLabel->reducedCost_ - Vehicle_->dual_)
            Vehicle_->bestReducedCost_ = newLabel->reducedCost_ - Vehicle_->dual_;
        outNode->activeLabels_.push_back(std::move(newLabel));

    }
    else{
        if (Terminate && newLabel->openNode_.empty()) {
            if (labelExtend(newLabel, &(*subGraph_->sinkNodes_[0]), false)){
                outNode->activeLabels_.push_back(newLabel);
                outNode->nbActiveLabels_++;
            }
            else {
                newLabel->status_ = DOMINATED;
                labelPool_.push_back(std::move(newLabel));
                return false;
            }
        }
        else {
            outNode->activeLabels_.push_back(newLabel);
            outNode->nbActiveLabels_++;
        }
    }
    return true;
}

void LabelingSubProblem::solveDynamic_pulling() {
    // create initial label
    int nbActive;
    while(true) {
        // create initial label
        initialization();

        while (!activeNodes_.empty()) {
            // select a node to pull other labels to it
            for (auto &currentNode: subGraph_->pickNodes_) {
                std::stable_sort(activeNodes_.begin(),activeNodes_.end(),[](const Node *lhs, const Node *rhs){
                    return lhs->travelTimeFromSource_ > rhs->travelTimeFromSource_;});
                for (int j = activeNodes_.size()-1; j >= 0; j--) {
                    if (activeNodes_[j]->nbActiveLabels_ == 0)
                        activeNodes_.erase(activeNodes_.begin() + j);
                    else {
                        for (int l = activeNodes_[j]->activeLabels_.size() - 1; l >= 0; l--) {
                            if (activeNodes_[j]->activeLabels_[l]->status_ == ACTIVE){
                                PLabel selectedLabel = activeNodes_[j]->activeLabels_[l];
                                // push to drop onboards
                                if (!selectedLabel->isDropExtend_) {
                                    if (!selectedLabel->openNode_.empty()) {
                                        for (auto &onboardNode: selectedLabel->openNode_) {
                                            if (selectedLabel->isTravelTimeFeasible(onboardNode, nbUnreachableDTrip_)) {
                                                nbActive = onboardNode->nbActiveLabels_;
                                                if (labelExtend(selectedLabel, onboardNode, true)) {
                                                    if ((onboardNode->nbActiveLabels_ == 1) && (nbActive == 0)) {
                                                        activeNodes_.push_back(onboardNode);
                                                    }
                                                }
                                            }
                                        }
                                    }
                                    selectedLabel->isDropExtend_ = true;
                                    // push to the sink
                                    /*else if (selectedLabel->openNode_.empty()) {
                                        labelExtend(selectedLabel, subGraph_->sinkNodes_[0], false);
                                        selectedLabel->isDropped_ = true;
                                    }*/
                                }
                                if (selectedLabel->numExtendCheck_ == subGraph_->pickNodes_.size() ||
                                    (selectedLabel->nbPickUp_ >= maxPickup_)) {
                                    selectedLabel->status_ = INACTIVE;
                                    activeNodes_[j]->nbActiveLabels_--;
                                    if (activeNodes_[j]->nbActiveLabels_ == 0) {
                                        activeNodes_.erase(activeNodes_.begin() + j);
                                        break;
                                    }
                                }
                                // pull all labels to the current node
                                else if (selectedLabel->isExtendFeasible(&(*currentNode), maxPickup_,
                                                                         solverOptions_->isSuccessorsLimited_,
                                                                         Vehicle_->capacity_, nbUnreachableDelay_,
                                                                         nbUnreachableDTrip_)) {
                                    nbActive = currentNode->nbActiveLabels_;
                                    if (labelExtend(selectedLabel, &(*currentNode), true)) {

                                        if ((currentNode->nbActiveLabels_ == 1) && (nbActive == 0)) {
                                            activeNodes_.push_back(&(*currentNode));
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                // decrease the number of active labels if truncated strategy is used
                if ((solverOptions_->isTruncated_) && (currentNode->nbActiveLabels_ > solverOptions_->MaxLabel_)){
                    truncateLabelList(&(*currentNode), solverOptions_->MaxLabel_, labelPool_);
                }
            }
        }
        break;
        if (!solverOptions_->areHeuristicsDisabled()) {
            if ((subGraph_->sinkNodes_[0])->bestLabelReduceCost_ - Vehicle_->dual_ >= -0.0001) {
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
void LabelingSubProblem::solveDynamic_pullingWave() {
    // create initial label
    int nbActive;
    while(true) {
        // create initial label
        initialization();
        while (!activeNodes_.empty()) {
            // select a node to pull other labels to it
            for (auto &currentNode: subGraph_->pickNodes_) {
                std::stable_sort(activeNodes_.begin(),activeNodes_.end(),[](const Node *lhs, const Node *rhs){
                    return lhs->travelTimeFromSource_ > rhs->travelTimeFromSource_;});
                for (int j = activeNodes_.size()-1; j >= 0; j--) {
                    if (activeNodes_[j]->nbActiveLabels_ == 0)
                        activeNodes_.erase(activeNodes_.begin() + j);
                    else {
                        for (int l = activeNodes_[j]->activeLabels_.size() - 1; l >= 0; l--) {
                            if (activeNodes_[j]->activeLabels_[l]->status_ == ACTIVE){
                                PLabel selectedLabel = activeNodes_[j]->activeLabels_[l];
                                if (selectedLabel->numExtendCheck_ == subGraph_->pickNodes_.size() ||
                                    (selectedLabel->nbPickUp_ >= maxPickup_))  {
                                    selectedLabel->status_ = INACTIVE;
                                    activeNodes_[j]->nbActiveLabels_--;
                                    if (activeNodes_[j]->nbActiveLabels_ == 0) {
                                        activeNodes_.erase(activeNodes_.begin() + j);
                                        break;
                                    }
                                }
                                // pull all labels to the current node
                                else if (selectedLabel->isExtendFeasible(&(*currentNode),maxPickup_,
                                                                         solverOptions_->isSuccessorsLimited_,
                                                                         Vehicle_->capacity_, nbUnreachableDelay_,
                                                                         nbUnreachableDTrip_)) {
                                    nbActive = currentNode->nbActiveLabels_;
                                    if (labelExtend(selectedLabel, &(*currentNode), true)) {

                                        if ((currentNode->nbActiveLabels_ == 1) && (nbActive == 0)) {
                                            activeNodes_.push_back(&(*currentNode));
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                // decrease the number of active labels if truncated strategy is used
                if ((solverOptions_->isTruncated_) && (currentNode->nbActiveLabels_ > solverOptions_->MaxLabel_)){
                    truncateLabelList(&(*currentNode), solverOptions_->MaxLabel_, labelPool_);
                }
            }
        }
        PNode initialNode = subGraph_->nodes_[initialNodeID_];
        std::vector<PNode> nodeList = subGraph_->pickNodes_;
        nodeList.push_back(initialNode);
        while (!nodeList.empty()){
            PNode currentNode = nodeList.back();
            nodeList.pop_back();
            for (auto & selectedLabel: currentNode->activeLabels_){
                /*if (selectedLabel->openNode_.empty())
                    labelExtend(selectedLabel, subGraph_->sinkNodes_[0], false);*/
                // drop onboards
                if (!selectedLabel->openNode_.empty()) {
                    for (auto &neighbourNode: selectedLabel->openNode_){
                        if (selectedLabel->isTravelTimeFeasible(neighbourNode, nbUnreachableDTrip_)) {
                            nbActive = neighbourNode->nbActiveLabels_;
                            if (labelExtend(selectedLabel, neighbourNode, true)) {
                                if ((neighbourNode->nbActiveLabels_ == 1) && (nbActive == 0)) {
                                    activeNodes_.push_back(neighbourNode);
                                }
                            }
                        }
                    }
                }
            }
        }

        while (!activeNodes_.empty()) {

            // select a node to extend active labels
            Node *currentNode = activeNodes_.back();
            activeNodes_.pop_back();

            // decrease the number of active labels if truncated strategy is used
            if ((solverOptions_->isTruncated_) && (currentNode->nbActiveLabels_ > solverOptions_->MaxLabel_)){
                truncateLabelList(currentNode, solverOptions_->MaxLabel_, labelPool_);
            }
            for (int j = currentNode->activeLabels_.size()-1; j >=0; j--) {
                if (currentNode->activeLabels_[j]->status_ == ACTIVE) {
                    PLabel selectedLabel = currentNode->activeLabels_[j];
                    currentNode->nbActiveLabels_--;
                    selectedLabel->status_ = INACTIVE;
                    // terminate to sink
                    /*if (selectedLabel->openNode_.empty())
                    labelExtend(selectedLabel, subGraph_->sinkNodes_[0], false);*/
                    // drop onboards
                    if (!selectedLabel->openNode_.empty()) {
                        for (auto &neighbourNode: selectedLabel->openNode_){
                            if (selectedLabel->isTravelTimeFeasible(neighbourNode, nbUnreachableDTrip_)) {
                                nbActive = neighbourNode->nbActiveLabels_;
                                if (labelExtend(selectedLabel, neighbourNode, true)) {
                                    if ((neighbourNode->nbActiveLabels_ == 1) && (nbActive == 0)) {
                                        activeNodes_.push_back(neighbourNode);
                                    }
                                }
                            }
                            /*if (!(*neighbourNode)->activeLabels_.empty())
                                (*neighbourNode)->activeLabels_.back()->isDropped_ = true;*/
                        }
                    }
                }
            }
        }
        break;
    }
}

//***************************************************************************************//
//                           P U S H I N G  S T R A T E G Y
//***************************************************************************************//
void LabelingSubProblem::solveDynamic_pushing() {
    int nbActive;
    while(true) {
        // create initial label
        initialization();

        while (!activeNodes_.empty()) {
            std::stable_sort(activeNodes_.begin(),activeNodes_.end(),[](const Node *lhs, const Node *rhs){
                return lhs->travelTimeFromSource_ < rhs->travelTimeFromSource_;});

            // select a node to extend active labels
            Node *currentNode = activeNodes_.back();
            activeNodes_.pop_back();

            // decrease the number of active labels if truncated strategy is used
            if ((solverOptions_->isTruncated_) && (currentNode->nbActiveLabels_ > solverOptions_->MaxLabel_)){
                truncateLabelList(currentNode, solverOptions_->MaxLabel_, labelPool_);
            }
            for (int j = currentNode->activeLabels_.size()-1; j >=0; j--) {
                if (currentNode->activeLabels_[j]->status_ == ACTIVE) {
                    PLabel selectedLabel = currentNode->activeLabels_[j];
                    currentNode->nbActiveLabels_--;
                    selectedLabel->status_ = INACTIVE;
                    // drop onboards
                    if (!selectedLabel->openNode_.empty()) {
                        for (auto &neighbourNode: selectedLabel->openNode_) {
                            if (selectedLabel->isTravelTimeFeasible(neighbourNode, nbUnreachableDTrip_)) {
                                nbActive = neighbourNode->nbActiveLabels_;
                                if (labelExtend(selectedLabel, neighbourNode, true)) {
                                    if ((neighbourNode->nbActiveLabels_ == 1) && (nbActive == 0)) {
                                        activeNodes_.push_back(neighbourNode);
                                    }
                                }
                            }
                        }
                    }
                    // terminate to sink
                    /*else
                        labelExtend(selectedLabel, subGraph_->sinkNodes_[0], false);*/
                    // push to pickup points
                    if (selectedLabel->load_ < Vehicle_->capacity_) {
                        for (auto &neighbourNode: selectedLabel->pathNode_.back()->successors_) {
                            if (selectedLabel->isExtendFeasible(neighbourNode, maxPickup_, solverOptions_->isSuccessorsLimited_,
                                                                Vehicle_->capacity_,nbUnreachableDelay_,
                                                                nbUnreachableDTrip_)) {
                                nbActive = neighbourNode->nbActiveLabels_;
                                if (labelExtend(selectedLabel, neighbourNode, true)){
                                    if ((neighbourNode->nbActiveLabels_ == 1) && (nbActive == 0)) {
                                        activeNodes_.push_back(neighbourNode);
                                    }
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
    int nbActive;
    while(true) {
        // create initial label
        initialization();
        while (!activeNodes_.empty()) {
            std::stable_sort(activeNodes_.begin(),activeNodes_.end(),[](const Node *lhs, const Node *rhs){
                return lhs->nbActiveLabels_ < rhs->nbActiveLabels_;});

            // select a node to extend active labels
            Node *currentNode = activeNodes_.back();
            activeNodes_.pop_back();

            // decrease the number of active labels if truncated strategy is used
            if ((solverOptions_->isTruncated_) && (currentNode->nbActiveLabels_ > solverOptions_->MaxLabel_)){
                truncateLabelList(currentNode, solverOptions_->MaxLabel_, labelPool_);
            }
            for (int j = currentNode->activeLabels_.size()-1; j >=0; j--) {
                if (currentNode->activeLabels_[j]->status_ == ACTIVE) {
                    PLabel selectedLabel = currentNode->activeLabels_[j];
                    currentNode->nbActiveLabels_--;
                    selectedLabel->status_ = INACTIVE;
                    // terminate to sink
                    /*if (selectedLabel->openNode_.empty())
                        labelExtend(selectedLabel, &(*subGraph_->sinkNodes_[0]), false);*/
                    // drop onboards and terminate to sink if empty
                    if (!selectedLabel->openNode_.empty()){
                        for (auto &neighbourNode: selectedLabel->openNode_){
                            if (selectedLabel->isTravelTimeFeasible(neighbourNode, nbUnreachableDTrip_)) {
                                nbActive = neighbourNode->nbActiveLabels_;
                                if (labelExtend(selectedLabel, neighbourNode, true)) {
                                    if ((neighbourNode->nbActiveLabels_ == 1) && (nbActive == 0)) {
                                        activeNodes_.push_back(neighbourNode);
                                    }
                                }
                            }
                        }
                    }
                    if (!selectedLabel->isDropped_) {
                        // push to pickup points
                        if (selectedLabel->load_ < Vehicle_->capacity_) {
                            for (auto &neighbourNode: selectedLabel->pathNode_.back()->successors_) {
                                if (selectedLabel->isExtendFeasible(neighbourNode, maxPickup_, solverOptions_->isSuccessorsLimited_,
                                                                    Vehicle_->capacity_, nbUnreachableDelay_,
                                                                    nbUnreachableDTrip_)) {
                                    nbActive = (neighbourNode)->nbActiveLabels_;
                                    if (labelExtend(selectedLabel, (neighbourNode), false)) {
                                        if ((neighbourNode->nbActiveLabels_ == 1) && (nbActive == 0)) {
                                            activeNodes_.push_back(neighbourNode);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        break;
        if (!solverOptions_->areHeuristicsDisabled()) {
            if ((subGraph_->sinkNodes_[0])->bestLabelReduceCost_ - Vehicle_->dual_ >= -0.0001)
                solverOptions_->disableHeuristics();
            else
                break;
        } else
            break;
    }
}

void LabelingSubProblem::solveDynamic_pushingWave() {
    // create initial label
    int nbActive;
    while(true) {
        // create initial label
        initialization();
        while (!activeNodes_.empty()) {
            std::stable_sort(activeNodes_.begin(),activeNodes_.end(),[](const Node *lhs, const Node *rhs){
                return lhs->travelTimeFromSource_ > rhs->travelTimeFromSource_;});

            // select a node to extend active labels
            Node *currentNode = activeNodes_.back();
            activeNodes_.pop_back();

            // decrease the number of active labels if truncated strategy is used
            if ((solverOptions_->isTruncated_) && (currentNode->nbActiveLabels_ > solverOptions_->MaxLabel_)){
                truncateLabelList(currentNode, solverOptions_->MaxLabel_, labelPool_);
            }
            for (int j = currentNode->activeLabels_.size()-1; j >=0; j--) {
                if (currentNode->activeLabels_[j]->status_ == ACTIVE) {
                    PLabel selectedLabel = currentNode->activeLabels_[j];
                    currentNode->nbActiveLabels_--;
                    selectedLabel->status_ = INACTIVE;
                    // push to pickup points
                    if (selectedLabel->load_ < Vehicle_->capacity_) {
                        for (auto &neighbourNode: selectedLabel->pathNode_.back()->successors_) {
                            if (selectedLabel->isExtendFeasible(neighbourNode, maxPickup_, solverOptions_->isSuccessorsLimited_,
                                                                Vehicle_->capacity_, nbUnreachableDelay_,
                                                                nbUnreachableDTrip_)) {
                                nbActive = neighbourNode->nbActiveLabels_;
                                if (labelExtend(selectedLabel, (neighbourNode), true)){
                                    if ((neighbourNode->nbActiveLabels_ == 1) && (nbActive == 0)) {
                                        activeNodes_.push_back(neighbourNode);
                                    }
                                }

                            }
                        }
                    }
                }
            }
        }

        PNode initialNode = subGraph_->nodes_[initialNodeID_];
        std::vector<PNode> nodeList = subGraph_->pickNodes_;
        nodeList.push_back(initialNode);
        while (!nodeList.empty()){
            PNode currentNode = nodeList.back();
            nodeList.pop_back();
            for (auto & selectedLabel: currentNode->activeLabels_){
                // drop onboards
                if (!selectedLabel->openNode_.empty()) {
                    for (auto &neighbourNode: selectedLabel->openNode_){
                        if (selectedLabel->isTravelTimeFeasible(neighbourNode, nbUnreachableDTrip_)) {
                            nbActive = (neighbourNode)->nbActiveLabels_;
                            if (labelExtend(selectedLabel, neighbourNode, true)) {
                                if ((neighbourNode->nbActiveLabels_ == 1) && (nbActive == 0)) {
                                    activeNodes_.push_back(neighbourNode);
                                }
                            }
                        }
                    }
                }
            }
        }
        activeNodes_.push_back(&(*initialNode));
        initialNode->activeLabels_[0]->status_ = ACTIVE;
        while (!activeNodes_.empty()) {
            // select a node to extend active labels
            Node *currentNode = activeNodes_.back();
            activeNodes_.pop_back();

            // decrease the number of active labels if truncated strategy is used
            if ((solverOptions_->isTruncated_) && (currentNode->nbActiveLabels_ > solverOptions_->MaxLabel_)){
                truncateLabelList(currentNode, solverOptions_->MaxLabel_, labelPool_);
            }
            for (int j = currentNode->activeLabels_.size()-1; j >=0; j--) {
                if (currentNode->activeLabels_[j]->status_ == ACTIVE) {
                    PLabel selectedLabel = currentNode->activeLabels_[j];
                    currentNode->nbActiveLabels_--;
                    selectedLabel->status_ = INACTIVE;

                    // drop onboards
                    if (!selectedLabel->openNode_.empty()) {
                        for (auto &neighbourNode: selectedLabel->openNode_){
                            if (selectedLabel->isTravelTimeFeasible(neighbourNode, nbUnreachableDTrip_)) {
                                nbActive = neighbourNode->nbActiveLabels_;
                                if (labelExtend(selectedLabel, neighbourNode, true)) {
                                    if ((neighbourNode->nbActiveLabels_ == 1) && (nbActive == 0)) {
                                        activeNodes_.push_back(neighbourNode);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        break;
        if (!solverOptions_->areHeuristicsDisabled()) {
            if ((subGraph_->sinkNodes_[0])->bestLabelReduceCost_ - Vehicle_->dual_ >= -0.0001)
                solverOptions_->disableHeuristics();
            else
                break;
        } else
            break;
    }
}

//***************************************************************************************//
//                           P U L L I N G  S T R A T E G Y
//***************************************************************************************//

void LabelingSubProblem::solveDynamic() {
    Vehicle_->bestReducedCost_ = INFINITY;
 //   subproTime_->start();
    if ((solverOptions_->LabelingStrategy_ == PUSHING)||(subRequests_.empty())){
        if (solverOptions_->isDropPickPossible_)
            this->solveDynamic_pushing();
        else
//            this->solveDynamic_pushingWave();
            this->solveDynamic_pushingDrop();
    }

    else if (solverOptions_->LabelingStrategy_ == PULLING){
        if (solverOptions_->isDropPickPossible_)
            this->solveDynamic_pulling();
        else
            this->solveDynamic_pullingWave();
    }


    /*if (!subGraph_->sinkNodes_[0]->activeLabels_.empty()) {
        bestReducedCost_ = subGraph_->sinkNodes_[0]->bestLabelReduceCost_ - (*Vehicle_)->dual_;
        (*Vehicle_)->bestReducedCost_ = bestReducedCost_;
    }*/
    nbNegativeColumns_ = 0;
    for (auto & labelObj : subGraph_->sinkNodes_[0]->activeLabels_) {
        if (labelObj->reducedCost_ - (Vehicle_)->dual_ < 0)
            nbNegativeColumns_ ++;
    }
    /*std::cout << "Best reduced cost: " << Vehicle_->bestReducedCost_ << std::endl;
    std::cout << "vehicle: " << Vehicle_->vehicleID_ << " : " << "nb Generated: " << nbGenerated_;
    std::cout << " - nb Dominated: " << nbDominated_ << " - nb Negative: " << nbNegativeColumns_ ;
    std::cout << " - nb final: " << subGraph_->sinkNodes_[0]->activeLabels_.size() << std::endl;*/
//    subproTime_->stop();
}

void LabelingSubProblem::SolutionToRoutes(PVehicle &vehicle, vector<PRoute> &availableRoutes, PInstance &pInst) {
    for (auto & labelObj : subGraph_->sinkNodes_[0]->activeLabels_) {
        availableRoutes.emplace_back(labelObj->labelToRoute(vehicle, pInst));
        availableRoutes.back()->createColumn();
        nbOutputs_++;
    }
    for (auto & nodeObj : subGraph_->nodes_){
        for (auto & labelObj : nodeObj.second->activeLabels_)
            labelPool_.push_back(std::move(labelObj));
    }
}

void LabelingSubProblem::solutionSummery(vector<int> &subProResults) {
    if (subProResults.empty()) {
        subProResults.push_back(nbGenerated_);
        subProResults.push_back(nbGenerated_);
        subProResults.push_back(nbDominated_);
        subProResults.push_back(subGraph_->nbNodes_);
        subProResults.push_back(subGraph_->pickNodes_.size());
    }
    else
    {
        subProResults[0] += nbOutputs_;
        subProResults[1] += nbGenerated_;
        subProResults[2] += nbDominated_;
        subProResults[4] += subGraph_->nbNodes_;
        subProResults[5] += subGraph_->pickNodes_.size();
    }
}

std::string LabelingSubProblem::toString() const {
    std::stringstream repStr;
    repStr << std::endl;
    repStr << "# ------------------------------------------------------------" << std::endl;
    repStr << "# SUB PROBLEM SOLUTION RESULT FOR VEHICLE: " << (Vehicle_)->vehicleID_ << std::endl;
    repStr << "#" << std::endl;
    repStr << "# Solution status = " << "Optimal (Label Setting Algorithm)" << std::endl;
    repStr << "# In total, " << nbGenerated_ << " labels were generated." << std::endl;
    repStr << "# " << nbDominated_ << " labels were removed via Domination " << std::endl;
    if (!subGraph_->sinkNodes_[0]->activeLabels_.empty())
        repStr << "# best objective value = " << subGraph_->sinkNodes_[0]->bestLabelReduceCost_ - (Vehicle_)->dual_ << std::endl;
    repStr << "# The solution pool contains = " << subGraph_->sinkNodes_[0]->activeLabels_.size() << " solutions." << std::endl;

    repStr << "# The solution pool contains = " << nbNegativeColumns_ << " solutions with negative reduced cost." << std::endl;
    return repStr.str();
}

std::string LabelingSubProblem::toStringOut(int epoch) const {
    std::stringstream repStr;
    repStr << epoch << ",";
    repStr << (Vehicle_)->vehicleID_ << ",";
    repStr << subRequests_.size() << ",";
    repStr << subGraph_->nbNodes_-2 << ",";
    repStr << maxPickup_ << ",";
    repStr << nbGenerated_ << ",";
    repStr << nbDominated_ << ",";
//    repStr << nbOutputs_ << ",";
    repStr << nbOutputs_ << "\n";
    /*repStr << subproTime_->dSinceInit().count();*/
    return repStr.str();
}

void LabelingSubProblem::truncateLabelList(Node *node, int MaxLabel, std::vector<PLabel> & labelPool) {
    if (solverOptions_->pathSort_ == RD_COST) {
        std::stable_sort(node->activeLabels_.begin(), node->activeLabels_.end(),
                         [](const PLabel &lhs, const PLabel &rhs) {
                             return lhs->reducedCost_ < rhs->reducedCost_;
                         });
    }
    if (solverOptions_->pathSort_ == LAMBDA) {
        std::stable_sort(node->activeLabels_.begin(), node->activeLabels_.end(),
                         [](const PLabel &lhs, const PLabel &rhs) {
                             return lhs->lambdaScore_ < rhs->lambdaScore_;
                         });
    }
    else {
        std::stable_sort(node->activeLabels_.begin(),node->activeLabels_.end(),[](const PLabel &lhs, const PLabel &rhs){
            return lhs->labelScore_ < rhs->labelScore_;});
    }


    for (int i = node->activeLabels_.size()-1; i >=0; i--){
        if (node->nbActiveLabels_ <= MaxLabel)
            break;
        else {
            if (node->activeLabels_[i]->status_ == ACTIVE){
                node->nbActiveLabels_--;
                node->activeLabels_[i]->status_ = DOMINATED;
                labelPool.push_back(std::move(node->activeLabels_[i]));
                node->activeLabels_.erase(node->activeLabels_.begin() + i);
            }
        }
    }
}




