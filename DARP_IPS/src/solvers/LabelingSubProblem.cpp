//
// Created by Ella on 1/28/2022.
//

#include "LabelingSubProblem.h"

#include <utility>



//-----------------------------------------------------------------------------
//  Labeling Sub problem class
//-----------------------------------------------------------------------------


LabelingSubProblem::LabelingSubProblem(const PVehicle &vehicle, const PSolverOption &solverOptions) :
        SubproModeler(vehicle), solverOptions_(solverOptions) {
    nbGenerated_ = 0;
    nbDominated_ = 0;
    nbEliminated_ = 0;
    nbPrunedPath_ = 0;
    nbPrunedArcs_ = 0;
    nbRecycledColumns_ = 0;
    nbOutputs_ = 0;
    maxPickup_ = solverOptions_->nbPick_;
    subproTime_ = new myTools::Timer(); subproTime_->init();
}
LabelingSubProblem::~LabelingSubProblem() {

    delete subproTime_;
}

void LabelingSubProblem::sortSuccessors(const std::vector<PNode> &nodeList, bool prunedArcs) const {
    for (auto & nodeObj: nodeList){
        if (nodeObj->type_ != SINK){
            nodeObj->successors_.clear();
            nodeObj->prunedArcs_.reset();
            for (auto &pickNodeObj : subGraph_->pickNodes_) {
                if (nodeObj->nodeID_ != pickNodeObj->nodeID_) {
                    if (prunedArcs) {
                        float minWait =
                                (Vehicle_)->departTime_ + nodeObj->travelTimeFromSource_ + nodeObj->serviceTime_ +
                                durationMatrix_[nodeObj->locationID_][pickNodeObj->locationID_] -
                                pickNodeObj->readyTime_;
                        if (minWait > pickNodeObj->related_Request_->penalty_)
                            nodeObj->prunedArcs_.set(pickNodeObj->related_Request_->taskIndexLabel_, true);
                    }
                    nodeObj->successors_.push_back(&(*pickNodeObj));
                }
            }
        }
    }
}


// reset that active lists of the nodes, create the first label at the source, add onboard requests
void LabelingSubProblem::initialization() {
    sortSuccessors(subGraph_->sourceNodes_, false);
    sortSuccessors(subGraph_->pickNodes_, solverOptions_->pruneArcs_);
    sortSuccessors(subGraph_->dropNodes_, solverOptions_->pruneArcs_);
    sortSuccessors(subGraph_->onboards_, solverOptions_->pruneArcs_);
    /*if (solverOptions_->isDropPickPossible_) {
        sortSuccessors(subGraph_->dropNodes_);
        sortSuccessors(subGraph_->onboards_);
    }*/

    // create the initial label at the source and add the source to the list active nodes
    PLabel initialLabel;
    if (!labelPool_.empty()){
        initialLabel = std::move(labelPool_.back());
        labelPool_.pop_back();
        initialLabel->copyLabel(Vehicle_, subGraph_->sourceNodes_[0]);
    }
    else
        initialLabel = std::make_shared<Label>(Vehicle_, subGraph_->sourceNodes_[0]);
    initialNodeID_ = subGraph_->sourceNodes_[0]->nodeID_;
    initialLabel->travelResources_.resize(nbTotalRequest_ + (Vehicle_)->onboards_.size(),0);
    // update travel resource for the initial label based on the onboard requests
    for (auto &nodeObj: subGraph_->onboards_) {
        initialLabel->openNode_.push_back(&(*nodeObj));
        initialLabel->openRequests_.set(nodeObj->related_Request_->taskIndexLabel_, true);
        float remainedTime = nodeObj->related_Request_->maxTravelTime_ - Vehicle_->departTime_ +
                             nodeObj->pairNode_->departTime_;

        initialLabel->travelResources_[nodeObj->related_Request_->taskIndexLabel_] = remainedTime;
    }

    /*if ((Vehicle_)->currentRoute_->routeSize_ > 1) {
        int i = 1;
        while ((Vehicle_)->currentRoute_->routeNodes_[i]->nodeStatus_ == COMMITTED){
            initialLabel->extend(&(*subGraph_->nodes_[(Vehicle_)->currentRoute_->routeNodes_[i]->nodeID_]), solverOptions_->isDropPickPossible_);
            initialNodeID_  = (Vehicle_)->currentRoute_->routeNodes_[i]->nodeID_;
            i++;
            if (i == (Vehicle_)->currentRoute_->routeSize_)
                break;
        }
    }*/

    initialLabel->pathNode_.back()->nbActiveLabels_++;
    initialLabel->pathNode_.back()->bestLabelReduceCost_ = initialLabel->reducedCost_;
    activeNodes_.clear();
    activeNodes_.push_back(initialLabel->pathNode_.back());
    if (reOptimize_  && solverOptions_->labelingReOptimizeStrategy_ == RE_INSERT &&  !subGraph_->newPickNodes_.empty()) {
        constructBaseLabels(initialLabel);
        initialLabel->numExtendCheck_ = 0;
    }
    initialLabel->pathNode_.back()->activeLabels_.push_back(std::move(initialLabel));
}


bool LabelingSubProblem::labelExtend(const PLabel &parentLabel, Node *outNode, bool Terminate) {
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
    if (!newLabel->isEliminated()) {
        if (!isLabelAdded(newLabel, outNode, Terminate))
            nbDominated_++;
    }
    else {
        nbEliminated_++;
        return false;
    }
    return true;
}

bool LabelingSubProblem::labelExtendPick(const PLabel &parentLabel, Node *outNode) {
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
    if (!newLabel->isEliminated()) {
        if (outNode->bestLabelReduceCost_ > newLabel->reducedCost_)
            outNode->bestLabelReduceCost_ = newLabel->reducedCost_;
        outNode->activeLabels_.push_back(newLabel);
        outNode->nbActiveLabels_++;
    }
    else {
        nbEliminated_++;
        newLabel->status_ = DOMINATED;
        labelPool_.push_back(std::move(newLabel));
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
        outNode->activeLabels_.push_back(newLabel);
        outNode->nbActiveLabels_++;
        if (Terminate && newLabel->openNode_.empty())
            labelExtend(newLabel, &(*subGraph_->sinkNodes_[0]), false);
        /*if (Terminate && newLabel->openNode_.empty()) {
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
        }*/
    }
    return true;
}

bool LabelingSubProblem::solveDynamic_pulling1(float availableTime) {
    // create initial label
    while(true) {
        // create initial label
        initialization();

        while (!activeNodes_.empty() && subproTime_->dSinceStart().count() <= availableTime) {
            // select a node to pull other labels to it
            for (auto &currentNode: subGraph_->pickNodes_) {
                std::stable_sort(activeNodes_.begin(),activeNodes_.end(),[](const Node *lhs, const Node *rhs){
                    return lhs->travelTimeFromSource_ > rhs->travelTimeFromSource_;});
                int nbActive = currentNode->nbActiveLabels_;
                for (int j = activeNodes_.size()-1; j >= 0; j--) {
                    if (activeNodes_[j]->nbActiveLabels_ == 0)
                        activeNodes_.erase(activeNodes_.begin() + j);
                    else {
                        for (int l = activeNodes_[j]->activeLabels_.size() - 1; l >= 0; l--) {
                            if (activeNodes_[j]->activeLabels_[l]->status_ == ACTIVE){
                                PLabel selectedLabel = activeNodes_[j]->activeLabels_[l];

                                if (!selectedLabel->isDropExtend_) {
                                    // push to drop onboards
                                    extendToDropOnboards(selectedLabel);
                                    selectedLabel->isDropExtend_ = true;
                                }

                                // pull all labels to the current node
                                if (selectedLabel->isExtendFeasible(&(*currentNode), maxPickup_,
                                                                    solverOptions_->discardSuboptimalPath_,
                                                                    Vehicle_->capacity_, nbPrunedPath_,
                                                                    nbEliminated_, nbPrunedArcs_)) {
                                    labelExtendPick(selectedLabel, &(*currentNode));
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
                            }
                        }
                    }
                }
                removeDominated(&(*currentNode), labelPool_);
                if ((currentNode->nbActiveLabels_ >= 1) && (nbActive == 0)) {
                    activeNodes_.push_back(&(*currentNode));
                }

                // decrease the number of active labels if truncated strategy is used
                else if ((solverOptions_->isTruncated_) && (currentNode->nbActiveLabels_ > solverOptions_->MaxLabel_)){
                    truncateLabelList(&(*currentNode), solverOptions_->MaxLabel_, solverOptions_->MaxCommittedLabel_, labelPool_);
                }

            }
        }
        break;
    }
    if (!activeNodes_.empty())
        return false;
    else
        return true;
}

bool LabelingSubProblem::solveDynamic_pulling(float availableTime) {
    // create initial label
    while(true) {
        // create initial label
        initialization();

        while (!activeNodes_.empty() && subproTime_->dSinceStart().count() <= availableTime) {
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
                                    extendToDropOnboards(selectedLabel);
                                    selectedLabel->isDropExtend_ = true;
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
                                                                         solverOptions_->discardSuboptimalPath_,
                                                                         Vehicle_->capacity_, nbPrunedPath_,
                                                                         nbEliminated_, nbPrunedArcs_)) {
                                    int nbActive = currentNode->nbActiveLabels_;
                                    if (labelExtend(selectedLabel, &(*currentNode), true)) {

                                        if (currentNode->nbActiveLabels_ == 1 && nbActive == 0) {
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
                    truncateLabelList(&(*currentNode), solverOptions_->MaxLabel_, solverOptions_->MaxCommittedLabel_, labelPool_);
                }
            }
        }
        break;
    }
    if (!activeNodes_.empty())
        return false;
    else
        return true;
}
bool LabelingSubProblem::solveDynamic_pullingWave(float availableTime) {
    // create initial label
    while(true) {
        // create initial label
        initialization();
        while (!activeNodes_.empty() && subproTime_->dSinceStart().count() <= availableTime) {
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
                                else if (selectedLabel->isExtendFeasible(&(*currentNode), maxPickup_,
                                                                         solverOptions_->discardSuboptimalPath_,
                                                                         Vehicle_->capacity_, nbPrunedPath_,
                                                                         nbEliminated_, nbPrunedArcs_)) {
                                    int nbActive = currentNode->nbActiveLabels_;
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

                // decrease the number of active labels if the truncated strategy is used
                if (solverOptions_->isTruncated_ && currentNode->nbActiveLabels_ > solverOptions_->MaxLabel_){
                    truncateLabelList(&(*currentNode), solverOptions_->MaxLabel_, solverOptions_->MaxCommittedLabel_, labelPool_);
                }
            }
        }
        PNode initialNode = subGraph_->nodes_[initialNodeID_];
        std::vector<PNode> nodeList = subGraph_->pickNodes_;
        nodeList.push_back(initialNode);
        while (!nodeList.empty() && subproTime_->dSinceStart().count() <= availableTime){
            PNode currentNode = nodeList.back();
            nodeList.pop_back();
            for (auto & selectedLabel: currentNode->activeLabels_){
                /*if (selectedLabel->openNode_.empty())
                    labelExtend(selectedLabel, subGraph_->sinkNodes_[0], false);*/
                // drop onboard requests
                extendToDropOnboards(selectedLabel);
            }
        }
        if (!nodeList.empty())
            return false;

        while (!activeNodes_.empty() && subproTime_->dSinceStart().count() <= availableTime) {

            // select a node to extend active labels
            Node *currentNode = activeNodes_.back();
            activeNodes_.pop_back();

            // decrease the number of active labels if truncated strategy is used
            /*if ((solverOptions_->isTruncated_) && (currentNode->nbActiveLabels_ > solverOptions_->MaxLabel_)){
                truncateLabelList(currentNode, solverOptions_->MaxLabel_, solverOptions_->MaxCommittedLabel_,labelPool_);
            }*/
            for (int j = currentNode->activeLabels_.size()-1; j >=0; j--) {
                if (currentNode->activeLabels_[j]->status_ == ACTIVE) {
                    PLabel selectedLabel = currentNode->activeLabels_[j];
                    currentNode->nbActiveLabels_--;
                    selectedLabel->status_ = INACTIVE;
                    // terminate to sink
                    /*if (selectedLabel->openNode_.empty())
                    labelExtend(selectedLabel, subGraph_->sinkNodes_[0], false);*/
                    // drop onboard requests
                    extendToDropOnboards(selectedLabel);
                }
            }
        }
        break;
    }
    if (!activeNodes_.empty())
        return false;
    return true;
}

bool LabelingSubProblem::solveDynamic_pullingWaveStep(float availableTime) {
    // create initial label
    while(true) {
        // create initial label
        initialization();

        // Wave 1: Pull towards Pickup nodes
        pullToPickups(availableTime, subGraph_->pickNodes_, true);
        PNode initialNode = subGraph_->nodes_[initialNodeID_];
        std::vector<PNode> nodeList = subGraph_->pickNodes_;
        nodeList.push_back(initialNode);

        // Wave 2: Push to drop on-boards
        pushToDrops(availableTime, nodeList);
        break;
    }
    if (!activeNodes_.empty())
        return false;
    return true;
}

bool LabelingSubProblem::ResolveDynamic_pullingWave(float availableTime) {
    // create initial label
    while(true) {
        // create initial label
        initialization();

//        std::cout << Vehicle_->vehicleID_ << " - " << subGraph_->newPickNodes_.size() << " - " << activeNodes_.size() << std::endl;
        while (!activeNodes_.empty() && subproTime_->dSinceStart().count() <= availableTime) {
            // select a node to pull other labels to it
            for (auto &currentNode: subGraph_->newPickNodes_) {
                for (int j = activeNodes_.size()-1; j >= 0; j--) {
                    if (activeNodes_[j]->nbActiveLabels_ == 0)
                        activeNodes_.erase(activeNodes_.begin() + j);
                    else {
                        for (int l = activeNodes_[j]->activeLabels_.size() - 1; l >= 0; l--) {
                            if (activeNodes_[j]->activeLabels_[l]->status_ == ACTIVE){
                                PLabel selectedLabel = activeNodes_[j]->activeLabels_[l];
                                if (selectedLabel->numExtendCheck_ == subGraph_->newPickNodes_.size() ||
                                    (selectedLabel->nbPickUp_ >= maxPickup_))  {
                                    selectedLabel->status_ = INACTIVE;
                                    activeNodes_[j]->nbActiveLabels_--;
                                    if (activeNodes_[j]->nbActiveLabels_ == 0) {
                                        activeNodes_.erase(activeNodes_.begin() + j);
                                        break;
                                    }
                                }
                                    // pull all labels to the current node
                                else if (selectedLabel->isExtendFeasible(&(*currentNode), maxPickup_,
                                                                         solverOptions_->discardSuboptimalPath_,
                                                                         Vehicle_->capacity_, nbPrunedPath_,
                                                                         nbEliminated_, nbPrunedArcs_)) {
                                    int nbActive = currentNode->nbActiveLabels_;
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
            }
        }
        PNode initialNode = subGraph_->nodes_[initialNodeID_];
        std::vector<PNode> nodeList = subGraph_->pickNodes_;
        nodeList.push_back(initialNode);
        while (!nodeList.empty() && subproTime_->dSinceStart().count() <= availableTime){
            PNode currentNode = nodeList.back();
            nodeList.pop_back();
            for (auto & selectedLabel: currentNode->activeLabels_){
                extendToDropOnboards(selectedLabel);
            }
        }
        if (!nodeList.empty())
            return false;

        while (!activeNodes_.empty() && subproTime_->dSinceStart().count() <= availableTime) {

            // select a node to extend active labels
            Node *currentNode = activeNodes_.back();
            activeNodes_.pop_back();

            for (int j = currentNode->activeLabels_.size()-1; j >=0; j--) {
                if (currentNode->activeLabels_[j]->status_ == ACTIVE) {
                    PLabel selectedLabel = currentNode->activeLabels_[j];
                    currentNode->nbActiveLabels_--;
                    selectedLabel->status_ = INACTIVE;
                    // drop onboard requests
                    extendToDropOnboards(selectedLabel);
                }
            }
        }
        break;
    }
    if (!activeNodes_.empty())
        return false;
    else
        return true;
}

bool LabelingSubProblem::ResolveDynamic_pullingWaveStep(float availableTime) {
    // create initial label
    while(true) {
        // create initial label
        initialization();
        // Wave 1: Pull towards Pickup nodes
        pullToPickups(availableTime, subGraph_->newPickNodes_, false);

        for (auto & currentNode : subGraph_->newPickNodes_) {
            for (int j = currentNode->activeLabels_.size()-1; j >=0; j--) {
                PLabel selectedLabel = currentNode->activeLabels_[j];
                // push to pickup points
                for (auto &neighbourNode: subGraph_->pickNodes_) {
                    if (selectedLabel->isExtendFeasible(&(*neighbourNode), maxPickup_,
                        solverOptions_->discardSuboptimalPath_, Vehicle_->capacity_, nbPrunedPath_,
                        nbEliminated_, nbPrunedArcs_)) {
                        int nbActive = neighbourNode->nbActiveLabels_;
                        if (labelExtend(selectedLabel, &(*neighbourNode), true)){
                            if (neighbourNode->nbActiveLabels_ == 1 && nbActive == 0) {
                                activeNodes_.push_back(&(*neighbourNode));
                            }
                        }
                    }
                }
            }
        }

        PNode initialNode = subGraph_->nodes_[initialNodeID_];
        std::vector<PNode> nodeList = subGraph_->pickNodes_;
        nodeList.push_back(initialNode);
        for (auto & newPickNode : subGraph_->newPickNodes_)
            nodeList.push_back(newPickNode);
        // Wave 2: Push to drop on-boards
        pushToDrops(availableTime, nodeList);
        break;
    }
    if (!activeNodes_.empty())
        return false;
    return true;
}

bool LabelingSubProblem::pushToDrops(float availableTime, std::vector<PNode> &pickNodeList) {
    while (!pickNodeList.empty() && subproTime_->dSinceStart().count() <= availableTime){
        PNode currentNode = pickNodeList.back();
        pickNodeList.pop_back();
        for (auto & selectedLabel: currentNode->activeLabels_){
            // drop onboard requests
            extendToDropOnboards(selectedLabel);
        }
    }
    // return false if the process has stoped due to lack of time
    if (!pickNodeList.empty())
        return false;

    while (!activeNodes_.empty() && subproTime_->dSinceStart().count() <= availableTime) {

        // select a node to extend active labels
        Node *currentNode = activeNodes_.back();
        activeNodes_.pop_back();
        for (int j = currentNode->activeLabels_.size()-1; j >=0; j--) {
            if (currentNode->activeLabels_[j]->status_ == ACTIVE) {
                PLabel selectedLabel = currentNode->activeLabels_[j];
                currentNode->nbActiveLabels_--;
                selectedLabel->status_ = INACTIVE;
                // drop onboard requests
                extendToDropOnboards(selectedLabel);
            }
        }
    }
    if (!activeNodes_.empty())
        return false;
    return true;
}

void LabelingSubProblem::pushToPickups(float availableTime, std::vector<PNode> &pickNodeList, bool doTruncation) {
    while (!activeNodes_.empty() && subproTime_->dSinceStart().count() <= availableTime) {
        if (doTruncation)
            std::stable_sort(activeNodes_.begin(),activeNodes_.end(),[](const Node *lhs, const Node *rhs){
            return lhs->travelTimeFromSource_ > rhs->travelTimeFromSource_;});

        // select a node to extend active labels
        Node *currentNode = activeNodes_.back();
        activeNodes_.pop_back();

        // decrease the number of active labels if truncated strategy is used
        if (doTruncation) {
            if (solverOptions_->isTruncated_ && currentNode->nbActiveLabels_ > solverOptions_->MaxLabel_){
                truncateLabelList(currentNode, solverOptions_->MaxLabel_, solverOptions_->MaxCommittedLabel_, labelPool_);
            }
        }
        for (int j = currentNode->activeLabels_.size()-1; j >=0; j--) {
            if (currentNode->activeLabels_[j]->status_ == ACTIVE) {
                PLabel selectedLabel = currentNode->activeLabels_[j];
                currentNode->nbActiveLabels_--;
                selectedLabel->status_ = INACTIVE;
                // push to pickup points
                for (auto &neighbourNode: pickNodeList) {
                    if (selectedLabel->isExtendFeasible(&(*neighbourNode), maxPickup_,
                        solverOptions_->discardSuboptimalPath_, Vehicle_->capacity_, nbPrunedPath_,
                        nbEliminated_, nbPrunedArcs_)) {
                        int nbActive = neighbourNode->nbActiveLabels_;
                        if (labelExtend(selectedLabel, &(*neighbourNode), true)){
                            if (neighbourNode->nbActiveLabels_ == 1 && nbActive == 0) {
                                activeNodes_.push_back(&(*neighbourNode));
                            }
                        }
                        }
                }
            }
        }
    }
}

void LabelingSubProblem::pullToPickups(float availableTime, std::vector<PNode> &pickNodeList, bool doTruncation) {
    while (!activeNodes_.empty() && subproTime_->dSinceStart().count() <= availableTime) {
        // select a node to pull other labels to it
        for (auto &currentNode: pickNodeList) {
            std::stable_sort(activeNodes_.begin(),activeNodes_.end(),[](const Node *lhs, const Node *rhs){
                return lhs->travelTimeFromSource_ > rhs->travelTimeFromSource_;});
            for (int j = activeNodes_.size()-1; j >= 0; j--) {
                if (activeNodes_[j]->nbActiveLabels_ == 0)
                    activeNodes_.erase(activeNodes_.begin() + j);
                else {
                    for (int l = activeNodes_[j]->activeLabels_.size() - 1; l >= 0; l--) {
                        if (activeNodes_[j]->activeLabels_[l]->status_ == ACTIVE){
                            PLabel selectedLabel = activeNodes_[j]->activeLabels_[l];
                            if (selectedLabel->numExtendCheck_ >= pickNodeList.size() ||
                                selectedLabel->nbPickUp_ >= maxPickup_)  {
                                selectedLabel->status_ = INACTIVE;
                                activeNodes_[j]->nbActiveLabels_--;
                                if (activeNodes_[j]->nbActiveLabels_ == 0) {
                                    activeNodes_.erase(activeNodes_.begin() + j);
                                    break;
                                }
                            }
                                // pull all labels to the current node
                            else if (selectedLabel->isExtendFeasible(&(*currentNode), maxPickup_,
                                                                     solverOptions_->discardSuboptimalPath_,
                                                                     Vehicle_->capacity_, nbPrunedPath_,
                                                                     nbEliminated_, nbPrunedArcs_)) {
                                int nbActive = currentNode->nbActiveLabels_;
                                if (labelExtend(selectedLabel, &(*currentNode), true)) {

                                    if (currentNode->nbActiveLabels_ == 1 && nbActive == 0) {
                                        activeNodes_.push_back(&(*currentNode));
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if (doTruncation) {
                // decrease the number of active labels if the truncated strategy is used
                if (solverOptions_->isTruncated_ && currentNode->nbActiveLabels_ > solverOptions_->MaxLabel_){
                    truncateLabelList(&(*currentNode), solverOptions_->MaxLabel_, solverOptions_->MaxCommittedLabel_, labelPool_);
                }
            }
        }
    }
}

bool LabelingSubProblem::solveDynamic_pullingWave1(float availableTime) {
    // create initial label
    while(true) {
        // create initial label
        initialization();
        while (!activeNodes_.empty() && subproTime_->dSinceStart().count() <= availableTime) {
            // select a node to pull other labels to it
            for (auto &currentNode: subGraph_->pickNodes_) {
                std::stable_sort(activeNodes_.begin(),activeNodes_.end(),[](const Node *lhs, const Node *rhs){
                    return lhs->travelTimeFromSource_ > rhs->travelTimeFromSource_;});
                if (subproTime_->dSinceStart().count() > availableTime)
                    return false;
                int nbActive = currentNode->nbActiveLabels_;
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
                                else if (selectedLabel->isExtendFeasible(&(*currentNode), maxPickup_,
                                                                         solverOptions_->discardSuboptimalPath_,
                                                                         Vehicle_->capacity_, nbPrunedPath_,
                                                                         nbEliminated_, nbPrunedArcs_)) {
                                    labelExtendPick(selectedLabel, &(*currentNode));
                                }
                            }
                        }
                    }
                }
                removeDominated(&(*currentNode), labelPool_);
                if ((currentNode->nbActiveLabels_ >= 1) && (nbActive == 0)) {
                    activeNodes_.push_back(&(*currentNode));
                }

                // decrease the number of active labels if truncated strategy is used
                if ((solverOptions_->isTruncated_) && (currentNode->nbActiveLabels_ > solverOptions_->MaxLabel_)){
                    truncateLabelList(&(*currentNode), solverOptions_->MaxLabel_,solverOptions_->MaxCommittedLabel_, labelPool_);
                }
            }
        }
        if (!activeNodes_.empty())
            return false;
        PNode initialNode = subGraph_->nodes_[initialNodeID_];
        std::vector<PNode> nodeList = subGraph_->pickNodes_;
        nodeList.push_back(initialNode);
        while (!nodeList.empty() && subproTime_->dSinceStart().count() < availableTime){
            PNode currentNode = nodeList.back();
            nodeList.pop_back();
            for (auto & selectedLabel: currentNode->activeLabels_){
                /*if (selectedLabel->openNode_.empty())
                    labelExtend(selectedLabel, subGraph_->sinkNodes_[0], false);*/
                // drop onboard requests
                extendToDropOnboards(selectedLabel);
            }
        }
        if (!nodeList.empty())
            return false;

        while (!activeNodes_.empty() && subproTime_->dSinceStart().count() <= availableTime) {

            // select a node to extend active labels
            Node *currentNode = activeNodes_.back();
            activeNodes_.pop_back();

            // decrease the number of active labels if truncated strategy is used
            if ((solverOptions_->isTruncated_) && (currentNode->nbActiveLabels_ > solverOptions_->MaxLabel_)){
                truncateLabelList(currentNode, solverOptions_->MaxLabel_, solverOptions_->MaxCommittedLabel_,labelPool_);
            }
            for (int j = currentNode->activeLabels_.size()-1; j >=0; j--) {
                if (currentNode->activeLabels_[j]->status_ == ACTIVE) {
                    PLabel selectedLabel = currentNode->activeLabels_[j];
                    currentNode->nbActiveLabels_--;
                    selectedLabel->status_ = INACTIVE;
                    // terminate to sink
                    /*if (selectedLabel->openNode_.empty())
                    labelExtend(selectedLabel, subGraph_->sinkNodes_[0], false);*/
                    // drop onboard requests
                    extendToDropOnboards(selectedLabel);
                }
            }
        }
        break;
    }
    if (!activeNodes_.empty())
        return false;
    else
        return true;
}

//***************************************************************************************//
//                           P U S H I N G  S T R A T E G Y
//***************************************************************************************//
bool LabelingSubProblem::solveDynamic_pushing(float availableTime) {
    while(true) {
        // create initial label
        initialization();

        while (!activeNodes_.empty() && subproTime_->dSinceStart().count() <= availableTime) {
            std::stable_sort(activeNodes_.begin(),activeNodes_.end(),[](const Node *lhs, const Node *rhs){
                return lhs->travelTimeFromSource_ < rhs->travelTimeFromSource_;});

            // select a node to extend active labels
            Node *currentNode = activeNodes_.back();
            activeNodes_.pop_back();

            // decrease the number of active labels if truncated strategy is used
            if ((solverOptions_->isTruncated_) && (currentNode->nbActiveLabels_ > solverOptions_->MaxLabel_)){
                truncateLabelList(currentNode, solverOptions_->MaxLabel_, solverOptions_->MaxCommittedLabel_,labelPool_);
            }
            for (int j = currentNode->activeLabels_.size()-1; j >=0; j--) {
                if (currentNode->activeLabels_[j]->status_ == ACTIVE) {
                    PLabel selectedLabel = currentNode->activeLabels_[j];
                    currentNode->nbActiveLabels_--;
                    selectedLabel->status_ = INACTIVE;
                    // drop onboard requests
                    extendToDropOnboards(selectedLabel);
                    // terminate to sink
                    /*else
                        labelExtend(selectedLabel, subGraph_->sinkNodes_[0], false);*/
                    // push to pickup points
                    if (selectedLabel->load_ < Vehicle_->capacity_) {
                        for (auto &neighbourNode: selectedLabel->pathNode_.back()->successors_) {
                            if (selectedLabel->isExtendFeasible(neighbourNode, maxPickup_, solverOptions_->discardSuboptimalPath_,
                                                                Vehicle_->capacity_, nbPrunedPath_,
                                                                nbEliminated_, nbPrunedArcs_)) {
                                int nbActive = neighbourNode->nbActiveLabels_;
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
    if (!activeNodes_.empty())
        return false;
    else
        return true;
}

bool LabelingSubProblem::solveDynamic_pushingDrop(float availableTime) {
    while(true) {
        // create initial label
        initialization();
        while (!activeNodes_.empty() && subproTime_->dSinceStart().count() <= availableTime) {
            std::stable_sort(activeNodes_.begin(),activeNodes_.end(),[](const Node *lhs, const Node *rhs){
                return lhs->nbActiveLabels_ < rhs->nbActiveLabels_;});

            // select a node to extend active labels
            Node *currentNode = activeNodes_.back();
            activeNodes_.pop_back();

            // decrease the number of active labels if truncated strategy is used
            if ((solverOptions_->isTruncated_) && (currentNode->nbActiveLabels_ > solverOptions_->MaxLabel_)){
                truncateLabelList(currentNode, solverOptions_->MaxLabel_,solverOptions_->MaxCommittedLabel_, labelPool_);
            }
            for (int j = currentNode->activeLabels_.size()-1; j >=0; j--) {
                if (currentNode->activeLabels_[j]->status_ == ACTIVE) {
                    PLabel selectedLabel = currentNode->activeLabels_[j];
                    currentNode->nbActiveLabels_--;
                    selectedLabel->status_ = INACTIVE;
                    // terminate to sink
                    /*if (selectedLabel->openNode_.empty())
                        labelExtend(selectedLabel, &(*subGraph_->sinkNodes_[0]), false);*/
                    // drop onboard requests and terminate to sink if empty
                    extendToDropOnboards(selectedLabel);
                    if (!selectedLabel->isDropped_) {
                        // push to pickup points
                        if (selectedLabel->load_ < Vehicle_->capacity_) {
                            for (auto &neighbourNode: selectedLabel->pathNode_.back()->successors_) {
                                if (selectedLabel->isExtendFeasible(neighbourNode, maxPickup_, solverOptions_->discardSuboptimalPath_,
                                                                    Vehicle_->capacity_, nbPrunedPath_,
                                                                    nbEliminated_, nbPrunedArcs_)) {
                                    int nbActive = (neighbourNode)->nbActiveLabels_;
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
    }
    if (!activeNodes_.empty())
        return false;
    else
        return true;
}

bool LabelingSubProblem::solveDynamic_pushingWave(float availableTime) {
    // create initial label
    while(true) {
        // create initial label
        initialization();
        while (!activeNodes_.empty() && subproTime_->dSinceStart().count() <= availableTime) {
            std::stable_sort(activeNodes_.begin(),activeNodes_.end(),[](const Node *lhs, const Node *rhs){
                return lhs->travelTimeFromSource_ > rhs->travelTimeFromSource_;});

            // select a node to extend active labels
            Node *currentNode = activeNodes_.back();
            activeNodes_.pop_back();

            // decrease the number of active labels if truncated strategy is used
            if ((solverOptions_->isTruncated_) && (currentNode->nbActiveLabels_ > solverOptions_->MaxLabel_)){
                truncateLabelList(currentNode, solverOptions_->MaxLabel_, solverOptions_->MaxCommittedLabel_, labelPool_);
            }
            for (int j = currentNode->activeLabels_.size()-1; j >=0; j--) {
                if (currentNode->activeLabels_[j]->status_ == ACTIVE) {
                    PLabel selectedLabel = currentNode->activeLabels_[j];
                    currentNode->nbActiveLabels_--;
                    selectedLabel->status_ = INACTIVE;
                    // push to pickup points
                    if (selectedLabel->load_ < Vehicle_->capacity_) {
                        for (auto &neighbourNode: selectedLabel->pathNode_.back()->successors_) {
                            if (selectedLabel->isExtendFeasible(neighbourNode, maxPickup_, solverOptions_->discardSuboptimalPath_,
                                                                Vehicle_->capacity_, nbPrunedPath_,
                                                                nbEliminated_, nbPrunedArcs_)) {
                                int nbActive = neighbourNode->nbActiveLabels_;
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
        while (!nodeList.empty() && subproTime_->dSinceStart().count() <= availableTime){
            PNode currentNode = nodeList.back();
            nodeList.pop_back();
            for (auto & selectedLabel: currentNode->activeLabels_){
                // drop onboard requests
                extendToDropOnboards(selectedLabel);
            }
        }
        if (!nodeList.empty())
            return false;
        activeNodes_.push_back(&(*initialNode));
        initialNode->activeLabels_[0]->status_ = ACTIVE;
        while (!activeNodes_.empty() && subproTime_->dSinceStart().count() <= availableTime) {
            // select a node to extend active labels
            Node *currentNode = activeNodes_.back();
            activeNodes_.pop_back();

            // decrease the number of active labels if truncated strategy is used
            if ((solverOptions_->isTruncated_) && (currentNode->nbActiveLabels_ > solverOptions_->MaxLabel_)){
                truncateLabelList(currentNode, solverOptions_->MaxLabel_, solverOptions_->MaxCommittedLabel_,labelPool_);
            }
            for (int j = currentNode->activeLabels_.size()-1; j >=0; j--) {
                if (currentNode->activeLabels_[j]->status_ == ACTIVE) {
                    PLabel selectedLabel = currentNode->activeLabels_[j];
                    currentNode->nbActiveLabels_--;
                    selectedLabel->status_ = INACTIVE;

                    // drop onboard requests
                    extendToDropOnboards(selectedLabel);
                }
            }
        }
        break;
    }
    if (!activeNodes_.empty())
        return false;
    return true;
}

bool LabelingSubProblem::solveDynamic_pushingWaveStep(float availableTime) {
    // create initial label
    while(true) {
        // create initial label
        initialization();

        // Wave 1: Push to Pickup nodes
        pushToPickups(availableTime, subGraph_->pickNodes_, true);

        PNode initialNode = subGraph_->nodes_[initialNodeID_];
        std::vector<PNode> nodeList = subGraph_->pickNodes_;
        nodeList.push_back(initialNode);

        // Wave 2: Push to drop on-boards
        pushToDrops(availableTime, nodeList);
        break;
    }
    if (!activeNodes_.empty())
        return false;
    return true;
}

//***************************************************************************************//
//                           P U L L I N G  S T R A T E G Y
//***************************************************************************************//

bool LabelingSubProblem::solveDynamic(float availableTime) {
    subproTime_->start();
    if (!reOptimize_ ) {
        if (solverOptions_->LabelingStrategy_ == PUSHING || subRequests_.empty()){
            if (solverOptions_->isDropPickPossible_) {
                if (!this->solveDynamic_pushing(availableTime)) {
                    subproTime_->stop();
                    return false;
                }
            }
            else {
                if (!this->solveDynamic_pushingWaveStep(availableTime)) {
                    subproTime_->stop();
                    return false;
                }
            }
        }

        else {
            if (solverOptions_->isDropPickPossible_) {
                if (!this->solveDynamic_pulling(availableTime)) {
                    subproTime_->stop();
                    return false;
                }
            }
            else {
                if (!this->solveDynamic_pullingWaveStep(availableTime)) {
                    subproTime_->stop();
                    return false;
                }
            }
        }
    }
    else {
        if (solverOptions_->labelingReOptimizeStrategy_ == RE_INSERT &&  !subGraph_->newPickNodes_.empty()) {
            if (!this->ResolveDynamic_pullingWaveStep(availableTime)) {
                subproTime_->stop();
                return false;
            }
        }
        else {
            if (!this->solveDynamic_pullingWaveStep(availableTime)) {
                subproTime_->stop();
                return false;
            }
        }
    }
    subproTime_->stop();
    return true;
}

void LabelingSubProblem::SolutionToRoutes(const PVehicle &vehicle, vector<PRoute> &availableRoutes,
    const PInstance &pInst, int nbRequests) {
    nbNegativeColumns_ = 0;
    availableRoutes.clear();
    for (auto & labelObj : subGraph_->sinkNodes_[0]->activeLabels_) {
        if (labelObj->numCompleted_ > 0) {
            PRoute newRoute = labelObj->labelToRoute(vehicle, pInst);
            newRoute->createColumn(nbRequests);
 //           if (vehicle->currentRoute_->routeRequests_.empty() || !newRoute->equal(*vehicle->currentRoute_)) {
                if (newRoute->reducedCost_ < -0.01)
                    nbNegativeColumns_ ++;
                nbOutputs_++;
                availableRoutes.emplace_back(std::move(newRoute));
//            }
        }
    }
    for (auto & nodeObj : subGraph_->nodes_){
        for (auto & labelObj : nodeObj.second->activeLabels_)
            labelPool_.push_back(std::move(labelObj));
    }
}

void LabelingSubProblem::CollectLabels() {
    for (auto & nodeObj : subGraph_->nodes_){
        for (auto & labelObj : nodeObj.second->activeLabels_)
            labelPool_.push_back(std::move(labelObj));
    }
}

void LabelingSubProblem::solutionSummery(vector<int> &subProResults) const {
    if (subProResults.empty()) {
        subProResults.push_back(nbGenerated_);
        subProResults.push_back(nbGenerated_);
        subProResults.push_back(nbDominated_);
        subProResults.push_back(subGraph_->nbNodes_);
        subProResults.push_back(static_cast<int>(subGraph_->pickNodes_.size()));
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
    repStr << nbEliminated_ << ",";
    repStr << nbPrunedArcs_ << ",";
    repStr << nbPrunedPath_ << ",";
    repStr << nbNegativeColumns_ << ",";
    repStr << nbOutputs_ << ",";
    repStr << Vehicle_->bestReducedCost_ << ",";
    repStr << subproTime_->dSinceStart().count() << "\n";
    return repStr.str();
}

void LabelingSubProblem::truncateLabelList(Node *node, int MaxLabel, std::vector<PLabel> & labelPool) {
    if (solverOptions_->sortPath_ == RD_COST) {
        std::stable_sort(node->activeLabels_.begin(), node->activeLabels_.end(),
                         [](const PLabel &lhs, const PLabel &rhs) {
                             return lhs->reducedCost_ < rhs->reducedCost_;
                         });
    }
    else if (solverOptions_->sortPath_ == LAMBDA) {
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
        if (node->activeLabels_[i]->status_ == ACTIVE){
            node->nbActiveLabels_--;
            node->activeLabels_[i]->status_ = DOMINATED;
            labelPool.push_back(std::move(node->activeLabels_[i]));
            node->activeLabels_.erase(node->activeLabels_.begin() + i);
        }
    }
}


void LabelingSubProblem::truncateLabelList(Node *node, int MaxLabel, int MaxCommittedLabel, std::vector<PLabel> & labelPool) const {
    if (solverOptions_->sortPath_ == RD_COST) {
        std::stable_sort(node->activeLabels_.begin(), node->activeLabels_.end(),
                         [](const PLabel &lhs, const PLabel &rhs) {
                             return lhs->reducedCost_ < rhs->reducedCost_;
                         });
    }
    else if (solverOptions_->sortPath_ == LAMBDA) {
        std::stable_sort(node->activeLabels_.begin(), node->activeLabels_.end(),
                         [](const PLabel &lhs, const PLabel &rhs) {
                             return lhs->lambdaScore_ < rhs->lambdaScore_;
                         });
    }
    else {
        std::stable_sort(node->activeLabels_.begin(),node->activeLabels_.end(),[](const PLabel &lhs, const PLabel &rhs){
            return lhs->labelScore_ < rhs->labelScore_;});
    }
    if (MaxCommittedLabel == 0) {
        // Iterate from best to worst (forward order after sorting)
        for (int i = node->activeLabels_.size()-1; i >=0; i--){
            if (node->nbActiveLabels_ <= MaxLabel)
                break;
            if (node->activeLabels_[i]->status_ == ACTIVE){
                node->nbActiveLabels_--;
                node->activeLabels_[i]->status_ = DOMINATED;
                labelPool.push_back(std::move(node->activeLabels_[i]));
                node->activeLabels_.erase(node->activeLabels_.begin() + i);
            }
        }
    }
    else {
        int keptCommittedLabels = 0;
        int keptNonCommittedLabels = 0;

        // Iterate from best to worst (forward order after sorting)
        for (int i = 0; i < node->activeLabels_.size(); i++){
            if (node->activeLabels_[i]->status_ != ACTIVE) {
                continue;
            }

            bool shouldKeep = false;

            if (node->activeLabels_[i]->nbCommitted_ > 0) {
                // This is a committed label
                if (keptCommittedLabels < MaxCommittedLabel) {
                    shouldKeep = true;
                    keptCommittedLabels++;
                }
            } else {
                // This is a non-committed label
                if (keptNonCommittedLabels < MaxLabel) {
                    shouldKeep = true;
                    keptNonCommittedLabels++;
                }
            }

            if (!shouldKeep) {
                node->nbActiveLabels_--;
                node->activeLabels_[i]->status_ = DOMINATED;
                labelPool.push_back(std::move(node->activeLabels_[i]));
                node->activeLabels_.erase(node->activeLabels_.begin() + i);
                i--; // Adjust index since we removed an element
            }
        }
    }
}
void LabelingSubProblem::removeDominated(Node *node, std::vector<PLabel> &labelPool) const {
    // Sort labels by increasing reduced cost
    std::stable_sort(node->activeLabels_.begin(), node->activeLabels_.end(),
                     [](const PLabel &lhs, const PLabel &rhs) {
                         return lhs->reducedCost_ < rhs->reducedCost_;
                     });

    for (int i = node->activeLabels_.size() - 1; i >= 0; --i) {
        // Compare the current label with all previous labels in the sorted list
        for (int j = 0; j < node->activeLabels_.size(); ++j) {
            if (i != j) {
                if (node->activeLabels_[i]->isDominated(node->activeLabels_[j],
                                                        this->solverOptions_)) { // Assuming solverOptions_ is accessible
                    if (node->activeLabels_[i]->status_ == ACTIVE)
                        node->nbActiveLabels_--; // Assuming you track the number of active labels
                    node->activeLabels_[i]->status_ = DOMINATED; // Assuming there is a status field
                    labelPool.push_back(std::move(node->activeLabels_[i])); // Move the dominated label to the pool
                    node->activeLabels_.erase(node->activeLabels_.begin() + i); // Erase from active labels
                    break;
                }
            }
        }
    }
}


void LabelingSubProblem::extendToDropOnboards(const PLabel &selectedLabel) {
    if (!selectedLabel->openNode_.empty()) {
        for (auto &neighbourNode : selectedLabel->openNode_) {
            if (selectedLabel->isTravelTimeFeasible(neighbourNode, nbEliminated_)) {
                int nbActive = neighbourNode->nbActiveLabels_;
                if (labelExtend(selectedLabel, neighbourNode, true)) {
                    if (neighbourNode->nbActiveLabels_ == 1 && nbActive == 0) {
                        activeNodes_.push_back(neighbourNode);
                    }
                }
            }
        }
    }
}

void LabelingSubProblem::constructLabels(const PLabel &initialLabel) {
    for (auto &route: availableRoutes_) {
        bool isValidRoute = true;

        // Check if all nodes in the route exist in the subGraph
        for (auto &node: route->routeNodes_) {
            if (subGraph_->nodes_.count(node->nodeID_) == 0) {
                isValidRoute = false;
                break;
            }
        }

        if (!isValidRoute) {
            continue; // Skip invalid routes
        }
        nbRecycledColumns_++;
        PLabel parentLabel = initialLabel;

        for (int i = 1; i < route->routeSize_; ++i) {
            // Get or create a new label
            PLabel newLabel;
            if (!labelPool_.empty()) {
                newLabel = std::move(labelPool_.back());
                labelPool_.pop_back();
                newLabel->copyLabel(*parentLabel);
            } else {
                newLabel = std::make_shared<Label>(*parentLabel);
            }

            // Extend the label
            newLabel->extend(&(*subGraph_->nodes_[route->routeNodes_[i]->nodeID_]),
                             solverOptions_->isDropPickPossible_);

            bool isRepeated = false;
            for (auto &labelObj: newLabel->pathNode_.back()->activeLabels_) {
                if (newLabel->isDominated(labelObj, this->solverOptions_)) {
                    isRepeated = true;
                    break;
                }
            }
            if (!isRepeated) {
                auto &pathNode = newLabel->pathNode_.back();

                // Update label statistics and active node list
                pathNode->nbActiveLabels_++;
                if (pathNode->bestLabelReduceCost_ > newLabel->reducedCost_) {
                    pathNode->bestLabelReduceCost_ = newLabel->reducedCost_;
                }

                if (pathNode->nbActiveLabels_ == 1) {
                    activeNodes_.push_back(pathNode);
                }

                pathNode->activeLabels_.push_back(newLabel);

                // Update parent label for pickup nodes
                if (route->routeNodes_[i]->type_ == PICKUP) {
                    parentLabel->extendCheck_.set(route->routeNodes_[i]->related_Request_->taskIndexLabel_, true);
                    parentLabel->numExtendCheck_++;
                }
            }

            // Update the parent label for the next iteration
            parentLabel = newLabel;
        }
    }

}

void LabelingSubProblem::constructBaseLabels(const PLabel &initialLabel) {
    for (auto &route: availableRoutes_) {
        nbRecycledColumns_++;
        PLabel parentLabel = initialLabel;

        for (int i = 1; i < route->routeSize_; ++i) {
            if (route->routeNodes_[i]->type_ == DROPOFF || route->routeNodes_[i]->related_Request_->solVehicleID_ == LARGE_CONSTANT)
                break;
            
            auto currentNode = subGraph_->nodes_[route->routeNodes_[i]->nodeID_];

            if (parentLabel->isExtendFeasible(&*currentNode, maxPickup_, solverOptions_->discardSuboptimalPath_,
                Vehicle_->capacity_, nbPrunedPath_,nbEliminated_, nbPrunedArcs_)) {
                int nbActive = currentNode->nbActiveLabels_;
                if (labelExtend(parentLabel, &(*currentNode), false)) {
                    // Update the parent label for the next iteration
                    parentLabel = currentNode->activeLabels_.back();

                    if ((currentNode->nbActiveLabels_ == 1) && (nbActive == 0)) {
                        activeNodes_.push_back(&(*currentNode));
                    }
                }
            }
        }
    }
    for (auto &nodeObj: activeNodes_) {
        for (auto &labelObj: nodeObj->activeLabels_) {
            labelObj->numExtendCheck_ = 0;
        }
    }
}


