//
// Created by Ella on 2/22/2022.
//

#include "Label.h"

unsigned int Label::labelCount_ = 0;

Label::Label(PVehicle *vehicle, PNode &source, int numRequests) : labelID_(labelCount_++), vehicle_(vehicle) {
    char* name2 = new char[255];
    strncpy(name2, std::to_string(labelID_).c_str(), 255);
    name_ = name2;

    load_ = (*vehicle)->numPassengers_;
    passedTime_ = (*vehicle)->departTime_;
    pathNodes_.push_back(source->nodeID_);
    pathNode_.push_back(&source);
 //   path_.push_back(&source);
    reducedCost_ = 0;
    totalDelay_ = 0;
    status_ = ACTIVE;
//    openNodes_.clear();
    openNode_.clear();
    openRequests_.clear();
 //   completedRequests_.clear();
 //   completedRequest_.clear();
    completeRequests_ = new myTools::BitVector(numRequests);
    extendCheck_ = new myTools::BitVector(numRequests);
    numExtendCheck_ = 0;
    currentNode_ = &source;
    nbPickUp_ = 0;
    nbPickMove_ = 0;
//    extendCheck_.insert(source->nodeID_);
//    parent_ = nullptr;
    isDropped_ = false;
    isDropExtend_ = false;
    nbUsed_ = 0;
    createTime_ = 0;
}

Label::Label(const Label &label) :labelID_(labelCount_++) {
    char* name2 = new char[255];
    strncpy(name2, std::to_string(labelID_).c_str(), 255);
    name_ = name2;
    status_ = ACTIVE;
    load_ = label.load_;
    passedTime_ = label.passedTime_;
    vehicle_ = label.vehicle_;
//    openReachTime_ = label.openReachTime_;
//    travelResource_ = label.travelResource_;
    travelResources_ = label.travelResources_;
    pathNodes_ = label.pathNodes_;
    pathNode_ = label.pathNode_;
 //   path_ = label.path_;
    reducedCost_ = label.reducedCost_;
    currentNode_ = label.currentNode_;
    totalDelay_ = label.totalDelay_;
//    openNodes_ = label.openNodes_;
    openNode_ = label.openNode_;
    openRequests_ = label.openRequests_;
//    completedRequests_ = label.completedRequests_;
    completeRequests_ = new myTools::BitVector(*label.completeRequests_);
    extendCheck_ = new myTools::BitVector(*label.completeRequests_);
    numCompleted_ = label.numCompleted_;
    numExtendCheck_ = label.numCompleted_;
//    extendCheck_ = completedRequests_;
//    if ((*currentNode_)->type_ == PICKUP)
//        extendCheck_[(*currentNode_)->related_Request_->taskIndexLabel_] = 1;
 //   completedRequest_ = label.completedRequest_;
//    requestIDToInt_ = label.requestIDToInt_;
    nbPickUp_ = label.nbPickUp_;
    nbPickMove_ = label.nbPickMove_;
    /*for (auto & nodeObj:label.pathNodes_) {
        if (nodeObj->type_ == PICKUP)
            extendCheck_.insert(nodeObj->nodeID_);
    }*/
    nbUsed_ = 0;
    isDropped_ = label.isDropped_;
    isDropExtend_ = false;
    createTime_ = 0;
}
void Label::copyLabel(const Label &label) {
    status_ = ACTIVE;
    load_ = label.load_;
    passedTime_ = label.passedTime_;
    vehicle_ = label.vehicle_;
//    parent_ = nullptr;
    travelResources_ = label.travelResources_;
    pathNodes_ = label.pathNodes_;
    pathNode_ = label.pathNode_;
 //   path_ = label.path_;
    reducedCost_ = label.reducedCost_;
    currentNode_ = label.currentNode_;
    totalDelay_ = label.totalDelay_;
    openNode_ = label.openNode_;
    openRequests_ = label.openRequests_;
//    completedRequests_ = label.completedRequests_;
    completeRequests_->copyValues(*label.completeRequests_);
    extendCheck_->copyValues(*label.completeRequests_);
    numCompleted_ = label.numCompleted_;
    numExtendCheck_ = label.numCompleted_;
//    extendCheck_ = label.completedRequests_;
//    if ((*currentNode_)->type_ == PICKUP)
//        extendCheck_[(*currentNode_)->related_Request_->taskIndexLabel_] = 1;
    nbPickUp_ = label.nbPickUp_;
    nbPickMove_ = label.nbPickMove_;
    isDropped_ = label.isDropped_;
    isDropExtend_ = false;
    nbUsed_ = 1;
}
Label::~Label() {
//    openNode_.clear();
//    pathNodes_.clear();
    delete[] name_;
    delete completeRequests_;
    delete extendCheck_;
}

unsigned int Label::getLabelId() const {
    return labelID_;
}

bool Label::operator () (const Label &rhs) const {
    return reducedCost_ < rhs.reducedCost_;
}

void Label:: extend(PNode &outNode) {
    load_ += outNode->nbPassengers_;
    float travelTime =  durationMatrix_[(*currentNode_)->locationID_][outNode->locationID_];
    float reachTime;
//    if (passedTime_ < outNode->requestTime_)
//        reachTime = outNode->requestTime_ + travelTime;
//    else
        reachTime = passedTime_ + travelTime;
    /*for (auto & item: travelResource_) {
        item.second -= travelTime;
    }*/
    for (auto &node: openNode_) {
        travelResources_[(*node)->related_Request_->taskIndexLabel_] -= (reachTime + outNode->deltaTime_ - passedTime_);
    }
    if (outNode->type_ == DROPOFF) {
        travelResources_[outNode->related_Request_->taskIndexLabel_] = 0;
        for (int i = 0; i < openNode_.size(); i++){
            if ((*openNode_[i])->nodeID_ == outNode->nodeID_){
                openNode_.erase(openNode_.begin()+i);
                break;
            }
        }
        openRequests_[outNode->related_Request_->taskIndexLabel_] = 0;
        if (!isDropped_ && travelTime > 0)
            isDropped_ = true;

    }
    else if (outNode->type_ == PICKUP){
 //       extendCheck_.insert(outNode->nodeID_);
        openNode_.push_back(outNode->pairNode_);
 //       openRequests_[requestIDToInt_[outNode->related_Request_->getRequestId()]] = 1;
//        completedRequests_[outNode->related_Request_->taskIndexLabel_] = 1;
        completeRequests_->add(outNode->related_Request_->taskIndexLabel_);
        numCompleted_++;
        extendCheck_->add(outNode->related_Request_->taskIndexLabel_);
        numExtendCheck_++;
 //       extendCheck_[outNode->related_Request_->taskIndexLabel_] = 1;
        openRequests_[outNode->related_Request_->taskIndexLabel_] = 1;
        reducedCost_ -= (outNode->related_Request_)->dual_;
        if (travelTime > 0){
            nbPickMove_++;
        }
        nbPickUp_ ++;
        totalDelay_ += (reachTime - outNode->requestTime_);
        reducedCost_ += (reachTime - outNode->requestTime_);
        travelResources_[outNode->related_Request_->taskIndexLabel_] = outNode->related_Request_->maxTravelTime_ + outNode->deltaTime_;
    }
    /*else if (outNode->type_ == SINK){
        passedTime_ = reachTime;
    }*/


    passedTime_ = reachTime + outNode->deltaTime_;

    pathNodes_.push_back(outNode->nodeID_);
    pathNode_.push_back(&outNode);
 //   path_.push_back(&outNode);
    currentNode_ = &outNode;
//    extendCheck_.insert(outNode->nodeID_);
}

// this function check the feasibility of the label before extension
bool Label::isExtendFeasible(PNode &outNode, int maxPickUp, bool usePick) {
    if (outNode->type_ == PICKUP){
        extendCheck_->add(outNode->related_Request_->taskIndexLabel_);
        numExtendCheck_++;
    }
    if ((load_ + outNode->nbPassengers_) > (*vehicle_)->capacity_)
        return false;
    if (outNode->type_ == PICKUP) {
        if (usePick){
            if (nbPickUp_ >= maxPickUp)
                return false;
        }
        else{
            if (nbPickMove_ >= maxPickUp && (*currentNode_)->locationID_ != outNode->locationID_)
                return false;
        }

 //       if (completedRequests_.count(outNode->related_Request_))
 //       if (completedRequests_[requestIDToInt_[outNode->related_Request_->getRequestId()]] == 1)
        if (completeRequests_->contains(outNode->related_Request_->taskIndexLabel_))
            return false;
        /*if (completedRequests_[outNode->related_Request_->taskIndexLabel_] == 1)
            return false;*/
    }
    if (outNode->type_ == DROPOFF) {
 //       if (openRequests_[requestIDToInt_[outNode->related_Request_->getRequestId()]] == 0)
        if (openRequests_[outNode->related_Request_->taskIndexLabel_] == 0)
 //       if (openNodes_.find(outNode) == openNodes_.end())
            return false;
    }
    if (outNode->type_ == SINK) {
//        if (openRequests_.size() > 0)
        /*if (!openNodes_.empty())
            return false;*/
        if (!openNode_.empty())
            return false;
    }
    if ((*currentNode_)->locationID_ != outNode->locationID_) {
        for (auto &nodeObj: openNode_) {
            float travelToDrop =
                    (*currentNode_)->deltaTime_ + durationMatrix_[(*currentNode_)->locationID_][outNode->locationID_] +
                    outNode->deltaTime_ + durationMatrix_[outNode->locationID_][(*nodeObj)->locationID_];
            /*if (travelResource_[nodeObj->nodeID_] <  travelToDrop)
                return false;*/
            if (travelResources_[(*nodeObj)->related_Request_->taskIndexLabel_] < travelToDrop)
                return false;
        }
    }
    else{
        for (auto &nodeObj: openNode_) {
            float travelToDrop = (*currentNode_)->deltaTime_ + outNode->deltaTime_ +
                    durationMatrix_[outNode->locationID_][(*nodeObj)->locationID_];
            if (travelResources_[(*nodeObj)->related_Request_->taskIndexLabel_] < travelToDrop)
                return false;
        }
    }

    /*for (auto & nodeObj: openNodes_) {
        float travelToDrop = currentNode_->deltaTime_ + durationMatrix_[currentNode_->locationID_][outNode->locationID_] +
                outNode->deltaTime_ + durationMatrix_[outNode->locationID_][nodeObj->locationID_];
        *//*if (travelResource_[nodeObj->nodeID_] <  travelToDrop)
            return false;*//*
        if (travelResources_[nodeObj->related_Request_->taskIndexLabel_] <  travelToDrop)
            return false;
    }*/




    /*for (auto & nodeObj : openNodes_) {
        float travelDuration = reachTime - openReachTime_[nodeObj->nodeID_] + outNode->deltaTime_ +
                travelMat->queryTravelTime(outNode, *nodeObj->pairNode_);
        if (travelDuration > (*nodeObj->related_Request_)->maxTravelTime_){
            return false;
        }

    }*/
    return true;
}

bool Label::isDominated(PLabel &otherLabel, PSolverOption &solverOption) const {
    if (currentNode_ != otherLabel->currentNode_)
        myTools::throwException("Label Domination error!!");

    if ((*currentNode_)->type_ == SINK){
        if (this->passedTime_ >= otherLabel->passedTime_) {
            if (this->reducedCost_ >= otherLabel->reducedCost_) {
                if (this->numCompleted_ >= otherLabel->numCompleted_) {
                    if (solverOption->isDominanceReleased_) {
                        return true;
                    }
                    else {
                        if (otherLabel->completeRequests_->isSubset(*this->completeRequests_))
                            return true;
                    }
                }
            }
        }
    }
    else {

        if (this->passedTime_ >= otherLabel->passedTime_) {
            if (this->reducedCost_ >= otherLabel->reducedCost_) {
                if (this->numCompleted_ >= otherLabel->numCompleted_) {
                    if (this->openRequests_ == otherLabel->openRequests_) {
                        if (solverOption->isDominanceReleased_) {
                            if (this->numCompleted_ > otherLabel->numCompleted_)
                                return true;
                        } else {
                            if (otherLabel->completeRequests_->isSubset(*this->completeRequests_)){
                                return true;
                                /*if ((this->passedTime_ == otherLabel->passedTime_) &&
                                    (this->reducedCost_ == otherLabel->reducedCost_)) {
                                    if (this->travelResources_ == otherLabel->travelResources_)
                                        return true;
                                    else
                                        return false;
                                } else
                                    return true;*/
                            }
                        }

                    }
                }
            }
        }
    }
    return false;
}
// this function examine the label to be sure that it leads to a route with negative reduced cost
bool Label::isEliminated() {
    /*for (auto & nodeObj: openNodes_) {
        float travelDuration = passedTime_ - openReachTime_[nodeObj->nodeID_] + (*currentNode_)->deltaTime_ +
                               durationMatrix_[(*currentNode_)->locationID_][nodeObj->locationID_];
        if (travelDuration > (*nodeObj->related_Request_)->maxTravelTime_)
            return true;
    }*/

    /*for (auto & nodeObj: openNodes_) {
        *//*float travelDuration = (*currentNode_)->deltaTime_ +
                               durationMatrix_[(*currentNode_)->locationID_][nodeObj->locationID_];*//*
        *//*if (travelResource_[nodeObj->nodeID_] < currentNode_->deltaTime_ + durationMatrix_[currentNode_->locationID_][nodeObj->locationID_])
            return true;*//*
        if (travelResources_[nodeObj->related_Request_->taskIndexLabel_] < currentNode_->deltaTime_ + durationMatrix_[currentNode_->locationID_][nodeObj->locationID_])
            return true;
    }*/
    for (auto & nodeObj: openNode_) {
        /*float travelDuration = (*currentNode_)->deltaTime_ +
                               durationMatrix_[(*currentNode_)->locationID_][nodeObj->locationID_];*/
        /*if (travelResource_[nodeObj->nodeID_] < currentNode_->deltaTime_ + durationMatrix_[currentNode_->locationID_][nodeObj->locationID_])
            return true;*/
        if (travelResources_[(*nodeObj)->related_Request_->taskIndexLabel_] < (*currentNode_)->deltaTime_ +
            durationMatrix_[(*currentNode_)->locationID_][(*nodeObj)->locationID_])
            return true;
    }

    /*for (auto & requestObj: openRequests_) {
        std::string pickID = myTools::createNodeID(requestObj->getRequestId(), PICKUP);
        std::string dropID = myTools::createNodeID(requestObj->getRequestId(), DROPOFF);

        float travelDuration = passedTime_ - openReachTime_[pickID] + (*currentNode_)->deltaTime_ +
                               durationMatrix_[(*currentNode_)->locationID_][graph->nodes_[dropID]->locationID_];
        if (travelDuration > requestObj->maxTravelTime_)
            return true;
    }*/

    /*if ((*currentNode_)->type_ != SINK) {
        float predictedReducedCost = reducedCost_;
        for (auto &nodeID : graph->intToNodeID_) {
            if (graph->nodes_[nodeID]->type_ == PICKUP) {
                if ((nbPickUp_ != maxPickUp) &&(!completedRequests_.count(*graph->nodes_[nodeID]->related_Request_))
                    &&(!openRequests_.count(*graph->nodes_[nodeID]->related_Request_))) {
                    float reachTime = passedTime_ + travelMat->queryTravelTime((*currentNode_), graph->nodes_[nodeID]) +
                                      (*currentNode_)->deltaTime_;
                    float addedValue = std::max(reachTime, graph->nodes_[nodeID]->requestTime_) - graph->nodes_[nodeID]->requestTime_ -
                                       (*graph->nodes_[nodeID]->related_Request_)->dual_;
                    if (addedValue < 0)
                        predictedReducedCost += addedValue;
                }
            }
        }
        if (predictedReducedCost < 0)
            return false;
        else
            return true;
    }*/
    return false;
}

// this function check whether the label is originated from a dominated parent or not
bool Label::haveDominatedParent() {
    /*PLabel  childLabel = parent_;
    if (childLabel != nullptr) {
        while (childLabel->parent_ != nullptr) {
            if (childLabel->status_ == DOMINATED)
                return true;
            else
                childLabel = childLabel->parent_;
        }
    }*/
    return false;
}

/*PRoute Label::labelToRoute(PVehicle &vehicle) {
    PRoute newRoute = std::make_shared<Route>(vehicle->vehicleID_);
    newRoute->reducedCost_ = reducedCost_ - vehicle->dual_;
    newRoute->addSource(pathNodes_[0], vehicle->departTime_, vehicle->numPassengers_);
    for (int i = 1; i < pathNodes_.size()-1; ++i) {
        newRoute->addNode(pathNodes_[i]);
    }
    newRoute->createTime_ = createTime_;
    if (totalDelay_ != newRoute->totalDelay_) {
        std::cout << "Total delay of the label partial path is not the same as the route delay" << std::endl;
        myTools::throwException("Label convert problem");
    }
    return newRoute;
}*/

PRoute Label::labelToRoute(PVehicle &vehicle, PInstance &pInst) {
    PRoute newRoute = std::make_shared<Route>(vehicle->vehicleID_);
    newRoute->totalLength_ = passedTime_ - vehicle->departTime_;
    newRoute->reducedCost_ = reducedCost_ - vehicle->dual_;
    newRoute->addSource(vehicle->departNode_, vehicle->departTime_, vehicle->numPassengers_);
    /*for (int i = 1; i < pathNodes_.size()-1; ++i) {
        newRoute->addNode(pInst->instGraph_->nodes_[pathNodes_[i]]);
    }*/
    for (int i = 1; i < pathNode_.size()-1; ++i) {
        if ((*pathNode_[i])->type_ == PICKUP)
            newRoute->addNode(pInst->instGraph_->pickNodes_[(*pathNode_[i])->related_Request_->getRequestId()]);
        else
            newRoute->addNode(pInst->instGraph_->dropNodes_[(*pathNode_[i])->related_Request_->getRequestId()]);
    }
    newRoute->createTime_ = createTime_;
    if (totalDelay_ != newRoute->totalDelay_) {
        std::cout << "Total delay of the label partial path is not the same as the route delay" << std::endl;
        myTools::throwException("Label convert problem");
    }
    return newRoute;
}

std::string Label::toString() const {
    std::stringstream repStr;

    repStr << "#" << std::left << std::endl;
    repStr << "#\t" << std::setw(24) << "- LABEL INFO" << " : " << std::endl;
    repStr << "# \t" <<"_____________________" << std::endl;
    repStr << "#\t" << std::setw(24) << "- LABEL_NUMBER" << " : " << labelID_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- CURRENT_NODE" << " : " << (*currentNode_)->nodeID_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- PASSED_TIME (seconds)" << " : " << passedTime_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- NUMBER_OF_STOPS" << " : " << pathNodes_.size() << std::endl;
    repStr << "#\t" << std::setw(24) << "- TOTAL_WAITING (seconds)" << " : " << totalDelay_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- REDUCED_COST" << " : " << reducedCost_ << std::endl;
    repStr << "#" << std::endl;
    repStr << "#\t" << std::setw(24) << "- OPEN_REQUESTS" << " : " ;
    for (auto & nodeObj : openNode_) {
        repStr << (*nodeObj)->related_Request_->getRequestId() << "  ";
    }
    repStr << std::endl;

    repStr << std::endl;
    repStr << "# ________________________________________________________________________" << std::endl;
    return repStr.str();
}





