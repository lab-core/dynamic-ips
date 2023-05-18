//
// Created by Ella on 2/22/2022.
//

#include "Label.h"

unsigned int Label::labelCount_ = 0;

Label::Label(Vehicle *vehicle, PNode &source, int numRequests) : labelID_(labelCount_++) {
    char* name2 = new char[255];
    strncpy(name2, std::to_string(labelID_).c_str(), 255);
    name_ = name2;

    load_ = vehicle->numPassengers_;
    passedTime_ = vehicle->departTime_;
    reachedTime_ = vehicle->departTime_;
//    pathNodes_.push_back(source->nodeID_);
    pathNode_.push_back(&(*source));
 //   path_.push_back(&source);
    reducedCost_ = 0;
    totalDelay_ = 0;
    status_ = ACTIVE;
//    openNodes_.clear();
    openNode_.clear();
    openRequests_.clear();
 //   completedRequests_.clear();
 //   completedRequest_.clear();
    /*completeRequests_ = std::make_shared<myTools::BitVector>(numRequests);
    extendCheck_ = std::make_shared<myTools::BitVector>(numRequests);*/
    completeRequests_.reset();
    extendCheck_.reset();
    this->numCompleted_ = 0;
    numExtendCheck_ = 0;
//    currentNode_ = &source;
    nbPickUp_ = 0;
    nbPickMove_ = 0;
//    extendCheck_.insert(source->nodeID_);
//    parent_ = nullptr;
    isDropped_ = false;
    isDropExtend_ = false;
    createTime_ = 0;
}

Label::Label(const Label &label) :labelID_(labelCount_++) {
    char* name2 = new char[255];
    strncpy(name2, std::to_string(labelID_).c_str(), 255);
    name_ = name2;
    status_ = ACTIVE;
    load_ = label.load_;
    passedTime_ = label.passedTime_;
    reachedTime_ = label.reachedTime_;
//    vehicle_ = label.vehicle_;
//    openReachTime_ = label.openReachTime_;
//    travelResource_ = label.travelResource_;
    travelResources_ = label.travelResources_;
//    pathNodes_ = label.pathNodes_;
    pathNode_ = label.pathNode_;
 //   path_ = label.path_;
    reducedCost_ = label.reducedCost_;
//    currentNode_ = label.currentNode_;
    totalDelay_ = label.totalDelay_;
//    openNodes_ = label.openNodes_;
    openNode_ = label.openNode_;
    openRequests_ = label.openRequests_;
//    completedRequests_ = label.completedRequests_;

    /*completeRequests_ = std::make_shared<myTools::BitVector>(*label.completeRequests_);
    extendCheck_ = std::make_shared<myTools::BitVector>(*label.completeRequests_);*/
    completeRequests_ = label.completeRequests_;
    extendCheck_ = label.completeRequests_;
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
    isDropped_ = label.isDropped_;
    isDropExtend_ = false;
    createTime_ = 0;
}
void Label::copyLabel(const Label &label) {
    status_ = ACTIVE;
    load_ = label.load_;
    passedTime_ = label.passedTime_;
    reachedTime_ = label.reachedTime_;
//    vehicle_ = label.vehicle_;
//    parent_ = nullptr;
    travelResources_ = label.travelResources_;
//    pathNodes_ = label.pathNodes_;
    pathNode_ = label.pathNode_;
 //   path_ = label.path_;
    reducedCost_ = label.reducedCost_;
//    currentNode_ = label.currentNode_;
    totalDelay_ = label.totalDelay_;
    openNode_ = label.openNode_;
    openRequests_ = label.openRequests_;
//    completedRequests_ = label.completedRequests_;
    /*completeRequests_.reset();
    extendCheck_.reset();
    completeRequests_ = std::make_shared<myTools::BitVector>(*label.completeRequests_);
    extendCheck_ = std::make_shared<myTools::BitVector>(*label.completeRequests_);*/
    /*completeRequests_->copyValues(*label.completeRequests_);
    extendCheck_->copyValues(*label.completeRequests_);*/
    completeRequests_ = label.completeRequests_;
    extendCheck_ = label.completeRequests_;
    numCompleted_ = label.numCompleted_;
    numExtendCheck_ = label.numCompleted_;
//    extendCheck_ = label.completedRequests_;
//    if ((*currentNode_)->type_ == PICKUP)
//        extendCheck_[(*currentNode_)->related_Request_->taskIndexLabel_] = 1;
    nbPickUp_ = label.nbPickUp_;
    nbPickMove_ = label.nbPickMove_;
    isDropped_ = label.isDropped_;
    isDropExtend_ = false;
}
Label::~Label() {
//    openNode_.clear();
//    pathNodes_.clear();
    delete[] name_;
}

unsigned int Label::getLabelId() const {
    return labelID_;
}

bool Label::operator () (const Label &rhs) const {
    return reducedCost_ < rhs.reducedCost_;
}

void Label::extend(Node *outNode) {
    load_ += outNode->nbPassengers_;
    float travelTime =  durationMatrix_[pathNode_.back()->locationID_][outNode->locationID_];
    reachedTime_ = passedTime_+travelTime;
    for (auto &node: openNode_) {
        travelResources_[(node)->related_Request_->taskIndexLabel_] -= (reachedTime_ + outNode->serviceTime_ - passedTime_);
    }
    if (outNode->type_ == DROPOFF) {
        travelResources_[outNode->related_Request_->taskIndexLabel_] = 0;
        for (int i = 0; i < openNode_.size(); i++){
            if ((openNode_[i])->nodeID_ == outNode->nodeID_){
                openNode_.erase(openNode_.begin()+i);
                break;
            }
        }
        openRequests_[outNode->related_Request_->taskIndexLabel_] = 0;
        if ((!isDropped_ && travelTime >0) || (travelTime == 0 && ServiceTime > 0))
            isDropped_ = true;

    }
    else if (outNode->type_ == PICKUP){
        openNode_.push_back(outNode->pairNode_);
//        completeRequests_->add(outNode->related_Request_->taskIndexLabel_);
        completeRequests_.set(outNode->related_Request_->taskIndexLabel_, true);
        numCompleted_++;
        extendCheck_.set(outNode->related_Request_->taskIndexLabel_, true);
//        extendCheck_->add(outNode->related_Request_->taskIndexLabel_);
        numExtendCheck_++;
        openRequests_[outNode->related_Request_->taskIndexLabel_] = 1;
        reducedCost_ -= (outNode->related_Request_)->dual_;
        if (travelTime > 0){
            nbPickMove_++;
        }
        nbPickUp_ ++;
        totalDelay_ += (reachedTime_ - outNode->requestTime_);
        reducedCost_ += (reachedTime_ - outNode->requestTime_);
        travelResources_[outNode->related_Request_->taskIndexLabel_] = outNode->related_Request_->maxTravelTime_;
    }
    /*else if (outNode->type_ == SINK){
        passedTime_ = reachTime;
    }*/


    passedTime_ = reachedTime_ + outNode->serviceTime_;

//    pathNodes_.push_back(outNode->nodeID_);
    pathNode_.push_back(outNode);
 //   path_.push_back(&outNode);
//    currentNode_ = &outNode;
//    extendCheck_.insert(outNode->nodeID_);
}

void Label::extend1(Node *outNode) {
    load_ += outNode->nbPassengers_;
    float travelTime =  durationMatrix_[pathNode_.back()->locationID_][outNode->locationID_];
    if ((travelTime > 0)||(pathNode_.back()->initialType_==SOURCE)){
        reachedTime_ = passedTime_ + travelTime;
        for (auto &node: openNode_) {
            travelResources_[(node)->related_Request_->taskIndexLabel_] -= (reachedTime_ + outNode->serviceTime_ -
                                                                            passedTime_);
        }
    }
    else
        reachedTime_ = passedTime_;

    if (outNode->type_ == DROPOFF) {
        travelResources_[outNode->related_Request_->taskIndexLabel_] = 0;
        for (int i = 0; i < openNode_.size(); i++){
            if ((openNode_[i])->nodeID_ == outNode->nodeID_){
                openNode_.erase(openNode_.begin()+i);
                break;
            }
        }
        openRequests_[outNode->related_Request_->taskIndexLabel_] = 0;
        if (!isDropped_ && travelTime > 0)
            isDropped_ = true;

    }
    else if (outNode->type_ == PICKUP){
        openNode_.push_back(outNode->pairNode_);
 //       completeRequests_->add(outNode->related_Request_->taskIndexLabel_);
        completeRequests_.set(outNode->related_Request_->taskIndexLabel_, true);
        numCompleted_++;
//        extendCheck_->add(outNode->related_Request_->taskIndexLabel_);
        extendCheck_.set(outNode->related_Request_->taskIndexLabel_, true);
        numExtendCheck_++;
        openRequests_[outNode->related_Request_->taskIndexLabel_] = 1;
        reducedCost_ -= (outNode->related_Request_)->dual_;
        if (travelTime > 0){
            nbPickMove_++;
        }
        nbPickUp_ ++;
        totalDelay_ += (reachedTime_ - outNode->requestTime_);
        reducedCost_ += (reachedTime_ - outNode->requestTime_);
        travelResources_[outNode->related_Request_->taskIndexLabel_] = outNode->related_Request_->maxTravelTime_;
    }
    if ((travelTime > 0)||(pathNode_.back()->initialType_==SOURCE))
        passedTime_ = reachedTime_ + outNode->serviceTime_;

//    pathNodes_.push_back(outNode->nodeID_);
    pathNode_.push_back(outNode);
//    currentNode_ = &outNode;
}

// this function check the feasibility of the label before extension
bool Label::isExtendFeasible(Node *outNode, int maxPickUp, bool usePick, int capacity) {
    if (outNode->type_ == PICKUP){
//        extendCheck_->add(outNode->related_Request_->taskIndexLabel_);
        extendCheck_.set(outNode->related_Request_->taskIndexLabel_, true);
        numExtendCheck_++;
    }
    if ((load_ + outNode->nbPassengers_) > capacity)
        return false;
    if (outNode->type_ == PICKUP) {
        if (usePick){
            if (nbPickUp_ >= maxPickUp)
                return false;
        }
        else{
            if (nbPickMove_ >= maxPickUp && pathNode_.back()->locationID_ != outNode->locationID_)
                return false;
        }

        if (completeRequests_.test(outNode->related_Request_->taskIndexLabel_))
            return false;
    }
    if (outNode->type_ == DROPOFF) {
        if (openRequests_[outNode->related_Request_->taskIndexLabel_] == 0)
            return false;
    }
    if (outNode->type_ == SINK) {
        if (!openNode_.empty())
            return false;
    }
    for (auto &nodeObj: openNode_) {
        float travelToDrop =
                durationMatrix_[pathNode_.back()->locationID_][outNode->locationID_] +
                outNode->serviceTime_ + durationMatrix_[outNode->locationID_][(nodeObj)->locationID_];

        if (travelResources_[(nodeObj)->related_Request_->taskIndexLabel_] < travelToDrop)
            return false;
    }
    /*if ((*currentNode_)->locationID_ != outNode->locationID_) {
        for (auto &nodeObj: openNode_) {
            float travelToDrop =
                    (*currentNode_)->serviceTime_ + durationMatrix_[(*currentNode_)->locationID_][outNode->locationID_] +
                    outNode->serviceTime_ + durationMatrix_[outNode->locationID_][(*nodeObj)->locationID_];

            if (travelResources_[(*nodeObj)->related_Request_->taskIndexLabel_] < travelToDrop)
                return false;
        }
    }
    else{
        for (auto &nodeObj: openNode_) {
            float travelToDrop = outNode->serviceTime_ +
                    durationMatrix_[outNode->locationID_][(*nodeObj)->locationID_];
            if (travelResources_[(*nodeObj)->related_Request_->taskIndexLabel_] < travelToDrop)
                return false;
        }
    }*/

    /*for (auto & nodeObj: openNodes_) {
        float travelToDrop = currentNode_->serviceTime_ + durationMatrix_[currentNode_->locationID_][outNode->locationID_] +
                outNode->serviceTime_ + durationMatrix_[outNode->locationID_][nodeObj->locationID_];
        *//*if (travelResource_[nodeObj->nodeID_] <  travelToDrop)
            return false;*//*
        if (travelResources_[nodeObj->related_Request_->taskIndexLabel_] <  travelToDrop)
            return false;
    }*/




    /*for (auto & nodeObj : openNodes_) {
        float travelDuration = reachTime - openReachTime_[nodeObj->nodeID_] + outNode->serviceTime_ +
                travelMat->queryTravelTime(outNode, *nodeObj->pairNode_);
        if (travelDuration > (*nodeObj->related_Request_)->maxTravelTime_){
            return false;
        }

    }*/
    return true;
}

bool Label::isExtendFeasible1(Node *outNode, int maxPickUp, bool usePick, int capacity) {
    if (outNode->type_ == PICKUP){
        extendCheck_.set(outNode->related_Request_->taskIndexLabel_, true);
        numExtendCheck_++;
    }
    if ((load_ + outNode->nbPassengers_) > capacity)
        return false;
    if (outNode->type_ == PICKUP) {
        if (usePick){
            if (nbPickUp_ >= maxPickUp)
                return false;
        }
        else{
            if (nbPickMove_ >= maxPickUp && durationMatrix_[pathNode_.back()->locationID_][outNode->locationID_] != 0)
                return false;
        }
        if (completeRequests_.test(outNode->related_Request_->taskIndexLabel_))
            return false;
    }
    if (outNode->type_ == DROPOFF) {
        if (openRequests_[outNode->related_Request_->taskIndexLabel_] == 0)
            return false;
    }
    if (outNode->type_ == SINK) {
        if (!openNode_.empty())
            return false;
    }
    float travelTime = durationMatrix_[pathNode_.back()->locationID_][outNode->locationID_];
    if (travelTime > 0){
        for (auto &nodeObj: openNode_) {
            float travelToDrop =
                    durationMatrix_[pathNode_.back()->locationID_][outNode->locationID_] +
                    outNode->serviceTime_ + durationMatrix_[outNode->locationID_][(nodeObj)->locationID_];

            if (travelResources_[(nodeObj)->related_Request_->taskIndexLabel_] < travelToDrop)
                return false;
        }
    }
    return true;
}

bool Label::isDominated(PLabel &otherLabel, PSolverOption &solverOption) const {
    if (pathNode_.back() != otherLabel->pathNode_.back())
        myTools::throwException("Label Domination error!!");

    if (pathNode_.back()->type_ == SINK){
        if (this->passedTime_ >= otherLabel->passedTime_) {
            if (this->reducedCost_ >= otherLabel->reducedCost_) {
                if (this->numCompleted_ >= otherLabel->numCompleted_) {
                    if (solverOption->isDominanceReleased_) {
                        return true;
                    }
                    else {
                        if ((otherLabel->completeRequests_ & this->completeRequests_) == otherLabel->completeRequests_)
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
                            if ((otherLabel->completeRequests_ & this->completeRequests_) == otherLabel->completeRequests_){
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

    for (auto & nodeObj: openNode_) {
        if (travelResources_[(nodeObj)->related_Request_->taskIndexLabel_] < durationMatrix_[pathNode_.back()->locationID_][(nodeObj)->locationID_])
            return true;
    }
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
    newRoute->addSource(pathNodes_[0], vehicle->departureTime_, vehicle->numPassengers_);
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
        if (pathNode_[i]->type_ == PICKUP)
            newRoute->addNode1(pInst->instGraph_->pickNodes_[pathNode_[i]->related_Request_->getRequestId()]);
        else
            newRoute->addNode1(pInst->instGraph_->dropNodes_[pathNode_[i]->related_Request_->getRequestId()]);
    }
    if (newRoute->routeRequests_.empty() && !pInst->vehicles_[newRoute->vehicleID_]->onboards_.empty())
        pInst->vehicles_[newRoute->vehicleID_]->emptyRoute_ = newRoute;
    newRoute->createTime_ = createTime_;
    if (totalDelay_ != newRoute->totalDelay_) {
        std::cout << "Total delay of the label partial path is not the same as the route delay" << std::endl;
        std::cout << "label: " << std::endl;
        std::cout << toString();
        std::cout << "route: " << std::endl;
        std::cout << newRoute->toString();
//        myTools::throwException("Label convert problem");
    }
    return newRoute;
}

std::string Label::toString() const {
    std::stringstream repStr;

    repStr << "#" << std::left << std::endl;
    repStr << "#\t" << std::setw(24) << "- LABEL INFO" << " : " << std::endl;
    repStr << "# \t" <<"_____________________" << std::endl;
    repStr << "#\t" << std::setw(24) << "- LABEL_NUMBER" << " : " << labelID_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- CURRENT_NODE" << " : " << pathNode_.back()->nodeID_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- PASSED_TIME (seconds)" << " : " << passedTime_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- NUMBER_OF_STOPS" << " : " << pathNode_.size() << std::endl;
    repStr << "#\t" << std::setw(24) << "- TOTAL_WAITING (seconds)" << " : " << totalDelay_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- REDUCED_COST" << " : " << reducedCost_ << std::endl;
    repStr << "#" << std::endl;
    if (!openNode_.empty()) {
        repStr << "#\t" << std::setw(24) << "- OPEN_REQUESTS" << " : ";
        for (auto &nodeObj: openNode_) {
            repStr << (nodeObj)->related_Request_->getRequestId() << "  ";
        }
    }
    repStr << "#\t" << std::setw(24) << "- PATH_NODES" << " : ";
    for (auto &nodeObj: pathNode_) {
        repStr << (nodeObj)->nodeID_ << "  ";
    }
    repStr << std::endl;

    repStr << std::endl;
    repStr << "# ________________________________________________________________________" << std::endl;
    return repStr.str();
}





