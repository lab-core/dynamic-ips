//
// Created by Ella on 2/22/2022.
//

#include "Label.h"

unsigned int Label::labelCount_ = 0;

Label::Label(const Vehicle *vehicle, PNode &source, int labelSize) : labelID_(labelCount_++) {
    char* name2 = new char[255];
    strncpy(name2, std::to_string(labelID_).c_str(), 255);
    name_ = name2;

    load_ = vehicle->numPassengers_;
    passedTime_ = vehicle->departTime_;
    reachedTime_ = vehicle->departTime_;
    pathNode_.push_back(&(*source));
    reducedCost_ = 0;
    labelScore_ = 0;
    lambdaScore_ = 0;
    totalDelay_ = 0;
    status_ = ACTIVE;
    openNode_.clear();
    nbCommitted_ = 0;
    openRequests_.reset();
    openRequests_.resize(labelSize);
    completeRequests_.reset();
    completeRequests_.resize(labelSize);
    extendCheck_.reset();
    extendCheck_.resize(labelSize);
//    prunedDirections_.reset();
    this->numCompleted_ = 0;
    numExtendCheck_ = 0;
    nbPickUp_ = 0;
    //   nbPickMove_ = 0;
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
    travelResources_ = label.travelResources_;
    pathNode_ = label.pathNode_;
    reducedCost_ = label.reducedCost_;
    labelScore_ = label.labelScore_;
    lambdaScore_ = label.lambdaScore_;
    totalDelay_ = label.totalDelay_;
    openNode_ = label.openNode_;
    openRequests_ = label.openRequests_;
    nbCommitted_ = label.nbCommitted_;

    completeRequests_ = label.completeRequests_;
//    prunedDirections_ = label.prunedDirections_;
    extendCheck_ = label.completeRequests_;
    numCompleted_ = label.numCompleted_;
    numExtendCheck_ = label.numCompleted_;

    nbPickUp_ = label.nbPickUp_;
    //   nbPickMove_ = label.nbPickMove_;

    isDropped_ = label.isDropped_;
    isDropExtend_ = false;
    createTime_ = 0;
}
void Label::copyLabel(const Vehicle *vehicle, PNode &source, int labelSize) {
    status_ = ACTIVE;
    load_ = vehicle->numPassengers_;
    passedTime_ = vehicle->departTime_;
    reachedTime_ = vehicle->departTime_;
    travelResources_.clear();
    pathNode_.clear();
    pathNode_.push_back(&(*source));
    reducedCost_ = 0;
    labelScore_ = 0;
    lambdaScore_ = 0;
    totalDelay_ = 0;
    openNode_.clear();
    openRequests_.reset();
    openRequests_.resize(labelSize);
    nbCommitted_ = 0;

    completeRequests_.reset();
    completeRequests_.resize(labelSize);
    extendCheck_.reset();
    extendCheck_.resize(labelSize);
//    prunedDirections_.reset();
    this->numCompleted_ = 0;
    numExtendCheck_ = 0;
    nbPickUp_ = 0;
    //   nbPickMove_ = 0;
    isDropped_ = false;
    isDropExtend_ = false;
    createTime_ = 0;
}

void Label::copyLabel(const Label &label) {
    status_ = ACTIVE;
    load_ = label.load_;
    passedTime_ = label.passedTime_;
    reachedTime_ = label.reachedTime_;
    travelResources_ = label.travelResources_;
    pathNode_ = label.pathNode_;
    reducedCost_ = label.reducedCost_;
    labelScore_ = label.labelScore_;
    lambdaScore_ = label.lambdaScore_;
    totalDelay_ = label.totalDelay_;
    openNode_ = label.openNode_;
    openRequests_.reset();
    openRequests_ = label.openRequests_;
    nbCommitted_ = label.nbCommitted_;
    completeRequests_.reset();
    completeRequests_ = label.completeRequests_;
 //   prunedDirections_ = label.prunedDirections_;
    extendCheck_.reset();
    extendCheck_ = label.completeRequests_;
    numCompleted_ = label.numCompleted_;
    numExtendCheck_ = label.numCompleted_;

    nbPickUp_ = label.nbPickUp_;
    //   nbPickMove_ = label.nbPickMove_;
    isDropped_ = label.isDropped_;
    isDropExtend_ = false;
}
Label::~Label() {
    delete[] name_;
}

unsigned int Label::getLabelId() const {
    return labelID_;
}

bool Label::operator () (const Label &rhs) const {
    return reducedCost_ < rhs.reducedCost_;
}

bool Label::checkSubsetOpen(const PLabel &otherLabel) const {
    for (auto & node : otherLabel->openNode_) {
        if (!this->openRequests_.test(node->related_Request_->taskIndexLabel_))
            return false;
    }
    return true;
}

bool Label::checkSubsetComplete(const PLabel &otherLabel) const {
    for (auto & node : otherLabel->pathNode_) {
        if (node->type_ != SINK && node->type_ != SOURCE && node->nodeStatus_ != PLANNED) {
            if (!this->completeRequests_.test(node->related_Request_->taskIndexLabel_))
                return false;
        }
    }
    return true;
}

void Label::extend(Node *outNode, bool isDropPickPossible) {
    load_ += outNode->nbPassengers_;
    float travelTime =  durationMatrix_[pathNode_.back()->locationID_][outNode->locationID_];
    if (outNode->type_ == PICKUP)
        reachedTime_ = std::max(outNode->related_Request_->earlyPick_,
                                reachedTime_ = std::max(outNode->related_Request_->requestTime_, passedTime_) + travelTime);
    else
        reachedTime_ = passedTime_ + travelTime;

    for (auto &node: openNode_) {
        travelResources_[(node)->related_Request_->taskIndexLabel_] -= (reachedTime_ + outNode->serviceTime_ -
                                                                        passedTime_);
    }
    if (outNode->type_ == DROPOFF) {
        travelResources_[outNode->related_Request_->taskIndexLabel_] = 0;
        for (int i = 0; i < openNode_.size(); i++){
            if ((openNode_[i])->nodeID_ == outNode->nodeID_){
                openNode_.erase(openNode_.begin()+i);
                break;
            }
        }
        openRequests_.set(outNode->related_Request_->taskIndexLabel_, false);
        isDropped_ = true;
    }
    else if (outNode->type_ == PICKUP){
        openNode_.push_back(outNode->pairNode_);
        completeRequests_.set(outNode->related_Request_->taskIndexLabel_, true);
        numCompleted_++;
        extendCheck_.set(outNode->related_Request_->taskIndexLabel_, true);
        numExtendCheck_++;
        openRequests_.set(outNode->related_Request_->taskIndexLabel_, true);
        reducedCost_ -= (outNode->related_Request_)->dual_;
        if (outNode->related_Request_->committedPickTime_ != LARGE_CONSTANT)
            nbCommitted_++;
        /*if (travelTime > 0){
            nbPickUp_++;
        }*/
        nbPickUp_ ++;
        totalDelay_ += (reachedTime_ - outNode->initialReadyTime_);
        reducedCost_ += (reachedTime_ - outNode->initialReadyTime_);
        travelResources_[outNode->related_Request_->taskIndexLabel_] = outNode->related_Request_->maxTravelTime_;
        labelScore_ = reducedCost_ / static_cast<float>(nbPickUp_);
        lambdaScore_ = totalDelay_ / (totalDelay_ - reducedCost_);
    }
    passedTime_ = reachedTime_ + outNode->serviceTime_;
    pathNode_.push_back(outNode);
}

// this function checks the feasibility of the label before extension
bool Label::isExtendFeasible(const Node *outNode, int maxPickUp, bool discardSuboptimalPath, int capacity,
                             int &nbPrunedPath, int &nbEliminated, int &nbPrunedArcs) {
    if (extendCheck_.test(outNode->related_Request_->taskIndexLabel_))
        return false;
    if (outNode->type_ == PICKUP){
        extendCheck_.set(outNode->related_Request_->taskIndexLabel_, true);
        numExtendCheck_++;
    }

    // check the arrival time window
    /*if (prunedDirections_.test(outNode->related_Request_->taskIndexLabel_)){
        nbPrunedPath ++;
        return false;
    }*/
    // check tha capacity of vehicle
    if ((load_ + outNode->nbPassengers_) > capacity)
        return false;

    if (outNode->type_ == PICKUP) {
        if (nbPickUp_ >= maxPickUp)
            return false;

        if (pathNode_.back()->prunedArcs_.test(outNode->related_Request_->taskIndexLabel_)) {
            nbPrunedArcs++;
            return false;
        }

        float timeToReach = std::max(outNode->related_Request_->earlyPick_,
                                     std::max(outNode->related_Request_->requestTime_, passedTime_) + durationMatrix_[
                                         pathNode_.back()->locationID_][outNode->locationID_]);
        if (discardSuboptimalPath) {
            if (timeToReach > outNode->related_Request_->latestPickup_) {
 //               prunedDirections_.set(outNode->related_Request_->taskIndexLabel_, true);
                nbPrunedPath ++;
                return false;
            }
        }
        if (completeRequests_.test(outNode->related_Request_->taskIndexLabel_))
            return false;
    }
    if (outNode->type_ == DROPOFF) {
        if (!openRequests_.test(outNode->related_Request_->taskIndexLabel_))
            return false;
    }
    if (outNode->type_ == SINK) {
        if (!openNode_.empty())
            return false;
    }
    // check travel time limitation
    if (!isTravelTimeFeasible(outNode, nbEliminated))
        return false;
    /*for (auto &nodeObj: openNode_) {
        float travelToDrop =
                durationMatrix_[pathNode_.back()->locationID_][outNode->locationID_] +
                outNode->serviceTime_ + durationMatrix_[outNode->locationID_][(nodeObj)->locationID_];

        if (travelResources_[(nodeObj)->related_Request_->taskIndexLabel_] < travelToDrop) {
            nbEliminated ++;
            return false;
        }
    }*/
    return true;
}

bool Label::isTravelTimeFeasible(const Node *outNode, int &nbEliminated) const {
    float timeToReach;
    if (outNode->type_ == PICKUP)
        timeToReach = std::max(outNode->related_Request_->earlyPick_,
                               std::max(outNode->related_Request_->requestTime_, passedTime_) + durationMatrix_[pathNode_.back()->locationID_][outNode->locationID_]);
    else
        timeToReach = passedTime_ + durationMatrix_[pathNode_.back()->locationID_][outNode->locationID_];


    // check travel time limitation
    for (auto &nodeObj: openNode_) {
        if (nodeObj->nodeID_ != outNode->nodeID_) {
            float timeToDrop = timeToReach - passedTime_ + outNode->serviceTime_ +
                               durationMatrix_[outNode->locationID_][(nodeObj)->locationID_];

            if (travelResources_[(nodeObj)->related_Request_->taskIndexLabel_] < timeToDrop) {
                nbEliminated++;
                return false;
            }
        }
    }
    return true;
}

bool Label::isDominated(const PLabel &otherLabel, const PSolverOption &solverOption) const {
    /*if (pathNode_.back() != otherLabel->pathNode_.back())
        throw myTools::myException("Label Domination error!!", __FILE__, __LINE__);*/

    if (this->passedTime_ >= otherLabel->passedTime_) {
        if (this->reducedCost_ >= otherLabel->reducedCost_) {
            if (this->numCompleted_ >= otherLabel->numCompleted_) {
                if (checkSubsetOpen(otherLabel)){
                    if (checkSubsetComplete(otherLabel)){
                        if (solverOption->isDominanceReleased_ || this->haveLessTravelResource(otherLabel))
                            return true;
                    }
                }
            }
        }
    }
    return false;
}
// this function examines the label to be sure that it leads to a route with negative reduced cost
bool Label::isEliminated() const {
 //   return false;
    for (auto & nodeObj: openNode_) {
        if (travelResources_[(nodeObj)->related_Request_->taskIndexLabel_] < durationMatrix_[pathNode_.back()->locationID_][(nodeObj)->locationID_]) {
            return true;
        }
    }
    return false;
}


PRoute Label::labelToRoute(const PVehicle &vehicle, const PInstance &pInst) const {
    PRoute newRoute = std::make_shared<Route>(vehicle->vehicleID_);
    newRoute->totalLength_ = passedTime_ - vehicle->departTime_;
    newRoute->reducedCost_ = reducedCost_ - vehicle->dual_;
    newRoute->addSource(vehicle->departNode_, vehicle->departTime_, vehicle->numPassengers_);

    for (int i = 1; i < pathNode_.size()-1; ++i) {
        if (pathNode_[i]->type_ == PICKUP)
            newRoute->addNode(pInst->instGraph_->pickNodes_[pathNode_[i]->related_Request_->getRequestId()]);
        else
            newRoute->addNode(pInst->instGraph_->dropNodes_[pathNode_[i]->related_Request_->getRequestId()]);
    }
    if (newRoute->routeRequests_.empty() && !pInst->vehicles_[newRoute->vehicleID_]->onboards_.empty())
        pInst->vehicles_[newRoute->vehicleID_]->emptyRoute_ = newRoute;
    if (!newRoute->routeRequests_.empty()) {
        newRoute->score_ = reducedCost_ / static_cast<float>(newRoute->routeRequests_.size());
        newRoute->waitScore_ = newRoute->totalDelay_ / static_cast<float>(newRoute->routeRequests_.size());
        newRoute->lambda_ = newRoute->totalDelay_ /(newRoute->totalDelay_ - reducedCost_ + 0.0001f);

    }
    else {
        newRoute->score_ = 0;
        newRoute->lambda_ = 0;
        newRoute->waitScore_ = 0;
    }
    newRoute->createTime_ = createTime_;
    if (totalDelay_ != newRoute->totalDelay_) {
        std::cout << "Total delay of the label partial path is not the same as the route delay" << std::endl;
        std::cout << "label: " << std::endl;
        std::cout << toString();
        std::cout << "route: " << std::endl;
        std::cout << newRoute->toString();
        throw myTools::myException("Label convert problem!!!", __FILE__,__LINE__);
    }
    return newRoute;
}

std::string Label::toString() const {
    std::stringstream repStr;

    repStr << "#" << std::left << std::endl;
//    repStr << "#\t" << std::setw(24) << "- LABEL INFO" << " : " << std::endl;
//    repStr << "# \t" <<"_____________________" << std::endl;
    repStr << "#\t" << std::setw(24) << "- LABEL_NUMBER" << " : " << labelID_ << std::endl;
//    repStr << "#\t" << std::setw(24) << "- CURRENT_NODE" << " : " << pathNode_.back()->nodeID_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- PASSED_TIME (seconds)" << " : " << passedTime_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- NUMBER_OF_STOPS" << " : " << pathNode_.size() << std::endl;
    repStr << "#\t" << std::setw(24) << "- TOTAL_WAITING (seconds)" << " : " << totalDelay_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- REDUCED_COST" << " : " << reducedCost_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- STATUS" << " : " << status_ << std::endl;
    if (!openNode_.empty()) {
        repStr << "#\t" << std::setw(24) << "- OPEN_REQUESTS" << " : ";
        for (auto &nodeObj: openNode_) {
            repStr << (nodeObj)->related_Request_->getRequestId() << "  ";
        }
        repStr << std::endl;
    }
    repStr << "#\t" << std::setw(24) << "- PATH_NODES" << " : ";
    for (auto &nodeObj: pathNode_) {
        repStr << (nodeObj)->nodeID_ << "  ";
    }
    repStr << std::endl;
    repStr << "# ________________________________________________________________________" << std::endl;
    return repStr.str();
}

bool Label::haveLessTravelResource(const PLabel &otherLabel) const {
    // Compare each element
    for (auto &nodeObj: otherLabel->openNode_) {
        for (size_t i = 0; i < travelResources_.size(); ++i) {
            if (travelResources_[(nodeObj)->related_Request_->taskIndexLabel_] > otherLabel->travelResources_[(nodeObj)->related_Request_->taskIndexLabel_]) {
                return false;
            }
        }
    }
    return true;
}
