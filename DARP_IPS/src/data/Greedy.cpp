//
// Created by Ella on 5/20/2022.
//

#include "Greedy.h"

#include <utility>

StopLabel::StopLabel(PNode currentNode, float reachTime, float departTime, int nbPassengers) :
        currentNode_(std::move(currentNode)), reachTime_(reachTime), leaveTime_(departTime),
        nbPassengers_(nbPassengers) {
    parent_ = nullptr;
    child_ = nullptr;
    pair_ = nullptr;
    travelResource_ = 0;
}

void StopLabel::setValues(PNode currentNode, float reachTime, float departTime, int nbPassengers) {
    currentNode_ = currentNode;
    reachTime_ = reachTime;
    nbPassengers_ = nbPassengers;
    parent_ = nullptr;
    child_ = nullptr;
    pair_ = nullptr;
    travelResource_ = 0;
    leaveTime_ = departTime;
}


GreedyRoute::GreedyRoute(PVehicle &vehicle, PInstance &pInst) : Vehicle_(&vehicle)  {
    PInitialStop_ = std::make_shared<StopLabel>((*Vehicle_)->departNode_,
                                                (*Vehicle_)->departNode_->reachTime_,
                                                vehicle->departTime_, vehicle->numPassengers_);
    idle_ = true;
    PCurrentStop_ = PInitialStop_;
    PLastStop_ = PInitialStop_;
    totalDelay_ = 0;
    idleTime_ = 0;
    departureTime_ = vehicle->departTime_;
    selected_ = false;
    for (auto &nodeID: (*Vehicle_)->onboards_) {
        idle_ = false;
        PNode onboardNode = pInst->instGraph_->nodes_[nodeID];
        float dropTime = labelToNodeReachTime(PLastStop_, onboardNode);
        PStopLabel newDropLabel = std::make_shared<StopLabel>(onboardNode, dropTime,
                                                                  dropTime+onboardNode->serviceTime_,
                                                                  PLastStop_->nbPassengers_ +
                                                                  onboardNode->nbPassengers_);
        newDropLabel->parent_ = PLastStop_;
        PLastStop_->child_ = newDropLabel;
        newDropLabel->travelResource_ = onboardNode->related_Request_->maxTravelTime_ - dropTime +
                onboardNode->pairNode_->departTime_;

        PLastStop_ = newDropLabel;
    }
}

GreedyRoute::GreedyRoute(PVehicle &vehicle, PInstance &pInst, std::vector<PStopLabel> &greedyLabelPool, bool greedyReOptimize) :
    Vehicle_(&vehicle) {
    idle_ = true;
    if (greedyLabelPool.empty())
        PInitialStop_ = std::make_shared<StopLabel>((*Vehicle_)->departNode_,
                                                    (*Vehicle_)->departNode_->reachTime_,
                                                    vehicle->departTime_,
                                                    vehicle->numPassengers_);
    else {
        PInitialStop_ = greedyLabelPool.back();
        greedyLabelPool.pop_back();
        PInitialStop_->setValues((*Vehicle_)->departNode_, (*Vehicle_)->departNode_->reachTime_,
                                 vehicle->departTime_, vehicle->numPassengers_);
    }
    selected_ = false;
    PCurrentStop_ = PInitialStop_;
    PLastStop_ = PInitialStop_;
    totalDelay_ = 0;
    idleTime_ = 0;
    departureTime_ = vehicle->departTime_;
    // just onboards are added and previous solution is not considered
    if (greedyReOptimize) {
        for (auto &nodeID: (*Vehicle_)->onboards_) {
            idle_ = false;
            PNode onboardNode = pInst->instGraph_->nodes_[nodeID];
            float dropTime = labelToNodeReachTime(PLastStop_, onboardNode);
            PStopLabel newDropLabel;
            if (greedyLabelPool.empty()) {
                newDropLabel = std::make_shared<StopLabel>(onboardNode, dropTime,
                                                           dropTime + onboardNode->serviceTime_,
                                                           PLastStop_->nbPassengers_ + onboardNode->nbPassengers_);
            } else {
                newDropLabel = greedyLabelPool.back();
                greedyLabelPool.pop_back();
                newDropLabel->setValues(onboardNode, dropTime,
                                        dropTime + onboardNode->serviceTime_, PLastStop_->nbPassengers_ +
                                                                              onboardNode->nbPassengers_);
            }
            newDropLabel->parent_ = PLastStop_;
            PLastStop_->child_ = newDropLabel;
            newDropLabel->travelResource_ = onboardNode->related_Request_->maxTravelTime_ - dropTime +
                                            onboardNode->pairNode_->departTime_;

            PLastStop_ = newDropLabel;
        }
    }
    // build previous solution
    else {
        for (int i = 1; i < (*Vehicle_)->currentRoute_->routeNodes_.size(); i++) {
            idle_ = false;
            PStopLabel newLabel = std::make_shared<StopLabel>((*Vehicle_)->currentRoute_->routeNodes_[i],
                                                              (*Vehicle_)->currentRoute_->plannedReachTime_[i],
                                                              (*Vehicle_)->currentRoute_->plannedDepartTime_[i],
                                                              (*Vehicle_)->currentRoute_->plannedPassengers_[i]);
            newLabel->parent_ = PLastStop_;
            PLastStop_->child_ = newLabel;
            PLastStop_ = newLabel;
            if (newLabel->currentNode_->nodeStatus_ == COMMITTED) {
                PCurrentStop_ = newLabel;
                departureTime_ = newLabel->leaveTime_;
            }
            if (newLabel->currentNode_->type_ == DROPOFF) {
                if (newLabel->currentNode_->related_Request_->requestStatus_ == NO_ACTION) {
                    PStopLabel currentLabel = PLastStop_;
                    while (currentLabel->parent_ != nullptr) {
                        if (currentLabel->parent_->currentNode_->type_ == PICKUP &&
                            currentLabel->parent_->currentNode_->related_Request_->getRequestId() ==
                            newLabel->currentNode_->related_Request_->getRequestId()) {
                            newLabel->pair_ = currentLabel->parent_;
                            currentLabel->parent_->pair_ = newLabel;
                            newLabel->travelResource_ = newLabel->currentNode_->related_Request_->maxTravelTime_ -
                                    (*Vehicle_)->currentRoute_->plannedReachTime_[i] +
                                    currentLabel->parent_->leaveTime_;
                            break;
                        }
                        currentLabel = currentLabel->parent_;
                    }
                } else {
                    newLabel->travelResource_ = newLabel->currentNode_->related_Request_->maxTravelTime_ -
                            (*Vehicle_)->currentRoute_->plannedReachTime_[i] +
                            (*Vehicle_)->currentRoute_->routeNodes_[i]->pairNode_->departTime_;
                }
            }
            else
                totalDelay_ += (newLabel->reachTime_ - newLabel->currentNode_->requestTime_);

        }
    }
}

GreedyRoute::GreedyRoute(const GreedyRoute &label) {
    selected_ = label.selected_;
    Vehicle_ = label.Vehicle_;
    departureTime_ = label.departureTime_;
    idleTime_ = label.idleTime_;
    PCurrentStop_ = label.PCurrentStop_;
    PLastStop_ = label.PLastStop_;
    PInitialStop_ = label.PInitialStop_;
    totalDelay_ = label.totalDelay_;
    idle_ = label.idle_;
}

GreedyRoute::~GreedyRoute() {
    /*PStopLabel currentLabel = PInitialStop_;
    while (currentLabel->child_ != nullptr) {
        currentLabel->pair_.reset();
        currentLabel->parent_.reset();
        currentLabel = currentLabel->child_;
    }
    currentLabel = PInitialStop_;
    while (currentLabel->child_ != nullptr) {
        PStopLabel nextLabel = currentLabel->child_;
        currentLabel->child_.reset();
        currentLabel.reset();
        currentLabel = nextLabel;
    }
    currentLabel.reset();*/
}

void GreedyRoute::resetLinkedGreedyLabels(vector<PStopLabel> &greedyLabelPool) {
    PStopLabel currentLabel = PInitialStop_;
    while (currentLabel->child_ != nullptr) {
//        currentLabel->currentNode_.reset();
        greedyLabelPool.push_back(currentLabel);
        currentLabel = currentLabel->child_;
    }
}

// this function find a position to insert pickup point and add drop off point at the end

/********************* New approach **********************/

bool GreedyRoute::isInsertPossible(PStopLabel &preLabel, PNode &newNode) const {
    // check the capacity
    if (preLabel->nbPassengers_ + newNode->nbPassengers_ > (*Vehicle_)->capacity_)
        return false;

    if (preLabel == PLastStop_)
        return true;

    float reachTime = labelToNodeReachTime(preLabel, newNode);
    float deltaT = nodeToLabelReachTime(reachTime, newNode, preLabel->child_) - preLabel->child_->reachTime_;

    PStopLabel nextLabel = preLabel->child_;
    while (nextLabel != nullptr) {
        if (nextLabel->currentNode_->type_ == DROPOFF) {
            if ((nextLabel->pair_ == nullptr) || (nextLabel->pair_->reachTime_ <= preLabel->reachTime_)) {
                if (nextLabel->travelResource_ < deltaT)
                    return false;
            }
        }
        nextLabel = nextLabel->child_;
    }
    return true;
}

bool GreedyRoute::isDropPossible(PStopLabel &preDrop, PStopLabel &pickLabel, PNode & dropNode, float maxDuration) const {
    if (!isInsertPossible(preDrop, dropNode))
        return false;
    else {
        float dropTime = preDrop->leaveTime_ + durationMatrix_[preDrop->currentNode_->locationID_][dropNode->locationID_];
        if (dropTime - pickLabel->leaveTime_ > maxDuration)
            return false;
    }
    return true;
}

/*bool GreedyRoute::isDropPossible1(PStopLabel &preDrop, PStopLabel &pickLabel, PNode & dropNode, float maxDuration) const {
    if (!isInsertPossible(preDrop, dropNode))
        return false;
    else {
        float departTime;
        if (preDrop->currentNode_->locationID_ == dropNode->locationID_)
            departTime = preDrop->reachTime_;
        else
            departTime = preDrop->reachTime_ + preDrop->currentNode_->serviceTime_;
        float dropTime = departTime + durationMatrix_[preDrop->currentNode_->locationID_][dropNode->locationID_];
        if (dropTime - pickLabel->leaveTime_ > maxDuration)
            return false;
    }
    return true;
}*/

void GreedyRoute::findInsertPlace(PNode &pickNode, PNode &dropNode, float maxDuration,
                                  std::vector<PStopLabel> &removedLabels, PInsertPosition & position) {
    float deltaDelay = INFINITY;

    // Tour length increase by adding pickup
    float pickDeltaT = INFINITY;

    // Tour length increase by adding drop off
    float dropDeltaT = INFINITY;
    position->updatePosition(PLastStop_, PLastStop_, deltaDelay, pickDeltaT + dropDeltaT);

    // define the initial position to add the request just after all
    PStopLabel prePick = PCurrentStop_;
    PStopLabel preDrop = PCurrentStop_;
    while (prePick != nullptr) {
        deltaDelay = INFINITY;
        if (prePick == PLastStop_) {
            if (prePick->nbPassengers_ + pickNode->nbPassengers_ > (*Vehicle_)->capacity_){
                std::cout << "Instance error! split the trip!" << std::endl;
                myTools::throwException("instance definition error");
            }
            // it stays at tail and then departs to the pickup point
            float pickTime = labelToNodeReachTime(prePick, pickNode);
            deltaDelay =  pickTime - pickNode->requestTime_;
            pickDeltaT = pickTime - departureTime_;
            dropDeltaT = pickNode->serviceTime_ + durationMatrix_[pickNode->locationID_][dropNode->locationID_];
            preDrop = PLastStop_;
            if (deltaDelay  < position->deltaDelay_) {
                if (pickDeltaT + dropDeltaT < position->deltaLength_)
                    position->updatePosition(prePick, preDrop, deltaDelay,pickDeltaT + dropDeltaT);
            }
        }
        else {
            //first we check to see is it possible to insert pickup after prePick
 //           if ((prePick->currentNode_->type_==SOURCE)||(prePick->currentNode_->related_Request_->minTravelTime_ != 0)) {
                if (isInsertPossible(prePick, pickNode)) {
                    // we have to find a place to insert drop point
                    // first we make a copy of the list and insert the pickup
                    float endTime = PLastStop_->reachTime_;
                    float curDelay = totalDelay_;

                    /*std::cout << "GreedyLinkList before checking" << std::endl;
                    std::cout << toString() << std::endl;*/
                    insertNode1(prePick, pickNode, removedLabels);
                    /*std::cout << "GreedyLinkList after insertion" << std::endl;
                    std::cout << toString() << std::endl;*/
                    deltaDelay = totalDelay_ - curDelay;
                    pickDeltaT = PLastStop_->reachTime_ - endTime;
                    preDrop = prePick->child_;
                    PStopLabel pickLabel = prePick->child_;

                    /*if (pickNode->related_Request_->minTravelTime_ == 0){
                        if (deltaDelay < position->deltaDelay_){
                            position->updatePosition(prePick, prePick, deltaDelay,pickDeltaT);
                        }
                    }
                    else {*/

                        // we have to check the vehicle capacity violation, if there is one, it should be dropped before that
                        PStopLabel endLabel = nullptr;
                        PStopLabel currentIndex = pickLabel;
                        while (currentIndex->child_ != nullptr) {
                            if (currentIndex->child_->nbPassengers_ > (*Vehicle_)->capacity_) {
                                endLabel = currentIndex->child_;
                                break;
                            } else
                                currentIndex = currentIndex->child_;
                        }
                        while (preDrop != endLabel) {
                            float increaseDelayByDrop = 0;
                            if (isDropPossible(preDrop, pickLabel, dropNode, maxDuration)) {
                                if (preDrop == PLastStop_)
                                    dropDeltaT = durationMatrix_[preDrop->currentNode_->locationID_][dropNode->locationID_]
                                                 + dropNode->serviceTime_;
                                else {
                                    dropDeltaT = labelToNodeReachTime(preDrop, dropNode) + dropNode->serviceTime_ +
                                                 durationMatrix_[dropNode->locationID_][preDrop->child_->currentNode_->locationID_] -
                                                 preDrop->child_->reachTime_;

                                    // calculate increase in wait time after adding the drop node
                                    PStopLabel currentLabel = preDrop;
                                    while (currentLabel->child_ != nullptr) {
                                        if (currentLabel->child_->currentNode_->type_ == PICKUP)
                                            increaseDelayByDrop += dropDeltaT;
                                        currentLabel = currentLabel->child_;
                                    }
                                }
                                if (deltaDelay + increaseDelayByDrop < position->deltaDelay_) {
                                    if (preDrop == pickLabel)
                                        position->updatePosition(prePick, prePick, deltaDelay + increaseDelayByDrop,
                                                                 pickDeltaT + dropDeltaT);
                                    else
                                        position->updatePosition(prePick, preDrop, deltaDelay + increaseDelayByDrop,
                                                                 pickDeltaT + dropDeltaT);
                                } else if (deltaDelay + increaseDelayByDrop == position->deltaDelay_) {
                                    if (pickDeltaT + dropDeltaT < position->deltaLength_) {
                                        if (preDrop == pickLabel)
                                            position->updatePosition(prePick, prePick, deltaDelay + increaseDelayByDrop,
                                                                     pickDeltaT + dropDeltaT);
                                        else
                                            position->updatePosition(prePick, preDrop, deltaDelay + increaseDelayByDrop,
                                                                     pickDeltaT + dropDeltaT);
                                    }
                                }
                            }
                            preDrop = preDrop->child_;
                        }
 //                   }
                    removeLabel(pickLabel, removedLabels);
                    /*std::cout << "GreedyLinkList after deletion" << std::endl;
                    std::cout << toString() << std::endl;*/
                }
//            }
        }
        prePick = prePick->child_;
    }
}

void GreedyRoute::findInsertPlace1(PNode &pickNode, PNode &dropNode, float maxDuration,
                                   std::vector<PStopLabel> &greedyLabelPool, PInsertPosition & position) {
    float deltaDelay = INFINITY;

    // Tour length increase by adding pickup
    float pickDeltaT = INFINITY;

    // Tour length increase by adding drop off
    float dropDeltaT = INFINITY;
    position->updatePosition(PLastStop_, PLastStop_, deltaDelay, pickDeltaT + dropDeltaT);

    // define the initial position to add the request just after all
    PStopLabel prePick = PCurrentStop_;
    PStopLabel preDrop = PCurrentStop_;
    while (prePick != nullptr) {
        deltaDelay = INFINITY;
        if (prePick == PLastStop_) {
            if (prePick->nbPassengers_ + pickNode->nbPassengers_ > (*Vehicle_)->capacity_){
                std::cout << "Instance error! split the trip!" << std::endl;
                myTools::throwException("instance definition error");
            }
            // it stays at tail and then departs to the pickup point
            float pickTime = labelToNodeReachTime(prePick, pickNode);
            deltaDelay =  pickTime - pickNode->requestTime_;
            pickDeltaT = pickTime - departureTime_;
            dropDeltaT = pickNode->serviceTime_ + durationMatrix_[pickNode->locationID_][dropNode->locationID_];
            preDrop = PLastStop_;
            if (deltaDelay  < position->deltaDelay_) {
                if (pickDeltaT + dropDeltaT < position->deltaLength_)
                    position->updatePosition(prePick, preDrop, deltaDelay,pickDeltaT + dropDeltaT);
            }
        }
        else {
            // first we check to see is it possible to insert pickup after prePick
            // if ((prePick->currentNode_->type_==SOURCE)||(prePick->currentNode_->related_Request_->minTravelTime_ != 0)) {
            if (isInsertPossible(prePick, pickNode)) {
                // we have to find a place to insert drop point
                // first we make a copy of the list and insert the pickup
                float endTime = PLastStop_->reachTime_;
                float curDelay = totalDelay_;

                insertNode1(prePick, pickNode, greedyLabelPool);

                // check travel resource and remove the node if there is violation
                PStopLabel nextLabel = prePick->child_->child_;
                bool isViolated = false;
                while (nextLabel != nullptr) {
                    if (nextLabel->currentNode_->type_ == DROPOFF) {
                        if (nextLabel->travelResource_ < 0) {
                            isViolated = true;
                            break;
                        }
                    }
                    nextLabel = nextLabel->child_;
                }

                deltaDelay = totalDelay_ - curDelay;
                pickDeltaT = PLastStop_->reachTime_ - endTime;
                preDrop = prePick->child_;
                PStopLabel pickLabel = prePick->child_;

                if (!isViolated) {
                    // we have to check the vehicle capacity violation, if there is one, it should be dropped before that
                    PStopLabel endLabel = nullptr;
                    PStopLabel currentIndex = pickLabel;
                    while (currentIndex->child_ != nullptr) {
                        if (currentIndex->child_->nbPassengers_ > (*Vehicle_)->capacity_) {
                            endLabel = currentIndex->child_;
                            break;
                        } else
                            currentIndex = currentIndex->child_;
                    }
                    while (preDrop != endLabel) {
                        float increaseDelayByDrop = 0;
                        if (isDropPossible(preDrop, pickLabel, dropNode, maxDuration)) {
                            if (preDrop->currentNode_->locationID_ != dropNode->locationID_)
                                dropDeltaT = 0;
                            else {
                                if (preDrop == PLastStop_)
                                    dropDeltaT =
                                            durationMatrix_[preDrop->currentNode_->locationID_][dropNode->locationID_]
                                            + dropNode->serviceTime_;
                                else {
                                    if (dropNode->locationID_ != preDrop->child_->currentNode_->locationID_)
                                        dropDeltaT = labelToNodeReachTime(preDrop, dropNode) + dropNode->serviceTime_ +
                                                     durationMatrix_[dropNode->locationID_][preDrop->child_->currentNode_->locationID_] -
                                                     preDrop->child_->reachTime_;
                                    else
                                        dropDeltaT = labelToNodeReachTime(preDrop, dropNode) + dropNode->serviceTime_
                                                     - preDrop->child_->reachTime_;
                                    // calculate increase in wait time after adding the drop node
                                    PStopLabel currentLabel = preDrop;
                                    while (currentLabel->child_ != nullptr) {
                                        if (currentLabel->child_->currentNode_->type_ == PICKUP)
                                            increaseDelayByDrop += dropDeltaT;
                                        currentLabel = currentLabel->child_;
                                    }
                                }
                            }


                            if (deltaDelay + increaseDelayByDrop < position->deltaDelay_) {
                                if (preDrop == pickLabel)
                                    position->updatePosition(prePick, prePick, deltaDelay + increaseDelayByDrop,
                                                             pickDeltaT + dropDeltaT);
                                else
                                    position->updatePosition(prePick, preDrop, deltaDelay + increaseDelayByDrop,
                                                             pickDeltaT + dropDeltaT);
                            } else if (deltaDelay + increaseDelayByDrop == position->deltaDelay_) {
                                if (pickDeltaT + dropDeltaT < position->deltaLength_) {
                                    if (preDrop == pickLabel)
                                        position->updatePosition(prePick, prePick, deltaDelay + increaseDelayByDrop,
                                                                 pickDeltaT + dropDeltaT);
                                    else
                                        position->updatePosition(prePick, preDrop, deltaDelay + increaseDelayByDrop,
                                                                 pickDeltaT + dropDeltaT);
                                }
                            }
                        }
                        preDrop = preDrop->child_;
                    }
                }
                removeLabel(pickLabel, greedyLabelPool);
            }
        }
        prePick = prePick->child_;
    }
}

void GreedyRoute::findInsertPlace2(PNode &pickNode, PNode &dropNode, float maxDuration,
                                   std::vector<PStopLabel> &greedyLabelPool, PInsertPosition & position) {
    float waitIncrease = INFINITY;

    // Tour length increase
    float lengthIncrease = INFINITY;

    position->updatePosition(PLastStop_, PLastStop_, waitIncrease, lengthIncrease);

    // define the initial position to add the request just after all
    PStopLabel prePick = PCurrentStop_;
    PStopLabel preDrop = PCurrentStop_;
    while (prePick != nullptr) {
        waitIncrease = INFINITY;
        if (prePick == PLastStop_) {
            if (prePick->nbPassengers_ + pickNode->nbPassengers_ > (*Vehicle_)->capacity_){
                std::cout << "Instance error! split the trip!" << std::endl;
                myTools::throwException("instance definition error");
            }
            // it stays at tail and then departs to the pickup point
            float pickTime = labelToNodeReachTime(prePick, pickNode);
            waitIncrease = pickTime - pickNode->requestTime_;
            lengthIncrease = (pickTime - departureTime_) +
                             (pickNode->serviceTime_ + durationMatrix_[pickNode->locationID_][dropNode->locationID_]);
            preDrop = PLastStop_;
            if (waitIncrease < position->deltaDelay_) {
                if (lengthIncrease < position->deltaLength_)
                    position->updatePosition(prePick, preDrop, waitIncrease, lengthIncrease);
            }
        }
        else {
            float endTime = PLastStop_->leaveTime_;
            float curWait = totalDelay_;
            // insert pick up node
            insertNode(prePick, pickNode, greedyLabelPool);
            PStopLabel pickLabel = prePick->child_;

            // check the feasibility
            if (!isAnyViolation(prePick->child_)){
                preDrop = prePick->child_;
                PStopLabel endLabel = nullptr;
                PStopLabel currentIndex = pickLabel;
                // find the position to insert drop node
                while (currentIndex->child_ != nullptr) {
                    if (currentIndex->child_->nbPassengers_ > (*Vehicle_)->capacity_) {
                        endLabel = currentIndex->child_;
                        break;
                    } else
                        currentIndex = currentIndex->child_;
                }
                while (preDrop != endLabel) {
                    // insert drop node
                    insertNode(preDrop, dropNode, greedyLabelPool);
                    PStopLabel dropLabel = preDrop->child_;
                    dropLabel->travelResource_ = maxDuration - (dropLabel->reachTime_ - pickLabel->leaveTime_);

                    // check the feasibility
                    if (!isAnyViolation(prePick->child_)){
                        waitIncrease = totalDelay_ - curWait;
                        lengthIncrease = PLastStop_->leaveTime_ - endTime;
                        if (waitIncrease < position->deltaDelay_) {
                            if (preDrop == pickLabel)
                                position->updatePosition(prePick, prePick, waitIncrease, lengthIncrease);
                            else
                                position->updatePosition(prePick, preDrop, waitIncrease, lengthIncrease);
                        } else if (waitIncrease == position->deltaDelay_) {
                            if (lengthIncrease < position->deltaLength_) {
                                if (preDrop == pickLabel)
                                    position->updatePosition(prePick, prePick, waitIncrease, lengthIncrease);
                                else
                                    position->updatePosition(prePick, preDrop, waitIncrease, lengthIncrease);
                            }
                        }
                    }
                    removeLabel(dropLabel, greedyLabelPool);
                    preDrop = preDrop->child_;
                }
            }
            removeLabel(pickLabel, greedyLabelPool);
        }
        prePick = prePick->child_;
    }
}


void GreedyRoute::insertNode(PStopLabel &preLabel, PNode &newNode, std::vector<PStopLabel> &greedyLabelPool) {
    // calculate reach time
    float reachTime = labelToNodeReachTime(preLabel, newNode);
    // update depart time
    if (preLabel->currentNode_->type_ != SOURCE){
        if (newNode->type_ == PICKUP){
            if (preLabel->leaveTime_ < newNode->requestTime_)
                preLabel->leaveTime_ = newNode->requestTime_;
        }
    }

    // create new label
    PStopLabel newLabel;
    if (!greedyLabelPool.empty()){
        newLabel = greedyLabelPool.back();
        greedyLabelPool.pop_back();
        newLabel->setValues(newNode, reachTime, reachTime+newNode->serviceTime_,
                            preLabel->nbPassengers_ + newNode->nbPassengers_);
    }
    else
        newLabel = std::make_shared<StopLabel>(newNode, reachTime, reachTime + newNode->serviceTime_,
                                                 preLabel->nbPassengers_ + newNode->nbPassengers_);
    if (preLabel->child_ == nullptr){
        newLabel->parent_ = preLabel;
        preLabel->child_= newLabel;
        PLastStop_ = newLabel;
    }
    else {

        preLabel->child_->parent_ = newLabel;
        newLabel->child_ = preLabel->child_;
        newLabel->parent_ = preLabel;
        preLabel->child_= newLabel;

        updateReachTimes(newLabel);
    }
    if (newNode->type_ == PICKUP){
        if ((reachTime - newNode->requestTime_) < 0 )
            std::cout << "error" ;
        totalDelay_ += (reachTime - newNode->requestTime_);
    }
}
void GreedyRoute::insertNode1(PStopLabel &preLabel, PNode &newNode, std::vector<PStopLabel> &greedyLabelPool) {
    // calculate reach time
    float reachTime = labelToNodeReachTime(preLabel, newNode);
    // update depart time
    if (preLabel->currentNode_->type_ != SOURCE){
        if (newNode->type_ == PICKUP){
            if (preLabel->leaveTime_ < newNode->requestTime_)
                preLabel->leaveTime_ = newNode->requestTime_;
        }
    }
    // calculating depart time
    float departTime;
    if ((durationMatrix_[preLabel->currentNode_->locationID_][newNode->locationID_] == 0)&&
            (preLabel->currentNode_->initialType_ != SOURCE)){
        departTime = reachTime;
    }
    else {
        departTime = reachTime + newNode->serviceTime_;
    }

    // create new label
    PStopLabel newLabel;
    if (!greedyLabelPool.empty()){
        newLabel = greedyLabelPool.back();
        greedyLabelPool.pop_back();
        newLabel->setValues(newNode, reachTime, departTime,
                            preLabel->nbPassengers_ + newNode->nbPassengers_);
    }
    else
        newLabel = std::make_shared<StopLabel>(newNode, reachTime, departTime,
                                                 preLabel->nbPassengers_ + newNode->nbPassengers_);
    if (preLabel->child_ == nullptr){
        newLabel->parent_ = preLabel;
        preLabel->child_= newLabel;
        PLastStop_ = newLabel;
    }
    else {

        preLabel->child_->parent_ = newLabel;
        newLabel->child_ = preLabel->child_;
        newLabel->parent_ = preLabel;
        preLabel->child_= newLabel;

        updateReachTimes1(newLabel);
    }
    if (newNode->type_ == PICKUP){
        if ((reachTime - newNode->requestTime_) < 0 )
            std::cout << "error" ;
        totalDelay_ += (reachTime - newNode->requestTime_);
    }
}

void GreedyRoute::removeLabel(PStopLabel &label, std::vector<PStopLabel> &greedyLabelPool) {
    greedyLabelPool.push_back(label);
    if (label->currentNode_->type_ == PICKUP) {
        totalDelay_ -= (label->reachTime_ - label->currentNode_->requestTime_);
    }
    if (label->child_ == nullptr){
        label->parent_->child_ = nullptr;
        PLastStop_ = label->parent_;
        label.reset();
    }
    else {
        label->child_->parent_ = label->parent_;
        label->parent_->child_ = label->child_;
        updateReachTimes(label->parent_);
        label.reset();
    }
}

void GreedyRoute::insertRequest(PInsertPosition &position, PNode &pickNode, PNode &dropNode, float maxDuration,
                                std::vector<PStopLabel> &greedyLabelPool) {
    PStopLabel pickLabel;
    PStopLabel dropLabel;
    // if pick up and drop off are inserted in the same slot
    if (position->prePickup_ == position->preDrop_) {
        insertNode(position->prePickup_, pickNode, greedyLabelPool);
        pickLabel = position->prePickup_->child_;
        insertNode(pickLabel, dropNode, greedyLabelPool);
        dropLabel = pickLabel->child_;
    }
    else {
        insertNode(position->prePickup_, pickNode, greedyLabelPool);
        pickLabel = position->prePickup_->child_;
        insertNode(position->preDrop_, dropNode, greedyLabelPool);
        dropLabel = position->preDrop_->child_;
    }

    pickLabel->pair_ = dropLabel;
    dropLabel->pair_ = pickLabel;
    dropLabel->travelResource_ = maxDuration - (dropLabel->reachTime_ - pickLabel->leaveTime_);
    if (dropLabel->travelResource_ < 0)
        myTools::throwException("Negative travel resource");
}
// this function calculate the reachTime from a Label to a node
float GreedyRoute::labelToNodeReachTime(PStopLabel &preLabel, PNode &Node) {
    if ((Node->type_ == PICKUP)&&(preLabel->leaveTime_ <= Node->requestTime_))
        return Node->requestTime_ + durationMatrix_[preLabel->currentNode_->locationID_][Node->locationID_];
    else
        return preLabel->leaveTime_ + durationMatrix_[preLabel->currentNode_->locationID_][Node->locationID_];
}


// this function calculate the reachTime from a node to a Label
float GreedyRoute::nodeToLabelReachTime(float nodeReachTime, PNode &preNode, PStopLabel &nextLabel) {
    if ((nextLabel->currentNode_->type_ == PICKUP) && (nodeReachTime + preNode->serviceTime_ < nextLabel->currentNode_->requestTime_))
        return nextLabel->currentNode_->requestTime_ + durationMatrix_[preNode->locationID_][nextLabel->currentNode_->locationID_];
    else
        return nodeReachTime + preNode->serviceTime_ +
               durationMatrix_[preNode->locationID_][nextLabel->currentNode_->locationID_];
}

float GreedyRoute::nodeToLabelReachTime1(float nodeReachTime, PNode &preNode, PStopLabel &nextLabel) {
    if (durationMatrix_[preNode->locationID_][nextLabel->currentNode_->locationID_] == 0){
        if ((nextLabel->currentNode_->type_ == PICKUP) && (nodeReachTime < nextLabel->currentNode_->requestTime_))
            return nextLabel->currentNode_->requestTime_;
        else
            return nodeReachTime;
    }
    else{
        if ((nextLabel->currentNode_->type_ == PICKUP) && (nodeReachTime + preNode->serviceTime_ < nextLabel->currentNode_->requestTime_))
            return nextLabel->currentNode_->requestTime_ + durationMatrix_[preNode->locationID_][nextLabel->currentNode_->locationID_];
        else
            return nodeReachTime + preNode->serviceTime_ +
                   durationMatrix_[preNode->locationID_][nextLabel->currentNode_->locationID_];
    }
}

// this function starts from a label in the list and update reachTimes and departTimes afterwards to the tail
void GreedyRoute::updateReachTimes(PStopLabel &preLabel) {
    PStopLabel currentLabel = preLabel;
    while (currentLabel->child_ != nullptr) {
        // calculate child reach time
        float childReachTime = labelToNodeReachTime(currentLabel, currentLabel->child_->currentNode_);
        if (currentLabel->currentNode_->type_ != SOURCE){
            if (currentLabel->child_->currentNode_->type_ == PICKUP){
                if (currentLabel->leaveTime_ < currentLabel->child_->currentNode_->requestTime_)
                    currentLabel->leaveTime_ = currentLabel->child_->currentNode_->requestTime_;
            }
        }
        // set current stop depart time
        /*if (currentLabel->currentNode_->type_ != SOURCE){
            if (currentLabel->child_->currentNode_->type_ == PICKUP)
                currentLabel->leaveTime_ = std::max(currentLabel->child_->currentNode_->requestTime_,
                                                     currentLabel->reachTime_ + currentLabel->currentNode_->serviceTime_);
            else
                currentLabel->leaveTime_ = currentLabel->reachTime_ + currentLabel->currentNode_->serviceTime_;
        }*/

        // update total delay
        if (currentLabel->child_->currentNode_->type_ == PICKUP) {
            float preDelay = currentLabel->child_->reachTime_ - currentLabel->child_->currentNode_->requestTime_;
            float newDelay = childReachTime - currentLabel->child_->currentNode_->requestTime_;
            totalDelay_ += (newDelay - preDelay);
            if (totalDelay_ < 0 ) {
                std::cout << "error";
                std::cout << toString() << std::endl;
            }
        }
        else if (currentLabel->child_->currentNode_->type_ == DROPOFF) {
            if (currentLabel->child_->pair_ == nullptr) {
                currentLabel->child_->travelResource_ = currentLabel->child_->currentNode_->related_Request_->maxTravelTime_ -
                                                        childReachTime + currentLabel->child_->currentNode_->pairNode_->departTime_;
            } else {
                currentLabel->child_->travelResource_ = currentLabel->child_->currentNode_->related_Request_->maxTravelTime_ -
                                                        (childReachTime - currentLabel->child_->pair_->leaveTime_);
            }
        }
        currentLabel->child_->reachTime_ = childReachTime;
        currentLabel->child_->leaveTime_ = childReachTime + currentLabel->child_->currentNode_->serviceTime_;
        currentLabel->child_->nbPassengers_ = currentLabel->nbPassengers_ + currentLabel->child_->currentNode_->nbPassengers_;

        currentLabel = currentLabel->child_;
    }
    if (totalDelay_ < 0 ) {
 //       myTools::throwException("Delay Error");
        std::cout << toString() << std::endl;
    }
}
void GreedyRoute::updateReachTimes1(PStopLabel &preLabel) {
    PStopLabel currentLabel = preLabel;
    while (currentLabel->child_ != nullptr) {
        // calculate child reach time
        float childReachTime = labelToNodeReachTime(currentLabel, currentLabel->child_->currentNode_);

        // set current stop depart time
        if (currentLabel->currentNode_->type_ != SOURCE){
            if (currentLabel->child_->currentNode_->type_ == PICKUP){
                if (currentLabel->leaveTime_ < currentLabel->child_->currentNode_->requestTime_)
                    currentLabel->leaveTime_ = currentLabel->child_->currentNode_->requestTime_;
            }
        }

        // update total delay
        if (currentLabel->child_->currentNode_->type_ == PICKUP) {
            float preDelay = currentLabel->child_->reachTime_ - currentLabel->child_->currentNode_->requestTime_;
            float newDelay = childReachTime - currentLabel->child_->currentNode_->requestTime_;
            totalDelay_ += (newDelay - preDelay);
            if (totalDelay_ < 0 ) {
                std::cout << "error";
                std::cout << toString() << std::endl;
            }
        }
        else if (currentLabel->child_->currentNode_->type_ == DROPOFF) {
            if (currentLabel->child_->pair_ == nullptr) {
                currentLabel->child_->travelResource_ = currentLabel->child_->currentNode_->related_Request_->maxTravelTime_ -
                                                        childReachTime + currentLabel->child_->currentNode_->pairNode_->departTime_;
            } else {
                currentLabel->child_->travelResource_ = currentLabel->child_->currentNode_->related_Request_->maxTravelTime_ -
                        (childReachTime - currentLabel->child_->pair_->leaveTime_);
            }
        }

        currentLabel->child_->reachTime_ = childReachTime;
        if ((durationMatrix_[currentLabel->currentNode_->locationID_][currentLabel->child_->currentNode_->locationID_] == 0)&&
            (currentLabel->currentNode_->initialType_ != SOURCE))
            currentLabel->child_->leaveTime_ = childReachTime;
        else
            currentLabel->child_->leaveTime_ = childReachTime + currentLabel->child_->currentNode_->serviceTime_;

        currentLabel->child_->nbPassengers_ = currentLabel->nbPassengers_ + currentLabel->child_->currentNode_->nbPassengers_;

        currentLabel = currentLabel->child_;
    }
    if (totalDelay_ < 0 ) {
 //       myTools::throwException("Delay Error");
        std::cout << toString() << std::endl;
    }
}

// this function convert a greedyLabel list to a route
PRoute GreedyRoute::greedyLabelToRoute(bool update) const {
    /*std::cout << "GreedyLinkList before converting to the route" << std::endl;
    std::cout << toString() << std::endl;*/
    PRoute newRoute = std::make_shared<Route>((*Vehicle_)->vehicleID_);
    newRoute->addSource(PInitialStop_->currentNode_, PInitialStop_->leaveTime_, (*Vehicle_)->numPassengers_);
    PStopLabel currentLabel = PInitialStop_->child_;
    while (currentLabel != nullptr) {
        newRoute->addNode(currentLabel->currentNode_);
        if (newRoute->plannedReachTime_.back() != currentLabel->reachTime_) {
            std::cout << "Connectivity constraint violated at node : ";
            std::cout << newRoute->routeNodes_.back()->nodeID_ << std::endl;
 //           myTools::throwException("Route-Validation");
        }

        if (currentLabel->parent_->leaveTime_ < currentLabel->currentNode_->requestTime_) {
            std::cout << "Depart time violated at node : ";
            std::cout << currentLabel->parent_->currentNode_->nodeID_ << std::endl;
 //           myTools::throwException("Route-Validation");
        }
        if (update) {
            newRoute->routeNodes_.back()->related_Request_->allocVehicleID_ = newRoute->vehicleID_;
            newRoute->routeNodes_.back()->nodeStatus_ = DONE;
            newRoute->routeNodes_.back()->reachTime_ = newRoute->plannedReachTime_.back();
            /*if (currentLabel->child_ == nullptr)
                newRoute->routeNodes_.back()->leaveTime_ = newRoute->routeNodes_.back()->reachTime_+
                        newRoute->routeNodes_.back()->serviceTime_;
            else*/
                newRoute->routeNodes_.back()->departTime_ = currentLabel->leaveTime_;
            if (newRoute->routeNodes_.back()->type_ == PICKUP)
                newRoute->routeNodes_.back()->related_Request_->pickTime_ = currentLabel->reachTime_;
            else if (newRoute->routeNodes_.back()->type_ == DROPOFF) {
                newRoute->routeNodes_.back()->related_Request_->dropTime_ = currentLabel->reachTime_;
                newRoute->routeNodes_.back()->related_Request_->requestStatus_ = COMPLETED;
            }
        }
        currentLabel = currentLabel->child_;
    }

    if (newRoute->totalDelay_ != totalDelay_){
        std::cout << "Total delay of the greedy solution is not the same as the route delay!";
        std::cout << "Vehicle ID: " << newRoute->vehicleID_ << std::endl;
        //           myTools::throwException("Route-Validation");
    }
    /*std::cout << "Created route from the GreedyLinkList" << std::endl;
    std::cout << newRoute->toString() << std::endl;*/
    return newRoute;
}

std::string GreedyRoute::toString() const {
    std::stringstream repStr;

    repStr << "#" << std::left << std::endl;
    repStr << "#\t" << std::setw(24) << "- VEHICLE_ID" << " : " << (*Vehicle_)->vehicleID_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- TOTAL_WAITING (seconds)" << " : " << totalDelay_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- IDLE_TIME (seconds)" << " : " << idleTime_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- DEPART_TIME (seconds)" << " : " << departureTime_ << std::endl;
    repStr << "#" << std::endl;

    // print table header
    repStr << "# ---------------------------------------------------------------------------------------------------------------" << std::endl;
    repStr << std::left << std::setw(6) << "#      ";
    repStr << std::left << std::setw(27) << "ACTION_DESCRIPTION";
    repStr << std::left << std::setw(11) << "NODE_ID" << std::right;
    repStr << std::right << std::setw(11) << " REACH_TIME"<< " (s)  ";
    repStr << std::right << std::setw(11) << " DEPART_TIME"<< " (s)  ";
    repStr << std::right << std::setw(11) << " TRAVEL_RESOURCE"<< " (s)  ";
    repStr << "#PASSENGERS" <<std::endl;
    repStr << "# ---------------------------------------------------------------------------------------------------------------" << std::endl;

    // print the source stop pint
    repStr << "#" << std::setw(4) << 1 << "  ";
    repStr << std::left << std::setw(27) << "(SOURCE ) departure";
    repStr << std::left << std::setw(11) << PInitialStop_->currentNode_->nodeID_;
    repStr << std::right << std::setw(11) << PInitialStop_->reachTime_ << " (s)  ";
    repStr << std::right << std::setw(11) << PInitialStop_->leaveTime_ << " (s)  ";
    repStr << std::right << std::setw(11) << PInitialStop_->travelResource_ << " (s)  ";
    repStr << std::setw(7) << PInitialStop_->nbPassengers_ << std::endl;

    // print the internal nodes of the route
    PStopLabel curr = PInitialStop_;
    int counter = 1;
    while (curr->child_ != nullptr) {
        repStr << "#" << std::setw(4) << counter + 1 << "  ";
        if (curr->child_->currentNode_->type_ == SINK)
            repStr << std::left << std::setw(27) << "(SINK   ) return";
        else {
            repStr << "(" << NodeTypeStr[curr->child_->currentNode_->type_] << ") Request_ID ";
            repStr << std::left << std::setw(6) << curr->child_->currentNode_->related_Request_->getRequestId();
        }
        repStr << std::left << std::setw(11) << curr->child_->currentNode_->nodeID_;
        repStr << std::right << std::setw(11) << curr->child_->reachTime_ << " (s)  ";
        if (curr->child_->reachTime_ != curr->child_->currentNode_->reachTime_ && curr->child_->currentNode_->reachTime_ != 0)
            std::cout << "error";
        repStr << std::right << std::setw(11) << curr->child_->leaveTime_ << " (s)  ";
        repStr << std::right << std::setw(11) << curr->child_->travelResource_ << " (s)  ";
        repStr << std::setw(7) << curr->child_->nbPassengers_ << std::endl;
        curr = curr->child_;
        counter++;
    }

    repStr << "=================================================================================================================" << std::endl;
    return repStr.str();
}

void GreedyRoute::findAssignedPlace(PNode &pickNode, PNode &dropNode, float maxDuration,
                                    vector<PStopLabel> &removedLabels, PInsertPosition &position) {
    float deltaDelay = INFINITY;
    bool notFound = true;

    // Tour length increase by adding pickup
    float pickDeltaT = INFINITY;

    // Tour length increase by adding drop off
    float dropDeltaT = INFINITY;
    position->updatePosition(PLastStop_, PLastStop_, deltaDelay, pickDeltaT + dropDeltaT);

    // define the initial position to add the request just after all
    PStopLabel prePick = PCurrentStop_;
    PStopLabel preDrop = PCurrentStop_;
    while ((prePick != nullptr) && (notFound)) {
        deltaDelay = INFINITY;
        if (prePick == PLastStop_) {
            // it stays at tail and then departs to the pickup point
            float pickTime = labelToNodeReachTime(prePick, pickNode);
            deltaDelay =  pickTime - pickNode->requestTime_;
            pickDeltaT = pickTime - departureTime_;
            preDrop = PLastStop_;
            if (deltaDelay  < position->deltaDelay_) {
                position->updatePosition(prePick, preDrop, deltaDelay,pickDeltaT);
            }
        }
        else {
            //first we check to see is it possible to insert pickup after prePick
            if (isInsertPossible(prePick, pickNode)) {
                // we have to find a place to insert drop point
                // first we make a copy of the list and insert the pickup
                float endTime = PLastStop_->reachTime_;
                float curDelay = totalDelay_;

                insertNode1(prePick,pickNode, removedLabels);
                deltaDelay =  totalDelay_ - curDelay;
                pickDeltaT = PLastStop_->reachTime_ - endTime;
                preDrop = prePick->child_;
                PStopLabel pickLabel = prePick->child_;

                // we have to check the vehicle capacity violation, if there is one, it should be dropped before that
                PStopLabel endLabel = nullptr;
                PStopLabel currentIndex = pickLabel;
                while (currentIndex->child_ != nullptr){
                    if (currentIndex->child_->nbPassengers_ > (*Vehicle_)->capacity_) {
                        endLabel = currentIndex->child_;
                        break;
                    }
                    else
                        currentIndex = currentIndex->child_;
                }
                while (preDrop != endLabel) {
                    if (isDropPossible(preDrop, pickLabel, dropNode, maxDuration)) {
                        if (deltaDelay  <= position->deltaDelay_) {
                            if (preDrop == pickLabel)
                                position->updatePosition(prePick, prePick, deltaDelay,
                                                         pickDeltaT + dropDeltaT);
                            else
                                position->updatePosition(prePick, preDrop, deltaDelay,
                                                         pickDeltaT + dropDeltaT);
                            notFound = false;
                            break;
                        }
                    }
                    else
                        preDrop = preDrop->child_;
                }
                removeLabel(pickLabel, removedLabels);
                preDrop = preDrop->child_;
            }
        }
        prePick = prePick->child_;
    }
}

void GreedyRoute::updateTailDepart() {
    if (PLastStop_->currentNode_->type_ != SOURCE)
        PLastStop_->leaveTime_ = PLastStop_->reachTime_ + PLastStop_->currentNode_->serviceTime_;
    departureTime_ = PLastStop_->leaveTime_;
}

void GreedyRoute::updateTailDepart1() {
    if (PLastStop_->currentNode_->type_ != SOURCE) {
        if (durationMatrix_[PLastStop_->parent_->currentNode_->locationID_][PLastStop_->currentNode_->locationID_] == 0)
            PLastStop_->leaveTime_ = PLastStop_->reachTime_;
        else
            PLastStop_->leaveTime_ = PLastStop_->reachTime_ + PLastStop_->currentNode_->serviceTime_;
    }
    departureTime_ = PLastStop_->leaveTime_;
}

bool GreedyRoute::isAnyViolation(PStopLabel &startLabel) {
    if (startLabel->nbPassengers_ > (*Vehicle_)->capacity_)
        return true;
    PStopLabel currentLabel = startLabel;
    while (currentLabel != nullptr) {
        if (currentLabel->currentNode_->type_ == DROPOFF) {
            if (currentLabel->travelResource_ < 0) {
                return true;
            }
        }
        currentLabel = currentLabel->child_;
    }
    return false;
}

void GreedyRoute::findAssignedPlace1(PNode &pickNode, PNode &dropNode, float maxDuration,
                                     vector<PStopLabel> &removedLabels, PInsertPosition &position) {
    float deltaDelay = INFINITY;
    bool notFound = true;
    // Tour length increase
    float DeltaT = INFINITY;

    position->updatePosition(PLastStop_, PLastStop_, deltaDelay, DeltaT);

    // define the initial position to add the request just after all
    PStopLabel prePick = PCurrentStop_;
    PStopLabel preDrop = PCurrentStop_;
    while ((prePick != nullptr) && (notFound)) {
        deltaDelay = INFINITY;
        if (prePick == PLastStop_) {
            if (prePick->nbPassengers_ + pickNode->nbPassengers_ > (*Vehicle_)->capacity_){
                std::cout << "Instance error! split the trip!" << std::endl;
                myTools::throwException("instance definition error");
            }
            // it stays at tail and then departs to the pickup point
            float pickTime = labelToNodeReachTime(prePick, pickNode);
            deltaDelay =  pickTime - pickNode->requestTime_;
            DeltaT = (pickTime - departureTime_) +
                     (pickNode->serviceTime_ + durationMatrix_[pickNode->locationID_][dropNode->locationID_]);
            preDrop = PLastStop_;
            if (deltaDelay  < position->deltaDelay_) {
                if (DeltaT < position->deltaLength_)
                    position->updatePosition(prePick, preDrop, deltaDelay,DeltaT);
            }
        }
        else {
            float endTime = PLastStop_->leaveTime_;
            float curDelay = totalDelay_;

            insertNode(prePick, pickNode, removedLabels);
            PStopLabel pickLabel = prePick->child_;
            if (!isAnyViolation(prePick->child_)){
                preDrop = prePick->child_;
                PStopLabel endLabel = nullptr;
                PStopLabel currentIndex = pickLabel;
                while (currentIndex->child_ != nullptr) {
                    if (currentIndex->child_->nbPassengers_ > (*Vehicle_)->capacity_) {
                        endLabel = currentIndex->child_;
                        break;
                    } else
                        currentIndex = currentIndex->child_;
                }
                while (preDrop != endLabel) {
                    insertNode(preDrop, dropNode, removedLabels);
                    PStopLabel dropLabel = preDrop->child_;
                    dropLabel->travelResource_ = maxDuration - (dropLabel->reachTime_ - pickLabel->leaveTime_);
                    if (!isAnyViolation(prePick->child_)){
                        deltaDelay = totalDelay_ - curDelay;
                        DeltaT = PLastStop_->leaveTime_ - endTime;

                        if (deltaDelay < position->deltaDelay_) {
                            if (preDrop == pickLabel)
                                position->updatePosition(prePick, prePick, deltaDelay,DeltaT);
                            else
                                position->updatePosition(prePick, preDrop, deltaDelay,DeltaT);
                            notFound = false;
                        }
                    }
                    removeLabel(dropLabel, removedLabels);
                    preDrop = preDrop->child_;
                }
            }
            removeLabel(pickLabel, removedLabels);
        }
        prePick = prePick->child_;
    }
}


insertPosition::insertPosition() {}
insertPosition::insertPosition(PStopLabel prePickup, PStopLabel preDrop, float waitIncrease,
                               float lengthIncrease) : prePickup_(std::move(prePickup)), preDrop_(std::move(preDrop)), deltaDelay_(waitIncrease),
                                                       deltaLength_(lengthIncrease) {}

void insertPosition::updatePosition(const PStopLabel &prePickup, const PStopLabel &preDrop, float waitIncrease,
                                    float lengthIncrease) {
    prePickup_ = prePickup;
    preDrop_ = preDrop;
    deltaDelay_ = waitIncrease;
    deltaLength_ = lengthIncrease;
}

