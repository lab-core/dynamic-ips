//
// Created by Ella on 5/20/2022.
//

#include "Greedy.h"

#include <utility>

GreedyLabel::GreedyLabel(PNode currentNode, float reachTime, float departTime, int nbPassengers) :
                            currentNode_(std::move(currentNode)), reachTime_(reachTime), departTime_(departTime),
                            nbPassengers_(nbPassengers) {
    parent_ = nullptr;
    child_ = nullptr;
    pair_ = nullptr;
    travelResource_ = 0;
}

void GreedyLabel::setValues(PNode currentNode, float reachTime, float departTime, int nbPassengers) {
    currentNode_ = currentNode;
    reachTime_ = reachTime;
    nbPassengers_ = nbPassengers;
    parent_ = nullptr;
    child_ = nullptr;
    pair_ = nullptr;
    travelResource_ = 0;
    departTime_ = departTime;
}


LinkedGreedyLabels::LinkedGreedyLabels(PVehicle &vehicle, PInstance &pInst) : Vehicle_(&vehicle)  {
    source_ = std::make_shared<GreedyLabel>((*Vehicle_)->departNode_,
                                            (*Vehicle_)->departNode_->reachTime_,
                                            vehicle->departTime_, vehicle->numPassengers_);
    head_ = source_;
    tail_ = source_;
    totalDelay_ = 0;
    idleTime_ = 0;
    departureTime_ = vehicle->departTime_;
    selected_ = false;
    for (auto &nodeID: (*Vehicle_)->onboards_) {
        PNode onboardNode = pInst->instGraph_->nodes_[nodeID];
        float dropTime = labelToNodeReachTime(tail_, onboardNode);
        PGreedyLabel newDropLabel = std::make_shared<GreedyLabel>(onboardNode, dropTime,
                                                                  dropTime+onboardNode->deltaTime_,
                                                                  tail_->nbPassengers_ +
                                                                  onboardNode->nbPassengers_);
        newDropLabel->parent_ = tail_;
        tail_->child_ = newDropLabel;
        newDropLabel->travelResource_ = onboardNode->related_Request_->maxTravelTime_ - dropTime +
                onboardNode->pairNode_->departTime_;

        tail_ = newDropLabel;
    }
}

LinkedGreedyLabels::LinkedGreedyLabels(PVehicle &vehicle, PInstance &pInst, std::vector<PGreedyLabel> &removedLabels) :
    Vehicle_(&vehicle) {
    if (removedLabels.empty())
        source_ = std::make_shared<GreedyLabel>((*Vehicle_)->departNode_,
                                                (*Vehicle_)->departNode_->reachTime_,
                                                vehicle->departTime_,
                                                vehicle->numPassengers_);
    else {
        source_ = removedLabels.back();
        removedLabels.pop_back();
        source_->setValues((*Vehicle_)->departNode_, (*Vehicle_)->departNode_->reachTime_,
                           vehicle->departTime_,vehicle->numPassengers_);
    }
    selected_ = false;
    head_ = source_;
    tail_ = source_;
    totalDelay_ = 0;
    idleTime_ = 0;
    departureTime_ = vehicle->departTime_;
    for (auto &nodeID: (*Vehicle_)->onboards_) {
        PNode onboardNode = pInst->instGraph_->nodes_[nodeID];
        float dropTime = labelToNodeReachTime(tail_, onboardNode);
        PGreedyLabel newDropLabel;
        if (removedLabels.empty()) {
            newDropLabel = std::make_shared<GreedyLabel>(onboardNode, dropTime,
                                                         dropTime + onboardNode->deltaTime_,
                                                         tail_->nbPassengers_ + onboardNode->nbPassengers_);
        }
        else {
            newDropLabel = removedLabels.back();
            removedLabels.pop_back();
            newDropLabel->setValues(onboardNode, dropTime,
                                    dropTime + onboardNode->deltaTime_,tail_->nbPassengers_ +
                                    onboardNode->nbPassengers_);
        }
        newDropLabel->parent_ = tail_;
        tail_->child_ = newDropLabel;
        newDropLabel->travelResource_ = onboardNode->related_Request_->maxTravelTime_ - dropTime +
                onboardNode->pairNode_->departTime_;

        tail_ = newDropLabel;
    }
}

LinkedGreedyLabels::LinkedGreedyLabels(const LinkedGreedyLabels &label) {
    selected_ = label.selected_;
    Vehicle_ = label.Vehicle_;
    departureTime_ = label.departureTime_;
    idleTime_ = label.idleTime_;
    head_ = label.head_;
    tail_ = label.tail_;
    source_ = label.source_;
    totalDelay_ = label.totalDelay_;
}

LinkedGreedyLabels::~LinkedGreedyLabels() {
    /*PGreedyLabel currentLabel = source_;
    while (currentLabel->child_ != nullptr) {
        currentLabel->pair_.reset();
        currentLabel->parent_.reset();
        currentLabel = currentLabel->child_;
    }
    currentLabel = source_;
    while (currentLabel->child_ != nullptr) {
        PGreedyLabel nextLabel = currentLabel->child_;
        currentLabel->child_.reset();
        currentLabel.reset();
        currentLabel = nextLabel;
    }
    currentLabel.reset();*/
}

void LinkedGreedyLabels::resetLinkedGreedyLabels(vector<PGreedyLabel> &removedLabels) {
    PGreedyLabel currentLabel = source_;
    while (currentLabel->child_ != nullptr) {
//        currentLabel->currentNode_.reset();
        removedLabels.push_back(currentLabel);
        currentLabel = currentLabel->child_;
    }
}

// this function find a position to insert pickup point and add drop off point at the end

/********************* New approach **********************/

bool LinkedGreedyLabels::isInsertPossible(PGreedyLabel &preLabel, PNode &newNode) const {
    // check the capacity
    if (preLabel->nbPassengers_ + newNode->nbPassengers_ > (*Vehicle_)->capacity_)
        return false;

    if (preLabel == tail_)
        return true;

    float reachTime = labelToNodeReachTime(preLabel, newNode);
    float deltaT = nodeToLabelReachTime1(reachTime, newNode, preLabel->child_) - preLabel->child_->reachTime_;

    PGreedyLabel nextLabel = preLabel->child_;
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

bool LinkedGreedyLabels::isDropPossible(PGreedyLabel &preDrop, PGreedyLabel &pickLabel, PNode & dropNode, float maxDuration) const {
    if (!isInsertPossible(preDrop, dropNode))
        return false;
    else {
        float dropTime = preDrop->departTime_ + durationMatrix_[preDrop->currentNode_->locationID_][dropNode->locationID_];
        if (dropTime - pickLabel->departTime_  > maxDuration)
            return false;
    }
    return true;
}

/*bool LinkedGreedyLabels::isDropPossible1(PGreedyLabel &preDrop, PGreedyLabel &pickLabel, PNode & dropNode, float maxDuration) const {
    if (!isInsertPossible(preDrop, dropNode))
        return false;
    else {
        float departTime;
        if (preDrop->currentNode_->locationID_ == dropNode->locationID_)
            departTime = preDrop->reachTime_;
        else
            departTime = preDrop->reachTime_ + preDrop->currentNode_->deltaTime_;
        float dropTime = departTime + durationMatrix_[preDrop->currentNode_->locationID_][dropNode->locationID_];
        if (dropTime - pickLabel->departTime_ > maxDuration)
            return false;
    }
    return true;
}*/

void LinkedGreedyLabels::findInsertPlace(PNode &pickNode, PNode &dropNode, float maxDuration,
                                                    std::vector<PGreedyLabel> &removedLabels, PInsertPosition & position) {
    float deltaDelay = INFINITY;

    // Tour length increase by adding pickup
    float pickDeltaT = INFINITY;

    // Tour length increase by adding drop off
    float dropDeltaT = INFINITY;
    position->updatePosition(tail_,tail_, deltaDelay, pickDeltaT + dropDeltaT);

    // define the initial position to add the request just after all
    PGreedyLabel prePick = head_;
    PGreedyLabel preDrop = head_;
    while (prePick != nullptr) {
        deltaDelay = INFINITY;
        if (prePick == tail_) {
            if (prePick->nbPassengers_ + pickNode->nbPassengers_ > (*Vehicle_)->capacity_){
                std::cout << "Instance error! split the trip!" << std::endl;
                myTools::throwException("instance definition error");
            }
            // it stays at tail and then departs to the pickup point
            float pickTime = labelToNodeReachTime(prePick, pickNode);
            deltaDelay =  pickTime - pickNode->requestTime_;
            pickDeltaT = pickTime - departureTime_;
            dropDeltaT = pickNode->deltaTime_ + durationMatrix_[pickNode->locationID_][dropNode->locationID_];
            preDrop = tail_;
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
                    float endTime = tail_->reachTime_;
                    float curDelay = totalDelay_;

                    /*std::cout << "GreedyLinkList before checking" << std::endl;
                    std::cout << toString() << std::endl;*/
                    insertNode1(prePick, pickNode, removedLabels);
                    /*std::cout << "GreedyLinkList after insertion" << std::endl;
                    std::cout << toString() << std::endl;*/
                    deltaDelay = totalDelay_ - curDelay;
                    pickDeltaT = tail_->reachTime_ - endTime;
                    preDrop = prePick->child_;
                    PGreedyLabel pickLabel = prePick->child_;

                    /*if (pickNode->related_Request_->minTravelTime_ == 0){
                        if (deltaDelay < position->deltaDelay_){
                            position->updatePosition(prePick, prePick, deltaDelay,pickDeltaT);
                        }
                    }
                    else {*/

                        // we have to check the vehicle capacity violation, if there is one, it should be dropped before that
                        PGreedyLabel endLabel = nullptr;
                        PGreedyLabel currentIndex = pickLabel;
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
                                if (preDrop == tail_)
                                    dropDeltaT = durationMatrix_[preDrop->currentNode_->locationID_][dropNode->locationID_]
                                                 + dropNode->deltaTime_;
                                else {
                                    dropDeltaT = labelToNodeReachTime(preDrop, dropNode) + dropNode->deltaTime_ +
                                                 durationMatrix_[dropNode->locationID_][preDrop->child_->currentNode_->locationID_] -
                                                 preDrop->child_->reachTime_;

                                    // calculate increase in wait time after adding the drop node
                                    PGreedyLabel currentLabel = preDrop;
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

void LinkedGreedyLabels::findInsertPlace1(PNode &pickNode, PNode &dropNode, float maxDuration,
                                         std::vector<PGreedyLabel> &removedLabels, PInsertPosition & position) {
    float deltaDelay = INFINITY;

    // Tour length increase by adding pickup
    float pickDeltaT = INFINITY;

    // Tour length increase by adding drop off
    float dropDeltaT = INFINITY;
    position->updatePosition(tail_,tail_, deltaDelay, pickDeltaT + dropDeltaT);

    // define the initial position to add the request just after all
    PGreedyLabel prePick = head_;
    PGreedyLabel preDrop = head_;
    while (prePick != nullptr) {
        deltaDelay = INFINITY;
        if (prePick == tail_) {
            if (prePick->nbPassengers_ + pickNode->nbPassengers_ > (*Vehicle_)->capacity_){
                std::cout << "Instance error! split the trip!" << std::endl;
                myTools::throwException("instance definition error");
            }
            // it stays at tail and then departs to the pickup point
            float pickTime = labelToNodeReachTime(prePick, pickNode);
            deltaDelay =  pickTime - pickNode->requestTime_;
            pickDeltaT = pickTime - departureTime_;
            dropDeltaT = pickNode->deltaTime_ + durationMatrix_[pickNode->locationID_][dropNode->locationID_];
            preDrop = tail_;
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
                float endTime = tail_->reachTime_;
                float curDelay = totalDelay_;

                insertNode1(prePick, pickNode, removedLabels);

                // check travel resource and remove the node if there is violation
                PGreedyLabel nextLabel = prePick->child_->child_;
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
                pickDeltaT = tail_->reachTime_ - endTime;
                preDrop = prePick->child_;
                PGreedyLabel pickLabel = prePick->child_;

                if (!isViolated) {
                    // we have to check the vehicle capacity violation, if there is one, it should be dropped before that
                    PGreedyLabel endLabel = nullptr;
                    PGreedyLabel currentIndex = pickLabel;
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
                                if (preDrop == tail_)
                                    dropDeltaT =
                                            durationMatrix_[preDrop->currentNode_->locationID_][dropNode->locationID_]
                                            + dropNode->deltaTime_;
                                else {
                                    if (dropNode->locationID_ != preDrop->child_->currentNode_->locationID_)
                                        dropDeltaT = labelToNodeReachTime(preDrop, dropNode) + dropNode->deltaTime_ +
                                                     durationMatrix_[dropNode->locationID_][preDrop->child_->currentNode_->locationID_] -
                                                     preDrop->child_->reachTime_;
                                    else
                                        dropDeltaT = labelToNodeReachTime(preDrop, dropNode) + dropNode->deltaTime_
                                                     - preDrop->child_->reachTime_;
                                    // calculate increase in wait time after adding the drop node
                                    PGreedyLabel currentLabel = preDrop;
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
                removeLabel(pickLabel, removedLabels);
            }
        }
        prePick = prePick->child_;
    }
}

void LinkedGreedyLabels::findInsertPlace2(PNode &pickNode, PNode &dropNode, float maxDuration,
                                          std::vector<PGreedyLabel> &removedLabels, PInsertPosition & position) {
    float deltaDelay = INFINITY;

    // Tour length increase
    float DeltaT = INFINITY;

    position->updatePosition(tail_,tail_, deltaDelay, DeltaT);

    // define the initial position to add the request just after all
    PGreedyLabel prePick = head_;
    PGreedyLabel preDrop = head_;
    while (prePick != nullptr) {
        deltaDelay = INFINITY;
        if (prePick == tail_) {
            if (prePick->nbPassengers_ + pickNode->nbPassengers_ > (*Vehicle_)->capacity_){
                std::cout << "Instance error! split the trip!" << std::endl;
                myTools::throwException("instance definition error");
            }
            // it stays at tail and then departs to the pickup point
            float pickTime = labelToNodeReachTime(prePick, pickNode);
            deltaDelay =  pickTime - pickNode->requestTime_;
            DeltaT = (pickTime - departureTime_) +
                    (pickNode->deltaTime_ + durationMatrix_[pickNode->locationID_][dropNode->locationID_]);
            preDrop = tail_;
            if (deltaDelay  < position->deltaDelay_) {
                if (DeltaT < position->deltaLength_)
                    position->updatePosition(prePick, preDrop, deltaDelay,DeltaT);
            }
        }
        else {
            float endTime = tail_->departTime_;
            float curDelay = totalDelay_;
            // insert pick up node
            insertNode1(prePick, pickNode, removedLabels);
            PGreedyLabel pickLabel = prePick->child_;

            // check the feasibility
            if (!isTimeViolated(prePick->child_)){
                preDrop = prePick->child_;
                PGreedyLabel endLabel = nullptr;
                PGreedyLabel currentIndex = pickLabel;
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
                    insertNode1(preDrop, dropNode, removedLabels);
                    PGreedyLabel dropLabel = preDrop->child_;
                    dropLabel->travelResource_ = maxDuration - (dropLabel->reachTime_ - pickLabel->departTime_);

                    // check the feasibility
                    if (!isTimeViolated(prePick->child_)){
                        deltaDelay = totalDelay_ - curDelay;
                        DeltaT = tail_->departTime_ - endTime;
                        if (deltaDelay < position->deltaDelay_) {
                            if (preDrop == pickLabel)
                                position->updatePosition(prePick, prePick, deltaDelay,DeltaT);
                            else
                                position->updatePosition(prePick, preDrop, deltaDelay,DeltaT);
                        } else if (deltaDelay == position->deltaDelay_) {
                            if (DeltaT < position->deltaLength_) {
                                if (preDrop == pickLabel)
                                    position->updatePosition(prePick, prePick, deltaDelay,DeltaT);
                                else
                                    position->updatePosition(prePick, preDrop, deltaDelay,DeltaT);
                            }
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

void LinkedGreedyLabels::insertNode(PGreedyLabel &preLabel, PNode &newNode, std::vector<PGreedyLabel> &removedLabels) {
    // calculate reach time
    float reachTime = labelToNodeReachTime(preLabel, newNode);
    // update depart time
    if (preLabel->currentNode_->type_ != SOURCE){
        if (newNode->type_ == PICKUP){
            if (preLabel->departTime_ < newNode->requestTime_)
                preLabel->departTime_ = newNode->requestTime_;
        }
    }

    // create new label
    PGreedyLabel newLabel;
    if (!removedLabels.empty()){
        newLabel = removedLabels.back();
        removedLabels.pop_back();
        newLabel->setValues(newNode, reachTime, reachTime+newNode->deltaTime_,
                            preLabel->nbPassengers_ + newNode->nbPassengers_);
    }
    else
        newLabel = std::make_shared<GreedyLabel>(newNode, reachTime, reachTime+newNode->deltaTime_,
                                                 preLabel->nbPassengers_ + newNode->nbPassengers_);
    if (preLabel->child_ == nullptr){
        newLabel->parent_ = preLabel;
        preLabel->child_= newLabel;
        tail_ = newLabel;
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
void LinkedGreedyLabels::insertNode1(PGreedyLabel &preLabel, PNode &newNode, std::vector<PGreedyLabel> &removedLabels) {
    // calculate reach time
    float reachTime = labelToNodeReachTime(preLabel, newNode);
    // update depart time
    if (preLabel->currentNode_->type_ != SOURCE){
        if (newNode->type_ == PICKUP){
            if (preLabel->departTime_ < newNode->requestTime_)
                preLabel->departTime_ = newNode->requestTime_;
        }
    }
    // calculating depart time
    float departTime;
    if ((durationMatrix_[preLabel->currentNode_->locationID_][newNode->locationID_] == 0)&&
            (preLabel->currentNode_->initialType_ != SOURCE)){
        departTime = reachTime;
    }
    else {
        departTime = reachTime + newNode->deltaTime_;
    }

    // create new label
    PGreedyLabel newLabel;
    if (!removedLabels.empty()){
        newLabel = removedLabels.back();
        removedLabels.pop_back();
        newLabel->setValues(newNode, reachTime, departTime,
                            preLabel->nbPassengers_ + newNode->nbPassengers_);
    }
    else
        newLabel = std::make_shared<GreedyLabel>(newNode, reachTime, departTime,
                                                 preLabel->nbPassengers_ + newNode->nbPassengers_);
    if (preLabel->child_ == nullptr){
        newLabel->parent_ = preLabel;
        preLabel->child_= newLabel;
        tail_ = newLabel;
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

void LinkedGreedyLabels::removeLabel(PGreedyLabel &label, std::vector<PGreedyLabel> &removedLabels) {
    removedLabels.push_back(label);
    if (label->currentNode_->type_ == PICKUP) {
        totalDelay_ -= (label->reachTime_ - label->currentNode_->requestTime_);
    }
    if (label->child_ == nullptr){
        label->parent_->child_ = nullptr;
        tail_ = label->parent_;
        label.reset();
    }
    else {
        label->child_->parent_ = label->parent_;
        label->parent_->child_ = label->child_;
        updateReachTimes1(label->parent_);
        label.reset();
    }
}

void LinkedGreedyLabels::insertRequest(PInsertPosition &position, PNode &pickNode, PNode &dropNode, float maxDuration,
                                       std::vector<PGreedyLabel> &removedLabels) {
    PGreedyLabel pickLabel;
    PGreedyLabel dropLabel;
    // if pick up and drop off are inserted in the same slot
    if (position->prePickup_ == position->preDrop_) {
        insertNode1(position->prePickup_, pickNode, removedLabels);
        pickLabel = position->prePickup_->child_;
        insertNode1(pickLabel, dropNode, removedLabels);
        dropLabel = pickLabel->child_;
    }
    else {
        insertNode1(position->prePickup_, pickNode, removedLabels);
        pickLabel = position->prePickup_->child_;
        insertNode1(position->preDrop_, dropNode, removedLabels);
        dropLabel = position->preDrop_->child_;
    }

    pickLabel->pair_ = dropLabel;
    dropLabel->pair_ = pickLabel;
    dropLabel->travelResource_ = maxDuration - (dropLabel->reachTime_ - pickLabel->departTime_);
    if (dropLabel->travelResource_ < 0)
        myTools::throwException("Negative travel resource");
}
// this function calculate the reachTime from a Label to a node
float LinkedGreedyLabels::labelToNodeReachTime(PGreedyLabel &preLabel, PNode &Node) {
    if ((Node->type_ == PICKUP)&&(preLabel->departTime_ <= Node->requestTime_))
        return Node->requestTime_ + durationMatrix_[preLabel->currentNode_->locationID_][Node->locationID_];
    else
        return preLabel->departTime_ + durationMatrix_[preLabel->currentNode_->locationID_][Node->locationID_];
}

/*float LinkedGreedyLabels::labelToNodeReachTime1(PGreedyLabel &preLabel, PNode &Node) {
    if ((preLabel->currentNode_->locationID_ == Node->locationID_)&&(preLabel->currentNode_->type_ != SOURCE)){
        if ((Node->type_ == PICKUP) && (preLabel->reachTime_<= Node->requestTime_))
            return Node->requestTime_;
        else
            return preLabel->reachTime_;
    }
    else {
        if ((Node->type_ == PICKUP) && (preLabel->departTime_<= Node->requestTime_))
            return Node->requestTime_ + durationMatrix_[preLabel->currentNode_->locationID_][Node->locationID_];
        else
            return preLabel->departTime_ + durationMatrix_[preLabel->currentNode_->locationID_][Node->locationID_];
    }
}*/

// this function calculate the reachTime from a node to a Label
float LinkedGreedyLabels::nodeToLabelReachTime(float nodeReachTime, PNode &preNode, PGreedyLabel &nextLabel) {
    if ((nextLabel->currentNode_->type_ == PICKUP) && (nodeReachTime + preNode->deltaTime_ < nextLabel->currentNode_->requestTime_))
        return nextLabel->currentNode_->requestTime_ + durationMatrix_[preNode->locationID_][nextLabel->currentNode_->locationID_];
    else
        return nodeReachTime + preNode->deltaTime_ +
                durationMatrix_[preNode->locationID_][nextLabel->currentNode_->locationID_];
}

float LinkedGreedyLabels::nodeToLabelReachTime1(float nodeReachTime, PNode &preNode, PGreedyLabel &nextLabel) {
    if (durationMatrix_[preNode->locationID_][nextLabel->currentNode_->locationID_] == 0){
        if ((nextLabel->currentNode_->type_ == PICKUP) && (nodeReachTime < nextLabel->currentNode_->requestTime_))
            return nextLabel->currentNode_->requestTime_;
        else
            return nodeReachTime;
    }
    else{
        if ((nextLabel->currentNode_->type_ == PICKUP) && (nodeReachTime + preNode->deltaTime_ < nextLabel->currentNode_->requestTime_))
            return nextLabel->currentNode_->requestTime_ + durationMatrix_[preNode->locationID_][nextLabel->currentNode_->locationID_];
        else
            return nodeReachTime + preNode->deltaTime_ +
                   durationMatrix_[preNode->locationID_][nextLabel->currentNode_->locationID_];
    }
}

// this function starts from a label in the list and update reachTimes and departTimes afterwards to the tail
void LinkedGreedyLabels::updateReachTimes(PGreedyLabel &preLabel) {
    PGreedyLabel currentLabel = preLabel;
    while (currentLabel->child_ != nullptr) {
        // calculate child reach time
        float childReachTime = labelToNodeReachTime(currentLabel, currentLabel->child_->currentNode_);
        if (currentLabel->currentNode_->type_ != SOURCE){
            if (currentLabel->child_->currentNode_->type_ == PICKUP){
                if (currentLabel->departTime_ < currentLabel->child_->currentNode_->requestTime_)
                    currentLabel->departTime_ = currentLabel->child_->currentNode_->requestTime_;
            }
        }
        // set current stop depart time
        /*if (currentLabel->currentNode_->type_ != SOURCE){
            if (currentLabel->child_->currentNode_->type_ == PICKUP)
                currentLabel->departTime_ = std::max(currentLabel->child_->currentNode_->requestTime_,
                                                     currentLabel->reachTime_ + currentLabel->currentNode_->deltaTime_);
            else
                currentLabel->departTime_ = currentLabel->reachTime_ + currentLabel->currentNode_->deltaTime_;
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
                                                        (childReachTime - currentLabel->child_->pair_->departTime_);
            }
        }
        currentLabel->child_->reachTime_ = childReachTime;
        currentLabel->child_->departTime_ = childReachTime + currentLabel->child_->currentNode_->deltaTime_;
        currentLabel->child_->nbPassengers_ = currentLabel->nbPassengers_ + currentLabel->child_->currentNode_->nbPassengers_;

        currentLabel = currentLabel->child_;
    }
    if (totalDelay_ < 0 ) {
 //       myTools::throwException("Delay Error");
        std::cout << toString() << std::endl;
    }
}
void LinkedGreedyLabels::updateReachTimes1(PGreedyLabel &preLabel) {
    PGreedyLabel currentLabel = preLabel;
    while (currentLabel->child_ != nullptr) {
        // calculate child reach time
        float childReachTime = labelToNodeReachTime(currentLabel, currentLabel->child_->currentNode_);

        // set current stop depart time
        if (currentLabel->currentNode_->type_ != SOURCE){
            if (currentLabel->child_->currentNode_->type_ == PICKUP){
                if (currentLabel->departTime_ < currentLabel->child_->currentNode_->requestTime_)
                    currentLabel->departTime_ = currentLabel->child_->currentNode_->requestTime_;
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
                        (childReachTime - currentLabel->child_->pair_->departTime_);
            }
        }

        currentLabel->child_->reachTime_ = childReachTime;
        if ((durationMatrix_[currentLabel->currentNode_->locationID_][currentLabel->child_->currentNode_->locationID_] == 0)&&
            (currentLabel->currentNode_->initialType_ != SOURCE))
            currentLabel->child_->departTime_ = childReachTime;
        else
            currentLabel->child_->departTime_ = childReachTime + currentLabel->child_->currentNode_->deltaTime_;

        currentLabel->child_->nbPassengers_ = currentLabel->nbPassengers_ + currentLabel->child_->currentNode_->nbPassengers_;

        currentLabel = currentLabel->child_;
    }
    if (totalDelay_ < 0 ) {
 //       myTools::throwException("Delay Error");
        std::cout << toString() << std::endl;
    }
}

// this function convert a greedyLabel list to a route
PRoute LinkedGreedyLabels::greedyLabelToRoute(bool update) const {
    /*std::cout << "GreedyLinkList before converting to the route" << std::endl;
    std::cout << toString() << std::endl;*/
    PRoute newRoute = std::make_shared<Route>((*Vehicle_)->vehicleID_);
    newRoute->addSource(source_->currentNode_, source_->departTime_, (*Vehicle_)->numPassengers_);
    PGreedyLabel currentLabel = source_->child_;
    while (currentLabel != nullptr) {
//        newRoute->addNode(currentLabel->currentNode_, currentLabel->reachTime_);
        newRoute->addNode1(currentLabel->currentNode_);
        if (newRoute->plannedReachTime_.back() != currentLabel->reachTime_) {
            std::cout << "Connectivity constraint violated at node : ";
            std::cout << newRoute->routeNodes_.back()->nodeID_ << std::endl;
 //           myTools::throwException("Route-Validation");
        }

        if (currentLabel->parent_->departTime_ < currentLabel->currentNode_->requestTime_) {
            std::cout << "Depart time violated at node : ";
            std::cout << currentLabel->parent_->currentNode_->nodeID_ << std::endl;
 //           myTools::throwException("Route-Validation");
        }
        if (update) {
//            newRoute->routeNodes_.back()->reachTime_ = currentLabel->reachTime_;
            newRoute->routeNodes_.back()->related_Request_->vehicleID_ = newRoute->vehicleID_;
            newRoute->routeNodes_.back()->nodeStatus_ = DONE;
            newRoute->routeNodes_.back()->reachTime_ = newRoute->plannedReachTime_.back();
            /*if (currentLabel->child_ == nullptr)
                newRoute->routeNodes_.back()->departTime_ = newRoute->routeNodes_.back()->reachTime_+
                        newRoute->routeNodes_.back()->deltaTime_;
            else*/
                newRoute->routeNodes_.back()->departTime_ = currentLabel->departTime_;
            if (newRoute->routeNodes_.back()->type_ == PICKUP)
                newRoute->routeNodes_.back()->related_Request_->pickTime_ = currentLabel->reachTime_;
            else if (newRoute->routeNodes_.back()->type_ == DROPOFF) {
                newRoute->routeNodes_.back()->related_Request_->dropTime_ = currentLabel->reachTime_;
                newRoute->routeNodes_.back()->related_Request_->requestStatus_ = COMPLETED;
            }
        }
        currentLabel = currentLabel->child_;
    }
    /*std::cout << "Created route from the GreedyLinkList" << std::endl;
    std::cout << newRoute->toString() << std::endl;*/
    return newRoute;
}

std::string LinkedGreedyLabels::toString() const {
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
    repStr << std::left << std::setw(11) << source_->currentNode_->nodeID_;
    repStr << std::right << std::setw(11) << source_->reachTime_ << " (s)  ";
    repStr << std::right << std::setw(11) << source_->departTime_ << " (s)  ";
    repStr << std::right << std::setw(11) << source_->travelResource_ << " (s)  ";
    repStr << std::setw(7) << source_->nbPassengers_ << std::endl;

    // print the internal nodes of the route
    PGreedyLabel curr = source_;
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
        repStr << std::right << std::setw(11) << curr->child_->departTime_ << " (s)  ";
        repStr << std::right << std::setw(11) << curr->child_->travelResource_ << " (s)  ";
        repStr << std::setw(7) << curr->child_->nbPassengers_ << std::endl;
        curr = curr->child_;
        counter++;
    }

    repStr << "=================================================================================================================" << std::endl;
    return repStr.str();
}

void LinkedGreedyLabels::findAssignedPlace(PNode &pickNode, PNode &dropNode, float maxDuration,
                                           vector<PGreedyLabel> &removedLabels, PInsertPosition &position) {
    float deltaDelay = INFINITY;
    bool notFound = true;

    // Tour length increase by adding pickup
    float pickDeltaT = INFINITY;

    // Tour length increase by adding drop off
    float dropDeltaT = INFINITY;
    position->updatePosition(tail_,tail_, deltaDelay, pickDeltaT + dropDeltaT);

    // define the initial position to add the request just after all
    PGreedyLabel prePick = head_;
    PGreedyLabel preDrop = head_;
    while ((prePick != nullptr) && (notFound)) {
        deltaDelay = INFINITY;
        if (prePick == tail_) {
            // it stays at tail and then departs to the pickup point
            float pickTime = labelToNodeReachTime(prePick, pickNode);
            deltaDelay =  pickTime - pickNode->requestTime_;
            pickDeltaT = pickTime - departureTime_;
            preDrop = tail_;
            if (deltaDelay  < position->deltaDelay_) {
                position->updatePosition(prePick, preDrop, deltaDelay,pickDeltaT);
            }
        }
        else {
            //first we check to see is it possible to insert pickup after prePick
            if (isInsertPossible(prePick, pickNode)) {
                // we have to find a place to insert drop point
                // first we make a copy of the list and insert the pickup
                float endTime = tail_->reachTime_;
                float curDelay = totalDelay_;

                insertNode1(prePick,pickNode, removedLabels);
                deltaDelay =  totalDelay_ - curDelay;
                pickDeltaT = tail_->reachTime_ - endTime;
                preDrop = prePick->child_;
                PGreedyLabel pickLabel = prePick->child_;

                // we have to check the vehicle capacity violation, if there is one, it should be dropped before that
                PGreedyLabel endLabel = nullptr;
                PGreedyLabel currentIndex = pickLabel;
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
            }
        }
        prePick = prePick->child_;
    }
}

void LinkedGreedyLabels::updateTailDepart() {
    if (tail_->currentNode_->type_!= SOURCE)
        tail_->departTime_ = tail_->reachTime_ + tail_->currentNode_->deltaTime_;
    departureTime_ = tail_->departTime_;
}

void LinkedGreedyLabels::updateTailDepart1() {
    if (tail_->currentNode_->type_!= SOURCE) {
        if (durationMatrix_[tail_->parent_->currentNode_->locationID_][tail_->currentNode_->locationID_] == 0)
            tail_->departTime_ = tail_->reachTime_;
        else
            tail_->departTime_ = tail_->reachTime_ + tail_->currentNode_->deltaTime_;
    }
    departureTime_ = tail_->departTime_;
}

bool LinkedGreedyLabels::isTimeViolated(PGreedyLabel &startLabel) {
    if (startLabel->nbPassengers_ > (*Vehicle_)->capacity_)
        return true;
    PGreedyLabel currentLabel = startLabel;
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

void LinkedGreedyLabels::findAssignedPlace1(PNode &pickNode, PNode &dropNode, float maxDuration,
                                            vector<PGreedyLabel> &removedLabels, PInsertPosition &position) {
    float deltaDelay = INFINITY;
    bool notFound = true;
    // Tour length increase
    float DeltaT = INFINITY;

    position->updatePosition(tail_,tail_, deltaDelay, DeltaT);

    // define the initial position to add the request just after all
    PGreedyLabel prePick = head_;
    PGreedyLabel preDrop = head_;
    while ((prePick != nullptr) && (notFound)) {
        deltaDelay = INFINITY;
        if (prePick == tail_) {
            if (prePick->nbPassengers_ + pickNode->nbPassengers_ > (*Vehicle_)->capacity_){
                std::cout << "Instance error! split the trip!" << std::endl;
                myTools::throwException("instance definition error");
            }
            // it stays at tail and then departs to the pickup point
            float pickTime = labelToNodeReachTime(prePick, pickNode);
            deltaDelay =  pickTime - pickNode->requestTime_;
            DeltaT = (pickTime - departureTime_) +
                     (pickNode->deltaTime_ + durationMatrix_[pickNode->locationID_][dropNode->locationID_]);
            preDrop = tail_;
            if (deltaDelay  < position->deltaDelay_) {
                if (DeltaT < position->deltaLength_)
                    position->updatePosition(prePick, preDrop, deltaDelay,DeltaT);
            }
        }
        else {
            float endTime = tail_->departTime_;
            float curDelay = totalDelay_;

            insertNode1(prePick, pickNode, removedLabels);
            PGreedyLabel pickLabel = prePick->child_;
            if (!isTimeViolated(prePick->child_)){
                preDrop = prePick->child_;
                PGreedyLabel endLabel = nullptr;
                PGreedyLabel currentIndex = pickLabel;
                while (currentIndex->child_ != nullptr) {
                    if (currentIndex->child_->nbPassengers_ > (*Vehicle_)->capacity_) {
                        endLabel = currentIndex->child_;
                        break;
                    } else
                        currentIndex = currentIndex->child_;
                }
                while (preDrop != endLabel) {
                    insertNode1(preDrop, dropNode, removedLabels);
                    PGreedyLabel dropLabel = preDrop->child_;
                    dropLabel->travelResource_ = maxDuration - (dropLabel->reachTime_ - pickLabel->departTime_);
                    if (!isTimeViolated(prePick->child_)){
                        deltaDelay = totalDelay_ - curDelay;
                        DeltaT = tail_->departTime_ - endTime;

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
insertPosition::insertPosition(PGreedyLabel prePickup, PGreedyLabel preDrop, float deltaDelay,
                               float deltaLength) : prePickup_(std::move(prePickup)), preDrop_(std::move(preDrop)), deltaDelay_(deltaDelay),
                                                    deltaLength_(deltaLength) {}

void insertPosition::updatePosition(const PGreedyLabel &prePickup, const PGreedyLabel &preDrop, float deltaDelay,
                                    float deltaLength) {
    prePickup_ = prePickup;
    preDrop_ = preDrop;
    deltaDelay_ = deltaDelay;
    deltaLength_ = deltaLength;
}

