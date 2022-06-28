//
// Created by Ella on 5/20/2022.
//

#include "Greedy.h"

#include <utility>

GreedyLabel::GreedyLabel(PNode currentNode, float reachTime, int nbPassengers) : currentNode_(std::move(currentNode)),
                                                                                        reachTime_(reachTime),
                                                                                        nbPassengers_(nbPassengers) {
    parent_ = nullptr;
    child_ = nullptr;
    pair_ = nullptr;
    travelResource_ = 0;
    departTime_ = reachTime;
}


LinkedGreedyLabels::LinkedGreedyLabels(PVehicle &vehicle, PInstance &pInst) : Vehicle_(&vehicle)  {
    source_ = std::make_shared<GreedyLabel>(pInst->instGraph_->nodes_[(*Vehicle_)->departID_], vehicle->departTime_, vehicle->onboards_.size());
    head_ = source_;
    tail_ = source_;
    totalDelay_ = 0;
    idleTime_ = 0;
    departTime_ = vehicle->departTime_;
    for (auto &nodeID: (*Vehicle_)->onboards_) {
        float dropTime = tail_->reachTime_ + tail_->currentNode_->deltaTime_ +
                         durationMatrix_[tail_->currentNode_->locationID_][pInst->instGraph_->nodes_[nodeID]->locationID_];
        PGreedyLabel newDropLabel = std::make_shared<GreedyLabel>(pInst->instGraph_->nodes_[nodeID], dropTime,
                                                                  tail_->nbPassengers_ +
                                                                  pInst->instGraph_->nodes_[nodeID]->nbPassengers_);
        newDropLabel->parent_ = tail_;
        tail_->child_ = newDropLabel;

        newDropLabel->travelResource_ = pInst->instGraph_->nodes_[nodeID]->related_Request_->maxTravelTime_ -
                dropTime + pInst->instGraph_->nodes_[nodeID]->related_Request_->pickTime_ +
                pInst->instGraph_->nodes_[nodeID]->deltaTime_;

        tail_ = newDropLabel;
    }
}

LinkedGreedyLabels::LinkedGreedyLabels(const LinkedGreedyLabels &label) {
    Vehicle_ = label.Vehicle_;
    departTime_ = label.departTime_;
    idleTime_ = label.idleTime_;
    head_ = label.head_;
    tail_ = label.tail_;
    source_ = label.source_;
    totalDelay_ = label.totalDelay_;

}

// this function find a position to insert pickup point and add drop off point at the end
PGreedyLabel LinkedGreedyLabels::findInsertPosition(PNode &pickNode, PNode &dropNode, float maxDuration) const {
    PGreedyLabel curLabel = head_;
    // find a position based on the available vehicle capacity
    while (curLabel->child_ != nullptr) {
        if (curLabel->nbPassengers_ + pickNode->nbPassengers_ > (*Vehicle_)->capacity_)
            curLabel = curLabel->child_;
        else
            break;
    }
    // find a position based on restriction of travel duration
    PGreedyLabel dropLabel = curLabel;
    while (dropLabel->child_ != nullptr) {
        float pickTime;
        if (curLabel->reachTime_ + curLabel->currentNode_->deltaTime_ < pickNode->requestTime_)
            pickTime = departTime_ + durationMatrix_[curLabel->currentNode_->locationID_][pickNode->locationID_];
        else
            pickTime = curLabel->reachTime_ + curLabel->currentNode_->deltaTime_ +
                    durationMatrix_[curLabel->currentNode_->locationID_][pickNode->locationID_];

        float deltaT = pickTime + pickNode->deltaTime_ +
                       durationMatrix_[pickNode->locationID_][curLabel->child_->currentNode_->locationID_] -
                       curLabel->child_->reachTime_;

        if (dropLabel->child_->travelResource_ < deltaT)
        {
            curLabel = dropLabel->child_;
            dropLabel = dropLabel->child_;
        }
        else
            dropLabel = dropLabel->child_;
    }
    // is it possible to drop it afterwards
    while (true) {
        if (curLabel->child_ != nullptr) {
            float pickTime;
            if (curLabel->reachTime_ + curLabel->currentNode_->deltaTime_ < pickNode->requestTime_)
                pickTime = departTime_ + durationMatrix_[curLabel->currentNode_->locationID_][pickNode->locationID_];
            else
                pickTime = curLabel->reachTime_ + curLabel->currentNode_->deltaTime_ +
                           durationMatrix_[curLabel->currentNode_->locationID_][pickNode->locationID_];
            float deltaT = pickTime + pickNode->deltaTime_ +
                           durationMatrix_[pickNode->locationID_][curLabel->child_->currentNode_->locationID_] -
                           curLabel->child_->reachTime_;

            float dropTime = tail_->reachTime_ + deltaT + tail_->currentNode_->deltaTime_ +
                             durationMatrix_[tail_->currentNode_->locationID_][dropNode->locationID_];
            if (dropTime - pickTime > maxDuration)
                curLabel = curLabel->child_;
            else
                break;
        }
        else
            break;
    }
    return curLabel;
}

void LinkedGreedyLabels::insertRequest(PGreedyLabel &preLabel, PNode &pickNode, PNode &dropNode, float maxDuration) {
    float pickTime = 0;
    if (preLabel->reachTime_ + preLabel->currentNode_->deltaTime_ < pickNode->requestTime_)
        pickTime = departTime_ + durationMatrix_[preLabel->currentNode_->locationID_][pickNode->locationID_];
    else
        pickTime = preLabel->reachTime_ + preLabel->currentNode_->deltaTime_ +
                   durationMatrix_[preLabel->currentNode_->locationID_][pickNode->locationID_];
    PGreedyLabel newPickLabel = std::make_shared<GreedyLabel>(pickNode, pickTime,
                                                                preLabel->nbPassengers_ + pickNode->nbPassengers_);
    if (preLabel->child_ == nullptr){
        newPickLabel->parent_ = preLabel;
        preLabel->child_= newPickLabel;

        float dropTime = pickTime + pickNode->deltaTime_ +
                         durationMatrix_[pickNode->locationID_][dropNode->locationID_];
        PGreedyLabel newDropLabel = std::make_shared<GreedyLabel>(dropNode, dropTime,
                                                                  newPickLabel->nbPassengers_ +
                                                                  dropNode->nbPassengers_);
        head_ = newPickLabel;
        tail_ = newDropLabel;
        newPickLabel->child_ = newDropLabel;
        newDropLabel->parent_ = newPickLabel;
        newPickLabel->pair_ = newDropLabel;
        newDropLabel->pair_ = newPickLabel;
        newDropLabel->travelResource_ = maxDuration - newDropLabel->reachTime_ + newPickLabel->reachTime_ + pickNode->deltaTime_;
    }
    else {
        float deltaT = pickTime + pickNode->deltaTime_ +
                durationMatrix_[pickNode->locationID_][preLabel->child_->currentNode_->locationID_] -
                preLabel->child_->reachTime_;
        preLabel->child_->parent_ = newPickLabel;
        newPickLabel->child_ = preLabel->child_;
        newPickLabel->parent_ = preLabel;
        preLabel->child_= newPickLabel;

        PGreedyLabel currentLabel = newPickLabel;
        while (currentLabel->child_ != nullptr) {
            currentLabel->child_->reachTime_ += deltaT;
            currentLabel->child_->nbPassengers_ += pickNode->nbPassengers_;
            currentLabel->child_->travelResource_ -= deltaT;
            currentLabel = currentLabel->child_;
        }
        float dropTime = tail_->reachTime_ + tail_->currentNode_->deltaTime_ +
                         durationMatrix_[tail_->currentNode_->locationID_][dropNode->locationID_];
        PGreedyLabel newDropLabel = std::make_shared<GreedyLabel>(dropNode, dropTime,
                                                                  tail_->nbPassengers_ +
                                                                  dropNode->nbPassengers_);
        head_ = newPickLabel;
        tail_->child_ = newDropLabel;
        newDropLabel->parent_ = tail_;
        tail_ = newDropLabel;
        newPickLabel->pair_ = newDropLabel;
        newDropLabel->pair_ = newPickLabel;
        newDropLabel->travelResource_ = maxDuration - newDropLabel->reachTime_ + newPickLabel->reachTime_ + pickNode->deltaTime_;
    }
    departTime_ = head_->reachTime_ + head_->currentNode_->deltaTime_;
    totalDelay_ += (pickTime - pickNode->requestTime_);
}

/********************* New approach **********************/

bool LinkedGreedyLabels::isInsertPossible(PGreedyLabel &preLabel, PNode &newNode) const {
    if (preLabel == tail_)
        return true;
    // check the capacity
    if (preLabel->nbPassengers_ + newNode->nbPassengers_ > (*Vehicle_)->capacity_)
        return false;

    float reachTime = preLabel->reachTime_ + preLabel->currentNode_->deltaTime_ +
                      durationMatrix_[preLabel->currentNode_->locationID_][newNode->locationID_];
    float deltaT = reachTime + newNode->deltaTime_ +
                   durationMatrix_[newNode->locationID_][preLabel->child_->currentNode_->locationID_] -
                   preLabel->child_->reachTime_;

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
        float dropTime = preDrop->reachTime_ + preDrop->currentNode_->deltaTime_ +
                         durationMatrix_[preDrop->currentNode_->locationID_][dropNode->locationID_];
        if (dropTime - (pickLabel->reachTime_ + pickLabel->currentNode_->deltaTime_) > maxDuration)
            return false;
    }
    return true;
}

PInsertPosition LinkedGreedyLabels::findInsertPlace(PNode &pickNode, PNode &dropNode, float maxDuration) {
    float deltaDelay = 999999;

    // Tour length increase by adding pickup
    float pickDeltaT = 999999;

    // Tour length increase by adding drop off
    float dropDeltaT = 999999;
    PInsertPosition position = std::make_shared<insertPosition>(tail_,tail_, deltaDelay, pickDeltaT + dropDeltaT);

    // define the initial position to add the request just after all
    PGreedyLabel prePick = head_;
    PGreedyLabel preDrop = head_;
    while (prePick != nullptr) {
        deltaDelay = 999999;
        if (prePick == tail_) {
            // it stays at tail and then departs to the pickup point
            float pickTime;
            if (prePick->reachTime_ + prePick->currentNode_->deltaTime_ < pickNode->requestTime_)
                pickTime = departTime_ + durationMatrix_[prePick->currentNode_->locationID_][pickNode->locationID_];
            else
                pickTime = prePick->reachTime_ + prePick->currentNode_->deltaTime_ + durationMatrix_[prePick->currentNode_->locationID_][pickNode->locationID_];
            deltaDelay =  pickTime - pickNode->requestTime_;
            pickDeltaT = pickTime - departTime_;
            dropDeltaT = pickNode->deltaTime_ + durationMatrix_[pickNode->locationID_][dropNode->locationID_];
            preDrop = tail_;
            if (deltaDelay  < position->deltaDelay_) {
                if (pickDeltaT + dropDeltaT < position->deltaLength_)
                    position->updatePosition(prePick, preDrop, deltaDelay,pickDeltaT + dropDeltaT);
            }
        }
        else {
            //first we check to see is it possible to insert pickup after prePick
            if (isInsertPossible(prePick, pickNode)) {
                // we have to find a place to insert drop point
                // first we make a copy of the list and insert the pickup
                float endTime = tail_->reachTime_;
                float curDelay = totalDelay_;
                std::cout << "GreedyLinkList before checking" << std::endl;
                std::cout << toString() << std::endl;
                insertNode(prePick,pickNode);
                deltaDelay =  totalDelay_ - curDelay;
                pickDeltaT = tail_->reachTime_ - endTime;
                preDrop = prePick->child_;
                PGreedyLabel pickLabel = prePick->child_;
                while (preDrop != nullptr) {
                    float increaseDelayByDrop = 0;
                    if (isDropPossible(preDrop, pickLabel, dropNode, maxDuration)) {
                        if (preDrop == tail_)
                            dropDeltaT = preDrop->currentNode_->deltaTime_ +
                                         durationMatrix_[preDrop->currentNode_->locationID_][dropNode->locationID_];
                        else {
                            dropDeltaT = preDrop->reachTime_ + preDrop->currentNode_->deltaTime_ + dropNode->deltaTime_ +
                                         durationMatrix_[preDrop->currentNode_->locationID_][dropNode->locationID_] +
                                         durationMatrix_[dropNode->locationID_][preDrop->child_->currentNode_->locationID_] -
                                         preDrop->child_->reachTime_;
                            PGreedyLabel currentLabel = preDrop;

                            while (currentLabel->child_ != nullptr) {
                                if (preDrop->child_->currentNode_->type_ == PICKUP)
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
                        }
                        else if (deltaDelay + increaseDelayByDrop == position->deltaDelay_) {
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
                std::cout << "GreedyLinkList after checking" << std::endl;
                std::cout << toString() << std::endl;
                removeLabel(pickLabel, pickDeltaT, deltaDelay);
                std::cout << "GreedyLinkList after deletion" << std::endl;
                std::cout << toString() << std::endl;
            }
        }
        prePick = prePick->child_;
    }
    return position;
}
void LinkedGreedyLabels::insertNode(PGreedyLabel &preLabel, PNode &newNode) {
    float reachTime = calculateReachTime(preLabel, newNode);
    PGreedyLabel newLabel = std::make_shared<GreedyLabel>(newNode, reachTime, preLabel->nbPassengers_ +
                                                                              newNode->nbPassengers_);
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

        PGreedyLabel currentLabel = newLabel;
        while (currentLabel->child_ != nullptr) {
            // calculate child reach time
            float childReachTime = calculateReachTime(currentLabel, currentLabel->child_->currentNode_);
            float childDeltaT = childReachTime - currentLabel->child_->reachTime_;
            currentLabel->child_->reachTime_ = childReachTime;
            currentLabel->child_->nbPassengers_ += newNode->nbPassengers_;
            if (currentLabel->child_->currentNode_->type_ == DROPOFF) {
                if ((currentLabel->child_->pair_ == nullptr) ||(currentLabel->child_->pair_->reachTime_ <= preLabel->reachTime_))
                    currentLabel->child_->travelResource_ -= childDeltaT;
            }
            if (currentLabel->child_->currentNode_->type_ == PICKUP)
                totalDelay_ += childDeltaT;
            currentLabel = currentLabel->child_;
        }
    }
    if (newNode->type_ == PICKUP)
        totalDelay_ += (reachTime - newNode->requestTime_);
}


void LinkedGreedyLabels::removeLabel(PGreedyLabel &Label, float deltaT, float deltaDelay) {
    totalDelay_ -= deltaDelay;
    if (Label->child_ == nullptr){
        Label->parent_->child_ = nullptr;
        tail_ = Label->parent_;
        Label.reset();
    }
    else {

        PGreedyLabel currentLabel = Label;
        while (currentLabel->child_ != nullptr) {
            currentLabel->child_->reachTime_ -= deltaT;
            currentLabel->child_->nbPassengers_ -= Label->currentNode_->nbPassengers_;
            if (currentLabel->child_->currentNode_->type_ == DROPOFF) {
                if ((currentLabel->child_->pair_ == nullptr) ||
                    (currentLabel->child_->pair_->reachTime_ <= Label->parent_->reachTime_))
                    currentLabel->child_->travelResource_ += deltaT;
            }
            currentLabel = currentLabel->child_;
        }
        Label->child_->parent_ = Label->parent_;
        Label->parent_->child_ = Label->child_;
        Label.reset();
    }
}

void LinkedGreedyLabels::insertRequest(PInsertPosition &position, PNode &pickNode, PNode &dropNode, float maxDuration) {
    PGreedyLabel pickLabel;
    PGreedyLabel dropLabel;
    // if pick up and drop off are inserted in the same slot
    if (position->prePickup_ == position->preDrop_) {
        insertNode(position->prePickup_, pickNode);
        pickLabel = position->prePickup_->child_;
        insertNode(pickLabel, dropNode);
        dropLabel = pickLabel->child_;
    }
    else {
        insertNode(position->prePickup_, pickNode);
        pickLabel = position->prePickup_->child_;
        insertNode(position->preDrop_, dropNode);
        dropLabel = position->preDrop_->child_;
    }

    pickLabel->pair_ = dropLabel;
    dropLabel->pair_ = pickLabel;
    dropLabel->travelResource_ = maxDuration - dropLabel->reachTime_ + pickLabel->reachTime_ + pickNode->deltaTime_;
    totalDelay_ += (pickLabel->reachTime_ - pickNode->requestTime_);
}

// this function calculate the reachTime from a Label to a node
float LinkedGreedyLabels::calculateReachTime(PGreedyLabel &preLabel, PNode &Node) {
    if ((preLabel->reachTime_ + preLabel->currentNode_->deltaTime_ < Node->requestTime_) && (Node->type_ == PICKUP))
        return Node->requestTime_ + durationMatrix_[preLabel->currentNode_->locationID_][Node->locationID_];
    else
        return preLabel->reachTime_ + preLabel->currentNode_->deltaTime_ +
                         durationMatrix_[preLabel->currentNode_->locationID_][Node->locationID_];
}


// this function convert a greedyLabel list to a route
PRoute LinkedGreedyLabels::greedyLabelToRoute() const {
    PRoute newRoute = std::make_shared<Route>((*Vehicle_)->vehicleID_);
    newRoute->addSource(source_->currentNode_, source_->reachTime_, (*Vehicle_)->numPassengers_);
    PGreedyLabel  currentLabel = source_->child_;
    while (currentLabel != nullptr) {
        newRoute->addNode(currentLabel->currentNode_, currentLabel->reachTime_);
        newRoute->routeNodes_.back()->reachTime_ = currentLabel->reachTime_;
        newRoute->routeNodes_.back()->departTime_ = currentLabel->departTime_;
        currentLabel = currentLabel->child_;
    }
    return newRoute;
}

std::string LinkedGreedyLabels::toString() const {
    std::stringstream repStr;

    repStr << "#" << std::left << std::endl;
    repStr << "#\t" << std::setw(24) << "- VEHICLE_ID" << " : " << (*Vehicle_)->vehicleID_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- TOTAL_WAITING (seconds)" << " : " << totalDelay_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- IDLE_TIME (seconds)" << " : " << idleTime_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- DEPART_TIME (seconds)" << " : " << departTime_ << std::endl;
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
        repStr << std::right << std::setw(11) << curr->child_->departTime_ << " (s)  ";
        repStr << std::right << std::setw(11) << curr->child_->travelResource_ << " (s)  ";
        repStr << std::setw(7) << curr->child_->nbPassengers_ << std::endl;
        curr = curr->child_;
        counter++;
    }

    repStr << "=================================================================================================================" << std::endl;
    return repStr.str();
}




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
