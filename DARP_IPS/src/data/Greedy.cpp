//
// Created by Ella on 5/20/2022.
//

#include "Greedy.h"
#include "utilities/InputPaths.h"
#include <utility>
//---------------------------------------------------------------------------------------------
//  StopLabel class
//  for Greedy method I use linked list for and I linked StopLabel to create routes and insertions
//---------------------------------------------------------------------------------------------

StopLabel::StopLabel(PNode currentNode, float reachTime, float departTime, int nbPassengers) :
        currentNode_(std::move(currentNode)), reachTime_(reachTime), leaveTime_(departTime),
        nbPassengers_(nbPassengers) {
    parent_ = nullptr;
    child_ = nullptr;
    pair_ = nullptr;
    travelResource_ = 0;
}

void StopLabel::setValues(PNode currentNode, float reachTime, float departTime, int nbPassengers) {
    currentNode_ = std::move(currentNode);
    reachTime_ = reachTime;
    nbPassengers_ = nbPassengers;
    parent_ = nullptr;
    child_ = nullptr;
    pair_ = nullptr;
    travelResource_ = 0;
    leaveTime_ = departTime;
}

//---------------------------------------------------------------------------------------------
//  GreedyRoute class
//  This class is used to define greedy labels as linked list
//---------------------------------------------------------------------------------------------

GreedyRoute::GreedyRoute(PVehicle &vehicle, const PInstance &pInst, std::vector<PStopLabel> &greedyLabelPool, bool greedyReOptimize) :
        Vehicle_(&vehicle) {
    if (greedyLabelPool.empty())
        initialStop_ = std::make_shared<StopLabel>((*Vehicle_)->departNode_,
                                                    (*Vehicle_)->departNode_->reachTime_,
                                                    vehicle->departTime_,
                                                    vehicle->numPassengers_);
    else {
        initialStop_ = greedyLabelPool.back();
        greedyLabelPool.pop_back();
        initialStop_->setValues((*Vehicle_)->departNode_, (*Vehicle_)->departNode_->reachTime_,
                                 vehicle->departTime_, vehicle->numPassengers_);
    }
    initialDepartStop_ = initialStop_;
    lastStop_ = initialStop_;
    totalWait_ = 0.0;
    totalTripDelay_ = 0.0;
    idleTime_ = 0;
    departureTime_ = vehicle->departTime_;
    // just onboards are added and the previous solution is not considered
    if (greedyReOptimize) {
        for (auto &nodeID: (*Vehicle_)->onboards_) {
            PNode onboardNode = pInst->instGraph_->nodes_[nodeID];
            float dropTime = labelToNodeReachTime(lastStop_, onboardNode);
            PStopLabel newDropLabel;
            if (greedyLabelPool.empty()) {
                newDropLabel = std::make_shared<StopLabel>(onboardNode, dropTime,
                                                           dropTime + onboardNode->serviceTime_,
                                                           lastStop_->nbPassengers_ + onboardNode->nbPassengers_);
            } else {
                newDropLabel = greedyLabelPool.back();
                greedyLabelPool.pop_back();
                newDropLabel->setValues(onboardNode, dropTime,
                                        dropTime + onboardNode->serviceTime_, lastStop_->nbPassengers_ +
                                                                              onboardNode->nbPassengers_);
            }
            newDropLabel->parent_ = lastStop_;
            lastStop_->child_ = newDropLabel;
            newDropLabel->travelResource_ = onboardNode->related_Request_->maxTravelTime_ - dropTime +
                                            onboardNode->pairNode_->departTime_;

            lastStop_ = newDropLabel;
            float tripDelay = dropTime - onboardNode->pairNode_->departTime_ - onboardNode->related_Request_->minTravelTime_;
            totalTripDelay_ += onboardNode->related_Request_->Req_W3_ * tripDelay;
        }
    }
    // build the previous solution
    else {
        for (int i = 1; i < (*Vehicle_)->currentRoute_->routeNodes_.size(); i++) {
            PStopLabel newLabel;
            if (greedyLabelPool.empty()) {
                newLabel = std::make_shared<StopLabel>((*Vehicle_)->currentRoute_->routeNodes_[i],
                                                              (*Vehicle_)->currentRoute_->plannedReachTime_[i],
                                                              (*Vehicle_)->currentRoute_->plannedDepartTime_[i],
                                                              (*Vehicle_)->currentRoute_->plannedPassengers_[i]);
            } else {
                newLabel = greedyLabelPool.back();
                greedyLabelPool.pop_back();
                newLabel->setValues((*Vehicle_)->currentRoute_->routeNodes_[i],
                    (*Vehicle_)->currentRoute_->plannedReachTime_[i],
                    (*Vehicle_)->currentRoute_->plannedDepartTime_[i],
                    (*Vehicle_)->currentRoute_->plannedPassengers_[i]);
            }
            newLabel->parent_ = lastStop_;
            lastStop_->child_ = newLabel;
            lastStop_ = newLabel;

            if (newLabel->currentNode_->type_ == DROPOFF) {
                if (newLabel->currentNode_->related_Request_->requestStatus_ == NO_ACTION) {
                    PStopLabel currentLabel = lastStop_;
                    while (currentLabel->parent_ != nullptr) {
                        if (currentLabel->parent_->currentNode_->type_ == PICKUP &&
                            currentLabel->parent_->currentNode_->related_Request_->getRequestId() ==
                            newLabel->currentNode_->related_Request_->getRequestId()) {
                            newLabel->pair_ = currentLabel->parent_;
                            currentLabel->parent_->pair_ = newLabel;
                            newLabel->travelResource_ = newLabel->currentNode_->related_Request_->maxTravelTime_ -
                                                        (*Vehicle_)->currentRoute_->plannedReachTime_[i] +
                                                        currentLabel->parent_->leaveTime_;
                            float tripDelay = (*Vehicle_)->currentRoute_->plannedReachTime_[i] -
                                currentLabel->parent_->leaveTime_ - newLabel->currentNode_->related_Request_->minTravelTime_;
                            totalTripDelay_ +=  newLabel->currentNode_->related_Request_->Req_W3_ * tripDelay;
                            break;
                        }
                        currentLabel = currentLabel->parent_;
                    }
                } else {
                    newLabel->travelResource_ = newLabel->currentNode_->related_Request_->maxTravelTime_ -
                                                (*Vehicle_)->currentRoute_->plannedReachTime_[i] +
                                                (*Vehicle_)->currentRoute_->routeNodes_[i]->pairNode_->departTime_;
                    float tripDelay = (*Vehicle_)->currentRoute_->plannedReachTime_[i] -
                        (*Vehicle_)->currentRoute_->routeNodes_[i]->pairNode_->departTime_ -
                            newLabel->currentNode_->related_Request_->minTravelTime_;
                    totalTripDelay_ += newLabel->currentNode_->related_Request_->Req_W3_ * tripDelay;
                }
            }
            else if (newLabel->currentNode_->type_ == PICKUP)
                totalWait_ += newLabel->currentNode_->related_Request_->Req_W3_ * (newLabel->reachTime_ - newLabel->currentNode_->initialReadyTime_);
            else if (newLabel->currentNode_->type_ == SINK)
                initialDepartStop_ = newLabel;
        }
    }
}

GreedyRoute::GreedyRoute(const GreedyRoute &label) {
    Vehicle_ = label.Vehicle_;
    departureTime_ = label.departureTime_;
    idleTime_ = label.idleTime_;
    lastStop_ = label.lastStop_;
    initialStop_ = label.initialStop_;
    initialDepartStop_ = label.initialDepartStop_;
    totalWait_ = label.totalWait_;
    totalTripDelay_ = label.totalTripDelay_;
}

GreedyRoute::~GreedyRoute() = default;

std::string GreedyRoute::toString() const {
    std::stringstream repStr;

    repStr << "#" << std::left << std::endl;
    repStr << "#\t" << std::setw(24) << "- VEHICLE_ID" << " : " << (*Vehicle_)->vehicleID_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- TOTAL_WAITING (seconds)" << " : " << totalWait_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- TRIP_DELAY (seconds)" << " : " << totalTripDelay_ << std::endl;
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
    repStr << std::left << std::setw(11) << initialStop_->currentNode_->nodeID_;
    repStr << std::right << std::setw(11) << initialStop_->reachTime_ << " (s)  ";
    repStr << std::right << std::setw(11) << initialStop_->leaveTime_ << " (s)  ";
    repStr << std::right << std::setw(11) << initialStop_->travelResource_ << " (s)  ";
    repStr << std::setw(7) << initialStop_->nbPassengers_ << std::endl;

    // print the internal nodes of the route
    PStopLabel curr = initialStop_;
    int counter = 1;
    while (curr->child_ != nullptr) {
        repStr << "#" << std::setw(4) << counter + 1 << "  ";
        if (curr->child_->currentNode_->type_ == SINK)
            repStr << std::left << std::setw(27) << "(SINK   ) return";
        else {
            repStr << "(" << eu::toString(curr->child_->currentNode_->type_) << ") Request_ID ";
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

void GreedyRoute::resetGreedyRoute(std::vector<PStopLabel> &greedyLabelPool) const {
    PStopLabel currentLabel = initialStop_;
    while (currentLabel->child_ != nullptr) {
        greedyLabelPool.push_back(currentLabel);
        currentLabel = currentLabel->child_;
    }
}

// this function finds a position to insert a pickup point and add a drop off point
void GreedyRoute::findInsertPlace(PNode &pickNode, PNode &dropNode, float maxDuration,
                                  std::vector<PStopLabel> &greedyLabelPool, const PInsertPosition & position,
                                  float wait_W1, float ride_W2) {

    position->updatePosition(lastStop_, lastStop_, LARGE_CONSTANT, LARGE_CONSTANT, wait_W1, ride_W2);
    // Quick capacity check
    if (lastStop_->nbPassengers_ + pickNode->nbPassengers_ > (*Vehicle_)->capacity_) {
        throw myTools::myException("Instance error: capacity exceeded, consider splitting trip.", __FILE__, __LINE__);
    }

    // define the initial position to add the request just after all
    PStopLabel prePick = initialDepartStop_;
    while (prePick != nullptr) {
        // Skip if already infeasible by capacity
        if (prePick->nbPassengers_ + pickNode->nbPassengers_ > (*Vehicle_)->capacity_) {
            prePick = prePick->child_;
            continue;
        }

        if (prePick == lastStop_) {
            // it stays at the tail and then departs to the pickup point
            float pickTime = labelToNodeReachTime(prePick, pickNode);
            float waitIncrease = pickNode->related_Request_->Req_W3_ * (pickTime - pickNode->initialReadyTime_);
            // trip delay is not increased due to the direct drop
            float objIncrease = wait_W1 *  waitIncrease + ride_W2 * 0.0;
            if (objIncrease < position->deltaObjective_) {
                position->updatePosition(prePick, lastStop_, waitIncrease, 0.0, wait_W1, ride_W2);
            }
        }
        else {
            // Try insertion at this position
            tryInsertionAt(prePick, pickNode, dropNode, maxDuration, greedyLabelPool, position, wait_W1, ride_W2);
        }
        prePick = prePick->child_;
    }
}

PStopLabel GreedyRoute::findCapacityLimit(const PStopLabel &startLabel) const {
    PStopLabel current = startLabel;
    while (current->child_ != nullptr) {
        if (current->child_->nbPassengers_ > (*Vehicle_)->capacity_) {
            return current->child_;
        }
        current = current->child_;
    }
    return nullptr;
}

void GreedyRoute::tryInsertionAt(PStopLabel &prePick, PNode &pickNode, PNode &dropNode, float maxDuration,
    std::vector<PStopLabel> &greedyLabelPool, const PInsertPosition &position, float wait_W1, float ride_W2) {
    float baseDelay = totalWait_;
    float baseTripDelay = totalTripDelay_;
    PStopLabel emptyLabel;

    // Insert pickup
    insertNode(prePick, pickNode, greedyLabelPool, emptyLabel);
    PStopLabel pickLabel = prePick->child_;

    if (isAnyViolation(pickLabel)) {
        removeLabel(pickLabel, greedyLabelPool, emptyLabel);
        return;
    }

    // Find valid drop positions
    PStopLabel preDrop = pickLabel;
    PStopLabel capacityLimit = findCapacityLimit(pickLabel);

    while (preDrop != capacityLimit) {
        insertNode(preDrop, dropNode, greedyLabelPool, pickLabel);
        PStopLabel dropLabel = preDrop->child_;
        dropLabel->travelResource_ = maxDuration - (dropLabel->reachTime_ - pickLabel->leaveTime_);

        if (!isAnyViolation(pickLabel)) {
            float waitIncrease = totalWait_ - baseDelay;
            float tripDelayIncrease = totalTripDelay_ - baseTripDelay;
            float objIncrease = wait_W1 * waitIncrease + ride_W2 * tripDelayIncrease;

            // Compare objective increase
            if (objIncrease < position->deltaObjective_) {
                PStopLabel dropPos = (preDrop == pickLabel) ? prePick : preDrop;
                position->updatePosition(prePick, dropPos, waitIncrease,
                                       tripDelayIncrease, wait_W1, ride_W2);
            }
            else if (objIncrease == position->deltaObjective_) {
                if (tripDelayIncrease < position->deltaTripDelay_) {
                    PStopLabel dropPos = (preDrop == pickLabel) ? prePick : preDrop;
                    position->updatePosition(prePick, dropPos, waitIncrease,
                                           tripDelayIncrease, wait_W1, ride_W2);
                }
            }
        }

        removeLabel(dropLabel, greedyLabelPool, pickLabel);
        preDrop = preDrop->child_;
    }

    removeLabel(pickLabel, greedyLabelPool, emptyLabel);
}

void GreedyRoute::insertNode(const PStopLabel &preLabel, PNode &newNode, std::vector<PStopLabel> &greedyLabelPool, PStopLabel &pickLabel) {
    // calculate reach time
    float reachTime = labelToNodeReachTime(preLabel, newNode);
    // update depart time
    if (preLabel->currentNode_->type_ != SOURCE && newNode->type_ == PICKUP){
        if (preLabel->leaveTime_ < newNode->related_Request_->requestTime_)
            preLabel->leaveTime_ = newNode->related_Request_->requestTime_;
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
        lastStop_ = newLabel;
    }
    else {

        preLabel->child_->parent_ = newLabel;
        newLabel->child_ = preLabel->child_;
        newLabel->parent_ = preLabel;
        preLabel->child_= newLabel;

        updateReachTimes(newLabel);
    }
    // Update totalDelay for PICKUP nodes
    if (newNode->type_ == PICKUP){
        if (reachTime - newNode->related_Request_->earlyPick_ < 0 )
            throw myTools::myException("Negative waiting time encountered in insertNode.", __FILE__, __LINE__);

        totalWait_ += newNode->related_Request_->Req_W3_ * (reachTime - newNode->initialReadyTime_);
    }
    // Update totalTripDelay for DROPOFF nodes
    else if (newNode->type_ == DROPOFF) {
        float rideTime;
        if (pickLabel == nullptr) {
            // Paired with node in graph (not in route yet)
            rideTime = reachTime - newNode->pairNode_->departTime_;
        } else {
            // Paired with pickup label in route
            rideTime = reachTime - pickLabel->leaveTime_;
        }
        float tripDelay = rideTime - newNode->related_Request_->minTravelTime_;
        totalTripDelay_ += newNode->related_Request_->Req_W3_ * tripDelay;
    }
}

void GreedyRoute::removeLabel(PStopLabel &label, std::vector<PStopLabel> &greedyLabelPool, PStopLabel &pickLabel) {
    greedyLabelPool.push_back(label);
    if (label->currentNode_->type_ == PICKUP) {
        totalWait_ -= label->currentNode_->related_Request_->Req_W3_ * (label->reachTime_ - label->currentNode_->initialReadyTime_);
    }
    else if (label->currentNode_->type_ == DROPOFF) {
        float rideTime;
        if (pickLabel == nullptr) {
            rideTime = label->reachTime_ - label->currentNode_->pairNode_->departTime_;
        } else {
            rideTime = label->reachTime_ - pickLabel->leaveTime_;
        }
        float tripDelay = rideTime - label->currentNode_->related_Request_->minTravelTime_;
        totalTripDelay_ -= label->currentNode_->related_Request_->Req_W3_ * tripDelay;
    }
    if (label->child_ == nullptr){
        label->parent_->child_ = nullptr;
        lastStop_ = label->parent_;
        label.reset();
    }
    else {
        label->child_->parent_ = label->parent_;
        label->parent_->child_ = label->child_;
        updateReachTimes(label->parent_);
        label.reset();
    }
}

void GreedyRoute::insertRequest(const PInsertPosition &position, PNode &pickNode, PNode &dropNode, float maxDuration,
                                std::vector<PStopLabel> &greedyLabelPool) {
    PStopLabel pickLabel;
    PStopLabel dropLabel;
    PStopLabel emptyLabel;
    // if pick up and drop off are inserted in the same slot
    if (position->prePickup_ == position->preDrop_) {
        insertNode(position->prePickup_, pickNode, greedyLabelPool, emptyLabel);
        pickLabel = position->prePickup_->child_;
        insertNode(pickLabel, dropNode, greedyLabelPool, pickLabel);
        dropLabel = pickLabel->child_;
    }
    else {
        insertNode(position->prePickup_, pickNode, greedyLabelPool, emptyLabel);
        pickLabel = position->prePickup_->child_;
        insertNode(position->preDrop_, dropNode, greedyLabelPool, pickLabel);
        dropLabel = position->preDrop_->child_;
    }

    pickLabel->pair_ = dropLabel;
    dropLabel->pair_ = pickLabel;
    dropLabel->travelResource_ = maxDuration - (dropLabel->reachTime_ - pickLabel->leaveTime_);
    if (dropLabel->travelResource_ < 0)
        throw myTools::myException("Negative travel resource", __FILE__, __LINE__);
}
// this function calculates the reachTime from a Label to a node
float GreedyRoute::labelToNodeReachTime(const PStopLabel &preLabel, const PNode &Node) {
    float reachTime = std::max(preLabel->leaveTime_, Node->related_Request_->requestTime_) +
        durationMatrix_[preLabel->currentNode_->locationID_][Node->locationID_];
    if (Node->type_ == PICKUP && Node->related_Request_->earlyPick_ > reachTime)
        return Node->related_Request_->earlyPick_;
    return reachTime;
}


// this function starts from a label in the list and updates reachTimes and departTimes up to the tail
void GreedyRoute::updateReachTimes(const PStopLabel &preLabel) {
    PStopLabel currentLabel = preLabel;
    while (currentLabel->child_ != nullptr) {
        PStopLabel childLabel = currentLabel->child_;
        PNode childNode = currentLabel->child_->currentNode_;
        // calculate child reach time
        float childReachTime = labelToNodeReachTime(currentLabel, childNode);

        // update total delay
        if (childNode->type_ == PICKUP) {
            // Handle request time constraint for departures
            if (currentLabel->currentNode_->type_ != SOURCE) {
                if (currentLabel->leaveTime_ < childNode->related_Request_->requestTime_)
                    currentLabel->leaveTime_ = childNode->related_Request_->requestTime_;
            }

            float oldDelay = childLabel->reachTime_ - childNode->initialReadyTime_;
            float newDelay = childReachTime - childNode->initialReadyTime_;

            totalWait_ += childNode->related_Request_->Req_W3_ * (newDelay - oldDelay);
            if (totalWait_ < -0.001f ) {
                throw myTools::myException("Negative total delay after updating route.", __FILE__, __LINE__);
            }
        }
        else if (childNode->type_ == DROPOFF) {
            // Calculate old trip delay
            float oldRideTime;
            if (childLabel->pair_ == nullptr) {
                oldRideTime = childLabel->reachTime_ - childNode->pairNode_->departTime_;
            } else {
                oldRideTime = childLabel->reachTime_ - childLabel->pair_->leaveTime_;
            }
            float oldTripDelay = oldRideTime - childNode->related_Request_->minTravelTime_;

            // Calculate new trip delay
            float newRideTime;
            if (childLabel->pair_ == nullptr) {
                newRideTime = childReachTime - childNode->pairNode_->departTime_;
                childLabel->travelResource_ = childNode->related_Request_->maxTravelTime_ - newRideTime;
            } else {
                newRideTime = childReachTime - childLabel->pair_->leaveTime_;
                childLabel->travelResource_ = childNode->related_Request_->maxTravelTime_ - newRideTime;
            }
            float newTripDelay = newRideTime - childNode->related_Request_->minTravelTime_;

            // Update total with delta
            totalTripDelay_ += childNode->related_Request_->Req_W3_ * (newTripDelay - oldTripDelay);
        }
        childLabel->reachTime_ = childReachTime;
        childLabel->leaveTime_ = childReachTime + childNode->serviceTime_;
        childLabel->nbPassengers_ = currentLabel->nbPassengers_ + childNode->nbPassengers_;

        currentLabel = currentLabel->child_;
    }
}

// this function converts a greedyLabel list to a route
PRoute GreedyRoute::greedyLabelToRoute(bool update) const {
    PRoute newRoute = std::make_shared<Route>((*Vehicle_)->vehicleID_);
    newRoute->addSource(initialStop_->currentNode_, initialStop_->leaveTime_, (*Vehicle_)->numPassengers_);
    PStopLabel currentLabel = initialStop_->child_;
    while (currentLabel != nullptr) {
        newRoute->addNode(currentLabel->currentNode_);
        if (newRoute->plannedReachTime_.back() != currentLabel->reachTime_) {
            std::cout << "Connectivity constraint violated at node : ";
            std::cout << newRoute->routeNodes_.back()->nodeID_ << std::endl;
            //           myTools::throwException("Route-Validation");
        }

        if (currentLabel->currentNode_->type_ == PICKUP) {
            if (currentLabel->parent_->leaveTime_ < currentLabel->currentNode_->related_Request_->requestTime_) {
                std::cout << "Depart time violated at node : ";
                std::cout << currentLabel->parent_->currentNode_->nodeID_ << std::endl;
                //           myTools::throwException("Route-Validation");
            }
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
            newRoute->routeNodes_.back()->related_Request_->solVehicleID_ = newRoute->vehicleID_;
        }
        currentLabel = currentLabel->child_;
    }

    if (newRoute->totalWait_ != totalWait_){
        std::cout << "Total delay of the greedy solution is not the same as the route delay!";
        std::cout << "Vehicle ID: " << newRoute->vehicleID_ << std::endl;
        // myTools::throwException("Route-Validation");
    }
    return newRoute;
}


void GreedyRoute::updateTailDepart() {
    if (lastStop_->currentNode_->type_ != SOURCE)
        lastStop_->leaveTime_ = lastStop_->reachTime_ + lastStop_->currentNode_->serviceTime_;
    departureTime_ = lastStop_->leaveTime_;
}

bool GreedyRoute::isAnyViolation(const PStopLabel &startLabel) const {
    if (startLabel->nbPassengers_ > (*Vehicle_)->capacity_)
        return true;
    PStopLabel currentLabel = startLabel;
    while (currentLabel != nullptr) {
        if (currentLabel->currentNode_->type_ == DROPOFF) {
            if (currentLabel->travelResource_ < 0) {
                return true;
            }
        }
        else if (currentLabel->currentNode_->type_ == PICKUP) {
            if (currentLabel->reachTime_ < currentLabel->currentNode_->readyTime_)
                return true;
            if (currentLabel->currentNode_->related_Request_->committedPickTime_ < LARGE_CONSTANT) {
                if (currentLabel->reachTime_ > currentLabel->currentNode_->related_Request_->latestPickup_)
                    return true;
            }
        }
        currentLabel = currentLabel->child_;
    }
    return false;
}

//---------------------------------------------------------------------------------------------
//  INSERT POSITION STRUCT
//  This structure is used to determine the best position to insert requests in a route.
//  For each request, the best position with the potential increase in delay and length is determined
//  then the best route and position to insert is selected
//---------------------------------------------------------------------------------------------


insertPosition::insertPosition() = default;

void insertPosition::updatePosition(const PStopLabel &prePickup, const PStopLabel &preDrop, float waitIncrease,
                                    float tripDelayIncrease, float wait_W1, float ride_W2) {
    prePickup_ = prePickup;
    preDrop_ = preDrop;
    deltaWait_ = waitIncrease;
    deltaTripDelay_ = tripDelayIncrease;

    deltaObjective_ = wait_W1 * waitIncrease + ride_W2 * tripDelayIncrease;
}
