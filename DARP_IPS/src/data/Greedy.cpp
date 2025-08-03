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
    PLastStop_ = PInitialStop_;
    totalDelay_ = 0;
    idleTime_ = 0;
    departureTime_ = vehicle->departTime_;
    // just onboards are added and the previous solution is not considered
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
    // build the previous solution
    else {
        for (int i = 1; i < (*Vehicle_)->currentRoute_->routeNodes_.size(); i++) {
            idle_ = false;
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
            newLabel->parent_ = PLastStop_;
            PLastStop_->child_ = newLabel;
            PLastStop_ = newLabel;
            /*if (newLabel->currentNode_->nodeStatus_ == COMMITTED) {
                PCurrentStop_ = newLabel;
                departureTime_ = newLabel->leaveTime_;
            }*/
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
            else if (newLabel->currentNode_->type_ == PICKUP)
                totalDelay_ += (newLabel->reachTime_ - newLabel->currentNode_->initialReadyTime_);

        }
    }
}

GreedyRoute::GreedyRoute(const GreedyRoute &label) {
    selected_ = label.selected_;
    Vehicle_ = label.Vehicle_;
    departureTime_ = label.departureTime_;
    idleTime_ = label.idleTime_;
    PLastStop_ = label.PLastStop_;
    PInitialStop_ = label.PInitialStop_;
    totalDelay_ = label.totalDelay_;
    idle_ = label.idle_;
}

GreedyRoute::~GreedyRoute() = default;

void GreedyRoute::resetGreedyRoute(std::vector<PStopLabel> &greedyLabelPool) const {
    PStopLabel currentLabel = PInitialStop_;
    while (currentLabel->child_ != nullptr) {
        greedyLabelPool.push_back(currentLabel);
        currentLabel = currentLabel->child_;
    }
}

// this function find a position to insert pickup point and add drop off point
void GreedyRoute::findInsertPlace(PNode &pickNode, PNode &dropNode, float maxDuration,
                                  std::vector<PStopLabel> &greedyLabelPool, const PInsertPosition & position) {
    float waitIncrease = INFINITY;

    // Tour length increase
    float lengthIncrease = INFINITY;

    position->updatePosition(PLastStop_, PLastStop_, waitIncrease, lengthIncrease);

    // define the initial position to add the request just after all
    PStopLabel prePick = PInitialStop_;
    PStopLabel preDrop;
    while (prePick != nullptr) {
        waitIncrease = INFINITY;
        if (prePick == PLastStop_) {
            if (prePick->nbPassengers_ + pickNode->nbPassengers_ > (*Vehicle_)->capacity_){
                throw myTools::myException("Instance error: capacity exceeded, consider splitting trip.", __FILE__, __LINE__);
            }
            // it stays at tail and then departs to the pickup point
            float pickTime = labelToNodeReachTime(prePick, pickNode);
            waitIncrease = pickTime - pickNode->initialReadyTime_;
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


void GreedyRoute::insertNode(const PStopLabel &preLabel, PNode &newNode, std::vector<PStopLabel> &greedyLabelPool) {
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
        if (reachTime - newNode->related_Request_->earlyPick_ < 0 )
            throw myTools::myException("Negative waiting time encountered in insertNode.", __FILE__, __LINE__);

        totalDelay_ += (reachTime - newNode->initialReadyTime_);
    }
}

void GreedyRoute::removeLabel(PStopLabel &label, std::vector<PStopLabel> &greedyLabelPool) {
    greedyLabelPool.push_back(label);
    if (label->currentNode_->type_ == PICKUP) {
        totalDelay_ -= label->reachTime_ - label->currentNode_->initialReadyTime_;
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

void GreedyRoute::insertRequest(const PInsertPosition &position, PNode &pickNode, PNode &dropNode, float maxDuration,
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
        // calculate child reach time
        float childReachTime = labelToNodeReachTime(currentLabel, currentLabel->child_->currentNode_);
        if (currentLabel->currentNode_->type_ != SOURCE){
            if (currentLabel->child_->currentNode_->type_ == PICKUP){
                if (currentLabel->leaveTime_ < currentLabel->child_->currentNode_->related_Request_->requestTime_)
                    currentLabel->leaveTime_ = currentLabel->child_->currentNode_->related_Request_->requestTime_;
            }
        }

        // update total delay
        if (currentLabel->child_->currentNode_->type_ == PICKUP) {
            float preDelay = currentLabel->child_->reachTime_ - currentLabel->child_->currentNode_->initialReadyTime_;
            float newDelay = childReachTime - currentLabel->child_->currentNode_->initialReadyTime_;
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
        throw myTools::myException("Negative total delay after updating route.", __FILE__, __LINE__);
    }
}

// this function converts a greedyLabel list to a route
PRoute GreedyRoute::greedyLabelToRoute(bool update) const {
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

        if (currentLabel->parent_->leaveTime_ < currentLabel->currentNode_->related_Request_->requestTime_) {
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
            newRoute->routeNodes_.back()->related_Request_->solVehicleID_ = newRoute->vehicleID_;
        }
        currentLabel = currentLabel->child_;
    }

    if (newRoute->totalDelay_ != totalDelay_){
        std::cout << "Total delay of the greedy solution is not the same as the route delay!";
        std::cout << "Vehicle ID: " << newRoute->vehicleID_ << std::endl;
        // myTools::throwException("Route-Validation");
    }
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


void GreedyRoute::updateTailDepart() {
    if (PLastStop_->currentNode_->type_ != SOURCE)
        PLastStop_->leaveTime_ = PLastStop_->reachTime_ + PLastStop_->currentNode_->serviceTime_;
    departureTime_ = PLastStop_->leaveTime_;
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
            if (currentLabel->reachTime_ > currentLabel->currentNode_->related_Request_->latestPickup_)
                return true;
        }
        currentLabel = currentLabel->child_;
    }
    return false;
}

void GreedyRoute::findAssignedPlace(PNode &pickNode, PNode &dropNode, float maxDuration,
                                    std::vector<PStopLabel> &removedLabels, const PInsertPosition &position) {
    float deltaDelay = INFINITY;
    bool notFound = true;
    // Tour length increase
    float DeltaT = INFINITY;

    position->updatePosition(PLastStop_, PLastStop_, deltaDelay, DeltaT);

    // define the initial position to add the request just after all
    PStopLabel prePick = PInitialStop_;
    PStopLabel preDrop;
    while ((prePick != nullptr) && (notFound)) {
        deltaDelay = INFINITY;
        if (prePick == PLastStop_) {
            if (prePick->nbPassengers_ + pickNode->nbPassengers_ > (*Vehicle_)->capacity_){
                throw myTools::myException("Instance error! split the trip!", __FILE__, __LINE__);
            }
            // it stays at tail and then departs to the pickup point
            float pickTime = labelToNodeReachTime(prePick, pickNode);
            deltaDelay =  pickTime - pickNode->initialReadyTime_;
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
//---------------------------------------------------------------------------------------------
//  INSERT POSITION STRUCT
//  This structure is used to determine the best position to insert requests in a route.
//  For each request, the best position with the potential increase in delay and length is determined
//  then the best route and position to insert is selected
//---------------------------------------------------------------------------------------------


insertPosition::insertPosition() = default;

void insertPosition::updatePosition(const PStopLabel &prePickup, const PStopLabel &preDrop, float waitIncrease,
                                    float lengthIncrease) {
    prePickup_ = prePickup;
    preDrop_ = preDrop;
    deltaDelay_ = waitIncrease;
    deltaLength_ = lengthIncrease;
}

