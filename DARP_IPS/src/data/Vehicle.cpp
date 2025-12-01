//
// Created by Ella on 2021-09-08.
//

#include "Vehicle.h"
//-----------------------------------------------------------------------------
//  vehicle class
//  contains the vehicle info like working time, capacity and passengers
//-----------------------------------------------------------------------------

// Constructor and Destructor
Vehicle::Vehicle(int vehicleId, int capacity, float departTime, float endTime, const PNode &departNode,
                 const PNode & sinkNode) : vehicleID_(vehicleId), endTime_(endTime), capacity_(capacity),
                                     departTime_(departTime), departNode_(departNode), sinkNode_(sinkNode){
    numPassengers_ = 0;
    dual_=0;
    InitialDual_ = 0;
    bestReducedCost_ = INFINITY;
    idleTime_ = 0;
    driveEmptyTime_ = 0;
    driveFullTime_ = 0;
    serviceTime_ = 0;
    returnEmptyTime_ = 0;
    startTime_ = 0;
    vehicleIndex_ = vehicleId;
    numPickup_ = 1;
    stateChanged_ = false;
    removeDrop_ = false;
    removePickup_ = false;
    preSolvePickLimit_ = 2;
}

Vehicle::~Vehicle() = default;

// Display function
std::string Vehicle::toString() const {
    std::stringstream repStr;
    repStr << std::left;
    repStr << "# VEHICLE ( " << vehicleID_ << " )" << std::endl;
    repStr << "#\t" << std::setw(24) << "- VEHICLE_CAPACITY" << " : " << capacity_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- START_TIME (seconds)" << " : " << startTime_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- DEPART_TIME (seconds)" << " : " << departTime_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- END_TIME (seconds)" << " : " << endTime_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- NUMBER_OF_ONBOARDS" << " : " << numPassengers_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- DEPART_NODE_ID" << " : " << departNode_->nodeID_ << std::endl;
    repStr << "#" << std::endl;
    return repStr.str();
}

// Setters
void Vehicle::setDepartTime(float departTime) {
    departTime_ = departTime;
}

// this function creates an empty route and sets it as the CurrentRoute_, used in initialization
void Vehicle::setEmptyRoute(const PInstance &pInst) {
    emptyRoute_ = std::make_shared<Route>(vehicleID_);
    emptyRoute_->addSource(departNode_, departTime_, numPassengers_);

    if (!onboards_.empty()) {
        if (currentRoute_!= nullptr) {
            if (currentRoute_->routeSize_ > 1) {
                for (int i = 1; i < currentRoute_->routeSize_; ++i) {
                    if (currentRoute_->routeNodes_[i]->nodeStatus_ == PLANNED) {
                        emptyRoute_->addNode(currentRoute_->routeNodes_[i]);
                    }
                }
            }
        }
        else{
            for (auto & nodeId : onboards_){
                emptyRoute_->addNode(pInst->instGraph_->nodes_[nodeId]);
            }
        }
    }
    emptyRoute_->calculateTripDelay(pInst->parameters_->Wait_W1_, pInst->parameters_->Ride_W2_);
    emptyRoute_->totalLength_ = emptyRoute_->plannedDepartTime_.back() - departTime_;
    emptyRoute_->createColumn(pInst->nbRequests_);
}

void Vehicle::setSolutionRoute() {
    solutionRoute_ = std::make_shared<Route>(vehicleID_);
    solutionRoute_->addSource(departNode_, departTime_, numPassengers_);
}

void Vehicle::setCurrentRoute(const PRoute &currentRoute) {
    currentRoute_ = currentRoute;
    for (int i = 0; i < currentRoute->routeRequests_.size(); ++i) {
        currentRoute->routeRequests_[i]->solVehicleID_  = vehicleID_;

  //      currentRoute->routeRequests_[i]->coveredVehicles_.set(vehicleID_,true);
        // currentRoute->routeRequests_[i]->plannedDelay_ = currentRoute->plannedDelay_[i];
    }
}

// function to update vehicle depart time at each time and
// update the situation of nodes and ride requests

void Vehicle::updateStateTime(const PInstance & mainInst, float elapsedTime, boost::dynamic_bitset<> &removedRequests) {
    float committedTime = 0;
    if (mainInst->parameters_->solutionMode_ == ANYTIME)
        committedTime = mainInst->parameters_->committedTime_;
    else
        committedTime = mainInst->parameters_->epochLength_;
    stateChanged_ = false;
    removePickup_ = false;
    removeDrop_ = false;

    if (currentRoute_->routeSize_ > 1) {
         if (currentRoute_->routeRequests_.empty() || currentRoute_->routeRequests_.size() > 1 || preSolvePickLimit_ != 1) {
            // this condition is useful for cases that the vehicle does not have any stops in the current epoch
            if (departTime_ < elapsedTime + committedTime) {
                onboards_.clear();
                int breakIndex = 0;
                for (int i = 1; i < currentRoute_->routeSize_; ++i) {
                    stateChanged_ = true;
                    mainInst->nbStateChanged_++;
                    PNode routeNode = currentRoute_->routeNodes_[i];
                    routeNode->nodeStatus_ = DONE;
                    routeNode->reachTime_ = currentRoute_->plannedReachTime_[i];
                    routeNode->departTime_ = currentRoute_->plannedDepartTime_[i];
                    solutionRoute_->addNode(routeNode);
                    if (routeNode->initialType_ == SINK) {
                        returnEmptyTime_ += (currentRoute_->plannedReachTime_[i] - currentRoute_->plannedDepartTime_[i-1]);
                        mainInst->nbReturn_++;
                    }
                    else {
                        serviceTime_ += routeNode->serviceTime_;
                        if (currentRoute_->plannedPassengers_[i-1] > 0)
                            driveFullTime_ += (currentRoute_->plannedReachTime_[i] - currentRoute_->plannedDepartTime_[i-1]);
                        else
                            driveEmptyTime_ += (currentRoute_->plannedReachTime_[i] - currentRoute_->plannedDepartTime_[i-1]);
                    }


                    // set request status
                    if (routeNode->type_ == PICKUP) {
                        removePickup_ = true;
 //                       stateChanged_ = true;
                        // routeNode->related_Request_->plannedDelay_ = currentRoute_->plannedDelay_[i];
                        if (mainInst->parameters_->mainAlgorithm_ != GREEDY)
                            removedRequests.set(routeNode->related_Request_->taskIndex_, true);
                        if (routeNode->related_Request_->committedPickTime_ == LARGE_CONSTANT) {
                            routeNode->related_Request_->commitTime_ = elapsedTime;
                            routeNode->related_Request_->committedPickTime_ = currentRoute_->plannedReachTime_[i];
                            if (currentRoute_->plannedReachTime_[i] - routeNode->related_Request_->initialEarlyPick_ > mainInst->parameters_->maxWait_)
                                mainInst->lastCommittedRequests_.push_back(routeNode->related_Request_);
                        }
                    }
                    else
                        removeDrop_ = true;

                    setRequestStatus(routeNode, currentRoute_->plannedReachTime_[i]);

                    if (i == currentRoute_->routeSize_ - 1 ||
                        (currentRoute_->plannedDepartTime_[i] >= elapsedTime + committedTime &&
                         routeNode->locationID_ !=
                         currentRoute_->routeNodes_[i + 1]->locationID_)) {
                        //at the departure point, the vehicle is ready to leave the stop location (delta has passed)
                        departTime_ = currentRoute_->plannedDepartTime_[i];
                        // if we have reached the end of the route, the next condition is checked
                        if (i == currentRoute_->routeSize_ - 1 && departTime_ < elapsedTime + committedTime) {
                            handleIdleState(elapsedTime + committedTime);
                        }
                        numPassengers_ = currentRoute_->plannedPassengers_[i];
                        departNode_ = routeNode;
                        routeNode->type_ = SOURCE;
                        breakIndex = i;
                        break;
                    }
                }
                for (int i = breakIndex + 1; i < currentRoute_->routeSize_; ++i) {
                    PNode routeNode = currentRoute_->routeNodes_[i];
                    if (routeNode->type_ != SOURCE && routeNode->type_ != SINK) {
                        if (routeNode->related_Request_->requestStatus_ == ON_BOARD) {
                            routeNode->nodeStatus_ = PLANNED;
                            onboards_.push_back(routeNode->nodeID_);
                            routeNode->related_Request_->latestDrop_ = routeNode->pairNode_->reachTime_ +
                                    routeNode->pairNode_->serviceTime_ + routeNode->related_Request_->maxTravelTime_;
                        }
                    }
                }
                currentRoute_->removeNode(breakIndex);
                currentRoute_->calculateTripDelay(mainInst->parameters_->Wait_W1_, mainInst->parameters_->Ride_W2_);
            }
        }
        if (currentRoute_->routeNodes_.size()-1 == onboards_.size())
            emptyRoute_ = currentRoute_;
    }
    else if (departTime_ <= elapsedTime + committedTime){
        handleIdleState(elapsedTime + committedTime);
    }
    for (int i = 1; i < currentRoute_->routeSize_; ++i) {
        PNode routeNode = currentRoute_->routeNodes_[i];
        if (routeNode->type_ == PICKUP && routeNode->related_Request_->committedPickTime_ == LARGE_CONSTANT) {
            if (elapsedTime - routeNode->related_Request_->earlyPick_ >= mainInst->parameters_->informTimeLimit_) {
                routeNode->related_Request_->committedPickTime_ = currentRoute_->plannedReachTime_[i];
                routeNode->related_Request_->earlyPick_ = currentRoute_->plannedReachTime_[i] -
                    mainInst->parameters_->pickupDeviationWindow_;
                routeNode->readyTime_ = routeNode->related_Request_->earlyPick_;
                routeNode->related_Request_->commitTime_ = elapsedTime;
            }
        }
    }
}

// this function is called at the end of the algorithm to set the final stos of the solution based on final epoch
void Vehicle::finalizeSolutionRoutes(float elapsedTime) {
    if (currentRoute_->routeSize_ == 1)
        solutionRoute_->routeNodes_.back()->departTime_ = solutionRoute_->plannedDepartTime_.back();
    if (solutionRoute_->routeNodes_.back()->type_ == SOURCE) {
        for (int i = 1; i < currentRoute_->routeSize_; ++i) {
            PNode routeNode = currentRoute_->routeNodes_[i];
            routeNode->nodeStatus_ = DONE;
            routeNode->reachTime_ = currentRoute_->plannedReachTime_[i];
            routeNode->departTime_ = currentRoute_->plannedDepartTime_[i];
            solutionRoute_->addNode(routeNode);

            if (routeNode->initialType_ == SINK)
                returnEmptyTime_ += (currentRoute_->plannedReachTime_[i] - currentRoute_->plannedDepartTime_[i-1]);
            else {
                serviceTime_ += routeNode->serviceTime_;
                if (currentRoute_->plannedPassengers_[i-1] > 0)
                    driveFullTime_ += (currentRoute_->plannedReachTime_[i] - currentRoute_->plannedDepartTime_[i-1]);
                else
                    driveEmptyTime_ += (currentRoute_->plannedReachTime_[i] - currentRoute_->plannedDepartTime_[i-1]);
            }

            if (routeNode->type_ == PICKUP) {
                if (routeNode->related_Request_->committedPickTime_ == LARGE_CONSTANT) {
                    routeNode->related_Request_->commitTime_ = elapsedTime;
                    routeNode->related_Request_->committedPickTime_ = currentRoute_->plannedReachTime_[i];
                }
                routeNode->related_Request_->committedPickTime_ = currentRoute_->plannedReachTime_[i];
                routeNode->related_Request_->pickTime_ = currentRoute_->plannedReachTime_[i];
                routeNode->related_Request_->allocVehicleID_ = vehicleID_;
                routeNode->related_Request_->requestStatus_ = COMPLETED;
            }
            else if (routeNode->type_ == DROPOFF) {
                routeNode->related_Request_->dropTime_ = currentRoute_->plannedReachTime_[i];
                routeNode->related_Request_->requestStatus_ = COMPLETED;
            }
        }
    }
}

void Vehicle::updateCurrentRoute(float elapsedTime, float wait_W1, float ride_W2) {
    float departure = std::max(elapsedTime + 5, departNode_->reachTime_+ departNode_->serviceTime_);
    if (departure < departTime_) {
        idleTime_ -= (departTime_ - departure);
        updateDepartTime(departure, wait_W1, ride_W2);
    }
}

void Vehicle::updateDepartTime(float departTime, float wait_W1, float ride_W2) {
    currentRoute_->plannedDepartTime_[0] = departTime;
    solutionRoute_->routeNodes_.back()->departTime_ = departTime;
    solutionRoute_->plannedDepartTime_.back() = departTime;
    departTime_ = departTime;
    currentRoute_->totalWait_ = 0;
    PRoute newRoute = std::make_shared<Route>(vehicleID_);
    newRoute->addSource(departNode_, departTime_, numPassengers_);
    for (int i = 1; i < currentRoute_->routeNodes_.size(); ++i) {
        newRoute->addNode(currentRoute_->routeNodes_[i]);
    }
    newRoute->calculateTripDelay(wait_W1, ride_W2);
    currentRoute_->plannedDepartTime_ = newRoute->plannedDepartTime_;
    currentRoute_->plannedReachTime_ = newRoute->plannedReachTime_;
    currentRoute_->totalWait_ = newRoute->totalWait_;
    currentRoute_->totalTripDelay_ = newRoute->totalTripDelay_;
}

void Vehicle::handleIdleState(float epochEndTime) {
    idleTime_ += (epochEndTime - departTime_);
    departTime_ = epochEndTime;
    currentRoute_->plannedDepartTime_[0] = departTime_;
    solutionRoute_->routeNodes_.back()->departTime_ = departTime_;
    solutionRoute_->plannedDepartTime_.back() = departTime_;
}


void Vehicle::setRequestStatus(const PNode &node, float reachTime) const {
    if (node->type_ == PICKUP) {
        node->related_Request_->requestStatus_ = ON_BOARD;
        node->related_Request_->pickTime_ = reachTime;
        node->related_Request_->latestDrop_ = reachTime + node->serviceTime_ + node->related_Request_->maxTravelTime_;
        node->related_Request_->allocVehicleID_ = vehicleID_;
    }

    else if (node->type_ == DROPOFF){
        node->related_Request_->requestStatus_ = COMPLETED;
        node->related_Request_->dropTime_ = reachTime;
        // check travelTime violation
        float travelTime = node->related_Request_->dropTime_ - node->pairNode_->departTime_;
        if (travelTime > node->related_Request_->maxTravelTime_){
            std::cout << "Trip delay constraint is violated by request: " ;
            std::cout << node->related_Request_->getRequestId() << std::endl;
//          myTools::throwException("Trip delay Validation");
        }
    }
}

void Vehicle::adjustDuals() {
    if (currentRoute_->routeRequests_.size() > 0) {
        float totalDual = 0.0;
        for (int i = 0; i < currentRoute_->routeRequests_.size(); ++i) {
            totalDual += currentRoute_->routeRequests_[i]->dual_;
        }
        for (int i = 0; i < currentRoute_->routeRequests_.size(); ++i) {
            currentRoute_->routeRequests_[i]->dual_ *= (currentRoute_->objCoef_ - dual_)/ totalDual;
        }
    }
    dual_ = 0.0;
}

std::string Vehicle::toStringOut(int epoch) const {
    std::stringstream repStr;
    float avgPassPerStop = 0.0;
    if (currentRoute_->routeSize_ > 1) {
        for (int i = 1; i < currentRoute_->plannedPassengers_.size(); ++i) {
            avgPassPerStop += currentRoute_->plannedPassengers_[i];
        }
        avgPassPerStop /= currentRoute_->plannedPassengers_.size() -1;
    }

    repStr << epoch << "," << vehicleID_ << ","
           << currentRoute_->getRouteId() << ","
           << numPassengers_ << ","
           << onboards_.size() << ","
           << currentRoute_->nbCommitted_ << ","
           << currentRoute_->routeRequests_.size() << ","
           << currentRoute_->routeSize_ << ","
           << currentRoute_->totalLength_ << ","
           << avgPassPerStop << ","
           << currentRoute_->totalWait_ << ","
           << currentRoute_->totalTripDelay_ << ","
           << currentRoute_->objCoef_ << "\n";
    return repStr.str();
}