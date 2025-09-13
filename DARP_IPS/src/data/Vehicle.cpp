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
    score_ = INFINITY;

    idleTime_ = 0;
    driveEmptyTime_ = 0;
    driveFullTime_ = 0;
    serviceTime_ = 0;
    returnEmptyTime_ = 0;

    startTime_ = 0;
    vehicleIndex_ = vehicleId;
    idle_ = true;
    numPickup_ = 1;
    stateChanged_ = false;
    preSolvePick_ = 2;
}

Vehicle::~Vehicle() = default;

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
        // currentRoute->routeRequests_[i]->plannedDelay_ = currentRoute->plannedDelay_[i];
    }
}



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

// function to update vehicle depart time at each time and
// update the situation of nodes and ride requests

void Vehicle::updateStateTime(const PInstance & mainInst, float elapsedTime, boost::dynamic_bitset<> &removedRequests) {
    float committedTime = 0;
    if (mainInst->parameters_->solutionMode_ == ANYTIME)
        committedTime = mainInst->parameters_->committedTime_;
    else
        committedTime = mainInst->parameters_->epochLength_;
    stateChanged_ = false;
    /*if (mainInst->parameters_->vehicleReturn_ && currentRoute_->plannedReachTime_[0]+ currentRoute_->routeNodes_.back()->serviceTime_ < elapsedTime - committedTime
    && currentRoute_->routeSize_ == 1){
        if (currentRoute_->routeNodes_.back()->locationID_ != sinkNode_->locationID_){
            idleTime_ += (elapsedTime + committedTime - departTime_);
            currentRoute_->plannedDepartTime_[0] = elapsedTime + committedTime;
            solutionRoute_->routeNodes_.back()->departTime_ = currentRoute_->plannedDepartTime_[0];
            solutionRoute_->plannedDepartTime_.back() = currentRoute_->plannedDepartTime_[0];
            currentRoute_->addSink(sinkNode_);
            mainInst->nbReturn_++;
        }
    }*/
    if (currentRoute_->routeSize_ > 1) {
         if (currentRoute_->routeRequests_.empty() || currentRoute_->routeRequests_.size() > 1 || preSolvePick_ != 1) {
            idle_ = false;
            // this condition is useful for the cases that the vehicle does not have any stop in the current epoch
            if (departTime_ < elapsedTime + committedTime) {
                onboards_.clear();
                int breakIndex = 0;
                for (int i = 1; i < currentRoute_->routeSize_; ++i) {
                    stateChanged_ = true;
                    mainInst->nbStateChanged_++;
                    currentRoute_->routeNodes_[i]->nodeStatus_ = DONE;
                    currentRoute_->routeNodes_[i]->reachTime_ = currentRoute_->plannedReachTime_[i];
                    currentRoute_->routeNodes_[i]->departTime_ = currentRoute_->plannedDepartTime_[i];
                    solutionRoute_->addNode(currentRoute_->routeNodes_[i]);
                    if (currentRoute_->routeNodes_[i]->initialType_ == SINK) {
                        returnEmptyTime_ += (currentRoute_->plannedReachTime_[i] - currentRoute_->plannedDepartTime_[i-1]);
                        mainInst->nbReturn_++;
                    }
                    else {
                        serviceTime_ += currentRoute_->routeNodes_[i]->serviceTime_;
                        if (currentRoute_->plannedPassengers_[i-1] > 0)
                            driveFullTime_ += (currentRoute_->plannedReachTime_[i] - currentRoute_->plannedDepartTime_[i-1]);
                        else
                            driveEmptyTime_ += (currentRoute_->plannedReachTime_[i] - currentRoute_->plannedDepartTime_[i-1]);
                    }


                    // set request status
                    if (currentRoute_->routeNodes_[i]->type_ == PICKUP) {
                        // currentRoute_->routeNodes_[i]->related_Request_->plannedDelay_ = currentRoute_->plannedDelay_[i];
                        if (mainInst->parameters_->mainAlgorithm_ != GREEDY)
                            removedRequests.set(currentRoute_->routeNodes_[i]->related_Request_->taskIndex_, true);
                        if (currentRoute_->routeNodes_[i]->related_Request_->committedPickTime_ == LARGE_CONSTANT) {
                            currentRoute_->routeNodes_[i]->related_Request_->commitTime_ = elapsedTime;
                            currentRoute_->routeNodes_[i]->related_Request_->committedPickTime_ = currentRoute_->plannedReachTime_[i];
                            if (currentRoute_->plannedReachTime_[i] - currentRoute_->routeNodes_[i]->related_Request_->initialEarlyPick_ > mainInst->parameters_->maxWait_)
                                mainInst->lastCommittedRequests_.push_back(currentRoute_->routeNodes_[i]->related_Request_);
                        }
                    }

                    setRequestStatus(currentRoute_->routeNodes_[i], currentRoute_->plannedReachTime_[i]);

                    if (i == currentRoute_->routeSize_ - 1 ||
                        ((currentRoute_->plannedDepartTime_[i] >= elapsedTime + committedTime) &&
                         (currentRoute_->routeNodes_[i]->locationID_ !=
                          currentRoute_->routeNodes_[i + 1]->locationID_))) {
                        //at the departure point, the vehicle is ready to leave the stop location (delta has passed)
                        departTime_ = currentRoute_->plannedDepartTime_[i];
                        // if we have reached the end of the route, the next condition is checked
                        if (i == currentRoute_->routeSize_ - 1 && departTime_ < elapsedTime + committedTime) {
                            handleIdleState(elapsedTime + committedTime);
                        }
                        numPassengers_ = currentRoute_->plannedPassengers_[i];
                        departNode_ = currentRoute_->routeNodes_[i];
                        currentRoute_->routeNodes_[i]->type_ = SOURCE;
                        breakIndex = i;
                        break;
                    }
                }
                for (int i = breakIndex + 1; i < currentRoute_->routeSize_; ++i) {
                    if (currentRoute_->routeNodes_[i]->type_ != SOURCE &&
                        currentRoute_->routeNodes_[i]->type_ != SINK) {
                        if (currentRoute_->routeNodes_[i]->related_Request_->requestStatus_ == ON_BOARD) {
                            currentRoute_->routeNodes_[i]->nodeStatus_ = PLANNED;
                            onboards_.push_back(currentRoute_->routeNodes_[i]->nodeID_);
                        }
                    }
                }
                currentRoute_->removeNode(breakIndex);
            }
        }
        if (currentRoute_->routeNodes_.size()-1 == onboards_.size())
            emptyRoute_ = currentRoute_;
    }
    else if (departTime_ <= elapsedTime + committedTime){
        handleIdleState(elapsedTime + committedTime);
    }
    for (int i = 1; i < currentRoute_->routeSize_; ++i) {
        if (currentRoute_->routeNodes_[i]->type_ == PICKUP && currentRoute_->routeNodes_[i]->related_Request_->committedPickTime_ == LARGE_CONSTANT) {
            if (elapsedTime - currentRoute_->routeNodes_[i]->related_Request_->earlyPick_ >= mainInst->parameters_->informTimeLimit_) {
                currentRoute_->routeNodes_[i]->related_Request_->committedPickTime_ = currentRoute_->plannedReachTime_[i];
                currentRoute_->routeNodes_[i]->related_Request_->earlyPick_ = currentRoute_->plannedReachTime_[i] -
                    mainInst->parameters_->pickupDeviationWindow_;
                currentRoute_->routeNodes_[i]->readyTime_ = currentRoute_->routeNodes_[i]->related_Request_->earlyPick_;
                currentRoute_->routeNodes_[i]->related_Request_->commitTime_ = elapsedTime;
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
            currentRoute_->routeNodes_[i]->nodeStatus_ = DONE;
            currentRoute_->routeNodes_[i]->reachTime_ = currentRoute_->plannedReachTime_[i];
            currentRoute_->routeNodes_[i]->departTime_ = currentRoute_->plannedDepartTime_[i];
            solutionRoute_->addNode(currentRoute_->routeNodes_[i]);

            if (currentRoute_->routeNodes_[i]->initialType_ == SINK)
                returnEmptyTime_ += (currentRoute_->plannedReachTime_[i] - currentRoute_->plannedDepartTime_[i-1]);
            else {
                serviceTime_ += currentRoute_->routeNodes_[i]->serviceTime_;
                if (currentRoute_->plannedPassengers_[i-1] > 0)
                    driveFullTime_ += (currentRoute_->plannedReachTime_[i] - currentRoute_->plannedDepartTime_[i-1]);
                else
                    driveEmptyTime_ += (currentRoute_->plannedReachTime_[i] - currentRoute_->plannedDepartTime_[i-1]);
            }

            if (currentRoute_->routeNodes_[i]->type_ == PICKUP) {
                if (currentRoute_->routeNodes_[i]->related_Request_->committedPickTime_ == LARGE_CONSTANT) {
                    currentRoute_->routeNodes_[i]->related_Request_->commitTime_ = elapsedTime;
                    currentRoute_->routeNodes_[i]->related_Request_->committedPickTime_ = currentRoute_->plannedReachTime_[i];
                }
                currentRoute_->routeNodes_[i]->related_Request_->committedPickTime_ = currentRoute_->plannedReachTime_[i];
                currentRoute_->routeNodes_[i]->related_Request_->pickTime_ = currentRoute_->plannedReachTime_[i];
                currentRoute_->routeNodes_[i]->related_Request_->allocVehicleID_ = vehicleID_;
                currentRoute_->routeNodes_[i]->related_Request_->requestStatus_ = COMPLETED;
            }
            else if (currentRoute_->routeNodes_[i]->type_ == DROPOFF) {
                currentRoute_->routeNodes_[i]->related_Request_->dropTime_ = currentRoute_->plannedReachTime_[i];
                currentRoute_->routeNodes_[i]->related_Request_->requestStatus_ = COMPLETED;
            }
        }
    }
}

void Vehicle::updateDepartTime(float departTime) {
    currentRoute_->plannedDepartTime_[0] = departTime;
    solutionRoute_->routeNodes_.back()->departTime_ = departTime;
    solutionRoute_->plannedDepartTime_.back() = departTime;
    departTime_ = departTime;
    idle_ = false;
    currentRoute_->totalDelay_ = 0;
    PRoute newRoute = std::make_shared<Route>(vehicleID_);
    newRoute->addSource(departNode_, departTime_, numPassengers_);
    for (int i = 1; i < currentRoute_->routeNodes_.size(); ++i) {
        newRoute->addNode(currentRoute_->routeNodes_[i]);
    }
    currentRoute_->plannedDepartTime_ = newRoute->plannedDepartTime_;
    currentRoute_->plannedReachTime_ = newRoute->plannedReachTime_;
    currentRoute_->totalDelay_ = newRoute->totalDelay_;
}

void Vehicle::updateCurrentRoute(float elapsedTime) {
    float departure = std::max(elapsedTime + 5, departNode_->reachTime_+ departNode_->serviceTime_);
    if (departure < departTime_) {
        idleTime_ -= (departTime_ - departure);
        updateDepartTime(departure);
    }
    idle_ = false;
}


// Handle idle state for vehicles with no stops
void Vehicle::handleIdleState(float epochEndTime) {
    idleTime_ += (epochEndTime - departTime_);
    departTime_ = epochEndTime;
    currentRoute_->plannedDepartTime_[0] = departTime_;
    solutionRoute_->routeNodes_.back()->departTime_ = departTime_;
    solutionRoute_->plannedDepartTime_.back() = departTime_;
    idle_ = true;
}


void Vehicle::setRequestStatus(const PNode &node, float reachTime) const {
    if (node->type_ == PICKUP) {
        node->related_Request_->requestStatus_ = ON_BOARD;
        node->related_Request_->pickTime_ = reachTime;
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
            currentRoute_->routeRequests_[i]->dual_ *= (currentRoute_->totalDelay_ - dual_)/ totalDual;
        }
    }
    dual_ = 0.0;
}

/*void Vehicle::checkCoveredRequests(std::vector<PRoute> &availableRoutes, int nbRequests) {
    coveredRequests.reset();
    coveredRequests.resize(nbRequests);
    for (auto & routeObj: availableRoutes) {
        for (auto & requestObj: routeObj->routeRequests_) {
            coveredRequests.set(requestObj->taskIndex_);
        }
    }
}*/
