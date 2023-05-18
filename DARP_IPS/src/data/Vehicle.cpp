//
// Created by Ella on 2021-09-08.
//

#include "Vehicle.h"

#include <utility>

//-----------------------------------------------------------------------------
//  vehicle class
//  contains the vehicle info like working time, capacity and passengers
//-----------------------------------------------------------------------------

// Constructor and Destructor
Vehicle::Vehicle(int vehicleId, int capacity, float departTime, float endTime, PNode &departNode,
                 std::string sinkID) : vehicleID_(vehicleId), capacity_(capacity), departTime_(departTime),
                                                   endTime_(endTime), departNode_(std::move(departNode)), sinkID_(std::move(sinkID)){
    numPassengers_ = 0;
    dual_=0;
    CPDual_ = 0;
    bestReducedCost_ = INFINITY;
    score_ = INFINITY;
    idleTime_ = 0;
    startTime_ = departTime;
    selected_ = false;
    vehicleIndex_ = -1;
}
Vehicle::Vehicle(int vehicleId, int capacity, float departTime, float endTime, PNode &departNode,
                 std::string sinkID, int zoneID) : vehicleID_(vehicleId), capacity_(capacity), departTime_(departTime),
                 endTime_(endTime), departNode_(std::move(departNode)), sinkID_(std::move(sinkID)) , zoneID_(zoneID){
    numPassengers_ = 0;
    dual_=0;
    CPDual_ = 0;
    bestReducedCost_ = INFINITY;
    score_ = INFINITY;
    idleTime_ = 0;
    startTime_ = departTime;
    selected_ = false;
    vehicleIndex_ = -1;
}

Vehicle::~Vehicle() = default;

// Setters
void Vehicle::setDepartTime(float departTime) {
    departTime_ = departTime;
}

// this function create an empty route and set it as the CurrentRoute_, used in initialization
void Vehicle::setEmptyRoute(PInstance &pInst) {
    emptyRoute_ = std::make_shared<Route>(vehicleID_);
    emptyRoute_->addSource(departNode_, departTime_, numPassengers_);

    if (!onboards_.empty()) {
        if (currentRoute_!= nullptr) {
            if (currentRoute_->routeSize_ > 1) {
                for (int i = 1; i < currentRoute_->routeSize_; ++i) {
                    if ((currentRoute_->routeNodes_[i]->nodeStatus_ == PLANNED) ||
                        (currentRoute_->routeNodes_[i]->nodeStatus_ == COMMITTED &&
                         currentRoute_->routeNodes_[i]->initialType_ == DROPOFF)) {
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
}

void Vehicle::setCurrentRoute(PRoute &currentRoute) {
    currentRoute_ = currentRoute;
}

void Vehicle::resetBestReducedCost() {
    bestReducedCost_ = INFINITY;
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

void Vehicle::updateState(int epoch, int &epochLength) {
    if (solutionRoute_ == nullptr) {
        solutionRoute_ = std::make_shared<Route>(vehicleID_);
        emptyRoute_->routeNodes_[0]->reachTime_ = startTime_;
        emptyRoute_->routeNodes_[0]->departTime_ = departTime_;
        solutionRoute_->addSource(emptyRoute_->routeNodes_[0], departTime_, numPassengers_);
        emptyRoute_->routeNodes_[0]->nodeStatus_ = DONE;
    }
    if (currentRoute_->routeSize_ > 1) {
        // the following condition is useful for the cases that the vehicle does not have any stop in current epoch
        if (departTime_ < startTime_ + static_cast<float>((epoch+1) * epochLength)) {
            onboards_.clear();
            int breakIndex = 0;
            for (int i = 1; i < currentRoute_->routeSize_; ++i) {
                if (currentRoute_->routeNodes_[i]->related_Request_->getRequestId() == 26855)
                    std::cout << "stop";
                currentRoute_->routeNodes_[i]->nodeStatus_ = DONE;
                currentRoute_->routeNodes_[i]->reachTime_ = currentRoute_->plannedReachTime_[i];
                currentRoute_->routeNodes_[i]->departTime_ = currentRoute_->plannedDepartTime_[i];
                solutionRoute_->addNode(currentRoute_->routeNodes_[i],currentRoute_->plannedReachTime_[i],
                                        currentRoute_->plannedDepartTime_[i]);

                // set request status
                if (currentRoute_->routeNodes_[i]->type_ == PICKUP) {
                    currentRoute_->routeNodes_[i]->related_Request_->requestStatus_ = ON_BOARD;
                    currentRoute_->routeNodes_[i]->related_Request_->pickTime_ = currentRoute_->plannedReachTime_[i];
                    currentRoute_->routeNodes_[i]->related_Request_->vehicleID_ = vehicleID_;
                }

                else if (currentRoute_->routeNodes_[i]->type_ == DROPOFF){
                    currentRoute_->routeNodes_[i]->related_Request_->requestStatus_ = COMPLETED;
                    currentRoute_->routeNodes_[i]->related_Request_->dropTime_ = currentRoute_->plannedReachTime_[i];
                    // check travelTime violation
                    float travelTime = currentRoute_->routeNodes_[i]->related_Request_->dropTime_ -
                            currentRoute_->routeNodes_[i]->pairNode_->departTime_;
                    if (travelTime > currentRoute_->routeNodes_[i]->related_Request_->maxTravelTime_){
                        std::cout << "Trip delay constraint is violated by request: " <<
                                  currentRoute_->routeNodes_[i]->related_Request_->getRequestId() << std::endl;
//                        myTools::throwException("Trip delay Validation");
                    }
                }

                if (i == currentRoute_->routeSize_-1 ||
                ((currentRoute_->plannedDepartTime_[i] >= startTime_ + static_cast<float>((epoch+1) * epochLength))&&
                (currentRoute_->plannedDepartTime_[i] != currentRoute_->plannedDepartTime_[i+1]))){
                    // at depart point the vehicle is ready to leave the stop location and delta time has passed
                    departTime_ = currentRoute_->plannedDepartTime_[i];

                    // if we have reached the end of the route, next condition is checked
                    if (i == currentRoute_->routeSize_-1 && departTime_ < startTime_ + static_cast<float>((epoch+1) * epochLength)) {
                        idleTime_ += (startTime_ + static_cast<float>((epoch+1) * epochLength) - departTime_);
                        departTime_ = startTime_ + static_cast<float>((epoch + 1) * epochLength);
              //          currentRoute_->routeNodes_[i]->leaveTime_ = leaveTime_;
                        currentRoute_->plannedDepartTime_[i] = departTime_;
                        solutionRoute_->routeNodes_.back()->departTime_ = departTime_;
                        solutionRoute_->plannedDepartTime_.back() = departTime_;
                    }
                    numPassengers_ = currentRoute_->plannedPassengers_[i];
                    departNode_ =  currentRoute_->routeNodes_[i];
                    currentRoute_->routeNodes_[i]->type_ = SOURCE;
                    breakIndex = i;
                    break;
                }
            }

            // update onboards for the rest of the route after break index
            for (int i = breakIndex + 1; i < currentRoute_->routeSize_; ++i) {
                if (currentRoute_->routeNodes_[i]->related_Request_->requestStatus_ == ON_BOARD) {
                    currentRoute_->routeNodes_[i]->nodeStatus_ = PLANNED;
                    onboards_.push_back(currentRoute_->routeNodes_[i]->nodeID_);
                }
            }
            currentRoute_->removeNode(breakIndex);
        }
        if (currentRoute_->routeNodes_.size()-1 == onboards_.size())
            emptyRoute_ = currentRoute_;
    }
    else if (departTime_ < startTime_ + static_cast<float>((epoch+1) * epochLength)){
        idleTime_ += (startTime_ + static_cast<float>((epoch+1) * epochLength) - departTime_);
        departTime_ = startTime_ + static_cast<float>((epoch+1) * epochLength);
        currentRoute_->plannedDepartTime_[0] = departTime_;
//        currentRoute_->routeNodes_.back()->leaveTime_ = leaveTime_;
//        solutionRoute_->routeNodes_.back()->reachTime_ = departureTime_;
        solutionRoute_->routeNodes_.back()->departTime_ = departTime_;
        solutionRoute_->plannedDepartTime_.back() = departTime_;
    }
}

void Vehicle::updateStateTime(float elapsedTime, float &epochLength) {
    if (solutionRoute_ == nullptr) {
        solutionRoute_ = std::make_shared<Route>(vehicleID_);
        solutionRoute_->addSource(emptyRoute_->routeNodes_[0], departTime_, numPassengers_);
        solutionRoute_->plannedDepartTime_.back() = solutionRoute_->plannedReachTime_.back();
    }
    if (currentRoute_->routeSize_ > 1) {
        // the following condition is useful for the cases that the vehicle does not have any stop in current epoch
        if (departTime_ < startTime_ + elapsedTime + epochLength) {
            onboards_.clear();
            int breakIndex = 0;
            for (int i = 1; i < currentRoute_->routeSize_; ++i) {
                currentRoute_->routeNodes_[i]->nodeStatus_ = DONE;
                currentRoute_->routeNodes_[i]->reachTime_ = currentRoute_->plannedReachTime_[i];
                currentRoute_->routeNodes_[i]->departTime_ = currentRoute_->plannedDepartTime_[i];
                solutionRoute_->addNode(currentRoute_->routeNodes_[i], currentRoute_->plannedReachTime_[i],
                                        currentRoute_->plannedDepartTime_[i]);

                // set request status
                if (currentRoute_->routeNodes_[i]->type_ == PICKUP) {
                    currentRoute_->routeNodes_[i]->related_Request_->requestStatus_ = ON_BOARD;
                    currentRoute_->routeNodes_[i]->related_Request_->pickTime_ = currentRoute_->plannedReachTime_[i];
                    currentRoute_->routeNodes_[i]->related_Request_->vehicleID_ = vehicleID_;
                }

                else if (currentRoute_->routeNodes_[i]->type_ == DROPOFF){
                    currentRoute_->routeNodes_[i]->related_Request_->requestStatus_ = COMPLETED;
                    currentRoute_->routeNodes_[i]->related_Request_->dropTime_ = currentRoute_->plannedReachTime_[i];
                }

                if (i == currentRoute_->routeSize_-1 ||
                    ((currentRoute_->plannedDepartTime_[i] >= startTime_ + elapsedTime + epochLength)&&
                     (currentRoute_->plannedDepartTime_[i] != currentRoute_->plannedDepartTime_[i+1]))){
                    // at depart point the vehicle is ready to leave the stop location and delta time has passed
                    departTime_ = currentRoute_->plannedDepartTime_[i];

                    // if we have reached the end of the route, next condition is checked
                    if (i == currentRoute_->routeSize_-1 && departTime_ < startTime_ + elapsedTime + epochLength) {
                        idleTime_ += (startTime_ + elapsedTime + epochLength - departTime_);
                        departTime_ = startTime_ + elapsedTime + epochLength;
                        currentRoute_->plannedDepartTime_[i] = departTime_;
                        solutionRoute_->routeNodes_.back()->departTime_ = departTime_;
                        solutionRoute_->plannedDepartTime_.back() = departTime_;
                        idle_ = true;
                    }
                    numPassengers_ = currentRoute_->plannedPassengers_[i];
                    departNode_ =  currentRoute_->routeNodes_[i];
                    currentRoute_->routeNodes_[i]->type_ = SOURCE;
                    breakIndex = i;
                    break;
                }
            }
            for (int i = breakIndex + 1; i < currentRoute_->routeSize_; ++i) {
                if (currentRoute_->routeNodes_[i]->related_Request_->requestStatus_ == ON_BOARD) {
                    currentRoute_->routeNodes_[i]->nodeStatus_ = PLANNED;
                    onboards_.push_back(currentRoute_->routeNodes_[i]->nodeID_);
                }
/*                if (currentRoute_->plannedReachTime_[i] < startTime_ + elapsedTime + epochLength)
                    currentRoute_->routeNodes_[i]->nodeStatus_ = COMMITTED;*/
            }

            currentRoute_->removeNode(breakIndex);
        }
        if (currentRoute_->routeNodes_.size()-1 == onboards_.size())
            emptyRoute_ = currentRoute_;
    }
    else if (departTime_ < startTime_ + elapsedTime + epochLength){
        idleTime_ += (startTime_ + elapsedTime + epochLength - departTime_);
        departTime_ = startTime_ + elapsedTime + epochLength;
        currentRoute_->plannedDepartTime_[0] = departTime_;
        solutionRoute_->routeNodes_.back()->departTime_ = departTime_;
        solutionRoute_->plannedDepartTime_.back() = departTime_;
        idle_ = true;
    }
}

// this function is called at the end of algorithm to set the final stos of the solution based on final epoch
void Vehicle::finalizeSolutionRoutes(PInstance & pInst) const {
    if (idle_)
        solutionRoute_->routeNodes_.back()->departTime_ = solutionRoute_->plannedDepartTime_.back();
    if (solutionRoute_->routeNodes_.back()->type_ == SOURCE) {
        for (int i = 1; i < currentRoute_->routeSize_; ++i) {
            currentRoute_->routeNodes_[i]->nodeStatus_ = DONE;
            currentRoute_->routeNodes_[i]->reachTime_ = currentRoute_->plannedReachTime_[i];
            currentRoute_->routeNodes_[i]->departTime_ = currentRoute_->plannedDepartTime_[i];
            solutionRoute_->addNode(currentRoute_->routeNodes_[i],currentRoute_->plannedReachTime_[i],
                                     currentRoute_->plannedDepartTime_[i]);

            if (currentRoute_->routeNodes_[i]->type_ == PICKUP) {
                currentRoute_->routeNodes_[i]->related_Request_->pickTime_ = currentRoute_->plannedReachTime_[i];
                currentRoute_->routeNodes_[i]->related_Request_->vehicleID_ = vehicleID_;
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
    if (currentRoute_->routeSize_ > 1 && idle_) {
        if (idle_) {
            float departure = std::max(startTime_ + elapsedTime, solutionRoute_->plannedDepartTime_.back());
            idleTime_ -= (departTime_ - (departure));
            updateDepartTime(departure);
        }
    }
}


