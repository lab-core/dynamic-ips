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
Vehicle::Vehicle(int vehicleId, int capacity, float departTime, float endTime, std::string departID,
                 std::string sinkID) : vehicleID_(vehicleId), capacity_(capacity), departTime_(departTime),
                 endTime_(endTime), departID_(std::move(departID)), sinkID_(std::move(sinkID)) {
    numPassengers_ = 0;
//    departID_ = Tools::createNodeID(0, SOURCE);
//    sinkID_ = Tools::createNodeID(0, SINK);
    dual_=0;
    bestReducedCost_ = 9999;
    idleTime_ = 0;
    startTime_ = departTime;
}

Vehicle::~Vehicle() = default;

// Setters
void Vehicle::setDepartTime(float departTime) {
    departTime_ = departTime;
}

// this function create an empty route and set it as the CurrentRoute_, used in initialization
void Vehicle::setEmptyRoute(PInstance &pInst) {
    PRoute newRoute = std::make_shared<Route>(vehicleID_);
    newRoute->addSource(pInst->instGraph_->nodes_[departID_], departTime_, numPassengers_);

    if (!onboards_.empty()) {
        for (auto &nodeID: onboards_) {
            newRoute->addNode(pInst->instGraph_->nodes_[nodeID]);
        }
    }
//    newRoute->addNode(pInst->instGraph_->nodes_[sinkID_], departTime_, nbPassengers_);

    emptyRoute_ = newRoute;
}

void Vehicle::setCurrentRoute(PRoute &currentRoute) {
    currentRoute_ = currentRoute;
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
    repStr << "#\t" << std::setw(24) << "- DEPART_NODE_ID" << " : " << departID_ << std::endl;
    repStr << "#" << std::endl;
    return repStr.str();
}

// function to update vehicle depart time at each time and
// update the situation of nodes and ride requests

void Vehicle::updateState(int epoch, int &epochLength) {
    if (solutionRoute_ == nullptr) {
        solutionRoute_ = std::make_shared<Route>(vehicleID_);
        solutionRoute_->addSource(emptyRoute_->routeNodes_[0], departTime_, numPassengers_);
        /*if (departTime_ < startTime_ + (epoch+1) * epochLength) {
            departTime_ = startTime_ + (epoch + 1) * epochLength;
            currentRoute_ = solutionRoute_;
        }*/
    }
    if (currentRoute_->routeSize_ > 1) {
        // the following constraint is useful for the cases that the vehicle does not have any stop in current epoch
        if (departTime_ < startTime_ + static_cast<float>((epoch+1) * epochLength)) {
            onboards_.clear();
            int breakIndex = 0;
            for (int i = 1; i < currentRoute_->routeSize_; ++i) {
                if (currentRoute_->routeNodes_[i]->related_Request_->getRequestId() == 260)
                    std::cout << "HI" << std::endl;
                currentRoute_->routeNodes_[i]->nodeStatus_ = DONE;
                currentRoute_->routeNodes_[i]->reachTime_ = currentRoute_->plannedReachTime_[i];
                currentRoute_->routeNodes_[i]->departTime_ = currentRoute_->plannedReachTime_[i];
                solutionRoute_->addNode(currentRoute_->routeNodes_[i],
                                        currentRoute_->plannedReachTime_[i]);

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

                if ((currentRoute_->plannedReachTime_[i] >= startTime_ + static_cast<float>((epoch+1) * epochLength))||(i == currentRoute_->routeSize_-1)){

                    // at depart point the vehicle is ready to leave the stop location and delta time has passed
                    departTime_ = currentRoute_->plannedReachTime_[i] + currentRoute_->routeNodes_[i]->deltaTime_;
                    if (departTime_ < startTime_ + static_cast<float>((epoch+1) * epochLength)) {
                        idleTime_ += startTime_ + static_cast<float>((epoch+1) * epochLength) - departTime_;
                        departTime_ = startTime_ + static_cast<float>((epoch + 1) * epochLength);
                    }
                    numPassengers_ = currentRoute_->plannedPassengers_[i];
                    departID_ =  currentRoute_->routeNodes_[i]->nodeID_;
                    currentRoute_->routeNodes_[i]->type_ = SOURCE;
                    breakIndex = i;
                    break;
                }
            }

            //          for (int i = breakIndex + 1; i < currentRoute_->routeSize_-1; ++i) {
            for (int i = breakIndex + 1; i < currentRoute_->routeSize_; ++i) {
                if (currentRoute_->routeNodes_[i]->related_Request_->requestStatus_ == ON_BOARD) {
                    currentRoute_->routeNodes_[i]->nodeStatus_ = PLANNED;
                    onboards_.push_back(currentRoute_->routeNodes_[i]->nodeID_);
                }
            }
            currentRoute_->removeNode(breakIndex);
        }
        if (currentRoute_->routeNodes_.size()-2 == onboards_.size())
            emptyRoute_ = currentRoute_;
    }
    else if (departTime_ < startTime_ + static_cast<float>((epoch+1) * epochLength)){
        idleTime_ += startTime_ + static_cast<float>((epoch+1) * epochLength) - departTime_;
        departTime_ = startTime_ + static_cast<float>((epoch+1) * epochLength);
        currentRoute_->plannedReachTime_[0] = departTime_;
//        solutionRoute_->routeNodes_.back()->reachTime_ = departTime_;
        solutionRoute_->routeNodes_.back()->departTime_ = departTime_;
        solutionRoute_->plannedReachTime_.back() = departTime_;
    }
}

void Vehicle::updateStateTime(float stopTime) {
    // the following constraint is useful for the cases that the vehicle does not have any stop in current epoch
    if (departTime_ < startTime_ + stopTime) {
        onboards_.clear();
        int breakIndex = 0;
        for (int i = 1; i < currentRoute_->routeSize_; ++i) {
            if (currentRoute_->routeNodes_[i]->related_Request_->getRequestId() == 260)
                std::cout << "HI" << std::endl;
            currentRoute_->routeNodes_[i]->nodeStatus_ = DONE;
            currentRoute_->routeNodes_[i]->reachTime_ = currentRoute_->plannedReachTime_[i];
            currentRoute_->routeNodes_[i]->departTime_ = currentRoute_->plannedReachTime_[i];
            solutionRoute_->addNode(currentRoute_->routeNodes_[i], currentRoute_->plannedReachTime_[i]);

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

            if ((currentRoute_->plannedReachTime_[i] >= startTime_ + stopTime)||(i == currentRoute_->routeSize_-1)){

                // at depart point the vehicle is ready to leave the stop location and delta time has passed
                departTime_ = currentRoute_->plannedReachTime_[i] + currentRoute_->routeNodes_[i]->deltaTime_;
                if (departTime_ < startTime_ + stopTime)
                    departTime_ = startTime_ + stopTime;
                numPassengers_ = currentRoute_->plannedPassengers_[i];
                departID_ =  currentRoute_->routeNodes_[i]->nodeID_;
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
        }
        currentRoute_->removeNode(breakIndex);
    }
}
