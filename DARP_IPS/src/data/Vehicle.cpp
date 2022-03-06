//
// Created by Ella on 2021-09-08.
//

#include "Vehicle.h"

//-----------------------------------------------------------------------------
//  vehicle class
//  contains the vehicle info like working time, capacity and passengers
//-----------------------------------------------------------------------------

// Constructor and Destructor
Vehicle::Vehicle(int vehicleId, int capacity, float startTime, float endTime) : vehicleID_(
        vehicleId), capacity_(capacity), startTime_(startTime), endTime_(endTime) {
    numPassengers_ = 0;
//    departTime_ = 2 * epochLength;
//    departTime_ = 100;
    departID_ = Tools::createNodeID(0, SOURCE);
    sinkID_ = Tools::createNodeID(0, SINK);
    dual_=0;
    bestReducedCost_ = 9999;
}

Vehicle::~Vehicle() {}

// Setters
void Vehicle::setDepartTime(float departTime) {
    departTime_ = departTime;
}

// this function create an empty route and set it as the CurrentRoute_, used in initialization
void Vehicle::setEmptyRoute(PInstance &pInst) {
    PRoute newRoute = std::make_shared<Route>(vehicleID_);
    newRoute->addSource(pInst->instGraph_->nodes_[departID_], departTime_, numPassengers_);

    if (onboards_.size() > 0) {
        for (auto &nodeID: onboards_) {
            newRoute->addNode(pInst->instGraph_->nodes_[nodeID]);
        }
    }
//    newRoute->addNode(pInst->instGraph_->nodes_[sinkID_], departTime_, numPassengers_);

    emptyRoute_ = newRoute;
}

void Vehicle::setCurrentRoute(PRoute currentRoute) {
    currentRoute_ = currentRoute;
}
void Vehicle::setEndTime(float endTime) {
    endTime_ = endTime;
}


// Display function
std::string Vehicle::toString() const {
    std::stringstream repStr;
    repStr << std::left;
    repStr << "# VEHICLE ( " << vehicleID_ << " )" << std::endl;;
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
    if (epoch == 1) {
        solutionRoute_ = std::make_shared<Route>(vehicleID_);
        solutionRoute_->addSource(emptyRoute_->routeNodes_[0], departTime_, numPassengers_);
    }
    else if (currentRoute_->routeSize_ > 1) {
        // the following constraint is useful for the cases that the vehicle does not have any stop in current epoch
        if (departTime_ < (epoch+1) * epochLength) {
            onboards_.clear();
            int breakIndex = 0;
            for (int i = 1; i < currentRoute_->routeSize_; ++i) {
                currentRoute_->routeNodes_[i]->nodeStatus_ = DONE;
                currentRoute_->routeNodes_[i]->reachTime_ = currentRoute_->plannedReachTime_[i];
                solutionRoute_->addNode(currentRoute_->routeNodes_[i]);

                // set request status
                if (currentRoute_->routeNodes_[i]->type_ == PICKUP) {
                    (*currentRoute_->routeNodes_[i]->related_Request_)->requestStatus_ = ON_BOARD;
                    (*currentRoute_->routeNodes_[i]->related_Request_)->pickTime_ = currentRoute_->plannedReachTime_[i];
                    (*currentRoute_->routeNodes_[i]->related_Request_)->vehicleID_ = vehicleID_;
                }

                else if (currentRoute_->routeNodes_[i]->type_ == DROPOFF){
                    (*currentRoute_->routeNodes_[i]->related_Request_)->requestStatus_ = COMPLETED;
                    (*currentRoute_->routeNodes_[i]->related_Request_)->dropTime_ = currentRoute_->plannedReachTime_[i];
                }

                if ((currentRoute_->plannedReachTime_[i] >= (epoch+1) * epochLength)||(i == currentRoute_->routeSize_-1)){

                    // at depart point the vehicle is ready to leave the stop location and delta time has passed
                    departTime_ = currentRoute_->plannedReachTime_[i] + currentRoute_->routeNodes_[i]->deltaTime_;
                    if (departTime_ < (epoch+1) * epochLength)
                        departTime_ = (epoch+1) * epochLength;
                    numPassengers_ = currentRoute_->plannedPassengers_[i];
                    departID_ =  currentRoute_->routeNodes_[i]->nodeID_;
                    currentRoute_->routeNodes_[i]->type_ = SOURCE;
                    breakIndex = i;
                    break;
                }
            }

            //          for (int i = breakIndex + 1; i < currentRoute_->routeSize_-1; ++i) {
            for (int i = breakIndex + 1; i < currentRoute_->routeSize_; ++i) {
                if ((*currentRoute_->routeNodes_[i]->related_Request_)->requestStatus_ == ON_BOARD) {
                    currentRoute_->routeNodes_[i]->nodeStatus_ = PLANNED;
                    onboards_.push_back(currentRoute_->routeNodes_[i]->nodeID_);
                }
            }
            currentRoute_->removeNode(breakIndex);
        }
        if (currentRoute_->routeNodes_.size()-2 == onboards_.size())
            emptyRoute_ = currentRoute_;
    }
}