//
// Created by Ella on 2021-10-26.
//

#include "Route.h"


//---------------------------------------------------------------------------------------------
//  Route class
//  Route for each vehicle
//---------------------------------------------------------------------------------------------

unsigned int Route::routeCount_ = 0;

// Constructor and Destructor
Route::Route(int vehicleId) : routeID_(routeCount_++), vehicleID_(vehicleId) {
    totalDelay_ = 0.0;
    reducedCost_ = 0.0;
    routeSize_ = 0;
    incompatibilityDegree_ = 0;
    char* name2 = new char[255];
    strncpy(name2, std::to_string(routeID_).c_str(), 255);
    name_ = name2;
    isCompatible_ = false;
}
Route::~Route(){
    delete[] name_;
}

// Getters and Setters
unsigned int Route::getRouteId() const {return routeID_;}

void Route::updateReducedCost(IloNumArray &requestDuals, IloNumArray &vehicleDual) {
    reducedCost_ = totalDelay_ - vehicleDual[vehicleID_];
    for (auto & requestObj : routeRequests_) {
        reducedCost_ -= requestDuals[requestObj->taskIndex_];
    }
}
// these functions are used to add nodes to the routes
void Route::addSource(PNode &node, float departTime, int departPassengers) {
    routeSize_ ++;
    if (node->type_ == SOURCE) {
        routeNodes_.push_back(node);
        plannedReachTime_.push_back(node->reachTime_);
        plannedDepartTime_.push_back(departTime);
        plannedPassengers_.push_back(departPassengers);
        /*if (node->initialType_ == SOURCE)
            node->reachTime_ = departTime;*/
        node->departTime_ = departTime;
    }
}
void Route::addNode(PNode &node) {
    routeSize_ ++;
    plannedPassengers_.push_back(plannedPassengers_.back() + node->nbPassengers_);
    if (node->initialType_ == PICKUP && plannedDepartTime_.back() < node->requestTime_)
        plannedDepartTime_.back() = node->requestTime_;
    float reachTime = plannedDepartTime_.back() + durationMatrix_[routeNodes_.back()->locationID_][node->locationID_];
    plannedReachTime_.push_back(reachTime);
    plannedDepartTime_.push_back(reachTime + node->deltaTime_);
    routeNodes_.push_back(node);

    if (node->initialType_ == PICKUP) {
        routeRequests_.push_back(node->related_Request_);
        totalDelay_ += (reachTime - node->requestTime_);
    }
}

void Route::addNode1(PNode &node) {
    routeSize_ ++;
    plannedPassengers_.push_back(plannedPassengers_.back() + node->nbPassengers_);
    if (node->initialType_ == PICKUP && plannedDepartTime_.back() < node->requestTime_)
        plannedDepartTime_.back() = node->requestTime_;

    float reachTime = plannedDepartTime_.back() + durationMatrix_[routeNodes_.back()->locationID_][node->locationID_];
    plannedReachTime_.push_back(reachTime);
    if ((durationMatrix_[routeNodes_.back()->locationID_][node->locationID_] == 0)&&(routeNodes_.back()->initialType_ != SOURCE))
        plannedDepartTime_.push_back(reachTime);
    else
        plannedDepartTime_.push_back(reachTime + node->deltaTime_);

    routeNodes_.push_back(node);
    if (node->initialType_ == PICKUP) {
        routeRequests_.push_back(node->related_Request_);
        totalDelay_ += (reachTime - node->requestTime_);
    }
}

void Route::addNode(PNode &node, float reachTime, float departTime) {
    routeSize_ ++;
    plannedPassengers_.push_back(plannedPassengers_.back() + node->nbPassengers_);
    plannedReachTime_.push_back(reachTime);
    plannedDepartTime_.push_back(departTime);
//    plannedDepartTime_.push_back(reachTime + node->deltaTime_);
    if (node->initialType_ == PICKUP) {
        routeRequests_.push_back(node->related_Request_);
        totalDelay_ += (reachTime - node->requestTime_);
    }
    routeNodes_.push_back(node);
}


// this function is used to remove completed nodes from the routes
void Route::removeNode(int nodeIndex) {
    routeSize_ = routeSize_ - nodeIndex;
    routeNodes_.erase(routeNodes_.begin(), routeNodes_.begin()+nodeIndex);
    plannedReachTime_.erase(plannedReachTime_.begin(), plannedReachTime_.begin()+nodeIndex);
    plannedDepartTime_.erase(plannedDepartTime_.begin(), plannedDepartTime_.begin()+nodeIndex);
    plannedPassengers_.erase(plannedPassengers_.begin(), plannedPassengers_.begin()+nodeIndex);
    routeRequests_.clear();
    totalDelay_ = 0;
    for (int i = 1; i < routeNodes_.size(); ++i) {
        if (routeNodes_[i]->initialType_ == PICKUP) {
            routeRequests_.push_back(routeNodes_[i]->related_Request_);
            totalDelay_ += (plannedReachTime_[i] - routeNodes_[i]->requestTime_);
        }

    }
}

// Display function
std::string Route::toString() const {
    std::stringstream repStr;

    repStr << std::left;
    repStr << "#\t" << std::setw(25) << "- ROUTE_NUMBER" << " : " << routeID_ << std::endl;
    repStr << "#\t" << std::setw(25) << "- VEHICLE_ID" << " : " << vehicleID_ << std::endl;
    repStr << "#\t" << std::setw(25) << "- NUMBER_OF_STOPS" << " : " << routeSize_ << std::endl;
    repStr << "#\t" << std::setw(25) << "- TOTAL_WAITING (seconds)" << " : " << totalDelay_ << std::endl;

    repStr << "#" << std::endl;

    // print table header
    repStr << "# ------------------------------------------------------------------------------------------------------------------" << std::endl;
    repStr << std::left << std::setw(6) << "#      ";
    repStr << std::left << std::setw(22) << "ACTION_DESCRIPTION";
    repStr << std::left << std::setw(9) << "NODE_ID" << std::right;
    repStr << std::right << std::setw(9) << " REACH_TIME"<< "(s)  ";
    repStr << std::right << std::setw(9) << " DEPART_TIME"<< "(s)  ";
    repStr << std::right << std::setw(9) << " TRAVEL_TIME"<< "(s)  ";
    repStr << std::right << std::setw(10) << " EARLy_PICK"<< "(s)  ";
    repStr << "#PASSENGERS" <<std::endl;
    repStr << "# ------------------------------------------------------------------------------------------------------------------" << std::endl;

    // print the source stop pint
    repStr << "#" << std::setw(4) << 1 << "  ";
    repStr << std::left << std::setw(23) << "(SOURCE ) departure";
    repStr << std::left << std::setw(9) << routeNodes_[0]->nodeID_;
    repStr << std::right << std::setw(9) << routeNodes_[0]->reachTime_ << " (s)  ";
    repStr << std::right << std::setw(9) << routeNodes_[0]->departTime_ << " (s)  ";
    repStr << std::right << std::setw(11) << "0" << " (s)  ";
    repStr << std::right << std::setw(11) << "0" << " (s)  ";
    repStr << std::setw(7) << plannedPassengers_[0] << std::endl;

    // print the internal nodes of the route
    for (int i = 1; i < routeSize_; ++i) {
        repStr << "#" << std::setw(4) << i + 1 << "  ";
        if (routeNodes_[i]->type_ == SINK)
            repStr << std::left << std::setw(24) << "(SINK   ) return";
        else {
            repStr << "(" << NodeTypeStr[routeNodes_[i]->initialType_] << ") REQ ";
            repStr << std::left << std::setw(9) << routeNodes_[i]->related_Request_->getRequestId();
        }
        repStr << std::left << std::setw(9) << routeNodes_[i]->nodeID_;
        repStr << std::right << std::setw(9) << routeNodes_[i]->reachTime_ << " (s)  ";
        repStr << std::right << std::setw(9) << routeNodes_[i]->departTime_ << " (s)  ";
        if ((routeNodes_[i]->departTime_ != plannedDepartTime_[i])||(routeNodes_[i]->reachTime_ != plannedReachTime_[i])){
            std::cout << "Connectivity constraint violated at node : ";
            std::cout << routeNodes_[i]->nodeID_ << std::endl;
 //           myTools::throwException("Route-Validation");
        }
        repStr << std::right << std::setw(11) << durationMatrix_[routeNodes_[i-1]->locationID_][routeNodes_[i]->locationID_] << " (s)  ";
        if (routeNodes_[i]->initialType_ == PICKUP)
            repStr << std::right << std::setw(11) << routeNodes_[i]->related_Request_->earlyPick_ << " (s)  ";
        else
            repStr << std::right << std::setw(11) << "-----" << " (s)  ";
        repStr << std::setw(7) << plannedPassengers_[i] << std::endl;
    }

    repStr << "====================================================================================================================" << std::endl;
    return repStr.str();
}

// this function is for testing the validation of the route
void Route::testRoute(PVehicle & vehicle, PParameters &parameters) {
    PRoute testRoute = std::make_shared<Route>(vehicleID_);
    testRoute->addSource(routeNodes_[0], vehicle->solutionRoute_->routeNodes_[0]->departTime_,
                         vehicle->solutionRoute_->plannedPassengers_[0]);

    for (int i = 1; i < routeSize_; ++i) {
        testRoute->addNode1(routeNodes_[i]);
        if (testRoute->routeNodes_.back()->reachTime_ != testRoute->plannedReachTime_.back()){
            std::cout << "Connectivity constraint violated at node : ";
            std::cout << testRoute->routeNodes_.back()->nodeID_ << std::endl;
 //           myTools::throwException("Route-Validation");
        }
        if (testRoute->plannedPassengers_.back() == 0){
            testRoute->plannedDepartTime_.back() = testRoute->routeNodes_.back()->departTime_;
        }

        if (testRoute->routeNodes_.back()->departTime_ != testRoute->plannedDepartTime_.back()){
            std::cout << "Connectivity constraint violated at node : ";
            std::cout << testRoute->routeNodes_.back()->nodeID_ << std::endl;
 //           myTools::throwException("Route-Validation");
        }

        // checking request data (pick up and drop off time)
        if (testRoute->routeNodes_.back()->initialType_ == PICKUP) {
            if (testRoute->routeNodes_.back()->reachTime_ !=
                testRoute->routeNodes_.back()->related_Request_->pickTime_) {
                std::cout << "Request pickup time is not equal to the pick node reach time at node : ";
                std::cout << testRoute->routeNodes_.back()->nodeID_ << std::endl;
 //               myTools::throwException("Route-Validation");
            }
        }
        else if (testRoute->routeNodes_.back()->initialType_ == DROPOFF) {
            if (testRoute->routeNodes_.back()->reachTime_ !=
                testRoute->routeNodes_.back()->related_Request_->dropTime_) {
                std::cout << "Request drop-off time is not equal to the drop node reach time at node : ";
                std::cout << testRoute->routeNodes_.back()->nodeID_ << std::endl;
 //               myTools::throwException("Route-Validation");
            }
        }
 /*//       if (mainAlgorithm != GREEDY){
//            if ((routeNodes_[i]->departureTime_ != testRoute->plannedReachTime_.back())&&(i != routeSize_-1))
            if ((routeNodes_[i]->departureTime_ != testRoute->plannedReachTime_.back()))
                testRoute->plannedReachTime_.back() = routeNodes_[i]->departureTime_;
//        }*/

        // checking capacity constraints
        if (testRoute->plannedPassengers_.back() > vehicle->capacity_){
            std::cout << "Capacity constraint violated at node : " << testRoute->routeNodes_.back()->nodeID_ << std::endl;
//            myTools::throwException("Route-Validation");
        }

        // checking trip delay constraint
        if (testRoute->routeNodes_.back()->initialType_ == DROPOFF){
            float travelTime = testRoute->routeNodes_.back()->related_Request_->dropTime_ -
                    testRoute->routeNodes_.back()->pairNode_->departTime_;
            if (travelTime > testRoute->routeNodes_.back()->related_Request_->maxTravelTime_ + 0.1){
                std::cout << "Trip delay constraint violated for request : " <<
                testRoute->routeNodes_.back()->related_Request_->getRequestId() << std::endl;
 //               myTools::throwException("Route-Validation");
            }
        }
    }

    if (testRoute->totalDelay_ != totalDelay_){
        std::cout << "Total delay is not the same" << std::endl;
 //       myTools::throwException("Route-Validation");
    }
    if (testRoute->plannedPassengers_.back() != plannedPassengers_.back()){
        std::cout << "Final Load is not the same" << std::endl;
 //       myTools::throwException("Route-Validation");
    }
    if (testRoute->plannedReachTime_.back()!= plannedReachTime_.back()){
//        std::cout << "End time is not the same" << std::endl;
//        myTools::throwException("Route-Validation");
    }


    /*std::cout << "####################### TEST Route ##########################" << std::endl;
    std::cout << testRoute->toString() << std::endl;
    std::cout << "##################### SOLUTION Route ########################" << std::endl;
    std::cout << toString() << std::endl;*/
}
// This function is to reset the status of the nodes in the route
void Route::resetRoute() {
    for (int i = routeSize_-1; i > 0; --i) {
        routeNodes_[i]->nodeStatus_ = DEFINED;
        if (routeNodes_[i]->type_ == DROPOFF) {
            routeNodes_[i]->related_Request_->requestStatus_ = ON_BOARD;
            routeNodes_[i]->related_Request_->dropTime_ = MAXReachTime;
        }
        else if (routeNodes_[i]->type_ == PICKUP) {
            routeNodes_[i]->related_Request_->requestStatus_ = NO_ACTION;
            routeNodes_[i]->related_Request_->pickTime_ = MAXReachTime;
            routeNodes_[i]->related_Request_->dropTime_ = MAXReachTime;
        }
    }
}

void Route::createColumn(int size) {
//    column_ = std::make_shared<myTools::BitVector>(size);
    column_.reset();
    /*for (auto & requestObj: routeRequests_)
        column_->add(requestObj->taskIndex_);*/
    for (auto & requestObj: routeRequests_)
        column_.set(requestObj->taskIndex_, true);
}


/*void Route::createFullPattern(std::unordered_map<unsigned int, int>& incRequestToOrder,
                              std::unordered_map<int, int> &incVehicleToOrder) {
    fullPattern_ = Eigen::MatrixXd::Zero((int) incRequestToOrder.size() + (int) incVehicleToOrder.size(),1);
    for (auto & requestObj : routeRequests_) {
        if (incRequestToOrder.count(requestObj) > 0)
            fullPattern_(incRequestToOrder[requestObj], 0) = 1;
    }
    if (incVehicleToOrder.count(vehicleID_)>0)
        fullPattern_(incVehicleToOrder[vehicleID_],0) = 1;
}*/


















