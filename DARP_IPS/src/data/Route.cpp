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
    incompatibilityDegree = 0;
    char* name2 = new char[255];
    strncpy(name2, std::to_string(routeID_).c_str(), 255);
    name_ = name2;
}
Route::~Route() {}

// Getters and Setters
void Route::setIncompatibilityDegree(float incompatibilityDegree) {
    Route::incompatibilityDegree = incompatibilityDegree;
}

const unsigned int Route::getRouteId() const {return routeID_;}

void Route::updateReducedCost(IloNumArray &requestDuals, IloNumArray &vehicleDual, std::map<int, int> &requestToOrder) {
    reducedCost_ = totalDelay_ - vehicleDual[vehicleID_];
    for (int r = 0; r < routeRequests.size(); ++r) {
        reducedCost_ -= requestDuals[requestToOrder[routeRequests[r]]];
    }
}

// this function is used to add nodes to the routes
void Route::addNode(PNode node, float departTime, int departPassengers) {

    routeSize_ ++;
    if (node->type_ == SOURCE) {
        routeNodes_.push_back(node);
        plannedReachTime_.push_back(departTime);
        plannedPassengers_.push_back(departPassengers);
    }

    else
    {
        plannedPassengers_.push_back(plannedPassengers_.back() + node->nbPassengers_);
        float reachTime = plannedReachTime_.back()+ routeNodes_.back()->deltaTime_ +
                          calcTravelTime(routeNodes_.back(), node);
        if (node->type_ == PICKUP) {
            routeRequests.push_back((*node->related_Request_)->getRequestId());
            // a request can not be picked up before its early pick time
            if (reachTime < node->requestTime_)
                plannedReachTime_.push_back(node->requestTime_);
            else {
                plannedReachTime_.push_back(reachTime);
                totalDelay_ += (reachTime - node->requestTime_);
            }
        }
        else   // for sink or drop off nodes
            plannedReachTime_.push_back(reachTime);
        routeNodes_.push_back(node);
    }
}

// Display function
std::string Route::toString() const {
    std::stringstream repStr;
    repStr << "==========================================================================" << std::endl;
    repStr << "#" << std::left << std::endl;
    repStr << "#\t" << std::setw(24) << "- ROUTE_NUMBER" << " : " << routeID_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- VEHICLE_ID" << " : " << vehicleID_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- NUMBER_OF_STOPS" << " : " << routeSize_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- TOTAL_DELAY (seconds)" << " : " << totalDelay_ << std::endl;
    repStr << "#" << std::endl;
    repStr << "#" << std::endl;

    // print table header
    repStr << "# ------------------------------------------------------------------------" << std::endl;
    repStr << std::left << std::setw(6) << "#      ";
    repStr << std::left << std::setw(27) << "ACTION_DESCRIPTION";
    repStr << std::left << std::setw(11) << "NODE_ID" << std::right;
    repStr << std::right << std::setw(11) << " REACH_TIME"<< " (s)  ";
    repStr << "#PASSENGERS" <<std::endl;
 //   repStr << "# ________________________________________________________________________" << std::endl;
    repStr << "# ------------------------------------------------------------------------" << std::endl;

    // print the source stop pint
    repStr << "#" << std::setw(4) << 1 << "  ";
    repStr << std::left << std::setw(27) << "(SOURCE ) departure";
    repStr << std::left << std::setw(11) << routeNodes_[0]->nodeID_;
    repStr << std::right << std::setw(11) << plannedReachTime_[0] << " (s)  ";
    repStr << std::setw(7) << plannedPassengers_[0] << std::endl;

    // print the internal nodes of the route
    for (int i = 1; i < routeSize_ -1; ++i) {
        repStr << "#" << std::setw(4) << i + 1 << "  ";
        repStr << "(" << NodeTypeStr[routeNodes_[i]->type_]<< ") Request_ID ";
        repStr << std::left << std::setw(6) <<  (*routeNodes_[i]->related_Request_)->getRequestId();
        repStr << std::left << std::setw(11) << routeNodes_[i]->nodeID_;
        repStr << std::right << std::setw(11) << plannedReachTime_[i] << " (s)  ";
        repStr << std::setw(7) << plannedPassengers_[i] << std::endl;
    }

    // print the sink stop point
    repStr << "#" << std::setw(4) << routeSize_ << "  ";
    repStr << std::left << std::setw(27) << "(SINK   ) return";
    repStr << std::left << std::setw(11) << routeNodes_.back()->nodeID_;
    repStr << std::right << std::setw(11) << plannedReachTime_.back() << " (s)  ";
    repStr << std::setw(7) << plannedPassengers_.back() << std::endl;
    repStr << "# ________________________________________________________________________" << std::endl;
    return repStr.str();
}










