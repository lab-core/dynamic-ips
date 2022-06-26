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
}
Route::~Route() = default;

// Getters and Setters
unsigned int Route::getRouteId() const {return routeID_;}

void Route::updateReducedCost(IloNumArray &requestDuals, IloNumArray &vehicleDual, std::unordered_map<unsigned int, int> &requestToOrder) {
    reducedCost_ = totalDelay_ - vehicleDual[vehicleID_];
    for (auto & requestID : routeRequests_) {
        reducedCost_ -= requestDuals[requestToOrder[requestID]];
    }
}
// these functions are used to add nodes to the routes
void Route::addSource(PNode &node, float departTime, int departPassengers) {
    routeSize_ ++;
    if (node->type_ == SOURCE) {
        routeNodes_.push_back(node);
        plannedReachTime_.push_back(departTime);
        plannedPassengers_.push_back(departPassengers);
        node->reachTime_ = departTime;
    }
}
void Route::addNode(PNode &node) {
    routeSize_ ++;
    plannedPassengers_.push_back(plannedPassengers_.back() + node->nbPassengers_);
    float reachTime = plannedReachTime_.back() + routeNodes_.back()->deltaTime_ +
            durationMatrix_[routeNodes_.back()->locationID_][node->locationID_];
    if (node->type_ == PICKUP) {
        routeRequests_.push_back(node->related_Request_->getRequestId());
        if (reachTime < node->requestTime_)
            plannedReachTime_.push_back(node->requestTime_);
        else {
            plannedReachTime_.push_back(reachTime);
            totalDelay_ += (reachTime - node->requestTime_);
        }
    }
    else
        plannedReachTime_.push_back(reachTime);
    routeNodes_.push_back(node);
}

void Route::addNode(PNode &node, float reachTime) {
    routeSize_ ++;
    plannedPassengers_.push_back(plannedPassengers_.back() + node->nbPassengers_);
    plannedReachTime_.push_back(reachTime);
    if (node->type_ == PICKUP) {
        routeRequests_.push_back(node->related_Request_->getRequestId());
        totalDelay_ += (reachTime - node->requestTime_);
    }
    routeNodes_.push_back(node);
}

void Route::addNode(PNode &node, float departTime, int departPassengers) {

    routeSize_ ++;
    if (node->type_ == SOURCE) {
        routeNodes_.push_back(node);
        plannedReachTime_.push_back(departTime);
        plannedPassengers_.push_back(departPassengers);
    }

    else
    {
        plannedPassengers_.push_back(plannedPassengers_.back() + node->nbPassengers_);
        float reachTime = plannedReachTime_.back() + routeNodes_.back()->deltaTime_ +
                          durationMatrix_[routeNodes_.back()->locationID_][node->locationID_];

        if (node->type_ == PICKUP) {
            routeRequests_.push_back(node->related_Request_->getRequestId());
            plannedReachTime_.push_back(std::max(node->requestTime_,reachTime));
            totalDelay_ += (plannedReachTime_.back() - node->requestTime_);

        }
        else
            plannedReachTime_.push_back(reachTime);
        routeNodes_.push_back(node);
    }
}

// this function is used to remove completed nodes from the routes
void Route::removeNode(int nodeIndex) {
    routeSize_ = routeSize_ - nodeIndex;
    routeNodes_.erase(routeNodes_.begin(), routeNodes_.begin()+nodeIndex);
    plannedReachTime_.erase(plannedReachTime_.begin(), plannedReachTime_.begin()+nodeIndex);
    plannedPassengers_.erase(plannedPassengers_.begin(), plannedPassengers_.begin()+nodeIndex);
    routeRequests_.clear();
    totalDelay_ = 0;
    for (int i = 1; i < routeNodes_.size(); ++i) {
        if (routeNodes_[i]->type_ == PICKUP) {
            routeRequests_.push_back(routeNodes_[i]->related_Request_->getRequestId());
            totalDelay_ += (plannedReachTime_[i] - routeNodes_[i]->requestTime_);
        }

    }
}

// Display function
std::string Route::toString() const {
    std::stringstream repStr;

    repStr << "#" << std::left << std::endl;
    repStr << "#\t" << std::setw(24) << "- ROUTE_NUMBER" << " : " << routeID_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- VEHICLE_ID" << " : " << vehicleID_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- NUMBER_OF_STOPS" << " : " << routeSize_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- TOTAL_WAITING (seconds)" << " : " << totalDelay_ << std::endl;
    repStr << "#" << std::endl;

    // print table header
    repStr << "# ------------------------------------------------------------------------" << std::endl;
    repStr << std::left << std::setw(6) << "#      ";
    repStr << std::left << std::setw(27) << "ACTION_DESCRIPTION";
    repStr << std::left << std::setw(11) << "NODE_ID" << std::right;
    repStr << std::right << std::setw(11) << " REACH_TIME"<< " (s)  ";
    repStr << "#PASSENGERS" <<std::endl;
    repStr << "# ------------------------------------------------------------------------" << std::endl;

    // print the source stop pint
    repStr << "#" << std::setw(4) << 1 << "  ";
    repStr << std::left << std::setw(27) << "(SOURCE ) departure";
    repStr << std::left << std::setw(11) << routeNodes_[0]->nodeID_;
    repStr << std::right << std::setw(11) << plannedReachTime_[0] << " (s)  ";
    repStr << std::setw(7) << plannedPassengers_[0] << std::endl;

    // print the internal nodes of the route
    for (int i = 1; i < routeSize_; ++i) {
        repStr << "#" << std::setw(4) << i + 1 << "  ";
        if (routeNodes_[i]->type_ == SINK)
            repStr << std::left << std::setw(27) << "(SINK   ) return";
        else {
            repStr << "(" << NodeTypeStr[routeNodes_[i]->type_] << ") Request_ID ";
            repStr << std::left << std::setw(6) << routeNodes_[i]->related_Request_->getRequestId();
        }
        repStr << std::left << std::setw(11) << routeNodes_[i]->nodeID_;
        repStr << std::right << std::setw(11) << plannedReachTime_[i] << " (s)  ";
        float reachTime = durationMatrix_[routeNodes_[i-1]->locationID_][routeNodes_[i]->locationID_] +
                routeNodes_[i]->requestTime_;
        if (reachTime > plannedReachTime_[i])
            std::cout << "Hi";
        repStr << std::setw(7) << plannedPassengers_[i] << std::endl;
    }

    repStr << "==========================================================================" << std::endl;
    return repStr.str();
}

















