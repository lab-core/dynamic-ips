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
    totalWait_ = 0.0;
    totalTripDelay_ = 0.0;
    reducedCost_ = 0.0;
    routeSize_ = 0;
    incompatibilityDegree_ = 0;
    char *name2 = new char[255];
    strncpy(name2, std::to_string(routeID_).c_str(), 255);
    name_ = name2;
    isCompatible_ = false;
    mpAdded_ = false;
    cpAdded_ = false;
    normal_RC_ = 0;
    lambda_ = 0;
    waitScore_ = 0;
    createTime_ = 0;
    totalLength_ = 0;
    isCompatible_ = true;
    nbCommitted_ = 0;
    objCoef_ = 0.0;
    IncScoreRatio_ = 0;
    IncScore_ = 0;
    keepMP_ = false;
}

Route::~Route(){
    delete[] name_;
}

void Route::setKey() {
    std::vector<int> ids;
    ids.reserve(routeRequests_.size());
    for (const auto &req : routeRequests_) {
        ids.push_back(req->getRequestId());
    }

    std::string key = std::to_string(objCoef_) + "|";
    for (size_t i = 0; i < ids.size(); ++i) {
        if (i) key += ",";
        key += std::to_string(ids[i]);
    }
    key_ = std::move(key);
}

void Route::swapState(Route &other) noexcept {
    using std::swap;

    swap(plannedDepartTime_, other.plannedDepartTime_);
    swap(plannedReachTime_,  other.plannedReachTime_);
    swap(plannedDelay_,      other.plannedDelay_);
    swap(totalWait_,         other.totalWait_);
    swap(totalTripDelay_,    other.totalTripDelay_);
    swap(objCoef_,           other.objCoef_);
    swap(routeRequests_,     other.routeRequests_);
    swap(routeNodes_,        other.routeNodes_);
    swap(plannedPassengers_, other.plannedPassengers_);
    swap(nbCommitted_,       other.nbCommitted_);
    swap(routeSize_,         other.routeSize_);
    swap(waitScore_,         other.waitScore_);
}

// Getters and Setters
unsigned int Route::getRouteId() const {return routeID_;}


// Display function
std::string Route::toString() const {
    std::stringstream repStr;

    repStr << std::left;
    repStr << "#\t" << std::setw(25) << "- ROUTE_NUMBER" << " : " << routeID_ << std::endl;
    repStr << "#\t" << std::setw(25) << "- VEHICLE_ID" << " : " << vehicleID_ << std::endl;
    repStr << "#\t" << std::setw(25) << "- NUMBER_OF_STOPS" << " : " << routeSize_ << std::endl;
    repStr << "#\t" << std::setw(25) << "- TOTAL_WAITING (seconds)" << " : " << totalWait_ << std::endl;
    repStr << "#\t" << std::setw(25) << "- TRIP_DELAY (seconds)" << totalTripDelay_ << std::endl;

    repStr << "#" << std::endl;

    // print table header
    repStr << "# ------------------------------------------------------------------------------------------------------------------" << std::endl;
    repStr << std::left << std::setw(6) << "#      ";
    repStr << std::left << std::setw(22) << "ACTION_DESCRIPTION";
    repStr << std::left << std::setw(9) << " NODE_ID" << std::right;
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
        if (routeNodes_[i]->initialType_ == SINK)
            repStr << std::left << std::setw(23) << "(SINK   ) return   ";
        else {
            repStr << "(" << eu::toString(routeNodes_[i]->initialType_) << ") REQ ";
            repStr << std::left << std::setw(9) << routeNodes_[i]->related_Request_->getRequestId();
        }
        repStr << std::left << std::setw(9) << routeNodes_[i]->nodeID_;
        repStr << std::right << std::setw(9) << plannedReachTime_[i] << " (s)  ";
        repStr << std::right << std::setw(9) << plannedDepartTime_[i] << " (s)  ";
        if (routeNodes_[i]->initialType_ != SINK && routeNodes_[i]->nodeStatus_ == DONE) {
            if ((routeNodes_[i]->departTime_ != plannedDepartTime_[i]) ||
                (routeNodes_[i]->reachTime_ != plannedReachTime_[i])) {
                std::cout << "Connectivity constraint violated at node : ";
                std::cout << routeNodes_[i]->nodeID_ << std::endl;
                //           myTools::throwException("Route-Validation");
            }
        }
        repStr << std::right << std::setw(11) << durationMatrix_[routeNodes_[i-1]->locationID_][routeNodes_[i]->locationID_] << " (s)  ";
        if (routeNodes_[i]->initialType_ == PICKUP)
            repStr << std::right << std::setw(11) << routeNodes_[i]->related_Request_->initialEarlyPick_ << " (s)  ";
        else
            repStr << std::right << std::setw(11) << "-----" << " (s)  ";
        repStr << std::setw(7) << plannedPassengers_[i] << std::endl;
    }

    repStr << "====================================================================================================================" << std::endl;
    return repStr.str();
}

std::string Route::routeMetricsToString(int epoch, int RMPCounter) const {
    std::stringstream repStr;
    repStr << routeID_ << "," << epoch << "," << RMPCounter << "," << vehicleID_ << ","
           << totalWait_ << "," << incompatibilityDegree_ << "," << reducedCost_ << ","
           << lambda_ << "," << normal_RC_ << "," << IncScoreRatio_ << "," << waitScore_ << ","
           << nbCommitted_ << "," << createTime_ << "," << routeRequests_.size() <<"\n";
    return repStr.str();
}

// these functions are used to add nodes to the routes
void Route::addSource(const PNode &node, float departTime, int departPassengers) {
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
void Route::addNode(const PNode &node) {
    const auto& prevNode = routeNodes_.back();
    const int prevLoc = prevNode->locationID_;
    const int nodeLoc = node->locationID_;
    float depart = plannedDepartTime_.back();

    routeSize_ ++;
    plannedPassengers_.push_back(plannedPassengers_.back() + node->nbPassengers_);
    if (node->initialType_ == PICKUP && depart < node->related_Request_->requestTime_)
        depart = node->related_Request_->requestTime_;
    plannedDepartTime_.back() = depart;
    if (prevNode->departTime_ != 0)
        prevNode->departTime_ = depart;
    const auto& row = durationMatrix_[prevLoc];
    float reachTime = depart + row[nodeLoc];

    if (node->initialType_ == PICKUP)
        reachTime = std::max(reachTime, node->related_Request_->earlyPick_);
    /*if (node->initialType_ == PICKUP && reachTime < node->related_Request_->earlyPick_) {
        plannedDepartTime_.back() += node->related_Request_->earlyPick_ - reachTime;
        reachTime = node->related_Request_->earlyPick_;
    }*/

    plannedReachTime_.push_back(reachTime);
    plannedDepartTime_.push_back(reachTime + node->serviceTime_);
    routeNodes_.push_back(node);

    if (node->initialType_ == PICKUP) {
        routeRequests_.push_back(node->related_Request_);
        plannedDelay_.push_back(reachTime - node->initialReadyTime_);
        totalWait_ += node->related_Request_->Req_W3_ * (reachTime - node->initialReadyTime_);
        if (node->related_Request_->committedPickTime_ != LARGE_CONSTANT)
            nbCommitted_++;
    }
}

bool Route::reConstructRoute(const PVehicle & vehicle){
    PRoute newRoute = std::make_shared<Route>(vehicleID_);
    newRoute->addSource(vehicle->departNode_, vehicle->departTime_, vehicle->numPassengers_);
    for (int i = 1; i < routeNodes_.size(); ++i) {
        newRoute->addNode(routeNodes_[i]);
        if (routeNodes_[i]->type_ == PICKUP) {
            if (newRoute->plannedReachTime_[i] < routeNodes_[i]->related_Request_->earlyPick_ ||
                newRoute->plannedReachTime_[i] > routeNodes_[i]->related_Request_->latestPickup_)
                return false;
        }
    }
    totalTripDelay_ = newRoute->totalTripDelay_;
    plannedDepartTime_ = newRoute->plannedDepartTime_;
    plannedReachTime_ = newRoute->plannedReachTime_;
    plannedDelay_ = newRoute->plannedDelay_;
    totalWait_ = newRoute->totalWait_;
    routeRequests_ = newRoute->routeRequests_;
    routeNodes_ = newRoute->routeNodes_;
    plannedPassengers_ = newRoute->plannedPassengers_;
    nbCommitted_ = newRoute->nbCommitted_;
    routeSize_ = newRoute->routeSize_;
    return true;
}

bool Route::reConstruct1(const PVehicle & vehicle, float wait_W1, float ride_W2){
    PRoute newRoute = std::make_shared<Route>(vehicleID_);
    newRoute->addSource(vehicle->departNode_, vehicle->departTime_, vehicle->numPassengers_);

    for (int i = vehicle->removeNodes_.size()+1; i < routeNodes_.size(); ++i) {
        newRoute->addNode(routeNodes_[i]);
        if (routeNodes_[i]->type_ == PICKUP) {
            if (newRoute->plannedReachTime_[i] > routeNodes_[i]->related_Request_->latestPickup_)
                return false;
        }
        else if (routeNodes_[i]->type_ == DROPOFF && routeNodes_[i]->related_Request_->requestStatus_ == ON_BOARD) {
            if (newRoute->plannedReachTime_[i] - routeNodes_[i]->pairNode_->departTime_ > routeNodes_[i]->related_Request_->maxTravelTime_)
                return false;
        }
    }
    if (newRoute->routeRequests_.empty())
        return false;
    newRoute->calculateTripDelay(wait_W1, ride_W2);
    plannedDepartTime_ = newRoute->plannedDepartTime_;
    plannedReachTime_ = newRoute->plannedReachTime_;
    plannedDelay_ = newRoute->plannedDelay_;
    totalWait_ = newRoute->totalWait_;
    totalTripDelay_ = newRoute->totalTripDelay_;
    objCoef_ = newRoute->objCoef_;
    routeRequests_ = newRoute->routeRequests_;
    routeNodes_ = newRoute->routeNodes_;
    plannedPassengers_ = newRoute->plannedPassengers_;
    nbCommitted_ = newRoute->nbCommitted_;
    routeSize_ = newRoute->routeSize_;
    waitScore_ = newRoute->totalWait_ / newRoute->routeRequests_.size();
    return true;
}

bool Route::reConstruct(const PVehicle& vehicle, float wait_W1, float ride_W2)
{
    Route tmp(vehicleID_); // stack temp: no make_shared, no heap alloc
    tmp.addSource(vehicle->departNode_, vehicle->departTime_, vehicle->numPassengers_);

    const std::size_t start = static_cast<std::size_t>(vehicle->removeNodes_.size()) + 1;
    if (start > routeNodes_.size()) return false;

    for (std::size_t k = start; k < routeNodes_.size(); ++k) {
        const auto& n = routeNodes_[k];
        tmp.addNode(n);

        const float reach = tmp.plannedReachTime_.back(); // <-- correct index

        if (n->type_ == PICKUP) {
            if (reach > n->related_Request_->latestPickup_)
                return false;
        } else if (n->type_ == DROPOFF &&
                   n->related_Request_->requestStatus_ == ON_BOARD) {
            if (reach - n->pairNode_->departTime_ > n->related_Request_->maxTravelTime_)
                return false;
                   }
    }

    if (tmp.routeRequests_.empty()) return false;

    tmp.calculateTripDelay(wait_W1, ride_W2);
    tmp.waitScore_ = tmp.totalWait_ / tmp.routeRequests_.size();
    tmp.mpAdded_ = mpAdded_;

    // Fast commit: move or swap
    this->swapState(tmp);              // simplest if move assignment is OK
    // or: this->swap(tmp);              // if you prefer explicit swap
    /*if (this->routeSize_ != (this->routeRequests_.size() * 2 + vehicle->onboards_.size() + 1))
        std::cout << "error";*/

    return true;
}

void Route::addNode(const PNode &node, float reachTime, float departTime) {
    routeSize_ ++;
    plannedPassengers_.push_back(plannedPassengers_.back() + node->nbPassengers_);
    plannedReachTime_.push_back(reachTime);
    plannedDepartTime_.push_back(departTime);
    if (node->initialType_ == PICKUP) {
        routeRequests_.push_back(node->related_Request_);
        plannedDelay_.push_back(reachTime - node->initialReadyTime_);
        totalWait_ += node->related_Request_->Req_W3_ * (reachTime - node->initialReadyTime_);
    }
    routeNodes_.push_back(node);
}

void Route::addSink(const PNode &node) {
    routeSize_ ++;
    plannedPassengers_.push_back(plannedPassengers_.back());
    float reachTime = plannedDepartTime_.back() + durationMatrix_[routeNodes_.back()->locationID_][node->locationID_];
    plannedReachTime_.push_back(reachTime);
    plannedDepartTime_.push_back(reachTime + node->serviceTime_);
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
    plannedDelay_.clear();
    totalWait_ = 0;
    for (int i = 1; i < routeNodes_.size(); ++i) {
        if (routeNodes_[i]->initialType_ == PICKUP) {
            routeRequests_.push_back(routeNodes_[i]->related_Request_);
            totalWait_ += routeNodes_[i]->related_Request_->Req_W3_ * (plannedReachTime_[i] - routeNodes_[i]->initialReadyTime_);
            plannedDelay_.push_back(plannedReachTime_[i] - routeNodes_[i]->initialReadyTime_);
        }
    }
}

// this function is for testing the validation of the route
void Route::testRoute(const PVehicle & vehicle) {
    PRoute testRoute = std::make_shared<Route>(vehicleID_);
    testRoute->addSource(routeNodes_[0], vehicle->solutionRoute_->routeNodes_[0]->departTime_,
                         vehicle->solutionRoute_->plannedPassengers_[0]);

    for (int i = 1; i < routeSize_; ++i) {
        testRoute->addNode(routeNodes_[i]);
        if (testRoute->plannedPassengers_.back() == 0){
            testRoute->plannedDepartTime_.back() = testRoute->routeNodes_.back()->departTime_;
        }
    }

    for (int i = 1; i < routeSize_; ++i) {
        if (testRoute->routeNodes_[i]->reachTime_ != testRoute->plannedReachTime_[i]){
            std::cout << "Connectivity constraint violated at node (reach time): ";
            std::cout << testRoute->routeNodes_[i]->nodeID_ << std::endl;
            std::cout << testRoute->routeNodes_[i]->reachTime_ << " - " << testRoute->plannedReachTime_[i] << std::endl;
  //          throw myTools::myException("Route-Validation", __FILE__,__LINE__);
        }

        if (testRoute->routeNodes_[i]->departTime_ != testRoute->plannedDepartTime_[i]){
            std::cout << "Connectivity constraint violated at node (depart time): ";
            std::cout << testRoute->routeNodes_[i]->nodeID_ << std::endl;
            std::cout << testRoute->routeNodes_[i]->departTime_ << " - " << testRoute->plannedDepartTime_[i] << std::endl;
 //           throw myTools::myException("Route-Validation", __FILE__,__LINE__);
        }

        // checking request data (pick up and drop off time)
        if (testRoute->routeNodes_[i]->initialType_ == PICKUP) {
            if (testRoute->routeNodes_[i]->reachTime_ !=
                testRoute->routeNodes_[i]->related_Request_->pickTime_) {
                std::cout << "Request pickup time is not equal to the pick node reach time at node : ";
                std::cout << testRoute->routeNodes_[i]->nodeID_ << std::endl;
 //               throw myTools::myException("Route-Validation", __FILE__,__LINE__);
            }
        }
        else if (testRoute->routeNodes_[i]->initialType_ == DROPOFF) {
            if (testRoute->routeNodes_[i]->reachTime_ !=
                testRoute->routeNodes_[i]->related_Request_->dropTime_) {
                std::cout << "Request drop-off time is not equal to the drop node reach time at node : ";
                std::cout << testRoute->routeNodes_[i]->nodeID_ << std::endl;
 //               throw myTools::myException("Route-Validation", __FILE__,__LINE__);
            }
        }

        // checking capacity constraints
        if (testRoute->plannedPassengers_[i] > vehicle->capacity_){
            std::cout << "Capacity constraint violated at node : " << testRoute->routeNodes_[i]->nodeID_ << std::endl;
 //           throw myTools::myException("Route-Validation", __FILE__,__LINE__);
        }

        // checking trip delay constraint
        if (testRoute->routeNodes_[i]->initialType_ == DROPOFF){
            float travelTime = testRoute->routeNodes_[i]->related_Request_->dropTime_ -
                               testRoute->routeNodes_[i]->pairNode_->departTime_;
            if (travelTime > testRoute->routeNodes_[i]->related_Request_->maxTravelTime_ + 0.1){
                std::cout << "Trip delay constraint violated for request : " <<
                          testRoute->routeNodes_[i]->related_Request_->getRequestId() << std::endl;
 //               throw myTools::myException("Route-Validation", __FILE__,__LINE__);
            }
        }
    }

    if (testRoute->totalWait_ != totalWait_){
        std::cout << "Total delay is not the same" << std::endl;
 //       throw myTools::myException("Route-Validation", __FILE__,__LINE__);
    }
    if (testRoute->plannedPassengers_.back() != plannedPassengers_.back()){
        std::cout << "Final Load is not the same" << std::endl;
 //       throw myTools::myException("Route-Validation", __FILE__,__LINE__);
    }
    if (testRoute->plannedReachTime_.back()!= plannedReachTime_.back()){
        std::cout << "End time is not the same" << std::endl;
 //       throw myTools::myException("Route-Validation", __FILE__,__LINE__);
    }

}
// This function is to reset the status of the nodes in the route
void Route::resetRoute() const {
    for (size_t i = routeSize_-1; i > 0; --i) {
        routeNodes_[i]->nodeStatus_ = DEFINED;
        if (routeNodes_[i]->type_ == DROPOFF) {
            routeNodes_[i]->related_Request_->requestStatus_ = ON_BOARD;
            routeNodes_[i]->related_Request_->dropTime_ = LARGE_CONSTANT;
        }
        else if (routeNodes_[i]->type_ == PICKUP) {
            routeNodes_[i]->related_Request_->requestStatus_ = NO_ACTION;
            routeNodes_[i]->related_Request_->pickTime_ = LARGE_CONSTANT;
            routeNodes_[i]->related_Request_->dropTime_ = LARGE_CONSTANT;
        }
    }
}

void Route::calcMarginalCosts(float wait_W1, float ride_W2) {
    if (routeRequests_.size() == 1) {
        routeRequests_[0]->marginalCost_ = objCoef_;
        routeRequests_[0]->dual_ = routeRequests_[0]->marginalCost_ * 1.5;
    }
    else if (routeRequests_.size() > 1) {
        for (size_t i = 1; i < routeNodes_.size(); ++i) {
            if (routeNodes_[i]->type_ != SINK && routeNodes_[i]->type_ != SOURCE) {
                float tripDelay = 0.0;
                float waitTime = 0.0;
                if (routeNodes_[i]->type_ == PICKUP) {
                    waitTime = plannedReachTime_[i] - routeNodes_[i]->related_Request_->requestTime_;
                    for (size_t j = i + 1; j < routeNodes_.size(); ++j) {
                        if (routeNodes_[i]->related_Request_->getRequestId() == routeNodes_[j]->related_Request_->getRequestId()) {
                            tripDelay = plannedReachTime_[j] - plannedDepartTime_[i] - routeNodes_[i]->related_Request_->minTravelTime_;
                            break;
                        }
                    }
                    routeNodes_[i]->related_Request_->marginalCost_ = routeNodes_[i]->related_Request_->Req_W3_ *
                        (wait_W1 * waitTime + ride_W2 * (tripDelay + routeNodes_[i]->related_Request_->Ride_W4_))/
                            routeNodes_[i]->related_Request_->Relative_W5_;
                    routeNodes_[i]->related_Request_->dual_ = routeNodes_[i]->related_Request_->marginalCost_ * 1.5;
                }
            }
        }
    }
}

void Route::createColumn(int nbRequests) {
    column_.reset();
    column_.resize(nbRequests);
    /*for (auto & requestObj: routeRequests_)
        column_->add(requestObj->taskIndex_);*/
    for (auto & requestObj: routeRequests_)
        column_.set(requestObj->taskIndex_, true);
}

void Route::calculateTripDelay(float wait_W1, float ride_W2) {
    totalTripDelay_ = 0.0;
    objCoef_ = 0.0;
    for (size_t i = 1; i < routeNodes_.size(); ++i) {
        if (routeNodes_[i]->type_ != SINK && routeNodes_[i]->type_ != SOURCE) {
            float tripDelay = 0.0;
            float waitTime = 0.0;
            if (routeNodes_[i]->type_ == DROPOFF && routeNodes_[i]->related_Request_->requestStatus_ == ON_BOARD) {
                tripDelay = plannedReachTime_[i] - routeNodes_[i]->related_Request_->minTravelTime_ -
                    (routeNodes_[i]->related_Request_->pickTime_ + routeNodes_[i]->serviceTime_);
            }
            else if (routeNodes_[i]->type_ == PICKUP) {
                waitTime = plannedReachTime_[i] - routeNodes_[i]->related_Request_->requestTime_;
                for (size_t j = i + 1; j < routeNodes_.size(); ++j) {
                    if (routeNodes_[i]->related_Request_->getRequestId() == routeNodes_[j]->related_Request_->getRequestId()) {
                        tripDelay = plannedReachTime_[j] - plannedDepartTime_[i] - routeNodes_[i]->related_Request_->minTravelTime_;
                        break;
                    }
                }
                routeNodes_[i]->related_Request_->marginalCost_ = routeNodes_[i]->related_Request_->Req_W3_ * (wait_W1 * waitTime + ride_W2 * (tripDelay + routeNodes_[i]->related_Request_->Ride_W4_))/routeNodes_[i]->related_Request_->Relative_W5_;
            }
            totalTripDelay_ += routeNodes_[i]->related_Request_->Req_W3_ * tripDelay;
            objCoef_ += routeNodes_[i]->related_Request_->Req_W3_ * (wait_W1 * waitTime + ride_W2 * (tripDelay + routeNodes_[i]->related_Request_->Ride_W4_))/routeNodes_[i]->related_Request_->Relative_W5_;
        }
    }
    // Refresh the unique key after objCoef_ and routeRequests_ are finalized
    setKey();
}
