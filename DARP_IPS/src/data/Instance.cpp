//
// Created by Ella on 2021-09-12.
//

#include "Instance.h"
vector2D<float> durationMatrix_;
//-----------------------------------------------------------------------------
//  Instance class
//  contains the instance data including vehicle info and requests
//-----------------------------------------------------------------------------
extern vector2D<float> durationMatrix_;
// Constructor and Destructor
Instance::Instance(std::string &name, float simulationStart, int nbVehicles, int nbOnboards, int nbReceived,
                   std::vector<PVehicle> &vehicles, int nbRequests, int nbLocations, PGraph &mainGraph) : name_(name),
                   simulationStartTime_(simulationStart), nbVehicles_(nbVehicles), nbOnboards_(nbOnboards),
                   nbWaiting_(nbReceived), vehicles_(vehicles), nbRequests_(nbRequests), nbLocations_(nbLocations),
                   instGraph_(mainGraph) {
    nbNewRequests_ = nbRequests;
    std::cout << "Instance created!"<< std::endl;
    requests_.reserve(nbRequests + nbOnboards);
    vehicles.reserve(nbVehicles);
}


Instance::Instance(const Instance &mainInst) : name_(mainInst.name_){
    nbVehicles_ = mainInst.nbVehicles_;
    vehicles_ = mainInst.vehicles_;
    parameters_ = mainInst.parameters_;
    simulationStartTime_ = mainInst.simulationStartTime_;
    nbRequests_ = 0;
    nbNewRequests_ = 0;
    nbWaiting_ = 0;
    instGraph_ = std::make_shared<Graph>();
    nbOnboards_ = mainInst.nbOnboards_;
    nbLocations_ = mainInst.nbLocations_;
    requests_.reserve(mainInst.requests_.size());
//    instGraph_->sinkNodes_ = mainInst.instGraph_->sinkNodes_;
}

Instance::Instance(const Instance &mainInst, int zoneID) : name_(mainInst.name_){
    for (auto & vehicleObj : mainInst.vehicles_){
        if (vehicleObj->zoneID_ == zoneID)
            vehicles_.push_back(vehicleObj);
    }
    nbVehicles_ = vehicles_.size();
    parameters_ = mainInst.parameters_;
    simulationStartTime_ = mainInst.simulationStartTime_;
    nbRequests_ = 0;
    nbNewRequests_ = 0;
    nbWaiting_ = 0;
    instGraph_ = std::make_shared<Graph>();
    nbOnboards_ = 0;
    nbLocations_ = mainInst.nbLocations_;
    requests_.reserve(mainInst.requests_.size());
    instGraph_->sinkNodes_ = mainInst.instGraph_->sinkNodes_;

}

void Instance::resetInstance() {
    nbRequests_ = 0;
    nbNewRequests_ = 0;
    nbWaiting_ = 0;
//    instGraph_ = std::make_shared<Graph>();

    requests_.clear();
    nameToRequest_.clear();
    orderToRequest_.clear();
    instGraph_->nodes_.clear();
    instGraph_->intToNodeID_.clear();
    instGraph_->pickNodes_.clear();
    instGraph_->dropNodes_.clear();
    instGraph_->sourceNodes_.clear();
    instGraph_->onboards_.clear();
    instGraph_->sinkNodes_.clear();
}

/*Instance::~Instance() {
    instGraph_.reset();
}*/

// Display function
std::string Instance::toString() {
    std::stringstream repStr;
    repStr << std::endl;
    repStr << "***************************************************************************" << std::endl;
    repStr << "**                             Instance Info                             **" << std::endl;
    repStr << "***************************************************************************" << std::endl;
    repStr << "# " << std::endl;
    repStr << "# INSTANCE_NAME       \t= " << name_ << std::endl;
    repStr << "# NUMBER_OF_VEHICLES  \t= " << nbVehicles_ <<std::endl;
    repStr << "# NUMBER_OF_REQUESTS  \t= " << nbRequests_ <<std::endl;
    repStr << "# " << std::endl;

    // display 3 requests information
    /*repStr << "--------------------- REQUESTS_INFO (Max 3 records) ---------------------" << std::endl;
    int n = std::min(3, nbRequests_);
    for (int i = 0; i < n; ++i) {
        repStr << requests_[i]->toString();
    }
    repStr << "--------------------- VEHICLES_INFO (Max 3 records) ---------------------" << std::endl;
    int m = std::min(3, nbVehicles_);
    for (int i = 0; i < m; ++i) {
        repStr << vehicles_[i]->toString();
    }

    repStr << "# " << std::endl;*/
    repStr << "------------------------ PARAMETERS AND OPTIONS -------------------------" << std::endl;
    repStr << parameters_->toString();
    repStr << "# " << std::endl;
    return repStr.str();
}

std::string Instance::solutionToString() {
    int numServed = 0;                  // total requests in routes
    float totalWaiting = 0;             // total route waiting times
    float totalWaitingPartial = 0;      // total waiting times after one hour / after simulation start
    float totalTripDelay = 0;           // total trip delay considering the initial onboards
    float totalTripDelayPartial = 0;    // total trip delay after one hour / after simulation start
    int totalNumServed = 0;             // total requests served considering initial onboards
    int totalNumServedPartial = 0;      // total requests served after one hour starting from scratch
    double penalty = 0;                 // total penalty of un-served
    float idleTime = 0;                 // total vehicle idle times
    int totalCustomers = 0;
    int totalCustomersPartial = 0;
    int nbIdle = 0;
    int totalStops = 0;
    int totalStopLoad = 0;

    std::stringstream repStr;

    instRepStr_ << "\n" << name_ << "," << mainAlgorithmName[parameters_->mainAlgorithm_] << ",";
    instRepStr_ << solutionModeName[parameters_->solutionMode_] << ",";
    instRepStr_ << nbRequests_ << "," << nbVehicles_ << "," << parameters_->nbThreads_ << ",";

    // print table header
    repStr << "# --------------------------------------------------------------------------------------------------------" << std::endl;
    repStr << "#  REQUEST_ID" << "  ";
    repStr << "REQUEST_TIME(s)" << "   ";
    repStr << "PICK_TIME(s)" << "   ";
    repStr << "DROP_TIME(s)" << "   ";
    repStr << "WAIT_TIME(s)" << "  ";
    repStr << "TRIP_DELAY(s)" << "   ";
    repStr << "#PASSENGERS" <<std::endl;
    repStr << "# --------------------------------------------------------------------------------------------------------" << std::endl;

    // print the internal nodes of the route
    for (int i = 0; i < nbRequests_; ++i) {

        repStr << std::fixed;
        repStr << std::setprecision(2);
        repStr << "#" << std::right << std::setw(9) << requests_[i]->getRequestId() << "       ";
        repStr << std::right << std::setw(9) << requests_[i]->earlyPick_ << " (s)  ";
        if (requests_[i]->requestStatus_ != NO_ACTION) {
            repStr << std::right << std::setw(9) << requests_[i]->pickTime_ << " (s)  ";
            repStr << std::right << std::setw(9) << requests_[i]->dropTime_ << " (s)  ";
            repStr << std::right << std::setw(9) << requests_[i]->pickTime_ - requests_[i]->earlyPick_ << " (s)  ";

//            float travelTime = requests_[i]->dropTime_ - requests_[i]->pickTime_ - requests_[i]->serviceTime_;
            float travelTime = instGraph_->dropNodes_[i]->reachTime_ - instGraph_->pickNodes_[i]->departTime_;
            if (durationMatrix_[instGraph_->pickNodes_[i]->locationID_][instGraph_->dropNodes_[i]->locationID_] == 0)
                travelTime = 0;
            repStr << std::right << std::setw(9) << travelTime - requests_[i]->minTravelTime_ << " (s)  ";
            if (travelTime > requests_[i]->maxTravelTime_ + 0.1){
                std::cout << "Trip delay constraint is violated by request: " << requests_[i]->getRequestId() << std::endl;
 //               myTools::throwException("Trip delay Validation");
            }
            if (travelTime - requests_[i]->minTravelTime_ < -0.1){
                std::cout << "Trip delay is negative for request: " << requests_[i]->getRequestId() << std::endl;
//                myTools::throwException("Trip delay Validation");
            }

            totalTripDelay += travelTime - requests_[i]->minTravelTime_;
            totalNumServed ++;
            if (parameters_->savePartial_) {
                if (requests_[i]->earlyPick_ >= simulationStartTime_ + 3600) {
                    totalNumServedPartial++;
                    totalWaitingPartial += requests_[i]->pickTime_ - requests_[i]->earlyPick_;
                    totalTripDelayPartial += travelTime - requests_[i]->minTravelTime_;
                    totalCustomersPartial += requests_[i]->nbPassengers_;
                }
            }
            else {
                if (requests_[i]->earlyPick_ >= simulationStartTime_) {
                    totalNumServedPartial++;
                    totalWaitingPartial += requests_[i]->pickTime_ - requests_[i]->earlyPick_;
                    totalTripDelayPartial += travelTime - requests_[i]->minTravelTime_;
                    totalCustomersPartial += requests_[i]->nbPassengers_;
                }
            }

            totalCustomers += requests_[i]->nbPassengers_;
        }
        else {
            repStr << std::right << std::setw(9) << "-------" << " (s)  ";
            repStr << std::right << std::setw(9) << "-------" << " (s)  ";
            repStr << std::right << std::setw(9) << "-------" << " (s)  ";
            penalty += requests_[i]->penalty_;
        }

        repStr << std::setw(7) << requests_[i]->nbPassengers_ << std::endl;
    }

    repStr << "# --------------------------------------------------------------------------------------------------------" << std::endl;

    for (const auto &vehicleObj : vehicles_) {
        totalWaiting += vehicleObj->solutionRoute_->totalDelay_;
        numServed += (int)vehicleObj->solutionRoute_->routeRequests_.size();
        idleTime += vehicleObj->idleTime_;
        if (vehicleObj->solutionRoute_->routeSize_ == 1)
            nbIdle++;
        totalStopLoad+= std::accumulate(vehicleObj->solutionRoute_->plannedPassengers_.begin(),
                                        vehicleObj->solutionRoute_->plannedPassengers_.end(),
                                        decltype(vehicleObj->solutionRoute_->plannedPassengers_)::value_type(0));
        totalStops += vehicleObj->solutionRoute_->routeSize_;
    }
    repStr << std::left << std::fixed << std::setprecision(2);
    repStr << "#" << std::endl;
    repStr << std::setw(sentenceSize) << "# FINAL OBJECTIVE VALUE" << " = " << penalty + totalWaiting << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL WAIT TIME" << " = " << totalWaiting << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL TRIP DELAY" << " = " << totalTripDelay << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL IDLE TIME" << " = " << idleTime << " (s)" << std::endl;
    repStr << "#" << std::endl;
    repStr << std::setw(sentenceSize) << "# NUMBER OF UNSERVED REQUESTS" << " = " << nbRequests_ - totalNumServed << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL NUMBER OF REQUESTS" << " = " << nbRequests_ << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL NUMBER OF SERVED PASSENGERS" << " = " << totalCustomers << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL NUMBER OF EMPTY VEHICLES" << " = " << nbIdle << std::endl;
    repStr << "#" << std::endl;
    repStr << "# ----------------------   AVERAGE VALUES   ----------------------" << std::endl;

    if (numServed != 0) {
        repStr << std::setw(sentenceSize) << "# WAIT TIME PER REQUEST" << " = " << totalWaiting/static_cast<float>(numServed) << " (s)" << std::endl;
        repStr << std::setw(sentenceSize) << "# WAIT TIME PER PASSENGER" << " = " << totalWaiting/static_cast<float>(totalCustomers) << " (s)" << std::endl;
        repStr << std::setw(sentenceSize) << "# TRIP DELAY PER REQUEST" << " = " << totalTripDelay/static_cast<float>(totalNumServed) << " (s)" << std::endl;
        repStr << std::setw(sentenceSize) << "# WAIT TIME PER REQUEST AFTER 1H" << " = " << totalWaitingPartial/static_cast<float>(totalNumServedPartial) << " (s)" << std::endl;
        repStr << std::setw(sentenceSize) << "# WAIT TIME PER PASSENGER AFTER 1H" << " = " << totalWaitingPartial/static_cast<float>(totalCustomersPartial) << " (s)" << std::endl;
        repStr << std::setw(sentenceSize) << "# TRIP DELAY PER REQUEST AFTER 1H" << " = " << totalTripDelayPartial/static_cast<float>(totalNumServedPartial) << " (s)" << std::endl;
    }
    repStr << std::setw(sentenceSize) << "# PASSENGERS IN VEHICLE" << " = " << totalStopLoad/static_cast<float>(totalStops) << std::endl;
    repStr << std::setw(sentenceSize) << "# IDLE TIME PER VEHICLE" << " = " << idleTime/static_cast<float>(nbVehicles_) << std::endl;
    repStr << "#" << std::endl;
    instRepStr_ << totalCustomers << ",";
    if (totalCustomersPartial > 135000)
        instRepStr_ << "135000 <" << ",";
    else if (totalCustomersPartial < 125000)
        instRepStr_ << "< 125000" << ",";
    else
        instRepStr_ << "125000 - 135000" << ",";
    instRepStr_ << totalNumServed << "," << totalWaiting/static_cast<float>(numServed) << ",";
    instRepStr_ << totalWaiting/static_cast<float>(totalCustomers) << ",";
    instRepStr_ << totalTripDelay/static_cast<float>(totalNumServed) << "," << totalNumServedPartial << ",";
    instRepStr_ << totalCustomersPartial << "," << totalWaitingPartial/static_cast<float>(totalNumServedPartial) << ",";
    instRepStr_ << totalWaitingPartial/static_cast<float>(totalCustomersPartial) << ",";
    instRepStr_ << totalTripDelayPartial/static_cast<float>(totalNumServedPartial) << ",";
    instRepStr_ << idleTime/static_cast<float>(nbVehicles_) << ",";
    instRepStr_ << nbIdle << ",";
    instRepStr_ << totalStopLoad/static_cast<float>(totalStops) << ",";
    return repStr.str();
}

// this function update the set of available requests, removed completed requests and update onboards
void Instance::buildPartialData(const PInstance &mainInst, std::vector<PRequest> &penaltyRequests, float elapsedTime, int lastRecRequests) {
    for (auto & vehicleObj : mainInst->vehicles_){
        instGraph_->addNewNode(vehicleObj->departNode_);
        instGraph_->addNewNode(vehicleObj->sinkNode_);

        vehicleObj->score_ = INFINITY;
    }
    nbNewRequests_ = 0;
    // add unscheduled requests
    for (auto & requestObj: penaltyRequests) {
        addRequest(requestObj);
        instGraph_->addNewNode(mainInst->instGraph_->pickNodes_[requestObj->getRequestId()]);
        instGraph_->addNewNode(mainInst->instGraph_->dropNodes_[requestObj->getRequestId()]);
    }

    if (mainInst->parameters_->mainAlgorithm_ != GREEDY || mainInst->parameters_->greedyReOptimize_) {
        // add unperformed requests
        for (auto &vehicleObj: mainInst->vehicles_) {
            if (vehicleObj->currentRoute_->routeSize_ > 1) {
                for (int i = 1; i < vehicleObj->currentRoute_->routeSize_; ++i) {
                    if (vehicleObj->currentRoute_->routeNodes_[i]->type_ == PICKUP) {
                        addRequest(vehicleObj->currentRoute_->routeNodes_[i]->related_Request_);
                        instGraph_->addNewNode(vehicleObj->currentRoute_->routeNodes_[i]);
//                    instGraph_->addNewNode(*vehicleObj->currentRoute_->routeNodes_[i]->pairNode_);
                        instGraph_->addNewNode(
                                mainInst->instGraph_->dropNodes_[vehicleObj->currentRoute_->routeNodes_[i]->related_Request_->getRequestId()]);
                    }
                        // adding onboard nodes to the graph
                    else if ((vehicleObj->currentRoute_->routeNodes_[i]->nodeStatus_ == PLANNED) ||
                             (vehicleObj->currentRoute_->routeNodes_[i]->nodeStatus_ == COMMITTED &&
                              vehicleObj->currentRoute_->routeNodes_[i]->initialType_ == DROPOFF)) {
                        instGraph_->nodes_.emplace(
                                std::pair<std::string, PNode>(vehicleObj->currentRoute_->routeNodes_[i]->nodeID_,
                                                              vehicleObj->currentRoute_->routeNodes_[i]));
                        instGraph_->onboards_.push_back(vehicleObj->currentRoute_->routeNodes_[i]);
                        vehicleObj->currentRoute_->routeNodes_[i]->nodeIndex_ = instGraph_->nbNodes_;
                        instGraph_->intToNodeID_.push_back(vehicleObj->currentRoute_->routeNodes_[i]->nodeID_);
                        instGraph_->nbNodes_++;
                    }
                }
            }
        }
    }

    // add new requests
    for (int i = lastRecRequests; i < mainInst->nbRequests_; ++i) {
        if (parameters_->solutionMode_ == ANYTIME) {
            if (mainInst->requests_[i]->earlyPick_ <= simulationStartTime_ + elapsedTime) {
                nbNewRequests_++;
                addRequest(mainInst->requests_[i]);
                instGraph_->addNewNode(mainInst->instGraph_->pickNodes_[i]);
                instGraph_->addNewNode(mainInst->instGraph_->dropNodes_[i]);

            }
            else
                break;
        }
        else {
            if (mainInst->requests_[i]->earlyPick_ < simulationStartTime_ + elapsedTime) {
                if (mainInst->requests_[i]->solVehicleID_ == 999999) {
                    nbNewRequests_++;
                    addRequest(mainInst->requests_[i]);
                    instGraph_->addNewNode(mainInst->instGraph_->pickNodes_[i]);
                    instGraph_->addNewNode(mainInst->instGraph_->dropNodes_[i]);
                }
            }
            else
                break;
        }
    }

    nbOnboards_ = instGraph_->onboards_.size();

    // calculate vehicle scores
    if (mainInst->parameters_->vehicle_portion_ < 1){
        for (int i = 0; i < instGraph_->pickNodes_.size() ; i++){
            for (auto & vehicleObj: mainInst->vehicles_){
                float earliestPick = vehicleObj->departTime_ + durationMatrix_[vehicleObj->departNode_->locationID_]
                [mainInst->instGraph_->pickNodes_[i]->locationID_] - (simulationStartTime_ + elapsedTime);
                if (earliestPick < vehicleObj->score_)
                    vehicleObj->score_ = earliestPick;
            }
        }
    }
    updateRequestOrder();
}

void Instance::buildStaticData(const PInstance &mainInst, int lastRecRequests) {
    for (auto & vehicleObj : mainInst->vehicles_){
        instGraph_->addNewNode(vehicleObj->departNode_);
        instGraph_->addNewNode(vehicleObj->sinkNode_);

        vehicleObj->score_ = INFINITY;
    }
    nbNewRequests_ = 0;

    if (mainInst->parameters_->mainAlgorithm_ != GREEDY || mainInst->parameters_->greedyReOptimize_) {
        // add unperformed requests
        for (auto &vehicleObj: mainInst->vehicles_) {
            if (vehicleObj->currentRoute_->routeSize_ > 1) {
                for (int i = 1; i < vehicleObj->currentRoute_->routeSize_; ++i) {
                    if (vehicleObj->currentRoute_->routeNodes_[i]->type_ == PICKUP) {
                        addRequest(vehicleObj->currentRoute_->routeNodes_[i]->related_Request_);
                        instGraph_->addNewNode(vehicleObj->currentRoute_->routeNodes_[i]);
//                    instGraph_->addNewNode(*vehicleObj->currentRoute_->routeNodes_[i]->pairNode_);
                        instGraph_->addNewNode(
                                mainInst->instGraph_->dropNodes_[vehicleObj->currentRoute_->routeNodes_[i]->related_Request_->getRequestId()]);
                    }
                        // adding onboard nodes to the graph
                    else if ((vehicleObj->currentRoute_->routeNodes_[i]->nodeStatus_ == PLANNED) ||
                             (vehicleObj->currentRoute_->routeNodes_[i]->nodeStatus_ == COMMITTED &&
                              vehicleObj->currentRoute_->routeNodes_[i]->initialType_ == DROPOFF)) {
                        instGraph_->nodes_.emplace(
                                std::pair<std::string, PNode>(vehicleObj->currentRoute_->routeNodes_[i]->nodeID_,
                                                              vehicleObj->currentRoute_->routeNodes_[i]));
                        instGraph_->onboards_.push_back(vehicleObj->currentRoute_->routeNodes_[i]);
                        vehicleObj->currentRoute_->routeNodes_[i]->nodeIndex_ = instGraph_->nbNodes_;
                        instGraph_->intToNodeID_.push_back(vehicleObj->currentRoute_->routeNodes_[i]->nodeID_);
                        instGraph_->nbNodes_++;
                    }
                }
            }
        }
    }

    // add new requests
    for (int i = lastRecRequests; i < mainInst->nbRequests_; ++i) {
        if (mainInst->requests_[i]->solVehicleID_ == 999999) {
            nbNewRequests_++;
            addRequest(mainInst->requests_[i]);
            instGraph_->addNewNode(mainInst->instGraph_->pickNodes_[i]);
            instGraph_->addNewNode(mainInst->instGraph_->dropNodes_[i]);
        }
    }

    nbOnboards_ = instGraph_->onboards_.size();

    updateRequestOrder();
}

void Instance::buildDataZone(const PInstance &mainInst, int zoneID) {
    for (auto & vehicleObj : vehicles_){
        instGraph_->addNewNode(vehicleObj->departNode_);
        instGraph_->addNewNode(vehicleObj->sinkNode_);
    }
    nbNewRequests_ = 0;

    for (auto & requestObj : mainInst->requests_) {
        if (requestObj->pickZoneID_ == zoneID) {
            nbNewRequests_++;
            addRequest(requestObj);
            std::string pickID = myTools::createNodeID(requestObj->getRequestId(), PICKUP);
            std::string dropID = myTools::createNodeID(requestObj->getRequestId(), DROPOFF);
            instGraph_->addNewNode(mainInst->instGraph_->nodes_[pickID]);
            instGraph_->addNewNode(mainInst->instGraph_->nodes_[dropID]);
        }
    }
}
// function to add requests from previous epochs to the current partial instance
void Instance::addRequest(PRequest &request) {
    nbRequests_++;
    requests_.push_back(request);
//    request->setPenaltyEpoch(epoch , parameters, simulationStart);
//    request->setPenaltyEpoch(epoch - request->readEpoch_, parameters, simulationStart);
    nameToRequest_[request->name_] = request;
 //   updateRequestOrder();
}


void Instance::setInitialTimes() {

    for (auto & requestObj: requests_) {

        requestObj->setMinTravelTime(durationMatrix_[requestObj->PickUpID_][requestObj->DropOffID_]);
        requestObj->setMaxTravelTime(parameters_->alphaParam_, parameters_->betaParam_);
    }

    // if the vehicles start from the source depart time is after the first epoch
    if (parameters_->solutionMode_ == DYNAMIC){
        for (auto & vehicleObj : vehicles_){
            if (vehicleObj->onboards_.empty() && vehicleObj->departTime_ < simulationStartTime_ + static_cast<float>(parameters_->epochLength_))
                vehicleObj->setDepartTime(simulationStartTime_ + static_cast<float>(parameters_->epochLength_));
        }
    }
    else if (parameters_->solutionMode_ == ANYTIME){
        for (auto & vehicleObj : vehicles_){
            if (vehicleObj->onboards_.empty()){
                if (vehicleObj->currentRoute_ == nullptr || vehicleObj->currentRoute_->routeSize_ <= 1) {
                    vehicleObj->idle_ = true;
                    vehicleObj->idleTime_ += (static_cast<float>(parameters_->committedTime_));
                    vehicleObj->setDepartTime(simulationStartTime_ + static_cast<float>(parameters_->committedTime_));
                }
            }
        }
    }
    else {
        for (auto & vehicleObj : vehicles_){
            if (vehicleObj->onboards_.empty() && vehicleObj->departTime_ < simulationStartTime_)
                vehicleObj->setDepartTime(simulationStartTime_);
        }
    }
}

// function to sort vehicles based on ID
void Instance::restVehicleOrder() {
    std::stable_sort(vehicles_.begin(), vehicles_.end(),[](const PVehicle &lhs, const PVehicle &rhs){
        return lhs->vehicleID_ < rhs->vehicleID_;});
}

void Instance::sortVehicles(SortVehicle sortBase) {
    switch(sortBase) {
        case DUAL :
            std::stable_sort(vehicles_.begin(), vehicles_.end(),[](const PVehicle &lhs, const PVehicle &rhs){
                return lhs->dual_ > rhs->dual_;});
            break;

        case DEPART_TIME :
            std::stable_sort(vehicles_.begin(), vehicles_.end(),[](const PVehicle &lhs, const PVehicle &rhs){
                return lhs->departTime_ < rhs->departTime_;});
            break;

        case ROURE_SIZE :
            std::stable_sort(vehicles_.begin(), vehicles_.end(),[](const PVehicle &lhs, const PVehicle &rhs){
                return lhs->currentRoute_->routeSize_ < rhs->currentRoute_->routeSize_;});
            break;
        case BEST_REDUCE_COST :
            std::stable_sort(vehicles_.begin(), vehicles_.end(),[](const PVehicle &lhs, const PVehicle &rhs){
                return lhs->bestReducedCost_ < rhs->bestReducedCost_;});
            break;
        case SCORE :
            std::stable_sort(vehicles_.begin(), vehicles_.end(),[](const PVehicle &lhs, const PVehicle &rhs){
                return lhs->score_ < rhs->score_;});
            break;
    }
}


// function to update penalties in rolling horizon approach
/*void Instance::updatePenaltiesEpoch(int epoch) {
    for (auto & requestObj : requests_)
        requestObj->setPenalty(parameters_->epochLength_ * epoch, parameters_, simulationStartTime_);
}*/

// function to update penalties in any time approach
void Instance::updatePenalties(float elapsedTime) {
    for (auto & requestObj : requests_)
        requestObj->setPenalty(elapsedTime, parameters_, simulationStartTime_);
}

//determine an order for requests to use in CPLEX modeling
void Instance::updateRequestOrder() {
    orderToRequest_.clear();
//    requestToOrder_.clear();

    int orderCounter = 0;
    for (auto & requestObj : requests_){
//        requestToOrder_[requestObj->getRequestId()] = orderCounter;
        requestObj->taskIndex_ = orderCounter;
        orderToRequest_.push_back(requestObj->getRequestId());
        orderCounter++;
    }
}

// print solutions in csv files
std::string Instance::saveSolutionRoutes() {
    std::stringstream repStr;
    repStr << "VehicleID,NodeID,RequestTime,ReachTime,NodeType, LocationID, DepartTime, nbPassengers, vehicleLoad, PlanedReach, PlanedDepart, TravelTime" << std::endl;
    for (auto & vehicleObj : vehicles_) {
        for (int j = 0; j< vehicleObj->solutionRoute_->routeNodes_.size(); j++) {
            repStr << vehicleObj->vehicleID_ << ",";
            repStr << vehicleObj->solutionRoute_->routeNodes_[j]->nodeID_ << ",";
            repStr << vehicleObj->solutionRoute_->routeNodes_[j]->requestTime_ << ",";
            repStr << vehicleObj->solutionRoute_->routeNodes_[j]->reachTime_ << ",";
            repStr << vehicleObj->solutionRoute_->routeNodes_[j]->initialType_ << ",";
            repStr << vehicleObj->solutionRoute_->routeNodes_[j]->locationID_ << ",";
            repStr << vehicleObj->solutionRoute_->routeNodes_[j]->departTime_ << ",";
            repStr << vehicleObj->solutionRoute_->routeNodes_[j]->nbPassengers_ << ",";
            repStr << vehicleObj->solutionRoute_->plannedPassengers_[j] << ",";
            repStr << vehicleObj->solutionRoute_->plannedReachTime_[j] << ",";
            repStr << vehicleObj->solutionRoute_->plannedDepartTime_[j] << ",";
            if (j == 0)
                repStr << 0 << "\n";
            else
                repStr << durationMatrix_[vehicleObj->solutionRoute_->routeNodes_[j-1]->locationID_][vehicleObj->solutionRoute_->routeNodes_[j]->locationID_] << "\n";
        }
    }
    return repStr.str();
}

std::string Instance::saveRequestsResults() {
    std::stringstream repStr;
    repStr << "RequestID,nbPassengers, PickupID,DropOffID,RequestTime,PickTime,"
              "DropTime, InVehicleID, VehicleID, WaitTime, TripDelay, MaxTravelTime, MinTravelTime, zoneID" << std::endl;

    for (auto & requestObj : requests_) {
        repStr << requestObj->getRequestId() << ",";
        repStr << requestObj->nbPassengers_ << ",";
        repStr << requestObj->PickUpID_ << ",";
        repStr << requestObj->DropOffID_ << ",";
        repStr << requestObj->earlyPick_ << ",";
        repStr << requestObj->pickTime_ << ",";
        repStr << requestObj->dropTime_ << ",";
        repStr << requestObj->initialVehicleID_ << ",";
        repStr << requestObj->allocVehicleID_ << ",";
        repStr << requestObj->pickTime_ - requestObj->earlyPick_ << ",";
        repStr << requestObj->dropTime_ - requestObj->pickTime_ - requestObj->minTravelTime_ << ",";
        repStr << requestObj->maxTravelTime_ << ",";
        repStr << requestObj->minTravelTime_ << ",";
        repStr << requestObj->pickZoneID_ << "\n";
    }
    return repStr.str();
}

std::string Instance::saveEpochRoutes(int epoch) {
    std::stringstream repStr;
    for (auto & vehicleObj : vehicles_) {
        for (auto & nodeObj : vehicleObj->solutionRoute_->routeNodes_) {
            repStr << epoch << ",";
            repStr << vehicleObj->vehicleID_ << ",";
            repStr << nodeObj->nodeID_ << ",";
            repStr << nodeObj->requestTime_ << ",";
            repStr << nodeObj->reachTime_ << ",";
            repStr << nodeObj->type_ << ",";
            repStr << nodeObj->locationID_ << "\n";
        }
    }
    return repStr.str();
}

std::string Instance::saveISUDRoutes(int epoch, int isudIter) {
    std::stringstream repStr;
    for (auto & vehicleObj : vehicles_) {
        for (auto & nodeObj : vehicleObj->currentRoute_->routeNodes_) {
            repStr << epoch << ",";
            repStr << isudIter << ",";
            repStr << vehicleObj->vehicleID_ << ",";
            repStr << nodeObj->nodeID_ << ",";
            repStr << nodeObj->requestTime_ << ",";
            repStr << nodeObj->reachTime_ << ",";
            repStr << nodeObj->type_ << ",";
            repStr << nodeObj->locationID_ << ",";
            repStr << vehicleObj->currentRoute_->getRouteId() << ",";
            repStr << vehicleObj->currentRoute_->incompatibilityDegree_ << "\n";
        }
    }
    return repStr.str();
}

std::string Instance::saveRoutesTimes(int epoch) {
    std::stringstream repStr;
    for (auto & vehicleObj : vehicles_) {
        repStr << epoch << ",";
        repStr << vehicleObj->vehicleID_ << ",";
        repStr << vehicleObj->currentRoute_->getRouteId() << ",";
        repStr << vehicleObj->currentRoute_->totalDelay_ << ",";
        repStr << vehicleObj->currentRoute_->routeRequests_.size() << ",";
        repStr << vehicleObj->currentRoute_->createTime_ << ",";
        repStr << vehicleObj->currentRoute_->totalLength_ << "\n";
    }
    return repStr.str();
}

void Instance::saveStatus(InputPaths &inputPaths, float simulationStart, float instDuration) {
    std::ofstream myFile;
    int setwSize = 15;
    int nbOnboards = 0;
    int nbWaiting = 0;
    int nbRequests = 0;

    // print vehicles
    myFile.open (inputPaths.getOutputVehicles(), std::ofstream::app);
    myFile << "COLUMNS\n\n" << "vehicle_ID\n" << "capacity\n" << "depart_Time\n" << "end_Time\n" << "depart_ID\n";
    myFile << "sink_ID\n" << "departZone_ID\n" << "sinkZone_ID\n" << "Route_size\n" << "LDual\n" << "IDuals\n\n" << "VEHICLES_INFO" << std::endl;

    for (auto & vehicleObj : vehicles_) {
        nbOnboards += int(vehicleObj->onboards_.size());
        myFile << std::left << std::setw(7) << vehicleObj->vehicleID_;
        myFile << std::setw(10) << vehicleObj->capacity_ ;
        myFile << std::setw(10) << vehicleObj->departTime_ ;
        myFile << std::setw(10) << vehicleObj->endTime_ ;
        myFile << std::setw(setwSize) << vehicleObj->departNode_->locationID_ ;
        myFile << std::setw(setwSize) << vehicleObj->sinkNode_->locationID_;
        myFile << std::setw(setwSize) << vehicleObj->departNode_->zoneID_ ;
        myFile << std::setw(setwSize) << vehicleObj->sinkNode_->zoneID_;
        myFile << std::setw(setwSize) << vehicleObj->currentRoute_->routeNodes_.size();
        myFile << std::setw(setwSize) << vehicleObj->InitialDual_;
        myFile << std::setw(setwSize) << vehicleObj->dual_ << "\n";
    }
    myFile.close();

    // print onboard requests
    myFile.open (inputPaths.getOutputOnboards(), std::ofstream::app);
    myFile << "COLUMNS\n\n" << "passenger_count\n" << "pickup_ID\n" << "dropoff_ID\n" << "request_time_sec\n";
    myFile << "pickup_time_sec\n" << "pickup_depart_sec\n" << "vehicle_ID\n" << "pickup_district\n" <<  "dropoff_district\n" << "position\n\n" << "REQUESTS_INFO" << std::endl;

    for (auto & vehicleObj : vehicles_) {
        for (int i = 1; i < vehicleObj->currentRoute_->routeNodes_.size(); ++i) {
            if (vehicleObj->currentRoute_->routeNodes_[i]->initialType_ != SINK && vehicleObj->currentRoute_->routeNodes_[i]->related_Request_->requestStatus_ == ON_BOARD){
                PRequest requestObj = vehicleObj->currentRoute_->routeNodes_[i]->related_Request_;
                myFile << std::left << std::setw(7) << requestObj->nbPassengers_ ;
                myFile << std::setw(10) << requestObj->PickUpID_ ;
                myFile << std::setw(10) << requestObj->DropOffID_ ;
                myFile << std::setw(10) << requestObj->earlyPick_ ;
                myFile << std::setw(10) << requestObj->pickTime_;
                myFile << std::setw(10) << (vehicleObj->currentRoute_->routeNodes_[i]->pairNode_)->departTime_;
                myFile << std::setw(setwSize) << vehicleObj->vehicleID_;
                myFile << std::setw(setwSize) << requestObj->pickZoneID_;
                myFile << std::setw(setwSize) << requestObj->dropZoneID_;
                myFile << std::setw(setwSize) << i << "\n";
            }
        }
    }
    myFile.close();

    // print waiting requests (requests that received in previous epochs and not served yet)
    myFile.open (inputPaths.getOutputWaitRequests(), std::ofstream::app);
    myFile << "COLUMNS\n\n" << "passenger_count\n" << "pickup_ID\n" << "dropoff_ID\n" << "request_time_sec\n";
    myFile << "pickup_district\n" << "dropoff_district\n" << "LDuals\n" << "IDuals\n" << "vehicle_ID\n" << "pick_position\n" << "drop_position\n\n";
    myFile << "REQUESTS_INFO" << std::endl;

    for (auto & vehicleObj : vehicles_) {
        for (int i = 1; i < vehicleObj->currentRoute_->routeNodes_.size(); ++i) {
            if (vehicleObj->currentRoute_->routeNodes_[i]->type_ == PICKUP &&
                vehicleObj->currentRoute_->routeNodes_[i]->related_Request_->requestStatus_ != ON_BOARD){
                PRequest requestObj = vehicleObj->currentRoute_->routeNodes_[i]->related_Request_;
                myFile << std::left << std::setw(7) << requestObj->nbPassengers_;
                myFile << std::setw(10) << requestObj->PickUpID_;
                myFile << std::setw(10) << requestObj->DropOffID_;
                myFile << std::setw(10) << requestObj->earlyPick_;
                myFile << std::setw(10) << requestObj->pickZoneID_;
                myFile << std::setw(10) << requestObj->dropZoneID_;
                myFile << std::setw(10) << requestObj->InitialDual_;
                myFile << std::setw(10) << requestObj->dual_;
                myFile << std::setw(10) << vehicleObj->vehicleID_;
                myFile << std::setw(10) << i;
                for (int j = i+1; j < vehicleObj->currentRoute_->routeNodes_.size(); ++j){
                    if (vehicleObj->currentRoute_->routeNodes_[j]->initialType_ != SINK) {
                        if (vehicleObj->currentRoute_->routeNodes_[i]->related_Request_->getRequestId() ==
                            vehicleObj->currentRoute_->routeNodes_[j]->related_Request_->getRequestId())
                            myFile << std::setw(10) << j << "\n";
                    }
                }
                nbWaiting ++;
            }
        }
    }
    for (auto & requestObj: requests_) {
        if ((requestObj->requestStatus_ == NO_ACTION)&&(requestObj->earlyPick_ < simulationStart) && requestObj->solVehicleID_ == 999999) {
            nbWaiting ++;
            myFile << std::left << std::setw(7) << requestObj->nbPassengers_;
            myFile << std::setw(10) << requestObj->PickUpID_;
            myFile << std::setw(10) << requestObj->DropOffID_;
            myFile << std::setw(10) << requestObj->earlyPick_;
            myFile << std::setw(10) << requestObj->pickZoneID_;
            myFile << std::setw(10) << requestObj->dropZoneID_;
            myFile << std::setw(10) << requestObj->InitialDual_;
            myFile << std::setw(10) << requestObj->dual_;
            myFile << std::setw(10) << 0;
            myFile << std::setw(10) << 0;
            myFile << std::setw(10) << 0  << "\n";
        }
        /*else if (requestObj->earlyPick_ > simulationStart)
            nbRequests++;*/
    }
    myFile.close();

    // print trip requests (requests that are not received yet)
    myFile.open (inputPaths.getOutputTrip(), std::ofstream::app);
    myFile << "COLUMNS\n\n" << "passenger_count\n" << "pickup_ID\n" << "dropoff_ID\n" << "request_time_sec\n";
    myFile << "pickup_district\n" <<  "dropoff_district\n\n";
    myFile << "REQUESTS_INFO" << std::endl;

    for (auto & requestObj: requests_) {
        if (requestObj->earlyPick_ >= simulationStart && requestObj->earlyPick_ <= simulationStart + instDuration) {
            myFile << std::left << std::setw(7) << requestObj->nbPassengers_;
            myFile << std::setw(10) << requestObj->PickUpID_;
            myFile << std::setw(10) << requestObj->DropOffID_;
            myFile << std::setw(10) << requestObj->earlyPick_;
            myFile << std::setw(10) << requestObj->pickZoneID_;
            myFile << std::setw(10) << requestObj->dropZoneID_ << "\n";
            nbRequests++;
        }
    }
    myFile.close();

    // print instance
    myFile.open (inputPaths.getOutputInstance(), std::ofstream::app);
    myFile << "INSTANCE = " << inputPaths.getInstanceNameOut() << std::endl << std::endl;
    myFile << "SIMULATION_START = " << simulationStart << std::endl << std::endl;
    myFile << "NUM_VEHICLES = " << nbVehicles_ << std::endl;
    myFile << "NUM_ONBOARDS = " << nbOnboards << std::endl;
    myFile << "NUM_RECEIVED = " << nbWaiting << std::endl;
    myFile << "NUM_REQUESTS = " << nbRequests << std::endl;
    myFile << "NUM_LOCATIONS = " << nbLocations_ << std::endl;
    myFile.close();
}

void Instance::updateTaskIndexLabeling() {
    int orderCounter = 0;
    /*std::stable_sort(requests_.begin(),requests_.end(),[](const PRequest &lhs, const PRequest &rhs){
        return lhs->dual_ > rhs->dual_;});*/

    for (auto & requestObj : requests_){
        requestObj->taskIndexLabel_ = orderCounter;
        orderCounter++;
    }
    int firstIndex = orderCounter;
    for (auto & vehicleObj : vehicles_){
        orderCounter = firstIndex;
        if (vehicleObj->currentRoute_->routeSize_ > 1) {
            for (int i = 1; i < vehicleObj->currentRoute_->routeSize_; ++i) {
                if ((vehicleObj->currentRoute_->routeNodes_[i]->nodeStatus_ == PLANNED)||
                    (vehicleObj->currentRoute_->routeNodes_[i]->nodeStatus_ == COMMITTED && vehicleObj->currentRoute_->routeNodes_[i]->type_ == DROPOFF)){
                    vehicleObj->currentRoute_->routeNodes_[i]->related_Request_->taskIndexLabel_ = orderCounter;
                    orderCounter++;
                }
            }
        }
    }
}

void Instance::resetVehicleSelection() {
    for (auto & vehicleObj : vehicles_)
        vehicleObj->selected_ = false;
}

int Instance::getNbUnselectedVehicles() {
    int nbUnselected = 0;
    for (auto & vehicleObj : vehicles_){
        if (!vehicleObj->selected_)
            nbUnselected++;
    }
    return nbUnselected;
}

std::string Instance::saveReqDuals(int epoch, int isudIter, string model) {
    std::stringstream repStr;
    for (auto & requestObj : requests_) {
        repStr << epoch << ",";
        repStr << isudIter << ",";
        repStr << requestObj->getRequestId() << ",";
        repStr << requestObj->dual_ << ",";
        repStr << model << ",";
        repStr << requestObj->penalty_ << "\n";
    }
    return repStr.str();
}

std::string Instance::saveVehDuals(int epoch, int isudIter, string model) {
    std::stringstream repStr;
    for (auto & vehicleObj : vehicles_) {
        repStr << epoch << ",";
        repStr << isudIter << ",";
        repStr << vehicleObj->vehicleID_ << ",";
        repStr << vehicleObj->dual_ << ",";
        repStr << model << "\n";
    }
    return repStr.str();
}

































