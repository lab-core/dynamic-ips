//
// Created by Ella on 2021-09-12.
//

#include "Instance.h"

#include <utility>
vector2D<float> durationMatrix_;
//-----------------------------------------------------------------------------
//  Instance class
//  contains the instance data, including vehicle info and requests
//-----------------------------------------------------------------------------
// Constructor and Destructor
Instance::Instance(std::string name, float simulationStart, int nbVehicles, int nbOnboards, int nbReceived,
                   std::vector<PVehicle> &vehicles, int nbRequests, int nbLocations, PGraph mainGraph) : name_(std::move(name)),
                                                                                                          simulationStartTime_(simulationStart), nbVehicles_(nbVehicles), vehicles_(vehicles),
                                                                                                          nbRequests_(nbRequests), nbOnboards_(nbOnboards), nbWaiting_(nbReceived), nbLocations_(nbLocations),
                                                                                                          instGraph_(std::move(mainGraph)) {
    nbNewRequests_ = nbRequests;
    std::cout << "Instance created!"<< std::endl;
    requests_.reserve(nbRequests + nbOnboards);
    vehicles.reserve(nbVehicles);
    nbTasks_ = nbRequests;
    nbZones_ = 0;
    nbRejected_ = 0;
    nbIdle_ = 0;
    passPerVehicle_ = 0;
    requestPerVehicle_ = 0;
    nodePerVehicle_ = 0;
    nbReturn_ = 0;
    nbStateChanged_ = 0;
    nbInitialOnboards_ = nbOnboards;
    nbCommitted_ = 0;
}


Instance::Instance(const Instance &mainInst) : name_(mainInst.name_) {
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
    nbTasks_ = 0;
    zones_ = mainInst.zones_;
    nbZones_ = mainInst.nbZones_;
    nbIdle_ = 0;
    nbReturn_ = 0;
    nbStateChanged_ = 0;
    nbRejected_ = 0;
    lastCommittedRequests_ = mainInst.lastCommittedRequests_;
    passPerVehicle_ = 0;
    requestPerVehicle_ = 0;
    nodePerVehicle_ = 0;
    nbCommitted_ = 0;
    nbInitialOnboards_ = 0;
}


void Instance::resetInstance() {
    nbRequests_ = 0;
    nbNewRequests_ = 0;
    nbWaiting_ = 0;

//    instGraph_ = std::make_shared<Graph>();

    requests_.clear();
    nameToRequest_.clear();
    nbTasks_ = 0;
    instGraph_->nodes_.clear();
    instGraph_->nbNodes_ = 0;
    instGraph_->pickNodes_.clear();
    instGraph_->dropNodes_.clear();
    instGraph_->sourceNodes_.clear();
    instGraph_->onboards_.clear();
    instGraph_->sinkNodes_.clear();
    instGraph_->newPickNodes_.clear();
    lastCommittedRequests_.clear();
}

/*Instance::~Instance() {
    instGraph_.reset();
}*/

// Display function
std::string Instance::toString() const {
    std::stringstream repStr;
    repStr << std::endl;
    repStr << "***************************************************************************" << std::endl;
    repStr << "**                             Instance Info                             **" << std::endl;
    repStr << "***************************************************************************" << std::endl;
    repStr << "# " << std::endl;
    repStr << "# INSTANCE_NAME       \t= " << name_ << std::endl;
    repStr << "# NUMBER_OF_VEHICLES  \t= " << nbVehicles_ <<std::endl;
    repStr << "# NUMBER_OF_REQUESTS  \t= " << nbRequests_ - nbInitialOnboards_ <<std::endl;
    repStr << "# " << std::endl;

    repStr << "------------------------ PARAMETERS AND OPTIONS -------------------------" << std::endl;
    repStr << parameters_->toString();
    repStr << "# " << std::endl;
    return repStr.str();
}

std::string Instance::solutionToString() {
    SolutionMetrics metrics;

    std::stringstream repStr;

    instRepStr_ << name_ << "," << "R" << nbRequests_ - nbInitialOnboards_ << ","
                << eu::toString(parameters_->mainAlgorithm_) << ","
                << eu::toString(parameters_->solutionMode_) << ","
                << nbVehicles_ << "," << nbRequests_ - nbInitialOnboards_ << ","
                << nbInitialOnboards_ << ",";

    // print table header
    repStr << "# --------------------------------------------------------------------------------------------------------" << std::endl;
    repStr << "#  REQUEST_ID" << "  ";
    repStr << "READY_TIME(s)" << "   ";
    repStr << "PICK_TIME(s)" << "   ";
    repStr << "DROP_TIME(s)" << "   ";
    repStr << "WAIT_TIME(s)" << "  ";
    repStr << "TRIP_DELAY(s)" << "   ";
    repStr << "#PASSENGERS" <<std::endl;
    repStr << "# --------------------------------------------------------------------------------------------------------" << std::endl;

    processRequestsAndCollectMetrics(repStr, metrics);
    repStr << "# --------------------------------------------------------------------------------------------------------" << std::endl;
    processVehiclesAndCollectMetrics(metrics);

    // Print summary statistics using the metrics object
    metrics.printTotalMetrics(repStr, SENTENCE_SIZE, nbVehicles_, nbRequests_, nbInitialOnboards_);
    metrics.printTotalMetricsInMinutes(repStr, SENTENCE_SIZE, SECONDS_PER_MINUTE);
    metrics.printAverageMetrics(repStr, SENTENCE_SIZE, nbVehicles_);

    repStr << "#" << std::endl;
    instRepStr_ << metrics.totalCustomers << ",";

    if (metrics.totalCustomers < 1000)
        instRepStr_ << "10000 <" << ",";

    else if (metrics.totalCustomers > 135000)
        instRepStr_ << "135000 <" << ",";
    else if (metrics.totalCustomers >= 120000 && metrics.totalCustomers <= 135000)
        instRepStr_ << "125000 - 135000" << ",";
    else if (metrics.totalCustomers >= 70000 && metrics.totalCustomers < 120000)
        instRepStr_ << "< 125000" << ",";
    else if (metrics.totalCustomers >= 50000 && metrics.totalCustomers < 70000)
        instRepStr_ << "< 50000 <" << ",";
    else if (metrics.totalCustomers >= 40000 && metrics.totalCustomers <= 50000)
        instRepStr_ << "40000 - 50000" << ",";
    else if (metrics.totalCustomers < 40000)
        instRepStr_ << "40000 <" << ",";

    instRepStr_ << metrics.totalNumServed << ","
                << metrics.numRejected << ","
                << metrics.totalRequestsWaiting / static_cast<float>(metrics.numServed) << ","
                << metrics.totalWeightedWaiting / static_cast<float>(metrics.totalCustomers) << ","
                << metrics.totalTripDelay / static_cast<float>(metrics.numServed) << ","
                << metrics.totalWeightedTripDelay / static_cast<float>(metrics.totalCustomers) << ","
                << metrics.idleTime / static_cast<float>(nbVehicles_) << ","
                << metrics.nbIdle << ","
                << static_cast<float>(metrics.totalStopLoad) / static_cast<float>(metrics.totalStops) << ",";
    return repStr.str();
}

void Instance::adjustParameters(const PConfig &config) const {
    parameters_->mainAlgorithm_ = static_cast<MainAlgorithm>(config->mainAlgo_);
    parameters_->solutionMode_ = static_cast<SolutionMode>(config->solMode_);
    if (parameters_->mainAlgorithm_ == F_ICG)
        parameters_->approach_ = ISUD;
    else if (parameters_->mainAlgorithm_ == GREEDY)
        parameters_->approach_ = Greedy;
    else if (parameters_->mainAlgorithm_ == MIP)
        parameters_->approach_ = MIP_SOLVER;
    else
        parameters_->approach_ = CG;
}

// this function updates the set of available requests, removed completed requests and updates onboards
void Instance::buildPartialData(const PInstance &mainInst, const std::vector<PRequest> &penaltyRequests, float elapsedTime,
                                int lastRecRequests) {
    nbReturn_ = mainInst->nbReturn_;
    nbStateChanged_ = mainInst->nbStateChanged_;

    mainInst->nbReturn_ = 0;
    mainInst->nbStateChanged_ = 0;

    for (auto & vehicleObj : mainInst->vehicles_){
        instGraph_->addNewNode(vehicleObj->departNode_);
        instGraph_->addNewNode(vehicleObj->sinkNode_);
    }
    nbNewRequests_ = 0;
    // add unscheduled requests
    for (auto & requestObj: penaltyRequests) {
        if (mainInst->parameters_->timeWindow_ == 0 || requestObj->latestPickup_ >= simulationStartTime_ + elapsedTime) {
            addRequest(requestObj);
            requestObj->coveredVehicles_.reset();
            requestObj->coveredVehicles_.resize(nbVehicles_);
            requestObj->insertedVehicles_.reset();
            requestObj->insertedVehicles_.resize(nbVehicles_);
            instGraph_->addNewNode(mainInst->instGraph_->pickNodes_[requestObj->getRequestId()]);
            instGraph_->addNewNode(mainInst->instGraph_->dropNodes_[requestObj->getRequestId()]);
        }
        else {
            requestObj->requestStatus_ = REJECTED;
        }
    }

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
                    if (this->requests_.back()->insertedVehicles_.empty())
                        this->requests_.back()->insertedVehicles_.resize(nbVehicles_);
                    if (this->requests_.back()->coveredVehicles_.empty())
                        this->requests_.back()->coveredVehicles_.resize(nbVehicles_);
                }
                // adding onboard nodes to the graph
                else if (vehicleObj->currentRoute_->routeNodes_[i]->nodeStatus_ == PLANNED)  {
                    instGraph_->nodes_.emplace(
                            std::pair<std::string, PNode>(vehicleObj->currentRoute_->routeNodes_[i]->nodeID_,
                                                          vehicleObj->currentRoute_->routeNodes_[i]));
                    instGraph_->onboards_.push_back(vehicleObj->currentRoute_->routeNodes_[i]);
                    instGraph_->nbNodes_++;
                }
            }
        }
    }



    // add new requests
    for (int i = lastRecRequests; i < mainInst->nbRequests_; ++i) {
        if (parameters_->solutionMode_ == ANYTIME) {
            if (mainInst->requests_[i]->requestTime_ <= simulationStartTime_ + elapsedTime) {
                if (mainInst->requests_[i]->solVehicleID_ == LARGE_CONSTANT) {
                    nbNewRequests_++;
                    addRequest(mainInst->requests_[i]);
                    mainInst->requests_[i]->insertedVehicles_.reset();
                    mainInst->requests_[i]->insertedVehicles_.resize(nbVehicles_);
                    mainInst->requests_[i]->coveredVehicles_.reset();
                    mainInst->requests_[i]->coveredVehicles_.resize(nbVehicles_);
                    instGraph_->addNewNode(mainInst->instGraph_->pickNodes_[i]);
                    instGraph_->addNewNode(mainInst->instGraph_->dropNodes_[i]);
                }

            }
            else
                break;
        }
        else {
            if (mainInst->requests_[i]->requestTime_ < simulationStartTime_ + elapsedTime) {
                if (mainInst->requests_[i]->solVehicleID_ == LARGE_CONSTANT) {
                    nbNewRequests_++;
                    addRequest(mainInst->requests_[i]);
                    mainInst->requests_[i]->insertedVehicles_.reset();
                    mainInst->requests_[i]->insertedVehicles_.resize(nbVehicles_);
                    mainInst->requests_[i]->coveredVehicles_.reset();
                    mainInst->requests_[i]->coveredVehicles_.resize(nbVehicles_);
                    instGraph_->addNewNode(mainInst->instGraph_->pickNodes_[i]);
                    instGraph_->addNewNode(mainInst->instGraph_->dropNodes_[i]);
                }
            }
            else
                break;
        }
    }

    nbOnboards_ = static_cast<int>(instGraph_->onboards_.size());
    updateRequestOrder();
}

void Instance::buildStaticData(const PInstance &mainInst, int lastRecRequests) {
    nbReturn_ = mainInst->nbReturn_;
    nbStateChanged_ = mainInst->nbStateChanged_;

    mainInst->nbReturn_ = 0;
    mainInst->nbStateChanged_ = 0;

    for (auto & vehicleObj : mainInst->vehicles_){
        instGraph_->addNewNode(vehicleObj->departNode_);
        instGraph_->addNewNode(vehicleObj->sinkNode_);
    }
    nbNewRequests_ = 0;


//    if (mainInst->parameters_->mainAlgorithm_ != GREEDY || mainInst->parameters_->greedyReOptimize_) {
        // add unperformed requests
        for (auto &vehicleObj: mainInst->vehicles_) {
            if (vehicleObj->currentRoute_->routeSize_ > 1) {
                for (int i = 1; i < vehicleObj->currentRoute_->routeSize_; ++i) {
                    if (vehicleObj->currentRoute_->routeNodes_[i]->type_ == PICKUP) {
                        addRequest(vehicleObj->currentRoute_->routeNodes_[i]->related_Request_);
                        instGraph_->addNewNode(vehicleObj->currentRoute_->routeNodes_[i]);
                        instGraph_->addNewNode(
                                mainInst->instGraph_->dropNodes_[vehicleObj->currentRoute_->routeNodes_[i]->related_Request_->getRequestId()]);
                    }
                        // adding onboard nodes to the graph
                    else if (vehicleObj->currentRoute_->routeNodes_[i]->nodeStatus_ == PLANNED)  {
                        instGraph_->nodes_.emplace(
                                std::pair<std::string, PNode>(vehicleObj->currentRoute_->routeNodes_[i]->nodeID_,
                                                              vehicleObj->currentRoute_->routeNodes_[i]));
                        instGraph_->onboards_.push_back(vehicleObj->currentRoute_->routeNodes_[i]);
                        instGraph_->nbNodes_++;
                    }
                }
            }
        }
//    }

    // add new requests
    for (int i = lastRecRequests; i < mainInst->nbRequests_; ++i) {
        if (mainInst->requests_[i]->solVehicleID_ == LARGE_CONSTANT) {
            nbNewRequests_++;
            addRequest(mainInst->requests_[i]);
            instGraph_->addNewNode(mainInst->instGraph_->pickNodes_[i]);
            instGraph_->addNewNode(mainInst->instGraph_->dropNodes_[i]);
        }
    }

    for (auto & requestObj : mainInst->requests_) {
        requestObj->coveredVehicles_.reset();
        requestObj->coveredVehicles_.resize(nbVehicles_);
        requestObj->insertedVehicles_.reset();
        requestObj->insertedVehicles_.resize(nbVehicles_);
    }

    nbOnboards_ = static_cast<int>(instGraph_->onboards_.size());
    updateRequestOrder();
}

void Instance::addNewRequests(const PInstance &mainInst, float elapsedTime, int lastRecRequests) {
    int orderCounter = requests_.back()->taskIndex_ + 1;
    // add new requests
    for (int i = lastRecRequests; i < mainInst->nbRequests_; ++i) {
        if (parameters_->solutionMode_ == ANYTIME) {
            if (mainInst->requests_[i]->requestTime_ <= simulationStartTime_ + elapsedTime) {
                if (mainInst->requests_[i]->solVehicleID_ == LARGE_CONSTANT) {
                    nbNewRequests_++;
                    addRequest(mainInst->requests_[i]);
                    instGraph_->addNewNode(mainInst->instGraph_->pickNodes_[i]);
                    instGraph_->addNewNode(mainInst->instGraph_->dropNodes_[i]);
                    requests_.back()->taskIndex_ = orderCounter;
                    orderCounter++;
                }

            }
            else
                break;
        }
        else {
            if (mainInst->requests_[i]->requestTime_ < simulationStartTime_ + elapsedTime) {
                if (mainInst->requests_[i]->solVehicleID_ == LARGE_CONSTANT) {
                    nbNewRequests_++;
                    addRequest(mainInst->requests_[i]);
                    instGraph_->addNewNode(mainInst->instGraph_->pickNodes_[i]);
                    instGraph_->addNewNode(mainInst->instGraph_->dropNodes_[i]);
                    requests_.back()->taskIndex_ = orderCounter;
                    orderCounter++;
                }
            }
            else
                break;
        }
    }
    nbTasks_ = static_cast<signed>(requests_.size());
}


// function to add requests from previous epochs to the current partial instance
void Instance::addRequest(const PRequest &request) {
    nbRequests_++;
    requests_.push_back(request);
//    request->setPenaltyEpoch(epoch , parameters, simulationStart);
//    request->setPenaltyEpoch(epoch - request->readEpoch_, parameters, simulationStart);
    nameToRequest_[request->name_] = request;
    //   updateRequestOrder();
}


void Instance::setInitialTimes(float commitTime) const {

    // if the vehicles start from the source, depart time is after the first epoch
    if (parameters_->solutionMode_ == STATIC){
        for (auto & vehicleObj : vehicles_){
            if (vehicleObj->onboards_.empty() && vehicleObj->departTime_ < simulationStartTime_)
                vehicleObj->setDepartTime(simulationStartTime_);
        }
    }
    else {
        for (auto & vehicleObj : vehicles_){
            if (vehicleObj->onboards_.empty()){
                if (vehicleObj->currentRoute_ == nullptr || vehicleObj->currentRoute_->routeSize_ <= 1) {
                    if (vehicleObj->departTime_ < simulationStartTime_ + commitTime) {
                        vehicleObj->idleTime_ += commitTime;
                        vehicleObj->setDepartTime(simulationStartTime_ + commitTime);
                    }
                }
            }
        }
    }
}


// function to sort vehicles based on ID
void Instance::resetVehicleOrder() {
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

        case ROUTE_SIZE :
            std::stable_sort(vehicles_.begin(), vehicles_.end(),[](const PVehicle &lhs, const PVehicle &rhs){
                return lhs->currentRoute_->routeSize_ < rhs->currentRoute_->routeSize_;});
            break;
        case BEST_REDUCE_COST :
            std::stable_sort(vehicles_.begin(), vehicles_.end(),[](const PVehicle &lhs, const PVehicle &rhs){
                return lhs->bestReducedCost_ < rhs->bestReducedCost_;});
            break;
    }
}

void Instance::sortZones() {

    for (auto & zoneObj : zones_){
        for (auto &nextZoneObj: zones_) {
            if (zoneObj.first != nextZoneObj.first) {
                nextZoneObj.second->travelToZone_ = durationMatrix_[zoneObj.second->centerLocationID_][nextZoneObj.
                    second->centerLocationID_];
                zoneObj.second->successors_.push_back(&(*nextZoneObj.second));
            }
        }
        std::stable_sort(zoneObj.second->successors_.begin(), zoneObj.second->successors_.end(), [](const Zone *lhs, const Zone *rhs) {
            return lhs->travelToZone_ < rhs->travelToZone_;
        });
    }
}

// function to update penalties in any time approach
void Instance::updatePenalties(float elapsedTime) const {
    for (auto & requestObj : requests_) {
        requestObj->setPenalty(elapsedTime, parameters_, simulationStartTime_);
        if (requestObj->committedPickTime_ == LARGE_CONSTANT)
            requestObj->latestPickup_ = requestObj->initialEarlyPick_ + requestObj->penalty_;
        else {
            requestObj->latestPickup_ = requestObj->committedPickTime_ + parameters_->pickupDeviationWindow_;

        }
        if (requestObj->solVehicleID_ == LARGE_CONSTANT) {
    //        requestObj->dual_ = requestObj->penalty_;
            requestObj->InitialDual_ = requestObj->Req_W3_ * requestObj->penalty_;
            requestObj->lastDual_ = requestObj->Req_W3_ * requestObj->penalty_;
        }
    }
}

//determine an order for requests to use in CPLEX modeling
void Instance::updateRequestOrder() {
    nbTasks_ = 0;
//    requestToOrder_.clear();

    int orderCounter = 0;
    nbTasks_ = static_cast<signed>(requests_.size());
    for (auto & requestObj : requests_){
        requestObj->taskIndex_ = orderCounter;
        orderCounter++;
    }
}

void Instance::updateRequestInc() {
    nbTasks_ = 0;
    //    requestToOrder_.clear();

    int orderCounter = 0;
    nbTasks_ = static_cast<signed>(requests_.size());

    for (auto &vehicleObj: vehicles_) {
        if (vehicleObj->currentRoute_->routeSize_ > 1) {
            for (int i = 1; i < vehicleObj->currentRoute_->routeSize_; ++i) {
                if (vehicleObj->currentRoute_->routeNodes_[i]->type_ == PICKUP) {
                    vehicleObj->currentRoute_->routeNodes_[i]->related_Request_->taskIndex_ = orderCounter;
                    orderCounter++;
                }
            }
        }
    }
    for (auto &requestObj: requests_) {
        if (requestObj->solVehicleID_ == LARGE_CONSTANT) {
            requestObj->taskIndex_ = orderCounter;
            orderCounter++;
        }
    }
}

void Instance::updateTaskIndexLabeling() const {
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
                if (vehicleObj->currentRoute_->routeNodes_[i]->nodeStatus_ == PLANNED){
                    vehicleObj->currentRoute_->routeNodes_[i]->related_Request_->taskIndexLabel_ = orderCounter;
                    orderCounter++;
                }
            }
        }
    }
}

void Instance::resetDuals() const {
    for (auto & requestObj : requests_) {
        requestObj->dual_ = requestObj->penalty_;
    }
    for (auto & vehicleObj : vehicles_) {
        vehicleObj->dual_ = 0;
    }
}

void Instance::calcVehicleMetric() {
    nbIdle_ = 0.0;
    passPerVehicle_ = 0.0;
    requestPerVehicle_ = 0.0;
    nodePerVehicle_ = 0.0;
    for (auto & vehicleObj : vehicles_) {
        requestPerVehicle_ += vehicleObj->currentRoute_->routeRequests_.size();
        nodePerVehicle_ += vehicleObj->currentRoute_->routeSize_ - 1;
        passPerVehicle_ += vehicleObj->numPassengers_;
        if (vehicleObj->currentRoute_->routeSize_ == 1)
            nbIdle_++;
    }
    float nbActiveVehicles = vehicles_.size() - nbIdle_;
    nodePerVehicle_ = nodePerVehicle_ / nbActiveVehicles;
    requestPerVehicle_ = requestPerVehicle_ / nbActiveVehicles;
    passPerVehicle_ = passPerVehicle_ / nbActiveVehicles;
}

void Instance::countCommittedRequests() {
    nbCommitted_ = 0;
    for (auto & requestObj : requests_) {
        if (requestObj->committedPickTime_ < LARGE_CONSTANT)
            ++nbCommitted_;
    }
}

void Instance::resetAssignedVehicles() const {
    for (auto & requestObj : requests_)
        requestObj->solVehicleID_ = LARGE_CONSTANT;
}

void Instance::setAssignedEpochVehicles(float assignTime) const {
    for (auto &vehicleObj : vehicles_) {
        for (auto & requestObj : vehicleObj->currentRoute_->routeRequests_) {
            if (requestObj->epochVehicleID_ == LARGE_CONSTANT) {
                requestObj->nbSwitch_ = 1;
                requestObj->epochVehicleID_  = vehicleObj->vehicleID_;
                requestObj->assignTime_ = assignTime;
            }
            else {
                if (requestObj->epochVehicleID_ != vehicleObj->vehicleID_) {
                    requestObj->nbSwitch_ ++;
                    requestObj->epochVehicleID_  = vehicleObj->vehicleID_;
                }
            }
        }
    }
}

void Instance::setNodeIndices() const {
    for (int i = 0; i < instGraph_->pickNodes_.size(); ++i) {
        PNode nodeObj = instGraph_->pickNodes_[i];
        nodeObj->nodeIndex_ = getIndex(nodeObj, i, static_cast<int>(instGraph_->pickNodes_.size()));
    }
    for (int i = 0; i < instGraph_->dropNodes_.size(); ++i) {
        PNode nodeObj = instGraph_->dropNodes_[i];
        nodeObj->nodeIndex_ = getIndex(nodeObj, i, static_cast<int>(instGraph_->dropNodes_.size()));
    }

    for (auto & vehicleObj : vehicles_){
        vehicleObj->departNode_->nodeIndex_ = getIndex(vehicleObj->departNode_, 0, static_cast<int>(vehicleObj->onboards_.size()));
        vehicleObj->sinkNode_->nodeIndex_ = getIndex(vehicleObj->sinkNode_, 0, static_cast<int>(vehicleObj->onboards_.size()));
        for (int i = 0; i < vehicleObj->onboards_.size(); ++i) {
            PNode nodeObj = instGraph_->nodes_[ vehicleObj->onboards_[i]];
            nodeObj->nodeIndex_ = getIndex(nodeObj, i, static_cast<int>(instGraph_->dropNodes_.size()));
        }
    }
}

//-----------------------------------------------------------------------------
// Functions to save results and outputs in csv and txt files
//-----------------------------------------------------------------------------
std::string Instance::saveSolutionRoutes() const {
    std::stringstream repStr;
    repStr << "VehicleID,NodeID,RequestTime,ReachTime,NodeType, LocationID, DepartTime, nbPassengers, vehicleLoad, PlanedReach, PlanedDepart, TravelTime" << std::endl;
    for (auto & vehicleObj : vehicles_) {
        for (int j = 0; j< vehicleObj->solutionRoute_->routeNodes_.size(); j++) {
            repStr << vehicleObj->vehicleID_ << ",";
            repStr << vehicleObj->solutionRoute_->routeNodes_[j]->nodeID_ << ",";
            repStr << vehicleObj->solutionRoute_->routeNodes_[j]->initialReadyTime_ << ",";
            repStr << vehicleObj->solutionRoute_->routeNodes_[j]->reachTime_ << ",";
            repStr << vehicleObj->solutionRoute_->routeNodes_[j]->initialType_ << ",";
            repStr << vehicleObj->solutionRoute_->routeNodes_[j]->locationID_ << ",";
            repStr << vehicleObj->solutionRoute_->routeNodes_[j]->nodeDepartTime_ << ",";
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

std::string Instance::saveRequestsResults() const {
    std::stringstream repStr;
    repStr << "RequestID,nbPassengers, PickupID,DropOffID,ReadyTime,PickTime,DropTime,EarliestPick,LatestPick,"
              "AssignTime,CommitWaitTime,plannedWaitTime,VehicleID,#VehicleSwitch,WaitTime,TripDelay,MaxTravelTime,"
              "MinTravelTime,minDual,maxDual,zoneID" << std::endl;

    for (auto & requestObj : requests_) {
        if (requestObj->getRequestId() >= nbInitialOnboards_) {
            repStr << requestObj->getRequestId() << ",";
            repStr << requestObj->nbPassengers_ << ",";
            repStr << requestObj->PickUpID_ << ",";
            repStr << requestObj->DropOffID_ << ",";
            repStr << requestObj->initialEarlyPick_ << ",";
            repStr << requestObj->pickTime_ << ",";
            repStr << requestObj->dropTime_ << ",";
            repStr << requestObj->earlyPick_ << ",";
            repStr << requestObj->latestPickup_ << ",";
            repStr << requestObj->assignTime_ - requestObj->initialEarlyPick_ << ",";
            repStr << requestObj->commitTime_ - requestObj->initialEarlyPick_ << ",";
            repStr << requestObj->committedPickTime_ - requestObj->initialEarlyPick_ << ",";
            repStr << requestObj->allocVehicleID_ << ",";
            repStr << requestObj->nbSwitch_ << ",";
            repStr << requestObj->pickTime_ - requestObj->initialEarlyPick_ << ",";
            repStr << requestObj->dropTime_ - requestObj->pickTime_ - requestObj->minTravelTime_ << ",";
            repStr << requestObj->maxTravelTime_ << ",";
            repStr << requestObj->minTravelTime_ << ",";
            repStr << requestObj->minDual_ << ",";
            repStr << requestObj->maxDual_ << ",";
            repStr << requestObj->pickZoneID_ << "\n";
        }
    }
    return repStr.str();
}


std::string Instance::saveVehicleResults() const {
    std::stringstream repStr;
    repStr << "VehicleID,startID,capacity,startTime,endTime,LastVisitTime,#Stops,#RequestsServed,WaitTime,"
              "TripDelay,idleTime,serviceTime,driveFullTime,driveEmptyTime,returnEmptyTime" << std::endl;

    for (auto & vehicleObj : vehicles_) {
        repStr << vehicleObj->vehicleID_ << ",";
        repStr << vehicleObj->sinkNode_->locationID_ << ",";
        repStr << vehicleObj->capacity_ << ",";
        repStr << simulationStartTime_ << ",";
        repStr << vehicleObj->endTime_ << ",";
        repStr << vehicleObj->solutionRoute_->routeNodes_.back()->nodeDepartTime_ << ",";
        repStr << vehicleObj->solutionRoute_->routeSize_ << ",";
        repStr << vehicleObj->solutionRoute_->routeRequests_.size() << ",";
        repStr << vehicleObj->solutionRoute_->totalWait_ << ",";
        repStr << vehicleObj->solutionRoute_->totalTripDelay_ << ",";
        repStr << vehicleObj->idleTime_ << ",";
        repStr << vehicleObj->serviceTime_ << ",";
        repStr << vehicleObj->driveFullTime_ << ",";
        repStr << vehicleObj->driveEmptyTime_ << ",";
        repStr << vehicleObj->returnEmptyTime_ << "\n";
    }
    return repStr.str();
}


void Instance::writeFinalOutputs(const InputPaths& inputPaths, const PConfig& config) const {

    std::cout << "Writing output files..." << std::endl;
    try {

        // Write solution routes CSV
        {
            Tools::LogOutput solutionRoutesStream(inputPaths.getOutputFinalRoutes());
            solutionRoutesStream << saveSolutionRoutes();
            solutionRoutesStream.close();
      //      std::cout << "✓ Solution routes written" << std::endl;
        }

        // Write request results CSV
        {
            Tools::LogOutput requestResultsStream(inputPaths.getOutputFinalRequests());
            requestResultsStream << saveRequestsResults();
            requestResultsStream.close();
     //       std::cout << "✓ Request results written" << std::endl;
        }

        // Write vehicle results CSV
        {
            Tools::LogOutput vehiclesResultsStream(inputPaths.getOutputFinalVehicles());
            vehiclesResultsStream << saveVehicleResults();
            vehiclesResultsStream.close();
     //       std::cout << "✓ Vehicle results written" << std::endl;
        }

        // Write summary CSV (with header if file doesn't exist)
        {

            Tools::LogOutput finalInstanceStream(inputPaths.getOutputSummary(), true); // append mode
            finalInstanceStream
                        << "VehicleFile,paramFile,Name,Instance,Algorithm,Mode,#vehicles,#requests,#initialOnboards,"
                           "#customers,customer Group,#served Req,#Rejected Req,wait/req,wait/cust,tripDelay/req,"
                           "tripDelay/cust,idle time/vehicle,#Idle Vehicles,#pass in vehicle,#epoch,#LMP Iter,"
                           "#IMP Iter,#RP Iter,#CP Iter,#Zoom Iter,#SP Iter ,MASTER time,RP time,CP time,Zoom time,"
                           "SP time,Greedy time,Rebalance time,Total time,RP/ISUD,CP/ISUD,MASTER/Total,SP/Total,"
                           "CPSuccess,CPFails,CGSuccess\n";

            // Write data row
            finalInstanceStream << config->vehicleFolder_ << "," << config->scenario_ << ",";
            finalInstanceStream << instRepStr_.str();
            finalInstanceStream.close();
     //       std::cout << "✓ Summary written" << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "Exception during output writing: " << e.what() << std::endl;
    }
}


std::string Instance::saveMPRoutes(int epoch, int mpIter) const {
    std::stringstream repStr;
    for (auto & vehicleObj : vehicles_) {
        for (auto & nodeObj : vehicleObj->currentRoute_->routeNodes_) {
            repStr << epoch << ",";
            repStr << mpIter << ",";
            repStr << vehicleObj->vehicleID_ << ",";
            repStr << nodeObj->nodeID_ << ",";
            repStr << nodeObj->initialReadyTime_ << ",";
            repStr << nodeObj->readyTime_ << ",";
            repStr << nodeObj->reachTime_ << ",";
            repStr << nodeObj->type_ << ",";
            repStr << nodeObj->locationID_ << ",";
            repStr << vehicleObj->currentRoute_->getRouteId() << ",";
            repStr << vehicleObj->currentRoute_->incompatibilityDegree_ << "\n";
        }
    }
    return repStr.str();
}


std::string Instance::saveRoutesTimes(int epoch) const {
    std::stringstream repStr;
    for (auto & vehicleObj : vehicles_) {
        repStr << epoch << ",";
        repStr << vehicleObj->vehicleID_ << ",";
        repStr << vehicleObj->currentRoute_->getRouteId() << ",";
        repStr << vehicleObj->currentRoute_->totalWait_ << ",";
        repStr << vehicleObj->currentRoute_->routeRequests_.size() << ",";
        repStr << vehicleObj->currentRoute_->createTime_ << ",";
        repStr << vehicleObj->currentRoute_->totalLength_ << "\n";
    }
    return repStr.str();
}

void Instance::saveStatus(InputPaths &inputPaths, float simulationStart, float instDuration, std::string prefix) const {
    inputPaths.makeInstanceOutput(prefix);
    if (parameters_->mainAlgorithm_ == GREEDY){
        for (auto & requestObj: requests_)
            requestObj->dual_ = requestObj->penalty_;
    }
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
        nbOnboards += static_cast<int>(vehicleObj->onboards_.size());
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
    myFile << "COLUMNS\n\n" << "passenger_count\n" << "pickup_ID\n" << "dropoff_ID\n" << "ready_time_sec\n";
    myFile << "pickup_time_sec\n" << "pickup_depart_sec\n" << "vehicle_ID\n" << "pickup_district\n" <<  "dropoff_district\n" << "position\n\n" << "REQUESTS_INFO" << std::endl;

    for (auto & vehicleObj : vehicles_) {
        for (int i = 1; i < vehicleObj->currentRoute_->routeNodes_.size(); ++i) {
            if (vehicleObj->currentRoute_->routeNodes_[i]->initialType_ != SINK && vehicleObj->currentRoute_->routeNodes_[i]->related_Request_->requestStatus_ == ON_BOARD){
                PRequest requestObj = vehicleObj->currentRoute_->routeNodes_[i]->related_Request_;
                myFile << std::left << std::setw(7) << requestObj->nbPassengers_ ;
                myFile << std::setw(10) << requestObj->PickUpID_ ;
                myFile << std::setw(10) << requestObj->DropOffID_ ;
                myFile << std::setw(10) << requestObj->initialEarlyPick_ ;
                myFile << std::setw(10) << requestObj->pickTime_;
                myFile << std::setw(10) << (vehicleObj->currentRoute_->routeNodes_[i]->pairNode_)->nodeDepartTime_;
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
    myFile << "COLUMNS\n\n" << "passenger_count\n" << "pickup_ID\n" << "dropoff_ID\n" << "ready_time_sec\n";
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
                myFile << std::setw(10) << requestObj->initialEarlyPick_;
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
        if ((requestObj->requestStatus_ == NO_ACTION)&&(requestObj->requestTime_ < simulationStart) && requestObj->solVehicleID_ == LARGE_CONSTANT) {
            //          if (requestObj->requestTime_ >= simulationStart - 120) {
            nbWaiting++;
            myFile << std::left << std::setw(7) << requestObj->nbPassengers_;
            myFile << std::setw(10) << requestObj->PickUpID_;
            myFile << std::setw(10) << requestObj->DropOffID_;
            myFile << std::setw(10) << requestObj->initialEarlyPick_;
            myFile << std::setw(10) << requestObj->pickZoneID_;
            myFile << std::setw(10) << requestObj->dropZoneID_;
            myFile << std::setw(10) << requestObj->InitialDual_;
            myFile << std::setw(10) << requestObj->dual_;
            myFile << std::setw(10) << 0;
            myFile << std::setw(10) << 0;
            myFile << std::setw(10) << 0 << "\n";
            //           }
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
        if (requestObj->requestTime_ >= simulationStart && requestObj->requestTime_ < simulationStart + instDuration) {
            myFile << std::left << std::setw(7) << requestObj->nbPassengers_;
            myFile << std::setw(10) << requestObj->PickUpID_;
            myFile << std::setw(10) << requestObj->DropOffID_;
            myFile << std::setw(10) << requestObj->initialEarlyPick_;
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

std::string Instance::saveReqDuals(int epoch, int isudIter, const string& model) const {
    std::stringstream repStr;
    for (auto & requestObj : requests_) {
        repStr << epoch << ",";
        repStr << isudIter << ",";
        repStr << requestObj->getRequestId() << ",";
        repStr << requestObj->InitialDual_ << ",";
        repStr << requestObj->dual_ << ",";
        repStr << requestObj->minDual_ << ",";
        repStr << requestObj->penalty_ << ",";
        repStr << requestObj->maxDual_ << ",";
        repStr << model << ",";
        repStr << requestObj->penalty_ << ",";
        repStr << requestObj->InitialDual_ -  requestObj->dual_<< "\n";
    }
    return repStr.str();
}

std::string Instance::saveVehDuals(int epoch, int isudIter, const string& model) const {
    std::stringstream repStr;
    for (auto & vehicleObj : vehicles_) {
        repStr << epoch << ",";
        repStr << isudIter << ",";
        repStr << vehicleObj->vehicleID_ << ",";
        repStr << vehicleObj->InitialDual_ << ",";
        repStr << vehicleObj->dual_ << ",";
        repStr << model << ",";
        repStr << vehicleObj->currentRoute_->totalWait_ << ",";
        repStr << vehicleObj->currentRoute_->totalTripDelay_ << ",";
        repStr << vehicleObj->currentRoute_->objCoef_ << ",";
        repStr << vehicleObj->currentRoute_->routeSize_ << ",";
        repStr << vehicleObj->currentRoute_->routeRequests_.size() << ",";
        repStr << vehicleObj->currentRoute_->totalLength_ << ",";
        repStr << vehicleObj->stateChanged_ << "\n";
        vehicleObj->InitialDual_ = vehicleObj->dual_;
    }
    return repStr.str();
}

void Instance::processRequestsAndCollectMetrics(std::stringstream &repStr, SolutionMetrics &metrics) const {
int startIndex = nbInitialOnboards_;
    // print the internal nodes of the route
    for (int i = startIndex; i < nbRequests_; ++i) {
        repStr << std::fixed;
        repStr << std::setprecision(2);
        repStr << "#" << std::right << std::setw(9) << requests_[i]->getRequestId() << "       ";
        repStr << std::right << std::setw(9) << requests_[i]->initialEarlyPick_ << " (s)  ";
        if (requests_[i]->requestStatus_ == COMPLETED) {
            repStr << std::right << std::setw(9) << requests_[i]->pickTime_ << " (s)  ";
            repStr << std::right << std::setw(9) << requests_[i]->dropTime_ << " (s)  ";
            repStr << std::right << std::setw(9) << requests_[i]->pickTime_ - requests_[i]->initialEarlyPick_ << " (s)  ";

//            float travelTime = requests_[i]->dropTime_ - requests_[i]->pickTime_ - requests_[i]->serviceTime_;
            float travelTime = instGraph_->dropNodes_[i]->reachTime_ - instGraph_->pickNodes_[i]->nodeDepartTime_;
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
            metrics.totalRequestsWaiting += requests_[i]->pickTime_ - requests_[i]->initialEarlyPick_;
            metrics.totalWeightedWaiting += requests_[i]->nbPassengers_ * (requests_[i]->pickTime_ - requests_[i]->initialEarlyPick_);
            metrics.totalTripDelay += travelTime - requests_[i]->minTravelTime_;
            metrics.totalWeightedTripDelay += requests_[i]->nbPassengers_ * (travelTime - requests_[i]->minTravelTime_);
            metrics.totalNumServed ++;
            metrics.totalCustomers += requests_[i]->nbPassengers_;
        }
        else {
            repStr << std::right << std::setw(9) << "-------" << " (s)  ";
            repStr << std::right << std::setw(9) << "-------" << " (s)  ";
            repStr << std::right << std::setw(9) << "-------" << " (s)  ";
            repStr << std::right << std::setw(9) << "-------" << " (s)  ";
            metrics.numRejected++;
            metrics.penalty += requests_[i]->Req_W3_ * requests_[i]->penalty_;
        }

        repStr << std::setw(7) << requests_[i]->nbPassengers_ << std::endl;
    }
}

void Instance::processVehiclesAndCollectMetrics(SolutionMetrics &metrics) const {
    for (const auto &vehicleObj : vehicles_) {
        metrics.totalRouteWaiting += vehicleObj->solutionRoute_->totalWait_;
        metrics.totalObj += vehicleObj->solutionRoute_->objCoef_;
        metrics.numServed += static_cast<int>(vehicleObj->solutionRoute_->routeRequests_.size());
        metrics.idleTime += vehicleObj->idleTime_;
        metrics.serviceTime += vehicleObj->serviceTime_;
        metrics.driveFullTime += vehicleObj->driveFullTime_;
        metrics.driveEmptyTime += vehicleObj->driveEmptyTime_;
        metrics.returnEmptyTime += vehicleObj->returnEmptyTime_;

        if (vehicleObj->solutionRoute_->routeSize_ == 1)
            metrics.nbIdle++;
        metrics.totalStopLoad += std::accumulate(vehicleObj->solutionRoute_->plannedPassengers_.begin(),
                                         vehicleObj->solutionRoute_->plannedPassengers_.end(),
                                         decltype(vehicleObj->solutionRoute_->plannedPassengers_)::value_type(0));
        metrics.totalStops += vehicleObj->solutionRoute_->routeSize_;
    }
}

// Function to get index based on node type and identifier
int getIndex(const PNode& node, int id, int nbPairs) {
    if (node->type_ == SOURCE) {
        return 0;
    } else if (node->type_ == SINK) {
        return 1;
    } else if (node->type_ == PICKUP) {
        return 2 + id;
    } else if (node->type_ == DROPOFF && node->nodeStatus_ != PLANNED) {
        return 2 + nbPairs + id;
    } else if (node->type_ == DROPOFF && node->nodeStatus_ == PLANNED) {
        return 2 + 2 * nbPairs + id;
    } else {
        throw std::invalid_argument("Unknown node type");
    }
}

// Function to get node information based on index
std::string getNode(const PInstance& pInst, int vehicleID, int index, int nbPairs) {
    if (index == 0) {
        return pInst->vehicles_[vehicleID]->departNode_->nodeID_;
    } else if (index == 1) {
        return pInst->vehicles_[vehicleID]->sinkNode_->nodeID_;
    } else if (index >= 2 && index < 2 + nbPairs) {
        return pInst->instGraph_->pickNodes_[index - 2]->nodeID_;
    } else if (index >= 2 + nbPairs && index < 2 + 2 * nbPairs) {
        return pInst->instGraph_->dropNodes_[index - 2 - nbPairs]->nodeID_;
    } else if (index >= 2 + 2 * nbPairs) {
        return pInst->instGraph_->nodes_[pInst->vehicles_[vehicleID]->onboards_[index - (2 + 2 * nbPairs)]]->nodeID_;
    } else {
        throw std::invalid_argument("Invalid index");
    }
}