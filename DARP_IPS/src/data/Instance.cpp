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
    repStr << "--------------------- REQUESTS_INFO (Max 3 records) ---------------------" << std::endl;
    int n = std::min(3, nbRequests_);
    for (int i = 0; i < n; ++i) {
        repStr << requests_[i]->toString();
    }
    repStr << "--------------------- VEHICLES_INFO (Max 3 records) ---------------------" << std::endl;
    int m = std::min(3, nbVehicles_);
    for (int i = 0; i < m; ++i) {
        repStr << vehicles_[i]->toString();
    }

    repStr << "# " << std::endl;
    repStr << "------------------------ PARAMETERS AND OPTIONS -------------------------" << std::endl;
    repStr << parameters_->toString();
    repStr << "# " << std::endl;
    return repStr.str();
}

std::string Instance::solutionToString() {
    int numServed = 0;
    float totalWaiting = 0;
    float totalTripDelay = 0;
    int totalNumServed = 0;
    double penalty = 0;
    float idleTime = 0;


    std::stringstream repStr;


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

/*            double travelTime =
                    instGraph_->nodes_[dropID]->reachTime_ - requests_[i]->pickTime_ - requests_[i]->deltaTime_;*/
            float travelTime = requests_[i]->dropTime_ - requests_[i]->pickTime_ - requests_[i]->deltaTime_;

            repStr << std::right << std::setw(9) << travelTime - requests_[i]->minTravelTime_ << " (s)  ";
            if (travelTime > requests_[i]->maxTravelTime_ + 0.1){
                std::cout << "Trip delay constraint is violated by request: " << requests_[i]->getRequestId() << std::endl;
                myTools::throwException("Trip delay Validation");
            }
//
            totalTripDelay += travelTime - requests_[i]->minTravelTime_;
            totalNumServed ++;
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
    repStr << "#" << std::endl;
    repStr << "# ----------------------   AVERAGE VALUES   ----------------------" << std::endl;

    if (numServed != 0) {
        repStr << std::setw(sentenceSize) << "# WAIT TIME PER REQUEST" << " = " << totalWaiting/static_cast<float>(numServed) << " (s)" << std::endl;
        repStr << std::setw(sentenceSize) << "# TRIP DELAY PER REQUEST" << " = " << totalTripDelay/static_cast<float>(totalNumServed) << " (s)" << std::endl;
    }

    repStr << std::setw(sentenceSize) << "# IDLE TIME PER VEHICLE" << " = " << idleTime/static_cast<float>(nbVehicles_) << std::endl;
    repStr << "#" << std::endl;
    return repStr.str();
}

// this function update the set of available requests, removed completed requests and update onboards
void Instance::buildPartialData(const PInstance &mainInst, std::vector<PRequest> &penaltyRequests, float elapsedTime, int lastRecRequests) {
    for (auto & vehicleObj : mainInst->vehicles_){
        instGraph_->addNewNode(vehicleObj->departNode_);
 //       instGraph_->addNewNode(mainInst->instGraph_->nodes_[vehicleObj->sinkID_]);
        instGraph_->addNewNode(mainInst->instGraph_->sinkNodes_[vehicleObj->vehicleID_]);
//        instGraph_->sourceNodes_.push_back(vehicleObj->departNode_);

        // adding onboard nodes to the graph
        /*for (auto & nodeID: vehicleObj->onboards_) {
 //           instGraph_->addNewNode(mainInst->instGraph_->nodes_[nodeID]);
            instGraph_->nodes_.emplace(std::pair<std::string, PNode> (nodeID, mainInst->instGraph_->nodes_[nodeID]));
            mainInst->instGraph_->nodes_[nodeID]->nodeIndex_ = instGraph_->nbNodes_;
            instGraph_->intToNodeID_.push_back(nodeID);
            instGraph_->nbNodes_++;
        }*/
        vehicleObj->score_ = INFINITY;
    }
    nbNewRequests_ = 0;

    // add unperformed requests
    for (auto & vehicleObj : mainInst->vehicles_) {
        if (vehicleObj->currentRoute_->routeSize_ > 1) {
            for (int i = 1; i < vehicleObj->currentRoute_->routeSize_; ++i) {
  //              instGraph_->addNewNode(vehicleObj->currentRoute_->routeNodes_[i]);
                if (vehicleObj->currentRoute_->routeNodes_[i]->type_ == PICKUP){
                    addRequest(vehicleObj->currentRoute_->routeNodes_[i]->related_Request_);
                    instGraph_->addNewNode(vehicleObj->currentRoute_->routeNodes_[i]);
                    instGraph_->addNewNode(*vehicleObj->currentRoute_->routeNodes_[i]->pairNode_);
 //                   instGraph_->pickNodes_.push_back(vehicleObj->currentRoute_->routeNodes_[i]);
 //                   instGraph_->dropNodes_.push_back(*vehicleObj->currentRoute_->routeNodes_[i]->pairNode_);
                }
                else if (vehicleObj->currentRoute_->routeNodes_[i]->nodeStatus_ == PLANNED){
                    instGraph_->nodes_.emplace(std::pair<std::string, PNode> (vehicleObj->currentRoute_->routeNodes_[i]->nodeID_, vehicleObj->currentRoute_->routeNodes_[i]));
                    instGraph_->onboards_.push_back(vehicleObj->currentRoute_->routeNodes_[i]);
                    vehicleObj->currentRoute_->routeNodes_[i]->nodeIndex_ = instGraph_->nbNodes_;
                    instGraph_->intToNodeID_.push_back(vehicleObj->currentRoute_->routeNodes_[i]->nodeID_);
                    instGraph_->nbNodes_++;
                }
            }
        }
    }

    // add unscheduled requests
    for (auto & requestObj: penaltyRequests) {
        addRequest(requestObj);
        instGraph_->addNewNode(mainInst->instGraph_->pickNodes_[requestObj->getRequestId()]);
        instGraph_->addNewNode(mainInst->instGraph_->dropNodes_[requestObj->getRequestId()]);
//        instGraph_->pickNodes_.push_back(mainInst->instGraph_->pickNodes_[requestObj->getRequestId()]);
//        instGraph_->dropNodes_.push_back(mainInst->instGraph_->dropNodes_[requestObj->getRequestId()]);
        /*std::string pickID = myTools::createNodeID(requestObj->getRequestId(), PICKUP);
        std::string dropID = myTools::createNodeID(requestObj->getRequestId(), DROPOFF);
        instGraph_->addNewNode(mainInst->instGraph_->nodes_[pickID]);
        instGraph_->addNewNode(mainInst->instGraph_->nodes_[dropID]);*/
    }

    // add new requests
    for (int i = lastRecRequests; i < mainInst->nbRequests_; ++i) {
        if (mainInst->requests_[i]->earlyPick_ <= simulationStartTime_ + elapsedTime ) {
            nbNewRequests_++;
            addRequest(mainInst->requests_[i]);
            instGraph_->addNewNode(mainInst->instGraph_->pickNodes_[i]);
            instGraph_->addNewNode(mainInst->instGraph_->dropNodes_[i]);
 //           instGraph_->pickNodes_.push_back(mainInst->instGraph_->pickNodes_[i]);
 //           instGraph_->dropNodes_.push_back(mainInst->instGraph_->dropNodes_[i]);
            /*std::string pickID = myTools::createNodeID(mainInst->requests_[i]->getRequestId(), PICKUP);
            std::string dropID = myTools::createNodeID(mainInst->requests_[i]->getRequestId(), DROPOFF);
            instGraph_->addNewNode(mainInst->instGraph_->nodes_[pickID]);
            instGraph_->addNewNode(mainInst->instGraph_->nodes_[dropID]);*/

            // calculate vehicle scores
            /*if (mainInst->parameters_->vehicle_portion_ < 1){
                for (auto & vehicleObj: mainInst->vehicles_){
                    float earliestPick = vehicleObj->departTime_ + durationMatrix_[vehicleObj->departNode_->locationID_]
                    [mainInst->instGraph_->pickNodes_[i]->locationID_] - (simulationStartTime_ + elapsedTime);
                    if (earliestPick < vehicleObj->score_)
                        vehicleObj->score_ = earliestPick;
                }
            }*/
        }
        else
            break;
    }
/*    for (auto & nodeObj: onboards){
        // adding onboard nodes to the graph
        instGraph_->dropNodes_.push_back(nodeObj);
    }*/
    nbOnboards_ = instGraph_->onboards_.size();
    /*for (auto & vehicleObj : mainInst->vehicles_){
        // adding onboard nodes to the graph
        for (auto & nodeID: vehicleObj->onboards_) {
            instGraph_->dropNodes_.push_back(instGraph_->nodes_[nodeID]);
        }
    }*/
    // calculate vehicle scores
    /*float earliestPick;
    int VehicleId;*/
    if (mainInst->parameters_->vehicle_portion_ < 1){
        /*for (int i = 0; i < instGraph_->pickNodes_.size() ; i++){
            earliestPick = INFINITY;
            for (auto & vehicleObj: mainInst->vehicles_){
                if (vehicleObj->score_ > 0){
                    float minReachTime = vehicleObj->departTime_ + durationMatrix_[vehicleObj->departNode_->locationID_]
                    [mainInst->instGraph_->pickNodes_[i]->locationID_] - (simulationStartTime_ + elapsedTime);
                    if ( minReachTime < earliestPick){
                        earliestPick = minReachTime;
                        VehicleId = vehicleObj->vehicleID_;
                    }
                }
            }
            mainInst->vehicles_[VehicleId]->score_ = 0;
            mainInst->vehicles_[VehicleId]->selected_ = false;
        }*/
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

void Instance::buildStaticData(const PInstance &mainInst) {
    for (auto & vehicleObj : mainInst->vehicles_){
        instGraph_->addNewNode(vehicleObj->departNode_);
        instGraph_->addNewNode(mainInst->instGraph_->nodes_[vehicleObj->sinkID_]);
    }
    nbNewRequests_ = 0;
    // add drop off onboards to the graph
//    instGraph_->onboards_ = mainInst->instGraph_->onboards_;
    for (auto & nodeObj : mainInst->instGraph_->onboards_){
        instGraph_->nodes_.emplace(std::pair<std::string, PNode> (nodeObj->nodeID_, nodeObj));
        instGraph_->onboards_.push_back(nodeObj);
        nodeObj->nodeIndex_ = instGraph_->nbNodes_;
        instGraph_->intToNodeID_.push_back(nodeObj->nodeID_);
        instGraph_->nbNodes_++;
    }
    /*for (auto & vehicleObj : mainInst->vehicles_) {
        if (vehicleObj->currentRoute_->routeSize_ > 1) {
            for (int i = 1; i < vehicleObj->currentRoute_->routeSize_; ++i)
                instGraph_->addNewNode(vehicleObj->currentRoute_->routeNodes_[i]);
        }
    }*/

    for (auto & requestObj : mainInst->requests_) {
        if (requestObj->requestStatus_ == NO_ACTION) {
            nbNewRequests_++;
            addRequest(requestObj);
            std::string pickID = myTools::createNodeID(requestObj->getRequestId(), PICKUP);
            std::string dropID = myTools::createNodeID(requestObj->getRequestId(), DROPOFF);
            instGraph_->addNewNode(mainInst->instGraph_->nodes_[pickID]);
            instGraph_->addNewNode(mainInst->instGraph_->nodes_[dropID]);
        }
    }
}

void Instance::buildDataZone(const PInstance &mainInst, int zoneID) {
    for (auto & vehicleObj : vehicles_){
        instGraph_->addNewNode(vehicleObj->departNode_);
        instGraph_->addNewNode(mainInst->instGraph_->nodes_[vehicleObj->sinkID_]);
    }
    nbNewRequests_ = 0;

    for (auto & requestObj : mainInst->requests_) {
        if (requestObj->zoneID_ == zoneID) {
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
    if ((nbOnboards_ == 0) &&(parameters_->mainAlgorithm_ != GREEDY)) {
        for (auto &vehicleObj: vehicles_) {
            vehicleObj->setDepartTime(vehicleObj->startTime_ + static_cast<float>(parameters_->epochLength_));
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
void Instance::saveSolutionRoutes(const std::string& routeResultDir) {
    std::ofstream myFile;
    myFile.open (routeResultDir);
    myFile << "VehicleID,NodeID,RequestTime,ReachTime,NodeType, LocationID, DepartTime, nbPassengers, vehicleLoad" << std::endl;
    for (auto & vehicleObj : vehicles_) {
        int i = 0;
        for (auto & nodeObj : vehicleObj->solutionRoute_->routeNodes_) {
            myFile << vehicleObj->vehicleID_ << ",";
            myFile << nodeObj->nodeID_ << ",";
            myFile << nodeObj->requestTime_ << ",";
            myFile << nodeObj->reachTime_ << ",";
            myFile << nodeObj->initialType_ << ",";
            myFile << nodeObj->locationID_ << ",";
            myFile << nodeObj->departTime_ << ",";
            myFile << nodeObj->nbPassengers_ << ",";
            myFile << vehicleObj->solutionRoute_->plannedPassengers_[i] << "\n";
            i++;
        }
    }
    myFile.close();
}

std::string Instance::saveSolutionRoutes() {
    std::stringstream repStr;
    repStr << "VehicleID,NodeID,RequestTime,ReachTime,NodeType, LocationID, DepartTime, nbPassengers, vehicleLoad" << std::endl;
    for (auto & vehicleObj : vehicles_) {
        int i = 0;
        for (auto & nodeObj : vehicleObj->solutionRoute_->routeNodes_) {
            repStr << vehicleObj->vehicleID_ << ",";
            repStr << nodeObj->nodeID_ << ",";
            repStr << nodeObj->requestTime_ << ",";
            repStr << nodeObj->reachTime_ << ",";
            repStr << nodeObj->initialType_ << ",";
            repStr << nodeObj->locationID_ << ",";
            repStr << nodeObj->departTime_ << ",";
            repStr << nodeObj->nbPassengers_ << ",";
            repStr << vehicleObj->solutionRoute_->plannedPassengers_[i] << "\n";
            i++;
        }
    }
    return repStr.str();
}

void Instance::saveRequestsResults(const std::string& requestResultDir) {
    std::ofstream myFile;
    myFile.open (requestResultDir);
    myFile << "RequestID,nbPassengers, PickupID,DropOffID,RequestTime,PickTime,"
              "DropTime, VehicleID, WaitTime, TripDelay, MaxTravelTime, MinTravelTime" << std::endl;

    for (auto & requestObj : requests_) {
        myFile << requestObj->getRequestId() << ",";
        myFile << requestObj->nbPassengers_ << ",";
        myFile << requestObj->PickUpID_ << ",";
        myFile << requestObj->DropOffID_ << ",";
        myFile << requestObj->earlyPick_ << ",";
        myFile << requestObj->pickTime_ << ",";
        myFile << requestObj->dropTime_ << ",";
        myFile << requestObj->vehicleID_ << ",";
        myFile << requestObj->pickTime_ - requestObj->earlyPick_ << ",";
        myFile << requestObj->dropTime_ - requestObj->pickTime_ - requestObj->minTravelTime_ << ",";
        myFile << requestObj->maxTravelTime_ << ",";
        myFile << requestObj->minTravelTime_ << "\n";
/*        if (requests_[i]->requestStatus_ != NO_ACTION) {
            myFile << requests_[i]->pickTime_ << ",";
            myFile << requests_[i]->dropTime_ << "\n";
        }
        else {
            myFile << "--" << ",";
            myFile << "--" << "\n";
        }*/
    }
    myFile.close();
}


std::string Instance::saveRequestsResults() {
    std::stringstream repStr;
    repStr << "RequestID,nbPassengers, PickupID,DropOffID,RequestTime,PickTime,"
              "DropTime, VehicleID, WaitTime, TripDelay, MaxTravelTime, MinTravelTime" << std::endl;

    for (auto & requestObj : requests_) {
        repStr << requestObj->getRequestId() << ",";
        repStr << requestObj->nbPassengers_ << ",";
        repStr << requestObj->PickUpID_ << ",";
        repStr << requestObj->DropOffID_ << ",";
        repStr << requestObj->earlyPick_ << ",";
        repStr << requestObj->pickTime_ << ",";
        repStr << requestObj->dropTime_ << ",";
        repStr << requestObj->vehicleID_ << ",";
        repStr << requestObj->pickTime_ - requestObj->earlyPick_ << ",";
        repStr << requestObj->dropTime_ - requestObj->pickTime_ - requestObj->minTravelTime_ << ",";
        repStr << requestObj->maxTravelTime_ << ",";
        repStr << requestObj->minTravelTime_ << "\n";
    }
    return repStr.str();
}

void Instance::saveEpochRoutes(const std::string& finalSolutionDir , int epoch) {
    std::ofstream myFile;
    myFile.open (finalSolutionDir, std::ofstream::app);
    for (auto & vehicleObj : vehicles_) {
        for (auto & nodeObj : vehicleObj->solutionRoute_->routeNodes_) {
            myFile << epoch << ",";
            myFile << vehicleObj->vehicleID_ << ",";
            myFile << nodeObj->nodeID_ << ",";
            myFile << nodeObj->requestTime_ << ",";
            myFile << nodeObj->reachTime_ << ",";
            myFile << nodeObj->type_ << ",";
            myFile << nodeObj->locationID_ << "\n";
        }
    }
    myFile.close();
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

void Instance::saveISUDRoutes(const std::string& isudSolutionDir, int epoch, int isudIter) {
    std::ofstream myFile;
    myFile.open (isudSolutionDir, std::ofstream::app);
    for (auto & vehicleObj : vehicles_) {
        for (auto & nodeObj : vehicleObj->currentRoute_->routeNodes_) {
            myFile << epoch << ",";
            myFile << isudIter << ",";
            myFile << vehicleObj->vehicleID_ << ",";
            myFile << nodeObj->nodeID_ << ",";
            myFile << nodeObj->requestTime_ << ",";
            myFile << nodeObj->reachTime_ << ",";
            myFile << nodeObj->type_ << ",";
            myFile << nodeObj->locationID_ << ",";
            myFile << vehicleObj->currentRoute_->getRouteId() << "\n";
        }
    }
    myFile.close();
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
            repStr << vehicleObj->currentRoute_->getRouteId() << "\n";
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
        repStr << vehicleObj->currentRoute_->createTime_ << "\n";
    }
    return repStr.str();
}

void Instance::saveStatus(InputPaths &inputPaths, float simulationStart) {
    std::ofstream myFile;
    int setwSize = 15;
    int nbOnboards = 0;
    int nbWaiting = 0;
    int nbRequests = 0;

    // print vehicles
    myFile.open (inputPaths.getOutputVehicles(), std::ofstream::app);
    myFile << "COLUMNS\n\n" << "vehicle_ID\n" << "capacity\n" << "depart_Time\n" << "end_Time\n" << "depart_ID\n";
    myFile << "sink_ID\n\n" << "VEHICLES_INFO" << std::endl;

    for (auto & vehicleObj : vehicles_) {
        nbOnboards += int(vehicleObj->onboards_.size());
        myFile << std::left << std::setw(7) << vehicleObj->vehicleID_;
        myFile << std::setw(10) << vehicleObj->capacity_ ;
        myFile << std::setw(10) << vehicleObj->departTime_ ;
        myFile << std::setw(10) << vehicleObj->endTime_ ;
        myFile << std::setw(setwSize) << vehicleObj->departNode_->locationID_ ;
        myFile << std::setw(setwSize) << instGraph_->nodes_[vehicleObj->sinkID_]->locationID_ << "\n";
    }
    myFile.close();

    // print onboard requests
    myFile.open (inputPaths.getOutputOnboards(), std::ofstream::app);
    myFile << "COLUMNS\n\n" << "passenger_count\n" << "pickup_ID\n" << "dropoff_ID\n" << "request_time_sec\n";
    myFile << "pickup_time_sec\n" << "vehicle_ID\n\n" << "REQUESTS_INFO" << std::endl;
    for (auto & vehicleObj : vehicles_) {
        for (auto & nodeID: vehicleObj->onboards_) {
            PRequest requestObj = instGraph_->nodes_[nodeID]->related_Request_;
            myFile << std::left << std::setw(7) << requestObj->nbPassengers_ ;
            myFile << std::setw(10) << requestObj->PickUpID_ ;
            myFile << std::setw(10) << requestObj->DropOffID_ ;
            myFile << std::setw(10) << requestObj->earlyPick_ ;
            myFile << std::setw(10) << requestObj->pickTime_;
            myFile << std::setw(setwSize) << vehicleObj->vehicleID_ << "\n";
        }
    }
    myFile.close();

    // print waiting requests (requests that received in previous epochs and not served yet)
    myFile.open (inputPaths.getOutputWaitRequests(), std::ofstream::app);
    myFile << "COLUMNS\n\n" << "passenger_count\n" << "pickup_ID\n" << "dropoff_ID\n" << "request_time_sec\n\n";
    myFile << "REQUESTS_INFO" << std::endl;

    for (auto & requestObj: requests_) {
        if ((requestObj->requestStatus_ == NO_ACTION)&&(requestObj->earlyPick_ <= simulationStart)) {
            nbWaiting ++;
            myFile << std::left << std::setw(7) << requestObj->nbPassengers_;
            myFile << std::setw(10) << requestObj->PickUpID_;
            myFile << std::setw(10) << requestObj->DropOffID_;
            myFile << std::setw(10) << requestObj->earlyPick_ << "\n";
        }
        /*else if (requestObj->earlyPick_ > simulationStart)
            nbRequests++;*/
    }
    myFile.close();

    // print trip requests (requests that are not received yet)
    myFile.open (inputPaths.getOutputTrip(), std::ofstream::app);
    myFile << "COLUMNS\n\n" << "passenger_count\n" << "pickup_ID\n" << "dropoff_ID\n" << "request_time_sec\n\n";
    myFile << "REQUESTS_INFO" << std::endl;

    for (auto & requestObj: requests_) {
        if (requestObj->earlyPick_ > simulationStart) {
            myFile << std::left << std::setw(7) << requestObj->nbPassengers_;
            myFile << std::setw(10) << requestObj->PickUpID_;
            myFile << std::setw(10) << requestObj->DropOffID_;
            myFile << std::setw(10) << requestObj->earlyPick_ << "\n";
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
    std::stable_sort(requests_.begin(),requests_.end(),[](const PRequest &lhs, const PRequest &rhs){
        return lhs->dual_ > rhs->dual_;});

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































