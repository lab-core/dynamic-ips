//
// Created by Ella on 2021-09-12.
//

#include "Instance.h"

//-----------------------------------------------------------------------------
//  Instance class
//  contains the instance data including vehicle info and requests
//-----------------------------------------------------------------------------

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
}
Instance::~Instance() {
    instGraph_.reset();
}

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
                Tools::throwException("Trip delay Validation");
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
    repStr << std::setw(sentenceSize) << "# Total waiting time before pickup" << " = " << totalWaiting << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# Total trip delay" << " = " << totalTripDelay << " (s)" << std::endl;
    if (numServed != 0) {
        repStr << std::setw(sentenceSize) << "# Average wait time per request" << " = " << totalWaiting/numServed << " (s)" << std::endl;
        repStr << std::setw(sentenceSize) << "# Average trip delay per request" << " = " << totalTripDelay/totalNumServed << " (s)" << std::endl;
    }
    repStr << std::setw(sentenceSize) << "# Number of unserved requests" << " = " << nbRequests_ - totalNumServed << std::endl;
    repStr << std::setw(sentenceSize) << "# Total number of requests" << " = " << nbRequests_ << std::endl;
    repStr << std::setw(sentenceSize) << "# Average Vehicle idle Time" << " = " << idleTime/nbVehicles_ << std::endl;
    repStr << std::setw(sentenceSize) << "# Final Objective Value" << " = " << penalty + totalWaiting << std::endl;
    std::cout << "#" << std::endl;
    return repStr.str();
}

// this function update the set of available requests, removed completed requests and update onboards
void Instance::buildPartialData(const PInstance &mainInst, std::vector<PRequest> &penaltyRequests, float elapsedTime, int lastRecRequests) {

    for (auto & vehicleObj : mainInst->vehicles_){
        instGraph_->addNewNode(mainInst->instGraph_->nodes_[vehicleObj->departID_]);
        instGraph_->addNewNode(mainInst->instGraph_->nodes_[vehicleObj->sinkID_]);

        // adding onboard nodes to the graph
        for (auto & nodeID: vehicleObj->onboards_) {
            instGraph_->addNewNode(mainInst->instGraph_->nodes_[nodeID]);
        }
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
                }
            }
        }
    }

    // add unscheduled requests
    for (auto & requestObj: penaltyRequests) {
        addRequest(requestObj);

        std::string pickID = Tools::createNodeID(requestObj->getRequestId(), PICKUP);
        std::string dropID = Tools::createNodeID(requestObj->getRequestId(), DROPOFF);
        instGraph_->addNewNode(mainInst->instGraph_->nodes_[pickID]);
        instGraph_->addNewNode(mainInst->instGraph_->nodes_[dropID]);
    }

    // add new requests
    for (int i = lastRecRequests; i < mainInst->nbRequests_; ++i) {
        if (mainInst->requests_[i]->earlyPick_ <= simulationStartTime_ + elapsedTime ) {
            nbNewRequests_++;
            addRequest(mainInst->requests_[i]);
            std::string pickID = Tools::createNodeID(mainInst->requests_[i]->getRequestId(), PICKUP);
            std::string dropID = Tools::createNodeID(mainInst->requests_[i]->getRequestId(), DROPOFF);
            instGraph_->addNewNode(mainInst->instGraph_->nodes_[pickID]);
            instGraph_->addNewNode(mainInst->instGraph_->nodes_[dropID]);
        }
        else
            break;
    }
    updateRequestOrder();
}

void Instance::buildStaticData(const PInstance &mainInst) {
    for (auto & vehicleObj : mainInst->vehicles_){
        instGraph_->addNewNode(mainInst->instGraph_->nodes_[vehicleObj->departID_]);
        instGraph_->addNewNode(mainInst->instGraph_->nodes_[vehicleObj->sinkID_]);
    }
    nbNewRequests_ = 0;
    // add drop off onboards to the graph
    for (auto & vehicleObj : mainInst->vehicles_) {
        if (vehicleObj->currentRoute_->routeSize_ > 1) {
            for (int i = 1; i < vehicleObj->currentRoute_->routeSize_; ++i)
                instGraph_->addNewNode(vehicleObj->currentRoute_->routeNodes_[i]);
        }
    }

    for (auto & requestObj : mainInst->requests_) {
        if (requestObj->requestStatus_ == NO_ACTION) {
            nbNewRequests_++;
            addRequest(requestObj);
            std::string pickID = Tools::createNodeID(requestObj->getRequestId(), PICKUP);
            std::string dropID = Tools::createNodeID(requestObj->getRequestId(), DROPOFF);
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
    sort(vehicles_.begin(), vehicles_.end(),[](const PVehicle &lhs, const PVehicle &rhs){
        return lhs->vehicleID_ < rhs->vehicleID_;});
}

void Instance::sortVehicles(SortVehicle sortBase) {
    switch(sortBase) {
        case DUAL :
            sort(vehicles_.begin(), vehicles_.end(),[](const PVehicle &lhs, const PVehicle &rhs){
                return lhs->dual_ > rhs->dual_;});
            break;

        case DEPART_TIME :
            sort(vehicles_.begin(), vehicles_.end(),[](const PVehicle &lhs, const PVehicle &rhs){
                return lhs->departTime_ < rhs->departTime_;});
            break;

        case ROURE_SIZE :
            sort(vehicles_.begin(), vehicles_.end(),[](const PVehicle &lhs, const PVehicle &rhs){
                return lhs->currentRoute_->routeSize_ < rhs->currentRoute_->routeSize_;});
            break;
        case BEST_REDUCE_COST :
            sort(vehicles_.begin(), vehicles_.end(),[](const PVehicle &lhs, const PVehicle &rhs){
                return lhs->bestReducedCost_ < rhs->bestReducedCost_;});
            break;
    }
}


// function to update penalties in rolling horizon approach
void Instance::updatePenaltiesEpoch(int epoch) {
    for (auto & requestObj : requests_)
        requestObj->setPenaltyEpoch(epoch, parameters_, simulationStartTime_);
}

// function to update penalties in any time approach
void Instance::updatePenalties(float elapsedTime, float length) {
    for (auto & requestObj : requests_)
        requestObj->setPenalty(elapsedTime, parameters_, simulationStartTime_, length);
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
        myFile << std::setw(setwSize) << instGraph_->nodes_[vehicleObj->departID_]->locationID_ ;
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
    sort(requests_.begin(),requests_.end(),[](const PRequest &lhs, const PRequest &rhs){
        return lhs->dual_ > rhs->dual_;});
    for (auto & requestObj : requests_){
        requestObj->taskIndexLabel_ = orderCounter;
        orderCounter++;
    }
}





















