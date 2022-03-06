//
// Created by Ella on 2021-09-12.
//

#include "Instance.h"

//-----------------------------------------------------------------------------
//  Instance class
//  contains the instance data including vehicle info and requests
//-----------------------------------------------------------------------------

// Constructor and Destructor
Instance::Instance(std::string &name, int nbVehicles, std::vector<PVehicle> &vehicles, int nbRequests,
                   PGraph &mainGraph) : name_(name), nbVehicles_(nbVehicles), vehicles_(vehicles),
                                        nbRequests_(nbRequests), instGraph_(mainGraph) {
    nbNewRequests_ = nbRequests;
    std::cout << "Instance created!"<< std::endl;
}


Instance::Instance(const Instance &mainInst) : name_(mainInst.name_){
    nbVehicles_ = mainInst.nbVehicles_;
    vehicles_ = mainInst.vehicles_;
    parameters_ = mainInst.parameters_;
    nbRequests_ = 0;
    nbNewRequests_ = 0;
    instGraph_ = std::make_shared<Graph>();
}
Instance::~Instance() {}

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
    double totalWaiting = 0;
    double totalTripDelay = 0;
    double totalMinWaiting = 0;
    double penalty = 0;

    std::stringstream repStr;


    // print table header
    repStr << "# ---------------------------------------------------------------------------------------------------" << std::endl;
    repStr << "#   REQUEST_ID" << "   ";
    repStr << "REQUEST_TIME (s)" << "   ";
    repStr << "PICK_TIME (s)" << "   ";
    repStr << "WAIT_TIME (s)" << "   ";
    repStr << "TRIP_DELAY (s)" << "   ";
    repStr << "#PASSENGERS" <<std::endl;
    repStr << "# ---------------------------------------------------------------------------------------------------" << std::endl;

    // print the internal nodes of the route
    for (int i = 0; i < nbRequests_; ++i) {

        repStr << std::fixed;
        repStr << std::setprecision(2);
        repStr << "#" << std::right << std::setw(9) << requests_[i]->getRequestId() << "       ";
        repStr << std::right << std::setw(12) << requests_[i]->earlyPick_ << " (s)  ";
        if (requests_[i]->requestStatus_ != NO_ACTION) {
            totalMinWaiting += requests_[i]->minReachTime_;
            repStr << std::right << std::setw(10) << requests_[i]->pickTime_ << " (s)  ";
            repStr << std::right << std::setw(10) << requests_[i]->pickTime_ - requests_[i]->earlyPick_ << " (s)  ";

/*            double travelTime =
                    instGraph_->nodes_[dropID]->reachTime_ - requests_[i]->pickTime_ - requests_[i]->deltaTime_;*/
            double travelTime = requests_[i]->dropTime_ - requests_[i]->pickTime_ - requests_[i]->deltaTime_;

            repStr << std::right << std::setw(11) << travelTime - requests_[i]->minTravelTime_ << " (s)  ";
            totalTripDelay += travelTime - requests_[i]->minTravelTime_;
        }
        else {
            repStr << std::right << std::setw(10) << "-------" << " (s)  ";
            repStr << std::right << std::setw(10) << "-------" << " (s)  ";
            repStr << std::right << std::setw(11) << "-------" << " (s)  ";
            penalty += requests_[i]->penalty_;
        }

        repStr << std::setw(7) << requests_[i]->nbPassengers_ << std::endl;
    }

    repStr << "# ---------------------------------------------------------------------------------------------------" << std::endl;

    for (const auto &vehicleObj : vehicles_) {
        totalWaiting += vehicleObj->solutionRoute_->totalDelay_;
        numServed += vehicleObj->solutionRoute_->routeRequests.size();
    }
    repStr << std::left << std::fixed << std::setprecision(2);
    repStr << std::setw(sentenceSize) << "# Total waiting time before pickup" << " = " << totalWaiting << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# Total trip delay" << " = " << totalTripDelay << " (s)" << std::endl;
    if (numServed != 0) {
        repStr << std::setw(sentenceSize) << "# Average wait time per request" << " = " << totalWaiting/numServed << " (s)" << std::endl;
        repStr << std::setw(sentenceSize) << "# Average trip delay per request" << " = " << totalTripDelay/numServed << " (s)" << std::endl;
    }
    repStr << std::setw(sentenceSize) << "# Number of served requests" << " = " << numServed << std::endl;
    repStr << std::setw(sentenceSize) << "# Total number of requests" << " = " << nbRequests_ << std::endl;
    repStr << std::setw(sentenceSize) << "# Final Objective Value" << " = " << penalty + totalWaiting << std::endl;
    std::cout << "#" << std::endl;
    return repStr.str();
}

// function to set the data of the partial instance based on the epoch
void Instance::buildPartialData(const PInstance &mainInst, std::vector<PRequest> penaltyRequests, int epoch, int lastRecRequests) {
    instGraph_->addNewNode(mainInst->instGraph_->nodes_[Tools::createNodeID(0, SOURCE)]);
    instGraph_->addNewNode(mainInst->instGraph_->nodes_[Tools::createNodeID(0, SINK)]);
    nbNewRequests_ = 0;

    if (epoch > 0) {
        for (auto & vehicleObj : mainInst->vehicles_) {
            instGraph_->addNewNode(mainInst->instGraph_->nodes_[vehicleObj->departID_]);
            if (vehicleObj->currentRoute_->routeSize_ > 1) {
                for (int i = 1; i < vehicleObj->currentRoute_->routeSize_; ++i) {
                    instGraph_->addNewNode(vehicleObj->currentRoute_->routeNodes_[i]);
                    if (vehicleObj->currentRoute_->routeNodes_[i]->type_ == PICKUP)
                        addRequest(*vehicleObj->currentRoute_->routeNodes_[i]->related_Request_, epoch, mainInst->parameters_);
                }
            }
        }

        for (auto & requestObj: penaltyRequests) {
            addRequest(requestObj, epoch, mainInst->parameters_);

            std::string pickID = Tools::createNodeID(requestObj->getRequestId(), PICKUP);
            std::string dropID = Tools::createNodeID(requestObj->getRequestId(), DROPOFF);
            instGraph_->addNewNode(mainInst->instGraph_->nodes_[pickID]);
            instGraph_->addNewNode(mainInst->instGraph_->nodes_[dropID]);
        }
    }

    for (int i = lastRecRequests; i < mainInst->nbRequests_; ++i) {
        if (mainInst->requests_[i]->earlyPick_ <= (epoch+1) * mainInst->parameters_->epochLength_ ) {
            mainInst->requests_[i]->readEpoch_ = epoch;
            nbNewRequests_++;
            addRequest(mainInst->requests_[i], epoch, mainInst->parameters_);
            std::string pickID = Tools::createNodeID(mainInst->requests_[i]->getRequestId(), PICKUP);
            std::string dropID = Tools::createNodeID(mainInst->requests_[i]->getRequestId(), DROPOFF);
            instGraph_->addNewNode(mainInst->instGraph_->nodes_[pickID]);
            instGraph_->addNewNode(mainInst->instGraph_->nodes_[dropID]);
        }
        else
            break;
    }
}

// function to add requests from previous epochs to the current partial instance
void Instance::addRequest(PRequest request, int epoch, PParameters &parameters) {
    nbRequests_++;
    requests_.push_back(request);
    request->setPenalty(epoch - request->readEpoch_, parameters);
    nameToRequest_[request->name_] = request;
    request->selectStatus_ = NOTSELECTED;
}


void Instance::setInitialTimes() {

    for (auto & requestObj: requests_) {
        requestObj->setMinTravelTime(durationMatrix_[requestObj->PickUpID_][requestObj->DropOffID_]);
        requestObj->setMinReachTime(durationMatrix_[2*nbRequests_][requestObj->PickUpID_]);
        requestObj->setMaxTravelTime(parameters_->alphaParam_, parameters_->betaParam_);
    }

    for (auto & vehicleObj: vehicles_) {
//        vehicleObj->setDepartTime(2 * parameters_->epochLength_);
        vehicleObj->setDepartTime(100);
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

void Instance::resetRequestsSelectStatus() {
    for (auto & requestObj: requests_)
        requestObj->selectStatus_ = NOTSELECTED;
}

// print solutions in csv files
void Instance::saveSolutionRoutes(string routeResultDir) {
    std::ofstream myFile;
    myFile.open (routeResultDir);
    myFile << "VehicleID,NodeID,RequestTime,ReachTime,NodeType,Latitude,Longitude, LocationID" << std::endl;
    for (auto & vehicleObj : vehicles_) {
        for (auto & nodeObj : vehicleObj->solutionRoute_->routeNodes_) {
            myFile << vehicleObj->vehicleID_ << ",";
            myFile << nodeObj->nodeID_ << ",";
            myFile << nodeObj->requestTime_ << ",";
            myFile << nodeObj->reachTime_ << ",";
            myFile << nodeObj->type_ << ",";
            myFile << nodeObj->locLatitude_ << ",";
            myFile << nodeObj->locLongitude_ << ",";
            myFile << nodeObj->locationID_ << "\n";

        }
    }
    myFile.close();
}

void Instance::saveRequestsResults(string requestResultDir) {
    std::ofstream myFile;
    myFile.open (requestResultDir);
    myFile << "RequestID,nbPassengers,PickLatitude,PickLongitude,DropLatitude,DropLongitude, PickupID,DropOffID,RequestTime,PickReachTime,"
              "DropReachTime, VehicleID" << std::endl;

    for (auto & requestObj : requests_) {
        myFile << requestObj->getRequestId() << ",";
        myFile << requestObj->nbPassengers_ << ",";
        myFile << requestObj->PickUpLatitude_ << ",";
        myFile << requestObj->PickUpLongitude_ << ",";
        myFile << requestObj->DropOffLatitude_ << ",";
        myFile << requestObj->DropOffLongitude_ << ",";
        myFile << requestObj->PickUpID_ << ",";
        myFile << requestObj->DropOffID_ << ",";
        myFile << requestObj->earlyPick_ << ",";
        myFile << requestObj->pickTime_ << ",";
        myFile << requestObj->dropTime_ << ",";
        myFile << requestObj->vehicleID_ << "\n";
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


void Instance::saveEpochRoutes(string finalSolutionDir , int epoch) {
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
            myFile << nodeObj->locLatitude_ << ",";
            myFile << nodeObj->locLongitude_ << ",";
            myFile << nodeObj->locationID_ << "\n";
        }
    }
    myFile.close();
}

void Instance::saveISUDRoutes(string isudSolutionDir, int epoch, int isudIter) {
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
            myFile << nodeObj->locLatitude_ << ",";
            myFile << nodeObj->locLongitude_ << ",";
            myFile << nodeObj->locationID_ << "\n";
        }
    }
    myFile.close();
}










