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


/*Instance::Instance(std::string &name, const PInstance &mainInst, int epoch, int lastRecRequests) : name_(name){
    nbVehicles_ = mainInst->nbVehicles_;
    vehicles_ = mainInst->vehicles_;
    nbRequests_ = 0;
    instGraph_ = std::make_shared<Graph>();
    instGraph_->addNewNode(mainInst->instGraph_->nodes_[Tools::createNodeID(0, SINK)]);
    for (int v = 0; v < mainInst->nbVehicles_; ++v) {
        if ((*mainInst->vehicles_[v]->currentRoute_)->routeSize_ > 2) {
            for (int i = (*mainInst->vehicles_[v]->currentRoute_)->routeSize_ - 2; i >= 1; --i) {
                if ((*mainInst->vehicles_[v]->currentRoute_)->routeNodes_[i]->nodeStatus_ != DONE)
                    instGraph_->addNewNode((*mainInst->vehicles_[v]->currentRoute_)->routeNodes_[i]);
                if ((*mainInst->vehicles_[v]->currentRoute_)->routeNodes_[i]->type_ != SOURCE) {
                    instGraph_->addNewNode((*mainInst->vehicles_[v]->currentRoute_)->routeNodes_[i]);
                    break;
                }
            }

            for (int i = (*mainInst->vehicles_[v]->currentRoute_)->routeSize_ - 2; i >= 1; --i) {
                if ((*(*mainInst->vehicles_[v]->currentRoute_)->routeNodes_[i]->related_Request_)->requestStatus_ != COMPLETED) {
                    nbRequests_++;
                    requests_.push_back(*(*mainInst->vehicles_[v]->currentRoute_)->routeNodes_[i]->related_Request_);
                }
            }
        }
    }

    for (int i = lastRecRequests; i < mainInst->nbRequests_; ++i) {
        if (mainInst->requests_[i]->earlyPick_ <= (epoch+1)*epochLength ) {
            nbRequests_++;
            requests_.push_back(mainInst->requests_[i]);
        }
        else
            break;
    }
}*/

Instance::Instance(const Instance &mainInst) : name_(mainInst.name_){
    nbVehicles_ = mainInst.nbVehicles_;
    vehicles_ = mainInst.vehicles_;
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
    return repStr.str();
}

std::string Instance::solutionToString() {
    int numServed = 0;
    double totalWaiting = 0;
    double totalTripDelay = 0;

    std::stringstream repStr;


    // print table header
    repStr << "# ---------------------------------------------------------------------------------------------------" << std::endl;
    repStr << "#   REQUEST_ID" << "   ";
    repStr << "REQUEST_TIME (s)" << "   ";
    repStr << "PICK_TIME (s)" << "   ";
    repStr << "WAIT_TIME (s)" << "   ";
    repStr << "TRIP_DELAY (s)" << "   ";
    repStr << "#PASSENGERS" <<std::endl;
    //   repStr << "# ________________________________________________________________________" << std::endl;
    repStr << "# ---------------------------------------------------------------------------------------------------" << std::endl;

    // print the internal nodes of the route
    for (int i = 0; i < nbRequests_; ++i) {

        repStr << std::fixed;
        repStr << std::setprecision(2);
 //       repStr << std::ios_base::badbit;
        repStr << "#" << std::right << std::setw(9) << requests_[i]->getRequestId() << "       ";
        repStr << std::right << std::setw(12) << requests_[i]->earlyPick_ << " (s)  ";
        if (requests_[i]->requestStatus_ != NO_ACTION) {
            repStr << std::right << std::setw(10) << requests_[i]->pickTime_ << " (s)  ";
            repStr << std::right << std::setw(10) << requests_[i]->pickTime_ - requests_[i]->earlyPick_ << " (s)  ";
            std::string dropID = Tools::createNodeID(requests_[i]->getRequestId(), DROPOFF);

            double travelTime =
                    instGraph_->nodes_[dropID]->reachTime_ - requests_[i]->pickTime_ - requests_[i]->deltaTime_;

            repStr << std::right << std::setw(11) << travelTime - requests_[i]->minTravelTime_ << " (s)  ";
            totalTripDelay += travelTime - requests_[i]->minTravelTime_;
        }
        else {
            repStr << std::right << std::setw(10) << "-------" << " (s)  ";
            repStr << std::right << std::setw(10) << "-------" << " (s)  ";
            repStr << std::right << std::setw(11) << "-------" << " (s)  ";
        }

        repStr << std::setw(7) << requests_[i]->nbPassengers_ << std::endl;
    }

    repStr << "# ---------------------------------------------------------------------------------------------------" << std::endl;

    for (int v = 0; v < nbVehicles_; ++v) {
        totalWaiting += vehicles_[v]->solutionRoute_->totalDelay_;
        numServed += vehicles_[v]->solutionRoute_->routeRequests.size();
    }
    repStr << std::left << std::fixed << std::setprecision(2);

    repStr << std::setw(37) << "# Total waiting time before pickup" << " = " << totalWaiting << " (s)" << std::endl;
    repStr << std::setw(37) << "# Total trip delay (s)" << " = " << totalTripDelay << " (s)" << std::endl;
    if (numServed != 0) {
        repStr << std::setw(37) << "# Average waiting time per customer" << " = " << totalWaiting/numServed << " (s)" << std::endl;
        repStr << std::setw(37) << "# Average trip delay   per customer" << " = " << totalTripDelay/numServed << " (s)" << std::endl;
    }
    repStr << std::setw(37) << "# Number of served requests" << " = " << numServed << std::endl;
    repStr << std::setw(37) << "# Total number of requests" << " = " << nbRequests_ << std::endl;
    std::cout << "#" << std::endl;

    return repStr.str();
}

// function to set the data of the partial instance based on the epoch
void Instance::buildPartialData(const PInstance &mainInst, std::vector<PRequest> penaltyRequests, int epoch, int lastRecRequests) {
    instGraph_->addNewNode(mainInst->instGraph_->nodes_[Tools::createNodeID(0, SOURCE)]);
    instGraph_->addNewNode(mainInst->instGraph_->nodes_[Tools::createNodeID(0, SINK)]);
    nbNewRequests_ = 0;

    if (epoch > 0) {
        for (int v = 0; v < mainInst->nbVehicles_; ++v) {
            instGraph_->addNewNode(mainInst->instGraph_->nodes_[mainInst->vehicles_[v]->departID_]);
            if (mainInst->vehicles_[v]->currentRoute_->routeSize_ > 2) {
                for (int i = 1; i < mainInst->vehicles_[v]->currentRoute_->routeSize_-1; ++i) {
                    instGraph_->addNewNode(mainInst->vehicles_[v]->currentRoute_->routeNodes_[i]);
                    if (mainInst->vehicles_[v]->currentRoute_->routeNodes_[i]->type_ == DROPOFF) {

                        addRequest(*mainInst->vehicles_[v]->currentRoute_->routeNodes_[i]->related_Request_, epoch);
                        /*nbRequests_++;
                        requests_.push_back(*mainInst->vehicles_[v]->currentRoute_->routeNodes_[i]->related_Request_);
                        requests_.back()->setPenalty(epoch);
                        nameToRequest_[requests_.back()->name_] = requests_.back();*/
                    }
                }
            }
        }

        for (int j = 0; j < penaltyRequests.size(); ++j) {
            /*nbRequests_++;
            requests_.push_back(penaltyRequests[j]);

            penaltyRequests[j]->setPenalty(epoch);
            nameToRequest_[penaltyRequests[j]->name_] = penaltyRequests[j];*/
            addRequest(penaltyRequests[j], epoch);

            std::string pickID = Tools::createNodeID(penaltyRequests[j]->getRequestId(), PICKUP);
            std::string dropID = Tools::createNodeID(penaltyRequests[j]->getRequestId(), DROPOFF);
            instGraph_->addNewNode(mainInst->instGraph_->nodes_[pickID]);
            instGraph_->addNewNode(mainInst->instGraph_->nodes_[dropID]);
        }
    }

    for (int i = lastRecRequests; i < mainInst->nbRequests_; ++i) {
        if (mainInst->requests_[i]->earlyPick_ <= (epoch+1)*epochLength ) {
            mainInst->requests_[i]->readEpoch_ = epoch;
            nbNewRequests_++;
            addRequest(mainInst->requests_[i], epoch);

            /*nbRequests_++;

            requests_.push_back(mainInst->requests_[i]);
            mainInst->requests_[i]->setPenalty(epoch);
            nameToRequest_[mainInst->requests_[i]->name_] = mainInst->requests_[i];*/

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
void Instance::addRequest(PRequest request, int epoch) {
    nbRequests_++;
    requests_.push_back(request);
    request->setPenalty(epoch - request->readEpoch_);
    nameToRequest_[request->name_] = request;
    request->subStatus_ = NOTSELECTED;
}


void Instance::updateMinTravelTimes() {

    for (int r = 0; r < nbRequests_; ++r) {
        int pickIndex = travelMat->nodeIDToInt_[Tools::createNodeID(requests_[r]->getRequestId(), PICKUP)];
        int dropIndex = travelMat->nodeIDToInt_[Tools::createNodeID(requests_[r]->getRequestId(), DROPOFF)];
        requests_[r]->setMinTravelTime(travelMat->durationValues_[pickIndex][dropIndex]);
    }
}

// print solutions in csv files
void Instance::saveSolutionRoutes(string saveDir) {
    std::ofstream myfile;
    myfile.open (saveDir);
    myfile << "VehicleID,NodeID,RequestTime,ReachTime,NodeType,Latitude,Longitude" << std::endl;
    for (int v = 0; v < nbVehicles_; ++v) {
        for (int i = 0; i < vehicles_[v]->solutionRoute_->routeNodes_.size(); ++i) {
            myfile << vehicles_[v]->vehicleID_ << ",";
            myfile << vehicles_[v]->solutionRoute_->routeNodes_[i]->nodeID_ << ",";
            myfile << vehicles_[v]->solutionRoute_->routeNodes_[i]->requestTime_ << ",";
            myfile << vehicles_[v]->solutionRoute_->routeNodes_[i]->reachTime_ << ",";
            myfile << vehicles_[v]->solutionRoute_->routeNodes_[i]->type_ << ",";
            myfile << vehicles_[v]->solutionRoute_->routeNodes_[i]->locLatitude_ << ",";
            myfile << vehicles_[v]->solutionRoute_->routeNodes_[i]->locLongitude_ << "\n";

        }
    }
    myfile.close();
}

void Instance::saveRequestsResults(string saveDir) {
    std::ofstream myfile;
    myfile.open (saveDir);
    myfile << "RequestID,nbPassengers,PickLatitude,PickLongitude,DropLatitude,DropLongitude,RequestTime,PickReachTime,"
              "DropReachTime" << std::endl;

    for (int i = 0; i < nbRequests_; ++i) {
        myfile << requests_[i]->getRequestId() << ",";
        myfile << requests_[i]->nbPassengers_ << ",";
        myfile << requests_[i]->PickUpLatitude_ << ",";
        myfile << requests_[i]->PickUpLongitude_ << ",";
        myfile << requests_[i]->DropOffLatitude_ << ",";
        myfile << requests_[i]->DropOffLongitude_ << ",";
        myfile << requests_[i]->earlyPick_ << ",";
        myfile << requests_[i]->pickTime_ << ",";
        myfile << requests_[i]->dropTime_ << "\n";
/*        if (requests_[i]->requestStatus_ != NO_ACTION) {
            myfile << requests_[i]->pickTime_ << ",";
            myfile << requests_[i]->dropTime_ << "\n";
        }
        else {
            myfile << "--" << ",";
            myfile << "--" << "\n";
        }*/
    }
    myfile.close();
}







