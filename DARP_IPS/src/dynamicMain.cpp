//
// Created by Ella on 2021-11-28.
//
#define CURL_STATICLIB

#include "utilities/ReadWrite.h"
#include "solvers/ISUDAlgorithm.h"
#include "solvers/CPLEXSubProblem.h"
#include "data/Instance.h"
#include <filesystem>
#include "solvers/MIPSolver.h"


using std::string;
using namespace std::chrono;

PTravelTime travelMat;
vector2D<float> durationMatrix_;

int main() {



    int nbReceivedRequest = 0;
    int epoch = 1;

    Tools::Timer *subProTime = new Tools::Timer(); subProTime->init();;

    string dataDir = "datasets/";
    string instanceName = "20150713_16-03m";

    // definition of results path
    string isudSolutionDir = dataDir + instanceName + "/epochSolution_" + instanceName + ".csv";
    string finalSolutionDir = dataDir + instanceName + "/finalSolution_" + instanceName + ".csv";
    std::string logfilePath = dataDir + instanceName + "/LogFile_" + instanceName + ".txt";
    std::string logInitial = dataDir + instanceName + "/LogFile_" + ".txt";
    std::ofstream myFile;
//    freopen (logInitial.c_str(),"w",stdout);

    myFile.open(isudSolutionDir);
  //  myFile << "Epoch, ISUDIter,VehicleID,NodeID,RequestTime,ReachTime,NodeType,Latitude,Longitude" << std::endl;
    myFile << "Epoch, ISUDIter,VehicleID,NodeID,RequestTime,ReachTime,NodeType,LocationID" << std::endl;
    myFile.close();

    myFile.open(finalSolutionDir);
//    myFile << "Epoch,VehicleID,NodeID,RequestTime,ReachTime,NodeType,Latitude,Longitude" << std::endl;
    myFile << "Epoch,VehicleID,NodeID,RequestTime,ReachTime,NodeType,LocationID" << std::endl;
    myFile.close();



    // build the path of input files
    InputPaths inputPaths(dataDir, instanceName);

    // Read data files and initialize instance
    std::cout << "# INITIALIZE OF THE MAIN INSTANCE" << std::endl;
    PInstance mainInst = ReadWrite::readInstance(inputPaths.getInstanceData());
    ReadWrite::readTripRequests(inputPaths.getTripData(), mainInst);
    ReadWrite::readDurations(inputPaths.getDurationData(), durationMatrix_, 2*mainInst->nbRequests_+1);

    // creating distance matrix based on the data from OSRM server
    /*travelMat = std::make_shared<TravelTime>();
    travelMat->setNodeIdToInt(mainInst->instGraph_->nodeIDToInt_);
    travelMat->setDurationMat(mainInst->instGraph_);*/

    // setting min travel times of requests based on the distance matrix
    // initializing vehicles empty route and current route
    mainInst->updateMinTravelTimes();
    for (int v = 0; v < mainInst->nbVehicles_; ++v) {
        mainInst->vehicles_[v]->setEmptyRoute(mainInst);
        mainInst->vehicles_[v]->setCurrentRoute(mainInst->vehicles_[v]->emptyRoute_);
    }


    std::cout << std::endl;
    std::cout << mainInst->toString();

    MIPSolver(mainInst);


    std::shared_ptr<ISUDAlgorithm> isudObj = std::make_shared<ISUDAlgorithm>();
//    MIPSolver(mainInst);
    double previousObj = 0;
    SubSolveStatus subStaus;
    while (nbReceivedRequest < mainInst->nbRequests_) {
        subStaus = RESTRICTED;
        std::cout << " *****************************  epoch " << std::setw(3) << epoch << "  *****************************" << std::endl;


        subProTime->start();
        for (int v = 0; v < mainInst->nbVehicles_; ++v) {
            mainInst->vehicles_[v]->updateStatus(epoch);
        }

        // creating a subInstance
        PInstance EpochInst = std::make_shared<Instance>(*mainInst);
        // reading the data received in previous epoch (epoch -1)
        EpochInst->buildPartialData(mainInst, isudObj->zSolution_ , epoch-1, nbReceivedRequest);
        nbReceivedRequest += EpochInst->nbNewRequests_;
        subProTime->stop();
        isudObj->initialization(EpochInst);
        std::cout << "# NUMBER OF RECEIVED REQUESTS: " << EpochInst->nbNewRequests_ << std::endl;
        std::cout << "# TOTAL NUMBER OF RECEIVED REQUESTS: " << nbReceivedRequest << std::endl;
        std::cout << "# Vehicle Routes after initialization: " << nbReceivedRequest << std::endl;
        for (auto & vehicleObj : EpochInst->vehicles_) {
            std::cout << vehicleObj->currentRoute_->toString();
        }

        // solving the reduced problem at the start of each epoch
        int noImprove = 0;
        /*sort(mainInst->vehicles_.begin(),mainInst->vehicles_.end(),[](const PVehicle &lhs, const PVehicle &rhs){
            return lhs->currentRoute_->routeSize_ > rhs->currentRoute_->routeSize_;});*/
  //      sort(EpochInst->vehicles_.begin(),EpochInst->vehicles_.end(),[](const PVehicle &lhs, const PVehicle &rhs){
  //          return lhs->dual_ > rhs->dual_;});


        while (noImprove == 0) {
            /*sort(EpochInst->vehicles_.begin(),EpochInst->vehicles_.end(),[](const PVehicle &lhs, const PVehicle &rhs){
                return lhs->departTime_ < rhs->departTime_;});*/
            /*sort(EpochInst->vehicles_.begin(),EpochInst->vehicles_.end(),[](const PVehicle &lhs, const PVehicle &rhs){
                return lhs->dual_ > rhs->dual_;});*/
            sort(EpochInst->vehicles_.begin(),EpochInst->vehicles_.end(),[](const PVehicle &lhs, const PVehicle &rhs){
                return std::tie(lhs->currentRoute_->routeSize_,lhs->departTime_) < std::tie(rhs->currentRoute_->routeSize_,rhs->departTime_);
            });
            subProTime->start();
            for (int i = 0; i < EpochInst->nbRequests_; ++i) {
                EpochInst->requests_[i]->subStatus_ = NOTSELECTED;
            }
            for (auto & vehicleObj : EpochInst->vehicles_) {
                /*PCPLEXsubPro subProblem = std::make_shared<CPLEXSubProblem>(vehicleObj);
                subProblem->initSubGraph(EpochInst, subStaus);
                int maxPick = EpochInst->nbRequests_;
                if (subStaus == RESTRICTED)
                    maxPick = floor(subProblem->subRequests_.size()/EpochInst->nbVehicles_)+2;

                subProblem->BuildModelCPLEX(isudObj->ReducedPro_->requestDuals_,
                                            isudObj->ReducedPro_->vehicleDuals_[vehicleObj->vehicleID_],
                                            isudObj->ReducedPro_->requestToOrder_, maxPick);
                subProblem->SolveCPLEX();
                std::cout << subProblem->toString();
                isudObj->availableRoutes_[vehicleObj->vehicleID_].clear();
                if (subProblem->SubProbCplex_.getObjValue() >= -0.0001) {
                    noImprove ++;
                }
                else
                    subProblem->SolutionToRoutes(isudObj->availableRoutes_[vehicleObj->vehicleID_], isudObj->generatedRoutes_);*/

                PLabelingSubPro subProblem = std::make_shared<LabelingSubProblem>(vehicleObj);
                subProblem->initSubGraph(EpochInst);
                int maxPick = EpochInst->nbRequests_;
                if (subStaus == RESTRICTED)
                    maxPick = floor(subProblem->subRequests_.size()/EpochInst->nbVehicles_)+2;
                subProblem->solveDynamic(maxPick);
                isudObj->availableRoutes_[vehicleObj->vehicleID_].clear();
                if (!subProblem->subGraph_->nodes_[vehicleObj->sinkID_]->activeLabels_.empty()) {
                    if(subProblem->subGraph_->nodes_[vehicleObj->sinkID_]->activeLabels_[0]->reducedCost_ >= -0.0001)
                        noImprove ++;
                    else
                        subProblem->SolutionToRoutes(isudObj->availableRoutes_[vehicleObj->vehicleID_], isudObj->generatedRoutes_);
                }
                else
                    noImprove ++;
            }

            subStaus = FULL;
            std::cout << "# ============================================================" << std::endl;
            std::cout << "# Time spent on solving subproblems:  = " << subProTime->dSinceStart().count() << " (seconds)" << std::endl;
            std::cout << "# ============================================================" << std::endl;
            std::cout << "#" << std::endl;
            subProTime->stop();
            if (noImprove == EpochInst->nbVehicles_) {
                std::cout << " *****************************  The Column Generation Terminated!  *****************************" << std::endl;
                break;
            }
            else {
                previousObj = isudObj->objValue_;
                sort(EpochInst->vehicles_.begin(),EpochInst->vehicles_.end(),[](const PVehicle &lhs, const PVehicle &rhs){
                    return lhs->vehicleID_ < rhs->vehicleID_;});
                isudObj->solveISUD(EpochInst, epoch, isudSolutionDir);
                std::cout << "# Solution Result after ISUD:" << std::endl;
                /*for (int r = 0; r < isudObj->routeSolution_.size(); ++r) {
                    std::cout << isudObj->routeSolution_[r]->toString();
                }*/
                for (auto & routeObj : isudObj->routeSolution_){
                    std::cout << routeObj->toString();
                }

                noImprove = 0;
            }
            if (previousObj == isudObj->objValue_)
                break;
        }

        for (auto & routeObj : isudObj->routeSolution_) {
            EpochInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
        }
        /*for (int r = 0; r < isudObj->routeSolution_.size(); ++r) {
            EpochInst->vehicles_[isudObj->routeSolution_[r]->vehicleID_]->setCurrentRoute(isudObj->routeSolution_[r]);
        }*/
        isudObj->setObjValue();
        std::cout << "# Solution of ISUD after epoch " << epoch << " : " << std::endl;
        std::cout << isudObj->toString();
        EpochInst->saveEpochRoutes(finalSolutionDir, epoch);
        epoch++;
    }


    /*sort(mainInst->vehicles_.begin(),mainInst->vehicles_.end(),[](const PVehicle &lhs, const PVehicle &rhs){
        return lhs->vehicleID_ < rhs->vehicleID_;});*/
    for (auto & vehicleObj : mainInst->vehicles_) {
        if (vehicleObj->solutionRoute_->routeNodes_.back()->type_ == SOURCE) {
            for (int i = 1; i < vehicleObj->currentRoute_->routeSize_; ++i) {
                vehicleObj->currentRoute_->routeNodes_[i]->nodeStatus_ = DONE;
                vehicleObj->currentRoute_->routeNodes_[i]->reachTime_ = vehicleObj->currentRoute_->plannedReachTime_[i];
                if (i != vehicleObj->currentRoute_->routeSize_ )
                    (*vehicleObj->currentRoute_->routeNodes_[i]->related_Request_)->requestStatus_ = COMPLETED;
                /*vehicleObj->solutionRoute_->addNode(vehicleObj->currentRoute_->routeNodes_[i],
                                                    vehicleObj->currentRoute_->plannedReachTime_[i],
                                                    vehicleObj->currentRoute_->plannedPassengers_[i]);*/
                vehicleObj->solutionRoute_->addNode(vehicleObj->currentRoute_->routeNodes_[i]);

                if (vehicleObj->currentRoute_->routeNodes_[i]->type_ == PICKUP) {
                    (*vehicleObj->currentRoute_->routeNodes_[i]->related_Request_)->pickTime_ =
                            vehicleObj->currentRoute_->plannedReachTime_[i];
                    (*vehicleObj->currentRoute_->routeNodes_[i]->related_Request_)->vehicleID_ = vehicleObj->vehicleID_;
                }
                else if (vehicleObj->currentRoute_->routeNodes_[i]->type_ == DROPOFF)
                    (*vehicleObj->currentRoute_->routeNodes_[i]->related_Request_)->dropTime_ =
                            vehicleObj->currentRoute_->plannedReachTime_[i];
            }
        }
        else
            /*vehicleObj->solutionRoute_->addNode(vehicleObj->currentRoute_->routeNodes_.back(),
                                                vehicleObj->currentRoute_->plannedReachTime_.back(),
                                                vehicleObj->currentRoute_->plannedPassengers_.back());*/
            vehicleObj->solutionRoute_->addNode(vehicleObj->currentRoute_->routeNodes_.back());

    }
    std::cout << std::endl << std::endl;

//    fclose (stdout);
    freopen (logfilePath.c_str(),"w",stdout);


    std::cout << "*************************************************************************************" << std::endl;
    std::cout << "*********************** FINAL VEHICLE ROUTES AFTER " << std::setw(3) << epoch << " EPOCHS *********************** " << std::endl;
    std::cout << "*************************************************************************************" << std::endl;
    std::cout << std::endl << std::endl;

    double finalObj_ = 0.0;
    for (int v = 0; v < mainInst->nbVehicles_; ++v) {
        std::cout << mainInst->vehicles_[v]->solutionRoute_->toString();
    }
    std::cout << "#" << std::endl;

    std::cout << mainInst->solutionToString();
    std::cout << std::left << std::fixed << std::setprecision(2);
    std::cout << std::setw(sentenceSize) << "# Time spent on ISUD improvement" << " = " << isudObj->isudTime_->dSinceInit().count() << " (s)" << std::endl;
    std::cout << std::setw(sentenceSize) << "# Time spent on solving Subproblems" << " = " << subProTime->dSinceInit().count() << " (s)" << std::endl;



    string outputRouteDir = dataDir + instanceName + "/Routes_" + instanceName + ".csv";
    string outputRequestDir = dataDir + instanceName + "/Requests_" + instanceName + ".csv";
    mainInst->saveSolutionRoutes(outputRouteDir);
    mainInst->saveRequestsResults(outputRequestDir);
    mainInst->saveEpochRoutes(finalSolutionDir, epoch);

    fclose (stdout);

}
