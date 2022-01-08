//
// Created by Ella on 2021-11-28.
//
#define CURL_STATICLIB

#include "utilities/ReadWrite.h"
#include "solvers/ISUDAlgorithm.h"
#include "solvers/SubProblem.h"
#include "data/Instance.h"


using std::string;
using namespace std::chrono;

PTravelTime travelMat;


int main() {



    int nbReceivedRequest = 0;
    int epoch = 1;

    Tools::Timer *subProTime = new Tools::Timer(); subProTime->init();;

    string dataDir = "datasets/";
    string instanceName = "20150713_16-03m";



    // build the path of input files
    InputPaths inputPaths(dataDir, instanceName);

    // Read data files and initialize instance
    std::cout << "# INITIALIZE OF THE MAIN INSTANCE" << std::endl;
    PInstance mainInst = ReadWrite::readInstance(inputPaths.getInstanceData());
    ReadWrite::readTripRequests(inputPaths.getTripData(), mainInst);

    // creating distance matrix based on the data from OSRM server
    travelMat = std::make_shared<TravelTime>();
    travelMat->setNodeIdToInt(mainInst->instGraph_->nodeIDToInt_);
    travelMat->setDurationMat(mainInst->instGraph_);

    // setting min travel times of requests based on the distance matrix
    // initializing vehicles empty route and current route
    mainInst->updateMinTravelTimes();
    for (int v = 0; v < mainInst->nbVehicles_; ++v) {
        mainInst->vehicles_[v]->setEmptyRoute(mainInst);
        mainInst->vehicles_[v]->setCurrentRoute(mainInst->vehicles_[v]->emptyRoute_);
    }


    std::cout << std::endl;
    std::cout << mainInst->toString();


    std::shared_ptr<ISUDAlgorithm> isudObj = std::make_shared<ISUDAlgorithm>();
//    MIPSolver(mainInst);
    double previousObj = 0;
    while (nbReceivedRequest < mainInst->nbRequests_) {
        std::cout << " *****************************  epoch " << std::setw(3) << epoch << "  *****************************" << std::endl;
        subProTime->start();
        for (int v = 0; v < mainInst->nbVehicles_; ++v) {
            mainInst->vehicles_[v]->updateStatus(epoch);
        }
        PInstance EpochInst = std::make_shared<Instance>(*mainInst);
        // reading the data received in previous epoch (epoch -1)
        EpochInst->buildPartialData(mainInst, isudObj->zSolution_ , epoch-1, nbReceivedRequest);
        nbReceivedRequest += EpochInst->nbNewRequests_;
        subProTime->stop();
        isudObj->initialization(EpochInst);
        std::cout << "# NUMBER OF RECEIVED REQUESTS: " << EpochInst->nbNewRequests_ << std::endl;
        std::cout << "# TOTAL NUMBER OF RECEIVED REQUESTS: " << nbReceivedRequest << std::endl;

        // solving the reduced problem at the start of each epoch
        int flagImprove = 0;
        /*sort(mainInst->vehicles_.begin(),mainInst->vehicles_.end(),[](const PVehicle &lhs, const PVehicle &rhs){
            return lhs->currentRoute_->routeSize_ > rhs->currentRoute_->routeSize_;});*/
        sort(mainInst->vehicles_.begin(),mainInst->vehicles_.end(),[](const PVehicle &lhs, const PVehicle &rhs){
            return lhs->dual_ > rhs->dual_;});

        while (flagImprove == 0) {
            subProTime->start();
            for (int i = 0; i < EpochInst->nbRequests_; ++i) {
                EpochInst->requests_[i]->subStatus_ = NOTSELECTED;
            }
            for (int v = 0; v < EpochInst->nbVehicles_; ++v) {
                /*if (nbReceivedRequest >= mainInst->nbRequests_)
                    EpochInst->vehicles_[v]->setEndTime(3600);
                else
                    EpochInst->vehicles_[v]->setEndTime(3 * epochLength * (epoch+1));*/
                PSubProblem subProblem = std::make_shared<SubProblem>(EpochInst->vehicles_[v]);
                subProblem->initSubGraph(EpochInst);
                subProblem->BuildModelCPLEX(isudObj->ReducedPro_->requestDuals_, isudObj->ReducedPro_->vehicleDuals_[v],
                                            isudObj->ReducedPro_->requestToOrder_);
                subProblem->SolveCPLEX();
                std::cout << subProblem->toString() << std::endl;
                if (subProblem->SubProbCplex_.getObjValue() >= -0.0001) {
                    flagImprove ++;
                }
                else
                    subProblem->SolutionToRoutes(isudObj->availableRoutes_[EpochInst->vehicles_[v]->vehicleID_], isudObj->generatedRoutes_);
            }
            subProTime->stop();
            if (flagImprove == EpochInst->nbVehicles_) {
                std::cout << " *****************************  The Column Generation Terminated!  *****************************" << std::endl;
                break;
            }
            else {
                isudObj->solveISUD(EpochInst, epoch);
                std::cout << "# Solution Result after ISUD:" << std::endl;
                for (int r = 0; r < isudObj->routeSolution_.size(); ++r) {
                    std::cout << isudObj->routeSolution_[r]->toString();
                }
                flagImprove = 0;
            }
            if (previousObj == isudObj->objValue_)
                break;
            previousObj = isudObj->objValue_;
        }
        sort(mainInst->vehicles_.begin(),mainInst->vehicles_.end(),[](const PVehicle &lhs, const PVehicle &rhs){
            return lhs->vehicleID_ < rhs->vehicleID_;});
        for (int r = 0; r < isudObj->routeSolution_.size(); ++r) {
            EpochInst->vehicles_[isudObj->routeSolution_[r]->vehicleID_]->setCurrentRoute(isudObj->routeSolution_[r]);
        }
        std::cout << "# Solution of ISUD after epoch " << epoch << " : " << std::endl;
        std::cout << isudObj->toString();
        epoch++;

    }

    sort(mainInst->vehicles_.begin(),mainInst->vehicles_.end(),[](const PVehicle &lhs, const PVehicle &rhs){
        return lhs->vehicleID_ < rhs->vehicleID_;});
    for (int v = 0; v < mainInst->nbVehicles_; ++v) {
        if (mainInst->vehicles_[v]->solutionRoute_->routeNodes_.back()->type_ == SOURCE) {
            for (int i = 1; i < mainInst->vehicles_[v]->currentRoute_->routeSize_; ++i) {
                mainInst->vehicles_[v]->currentRoute_->routeNodes_[i]->nodeStatus_ = DONE;
                mainInst->vehicles_[v]->currentRoute_->routeNodes_[i]->reachTime_ =
                        mainInst->vehicles_[v]->currentRoute_->plannedReachTime_[i];
                if (i != mainInst->vehicles_[v]->currentRoute_->routeSize_ - 1)
                    (*mainInst->vehicles_[v]->currentRoute_->routeNodes_[i]->related_Request_)->requestStatus_ = COMPLETED;
                mainInst->vehicles_[v]->solutionRoute_->addNode(mainInst->vehicles_[v]->currentRoute_->routeNodes_[i],
                                                                mainInst->vehicles_[v]->currentRoute_->plannedReachTime_[i],
                                                                mainInst->vehicles_[v]->currentRoute_->plannedPassengers_[i]);

                if (mainInst->vehicles_[v]->currentRoute_->routeNodes_[i]->type_ == PICKUP) {
                    (*mainInst->vehicles_[v]->currentRoute_->routeNodes_[i]->related_Request_)->pickTime_ =
                            mainInst->vehicles_[v]->currentRoute_->plannedReachTime_[i];
                }
                else if (mainInst->vehicles_[v]->currentRoute_->routeNodes_[i]->type_ == DROPOFF)
                    (*mainInst->vehicles_[v]->currentRoute_->routeNodes_[i]->related_Request_)->dropTime_ =
                            mainInst->vehicles_[v]->currentRoute_->plannedReachTime_[i];
            }
        }
        else
            mainInst->vehicles_[v]->solutionRoute_->addNode(mainInst->vehicles_[v]->currentRoute_->routeNodes_.back(),
                                                            mainInst->vehicles_[v]->currentRoute_->plannedReachTime_.back(),
                                                            mainInst->vehicles_[v]->currentRoute_->plannedPassengers_.back());

    }
    std::cout << std::endl << std::endl;

    std::string logfilePath = dataDir + instanceName + "/LogFile_" + instanceName + ".txt";
    freopen (logfilePath.c_str(),"w",stdout);


    std::cout << "*************************************************************************************" << std::endl;
    std::cout << "*********************** FINAL VEHICLE ROUTES AFTER " << std::setw(3) << epoch << " EPOCHS *********************** " << std::endl;
    std::cout << "*************************************************************************************" << std::endl;
    std::cout << std::endl << std::endl;

    for (int v = 0; v < mainInst->nbVehicles_; ++v) {
        std::cout << mainInst->vehicles_[v]->solutionRoute_->toString();
    }
    std::cout << "#" << std::endl;



    std::cout << mainInst->solutionToString();
    std::cout << std::left << std::fixed << std::setprecision(2);
    std::cout << std::setw(37) << "# Time spent on ISUD improvement" << " = " << isudObj->isudTime_->dSinceInit().count() << " (s)" << std::endl;
    std::cout << std::setw(37) << "# Time spent on solving subproblems" << " = " << subProTime->dSinceInit().count() << " (s)" << std::endl;

    string outputRouteDir = dataDir + instanceName + "/Routes_" + instanceName + ".csv";
    string outputRequestDir = dataDir + instanceName + "/Requests_" + instanceName + ".csv";
    mainInst->saveSolutionRoutes(outputRouteDir);
    mainInst->saveRequestsResults(outputRequestDir);

    fclose (stdout);

}
