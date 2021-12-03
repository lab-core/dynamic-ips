//
// Created by Ella on 2021-11-28.
//

#include "utilities/ReadWrite.h"
#include <iostream>
#include "solvers/ISUDAlgorithm.h"
#include "solvers/SubProblem.h"
#include "data/Instance.h"


using std::string;
using namespace std::chrono;

int main() {

    int nbReceivedRequest = 0;
    int epoch = 0;

    Tools::Timer *subProTime = new Tools::Timer(); subProTime->init();;

    string dataDir = "datasets/";
    string instanceName = "20150713_16-03m";

    // build the path of input files
    InputPaths inputPaths(dataDir, instanceName);

    // Read data files and initialize instance
    std::cout << "# INITIALIZE OF THE MAIN INSTANCE" << std::endl;
    PInstance mainInst = ReadWrite::readInstance(inputPaths.getInstanceData());
    ReadWrite::readTripRequests(inputPaths.getTripData(), mainInst);
    // updating vehicle data and setting their current route to an empty route

    std::cout << std::endl;
    std::cout << mainInst->toString();

    std::shared_ptr<ISUDAlgorithm> isudObj = std::make_shared<ISUDAlgorithm>();

    double previousObj = 0;
    while (nbReceivedRequest < mainInst->nbRequests_) {
        std::cout << "************************************************************************************" << std::endl;
        std::cout << " *****************************  epoch " << epoch << "  *****************************" << std::endl;
        std::cout << "************************************************************************************" << std::endl;
        subProTime->start();
        for (int v = 0; v < mainInst->nbVehicles_; ++v) {
            mainInst->vehicles_[v]->updateStatus(epoch);
        }
        PInstance EpochInst = std::make_shared<Instance>(*mainInst);
        EpochInst->buildPartialData(mainInst, isudObj->zSolution_ , epoch, nbReceivedRequest);
        nbReceivedRequest += EpochInst->nbNewRequests_;
        subProTime->stop();
        isudObj->initialization(EpochInst);
        std::cout << "# NUMBER OF RECEIVED REQUESTS: " << EpochInst->nbNewRequests_ << std::endl;
        std::cout << "# TOTAL NUMBER OF RECEIVED REQUESTS: " << EpochInst->nbRequests_ << std::endl;

        // solving the reduced problem at the start of each epoch
        int flagImprove = 0;
        sort(mainInst->vehicles_.begin(),mainInst->vehicles_.end(),[](const PVehicle &lhs, const PVehicle &rhs){
            return lhs->currentRoute_->routeSize_ > rhs->currentRoute_->routeSize_;});

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
                if (subProblem->SubProbCplex_.getObjValue() >= -0.00001) {
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
        std::cout << "# Solution after ISUD:" << std::endl;
        std::cout << isudObj->toString();
        epoch++;

    }

    sort(mainInst->vehicles_.begin(),mainInst->vehicles_.end(),[](const PVehicle &lhs, const PVehicle &rhs){
        return lhs->vehicleID_ < rhs->vehicleID_;});
    for (int v = 0; v < mainInst->nbVehicles_; ++v) {
        if (mainInst->vehicles_[v]->solutionRoute_->routeNodes_.back()->type_ == SOURCE) {
            for (int i = 1; i < mainInst->vehicles_[v]->currentRoute_->routeSize_; ++i) {
                mainInst->vehicles_[v]->solutionRoute_->addNode(mainInst->vehicles_[v]->currentRoute_->routeNodes_[i],
                                                                mainInst->vehicles_[v]->currentRoute_->plannedReachTime_[i],
                                                                mainInst->vehicles_[v]->currentRoute_->plannedPassengers_[i]);
            }
        }
        else
            mainInst->vehicles_[v]->solutionRoute_->addNode(mainInst->vehicles_[v]->currentRoute_->routeNodes_.back(),
                                                            mainInst->vehicles_[v]->currentRoute_->plannedReachTime_.back(),
                                                            mainInst->vehicles_[v]->currentRoute_->plannedPassengers_.back());

    }

    int numServed = 0;
    double totalDelay = 0;
    std::cout << "# Final solution of the routes after " << epoch << " epochs: " << std::endl;
    for (int v = 0; v < mainInst->nbVehicles_; ++v) {
        std::cout << mainInst->vehicles_[v]->solutionRoute_->toString();
        totalDelay += mainInst->vehicles_[v]->solutionRoute_->totalDelay_;
        numServed += mainInst->vehicles_[v]->solutionRoute_->routeRequests.size();
    }
    std::cout << "# Time spent on ISUD improvement    = " << isudObj->isudTime_->dSinceInit().count() << " (seconds)" << std::endl;
    std::cout << "# Time spent on solving subproblems = " << subProTime->dSinceInit().count() << " (seconds)" << std::endl;
    std::cout << "# Total delay                       = " << totalDelay << std::endl;
    std::cout << "# Number of served requests       = " << numServed << std::endl;
    std::cout << "# Total number of requests          = " << mainInst->nbRequests_ << std::endl;
}
