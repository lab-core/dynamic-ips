//
// Created by Elahe Amiri on 2025-11-02.
//

#include "AnytimeSolver.h"
#include "MasterAlgorithm.h"


void AnytimeSolver::AnytimeHorizon(PInstance &mainInst, InputPaths &inputPaths, bool middleSave, float saveTime) {
    // define required variables
    epoch_ = 0;
    rebalanceTime_ = 0;
    rebalancingTime_->start();
    std::vector<float> EpochTime = {1,1,1};

    int instance_count = 1;

    mainInst->setInitialTimes(mainInst->parameters_->committedTime_);
    for (auto &vehicleObj : mainInst->vehicles_) {
        vehicleObj->setEmptyRoute(mainInst);
        vehicleObj->setSolutionRoute();
        if (vehicleObj->currentRoute_ == nullptr)
            vehicleObj->setCurrentRoute(vehicleObj->emptyRoute_);
        for (auto & routeNode : vehicleObj->currentRoute_->routeNodes_) {
            if (routeNode->type_ == DROPOFF && routeNode->nodeStatus_ == PLANNED) {
                routeNode->related_Request_->committedPickTime_ = routeNode->related_Request_->pickTime_;

                routeNode->related_Request_->latestDrop_ = routeNode->pairNode_->reachTime_ +
                    routeNode->pairNode_->serviceTime_ + routeNode->related_Request_->maxTravelTime_;
            }
        }
    }

    int nbReceivedRequest = mainInst->nbOnboards_;
    PInstance EpochInst = std::make_shared<Instance>(*mainInst);

    while (nbReceivedRequest < mainInst->nbRequests_){
        nextEpoch:
        // start simulation timer
        simulationTime_->start();
        elapsedTime_ = simulationTime_->dSinceInit().count();

        /******************************* Log Info ****************************/
        logEpochInfo(epoch_, elapsedTime_, simulationTime_->dSinceInit().count(), runtimeMetrics_->epochRuntime_,
            inputPaths);

        EpochTime[epoch_ % EpochTime.size()] = runtimeMetrics_->epochRuntime_;
        float avg = ceil(std::accumulate(EpochTime.begin(), EpochTime.end(),0.0f) / static_cast<float>(EpochTime.size()));
        if (mainInst->parameters_->committedTime_ > std::max(avg, mainInst->parameters_->epochLength_)) {
            mainInst->parameters_->committedTime_ = std::max(avg, mainInst->parameters_->epochLength_);
            std::cout << "dec commit time: " << mainInst->parameters_->committedTime_ << std::endl;
        }
        if (mainInst->parameters_->committedTime_ < runtimeMetrics_->epochRuntime_) {
            mainInst->parameters_->committedTime_ = ceil(runtimeMetrics_->epochRuntime_ + 2);
            std::cout << "inc commit time: " << mainInst->parameters_->committedTime_ << std::endl;
        }

        // update vehicle status
        mainInst->nbOnboards_ = 0;
        boost::dynamic_bitset<> removedRequests;
        removedRequests.resize(EpochInst->nbRequests_);

        for (auto &vehicleObj: mainInst->vehicles_) {
            vehicleObj->updateStateTime(mainInst, mainInst->simulationStartTime_ + elapsedTime_, removedRequests);
            mainInst->nbOnboards_ += static_cast<int>(vehicleObj->onboards_.size());
        }
        if (mainInst->parameters_->routeRecycle_) {
            if (MP_solver_->availableRoutes_.size() > 0) {
                for (auto &vehicleObj: mainInst->vehicles_) {
                    if (vehicleObj->stateChanged_)
                        MP_solver_->availableRoutes_[vehicleObj->vehicleID_].clear();
                }
                /*if (removedRequests.count()) {
                    updateAvailableRoutes(removedRequests, MP_solver_->availableRoutes_);
                }*/
            }
        }

        buildEpochInstance(mainInst, EpochInst, elapsedTime_, nbReceivedRequest);

        // saving the status in the middle of running
        if (middleSave && elapsedTime_ >= saveTime) {
            mainInst->saveStatus(inputPaths, EpochInst->simulationStartTime_ + elapsedTime_,
                mainInst->parameters_->epochLength_, std::to_string(instance_count));

            saveTime += 120;
            if (instance_count >= 30)
                break;
            instance_count ++;
        }

        if (EpochInst->nbNewRequests_ == 0) {
            if (EpochInst->parameters_->mainAlgorithm_ != GREEDY)
                MP_solver_->availableRoutes_.clear();
            simulationTime_->stop();
            simulationTime_->addTime(mainInst->requests_[nbReceivedRequest]->requestTime_ -
                    mainInst->simulationStartTime_ - simulationTime_->dSinceInit().count());
            epoch_++;
            goto nextEpoch;
        }

        switch (EpochInst->parameters_->mainAlgorithm_) {
            case GREEDY:
                GreedyModel_->GreedySolver(EpochInst);
                handleVehicleReturn(EpochInst);

                break;
            case MIP_CPLEX:
                MIPModel_ = std::make_unique<MIPSolver>();
                MIPModel_->SolveMIP(EpochInst, inputPaths);
  //              masterModel_->zSolution_ = MIPModel_->zSolution_;
  //              masterModel_->routeSolution_ = MIPModel_->routeSolution_;
                MIPModel_.reset();
                break;
            default:
  //              solveCG_Epoch_CPLEX(EpochInst, mainInst, inputPaths);
                solveEpoch(EpochInst, mainInst, inputPaths);
                break;
        }

        elapsedTime_ = simulationTime_->dSinceInit().count();
        EpochInst->setAssignedEpochVehicles(mainInst->simulationStartTime_ + elapsedTime_);
        if (EpochInst->parameters_->mainAlgorithm_ != GREEDY)
            *pLogRunTimesStream_ << saveRuntimes(EpochInst);
        else
            *pLogRunTimesStream_ << saveRuntimesGreedy(EpochInst);

        epoch_++;
        simulationTime_->stop();
    }

    for (auto & vehicleObj : mainInst->vehicles_) {
        vehicleObj->finalizeSolutionRoutes(mainInst->simulationStartTime_+ elapsedTime_);
        vehicleObj->solutionRoute_->calculateTripDelay(mainInst->parameters_->Wait_W1_,mainInst->parameters_->Ride_W2_);
        // Test the solution route
        vehicleObj->solutionRoute_->testRoute(vehicleObj);
    }
    if (!middleSave) {
        if (mainInst->parameters_->timeWindow_ > 0) {
            for (auto& requestObj : mainInst->requests_) {
                if (requestObj->requestStatus_ != COMPLETED)
                    requestObj->requestStatus_ = REJECTED;
            }
        }
    }
    rebalancingTime_->stop();
}

void AnytimeSolver::doSimulation(PInstance &mainInst, InputPaths &inputPaths, bool middleSave, float saveTime) {
    std::cout << "====================================================================="<< std::endl;
    std::cout << "               Rolling Horizon with flexible Batch size" << std::endl;
    std::cout << "====================================================================="<< std::endl;
    AnytimeHorizon(mainInst, inputPaths, middleSave, saveTime);
}
