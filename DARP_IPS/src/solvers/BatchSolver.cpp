//
// Created by Elahe Amiri on 2025-11-02.
//

#include "BatchSolver.h"
#include "MasterAlgorithm.h"

void BatchSolver::BatchHorizon(PInstance &mainInst, InputPaths &inputPaths, bool middleSave, float saveTime) {
    // define required variables
    epoch_ = 0;
    rebalanceTime_ = 0;
    rebalancingTime_->start();
 //   int instance_count = 1;
    int instance_count = mainInst->nbVehicles_;

    mainInst->setInitialTimes(mainInst->parameters_->epochLength_);
    for (auto &vehicleObj : mainInst->vehicles_) {
        vehicleObj->setEmptyRoute(mainInst);
        vehicleObj->setSolutionRoute();
        if (vehicleObj->currentRoute_ == nullptr)
            vehicleObj->setCurrentRoute(vehicleObj->emptyRoute_);
    }

    int nbReceivedRequest = mainInst->nbOnboards_;
    PInstance EpochInst = std::make_shared<Instance>(*mainInst);
    std::vector<PRequest> zSolution;
    while (nbReceivedRequest < mainInst->nbRequests_ || !zSolution.empty()){
        if (EpochInst->parameters_->mainAlgorithm_ != GREEDY)
            zSolution = MP_solver_->zSolution_;
        else
            zSolution = GreedyModel_->zSolution_;
        nextEpoch:
        // start simulation timer
        simulationTime_->start();
        elapsedTime_ = static_cast<float>(epoch_) * mainInst->parameters_->epochLength_;

        /******************************* Log Info ****************************/
        logEpochInfo(epoch_, elapsedTime_, simulationTime_->dSinceInit().count(), runtimeMetrics_->epochRuntime_,
            inputPaths);

        // update vehicle status
        mainInst->nbOnboards_ = 0;
        boost::dynamic_bitset<> removedRequests;
        removedRequests.resize(EpochInst->nbRequests_);

        for (auto &vehicleObj: mainInst->vehicles_) {
            vehicleObj->updateStateTime(mainInst, mainInst->simulationStartTime_ + elapsedTime_, removedRequests);
            mainInst->nbOnboards_ += static_cast<int>(vehicleObj->onboards_.size());
        }
        if (mainInst->parameters_->routeRecycle_) {
            if (!MP_solver_->availableRoutes_.empty()) {
                updateAvailableRoutes(removedRequests, MP_solver_->availableRoutes_, mainInst);
            }
        }
        else if (EpochInst->parameters_->mainAlgorithm_ != GREEDY)  {
            MP_solver_->availableRoutes_.clear();
            MP_solver_->availableRoutes_.resize(EpochInst->nbVehicles_);
        }
        if (EpochInst->parameters_->mainAlgorithm_ != GREEDY) {
            MP_solver_->duplicatesRoutes_.clear();
            MP_solver_->duplicatesRoutes_.resize(EpochInst->nbVehicles_);
        }


        buildEpochInstance(mainInst, EpochInst, elapsedTime_, nbReceivedRequest);

        // saving the status in the middle of running
        if (middleSave && elapsedTime_ >= saveTime) {
            mainInst->saveStatus(inputPaths, EpochInst->simulationStartTime_ + elapsedTime_,
                mainInst->parameters_->epochLength_, std::to_string(instance_count));

            break;
            saveTime += 90;
            if (instance_count >= 30)
                break;
            instance_count ++;
        }

        if (EpochInst->nbRequests_ == 0) {
            if (EpochInst->parameters_->mainAlgorithm_ != GREEDY)
                MP_solver_->availableRoutes_.clear();
            simulationTime_->stop();
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
                solveEpoch(EpochInst, mainInst, inputPaths);
                break;
        }

        elapsedTime_ = static_cast<float>(epoch_+1) * mainInst->parameters_->epochLength_;
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
    rebalancingTime_->stop();
    if (!middleSave) {
        if (mainInst->parameters_->timeWindow_ > 0) {
            for (auto& requestObj : mainInst->requests_) {
                if (requestObj->requestStatus_ != COMPLETED)
                    requestObj->requestStatus_ = REJECTED;
            }
        }
    }
}

void BatchSolver::doSimulation(PInstance &mainInst, InputPaths &inputPaths, bool middleSave, float saveTime) {
    std::cout << "====================================================================="<< std::endl;
    std::cout << "                 Rolling Horizon with Batch of "<< mainInst->parameters_->epochLength_ << "(s)"<< std::endl;
    std::cout << "====================================================================="<< std::endl;
    BatchHorizon(mainInst, inputPaths, middleSave, saveTime);
}
