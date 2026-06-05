//
// Created by Elahe Amiri on 2025-11-02.
//

#include "OfflineSolver.h"

#include "GreedyModeler.h"

void OfflineSolver::staticSolver(PInstance &mainInst, InputPaths &inputPaths, bool middleSave, float saveTime) {
    // define required variables
    epoch_ = 0;

    // start simulation timer
    mainInst->setInitialTimes(mainInst->parameters_->epochLength_);
    for (int v = 0; v < mainInst->nbVehicles_; ++v) {
        mainInst->vehicles_[v]->setEmptyRoute(mainInst);
        mainInst->vehicles_[v]->setSolutionRoute();
        if (mainInst->vehicles_[v]->currentRoute_ == nullptr)
            mainInst->vehicles_[v]->setCurrentRoute(mainInst->vehicles_[v]->emptyRoute_);
    }

    simulationTime_->start();

    int nbReceivedRequest = mainInst->nbOnboards_;
    PInstance StaticInst = std::make_shared<Instance>(*mainInst);
    StaticInst->buildStaticData(mainInst, nbReceivedRequest);

    if (MP_solver_) {
        MP_solver_->duplicatesRoutes_.clear();
        MP_solver_->duplicatesRoutes_.resize(StaticInst->nbVehicles_);
    }

    for (auto &vehicleObj: StaticInst->vehicles_){
        if (StaticInst->nbRequests_ != 0) {
            vehicleObj->currentRoute_->createColumn(StaticInst->nbRequests_);
            vehicleObj->emptyRoute_->createColumn(StaticInst->nbRequests_);
        }
    }

    if (StaticInst->parameters_->timeWindow_ == 0)
        StaticInst->updatePenalties(0);

//    preprocessTime_->stop();
    switch(StaticInst->parameters_->mainAlgorithm_) {
        case MIP:
#if defined(DARP_USE_CPLEX)
            MIPModel_ = std::make_unique<MIPSolver_Cplex>();
#elif defined(DARP_USE_GUROBI)
            MIPModel_ = std::make_unique<MIPSolver_Gurobi>();
#endif
            MIPModel_->setTimeLimit(static_cast<float>(StaticInst->parameters_->solveTimeLimit_));
            MIPSolveTime_->start();
            MIPModel_->SolveMIP(StaticInst, inputPaths);
            MIPSolveTime_->stop();
            totalMIPSolveTime_ += MIPSolveTime_->dSinceStart().count();
            *pLogRunTimesStream_ << saveRuntimesMIP(StaticInst);
            MIPModel_.reset();
            break;
        case GREEDY:
            GreedyModel_->GreedySolver(StaticInst);
            std::cout << "# FINAL SOLUTION OF Greedy solution : " << GreedyModel_->objValue_<< std::endl;
            for (auto & vehicleObj : StaticInst->vehicles_)
                vehicleObj->solutionRoute_ = vehicleObj->currentRoute_;
            std::cout << "# TIME SPENT ON GREEDY " << "=" << GreedyModel_->greedyTime_->dSinceInit().count() << " (seconds)" << std::endl;
            break;
        default: // CG_CPLEX and CG_ISUD
            solveEpoch(StaticInst, mainInst, inputPaths);
            break;
    }
    StaticInst->setAssignedEpochVehicles(simulationTime_->dSinceInit().count());
    simulationTime_->stop();
    if (!middleSave && StaticInst->parameters_->mainAlgorithm_ != GREEDY) {
        for (auto & vehicleObj : mainInst->vehicles_) {
            vehicleObj->departNode_->nodeDepartTime_ = vehicleObj->currentRoute_->plannedDepartTime_[0];
            vehicleObj->finalizeSolutionRoutes(mainInst->simulationStartTime_ + saveTime);
            vehicleObj->solutionRoute_->calculateTripDelay(mainInst->parameters_->Wait_W1_,mainInst->parameters_->Ride_W2_);
        }
    }
    // Test the solution route
    for (auto& vehicleObj : mainInst->vehicles_)
        vehicleObj->solutionRoute_->testRoute(vehicleObj);
}

void OfflineSolver::doSimulation(PInstance &mainInst, InputPaths &inputPaths, bool middleSave, float saveTime) {
    std::cout << "====================================================================="<< std::endl;
    std::cout << "               Solving Static DARP (Offline mode)" << std::endl;
    std::cout << "====================================================================="<< std::endl;
    staticSolver(mainInst, inputPaths, middleSave, saveTime);
}
