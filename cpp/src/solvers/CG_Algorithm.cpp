//
// Created by Elahe Amiri on 2025-06-26.
//

#include "CG_Algorithm.h"

#ifdef DARP_USE_CPLEX
#include "CplexSolver/MP_Cplex.h"
#endif
#ifdef DARP_USE_GUROBI
#include "GurobiSolver/MP_Gurobi.h"
#endif

CG_Algorithm::CG_Algorithm(const InputPaths &inputPaths) : MasterAlgorithm(inputPaths){
#ifdef DARP_USE_CPLEX
    MasterProblem_ = std::make_shared<MP_Cplex>();
#endif
#ifdef DARP_USE_GUROBI
    MasterProblem_ = std::make_shared<MP_Gurobi>(inputPaths.getOutputSolverLog());
#endif

    LPIter_ = 0;
    MIPIter_ = 0;
}

void CG_Algorithm::epochInitialization(PInstance &pInst, InputPaths &inputPaths, int epoch,
    const PGreedyModeler &GreedyModel) {
    initializations(pInst, inputPaths, epoch, GreedyModel);

    RMPCounter_++;
    MPnbRoutes_ = 0;
    lpObjValue_ = objValue_;

    if (availableRoutes_.empty())
        availableRoutes_.resize(pInst->nbVehicles_);
}

void CG_Algorithm::solveRMP_IP(const PInstance &pInst, int epoch, const InputPaths &inputPaths, float subProTime) {
    masterTime_->start();
    setObjValue();
    previousObj_ = objValue_;

    /************************************************************************************************/
    //                                     MASTER PROBLEM
    /************************************************************************************************/
    setAvailableTime();
    if (availableTime_ > 0) {
        // solve RP with MIP solver
        if (availableTime_ < 1)
            availableTime_ = 2;
        // solve the model in Integer mode
        int nbColumns = 0;
        MasterProblem_->solveModelInt(pInst, zSolution_, routeSolution_, inputPaths, availableTime_, previousObj_);
        MPEpochSolveTime_ += MasterProblem_->solveTime_->dSinceStart().count();
        nbColumns = MasterProblem_->compRoutes_.size();

        setCurrentRoutes(pInst);
        MIPIter_++;
        setObjValue();
        epochTime_ += masterTime_->dSinceStart().count();

        (*pLogMPResultsStream_) << save_MPResults(epoch, "CG", nbColumns,
            masterTime_->dSinceStart().count(), subProTime, 0.0);
        RMPCounter_++;
    }
    masterTime_->stop();
}

void CG_Algorithm::solveRMP_LP(PInstance &pInst, int epoch, const InputPaths &inputPaths, float subProTime) {
    masterTime_->start();
    iterTime_ = masterTime_->dSinceStart().count();
    if (SPIter_ == 0)
        lpObjValue_ = objValue_;
    /************************************************************************************************/
    //                                     MASTER PROBLEM
    /************************************************************************************************/
    setAvailableTime();
    if (availableTime_ > 1) {
        solveMP_LP(pInst, inputPaths, epoch, subProTime);

    }
    masterTime_->stop();
}

void CG_Algorithm::solve(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime) {
    timeLimit_ = availableTime_;
    epochTime_ += subProTime;
    if (pInst->parameters_->mainAlgorithm_ == RT_CG)
        solveRMP_LP(pInst, epoch, inputPaths, subProTime);
    else if (pInst->parameters_->mainAlgorithm_ == A_CG)
        solveMP_CG(pInst, epoch, inputPaths, subProTime);
}

void CG_Algorithm::getIPSolution(const PInstance &pInst, int epoch, const InputPaths &inputPaths, float subProTime) {
    setObjValue();

    if (pInst->parameters_->mainAlgorithm_ == RT_CG) {
        timeLimit_ = std::max(availableTime_, 10.0f);

        solveRMP_IP(pInst, epoch, inputPaths, subProTime);
    }
}

bool CG_Algorithm::shouldTerminate(const PInstance &pInst, float previousObj, float previousLpObj, int iter) {
    if (pInst->parameters_->numIter_ == iter){
        return true;
    }

    if (pInst->parameters_->mainAlgorithm_ == A_CG){
        if (previousObj == objValue_) {
            CGSuccess_++;
            std::cout << "No changes in Objective" << std::endl;
            return true;
        }
    }
    else if (pInst->parameters_->mainAlgorithm_ == RT_CG){
        if (previousLpObj == lpObjValue_) {
            std::cout << "No changes in LP Objective" << std::endl;
            return true;
        }
    }
    return false;
}

void CG_Algorithm::resetModels() {
    if (MasterProblem_)
        MasterProblem_->resetForNextIteration();
}

void CG_Algorithm::initializations(PInstance &pInst, InputPaths &inputPaths, int epoch,
    const PGreedyModeler &GreedyModel) {
    initialization(pInst, inputPaths, GreedyModel);
    previousObj_ = objValue_;
    masterTime_->start();

    for (auto & vehicleObj : pInst->vehicles_) {
        if (vehicleObj->currentRoute_->routeSize_ != vehicleObj->emptyRoute_->routeSize_)
            MasterProblem_->routesToAdd_.push_back(vehicleObj->emptyRoute_);
    }

    // build MP model
    MPBuildTime_->start();
    MasterProblem_->buildModelMP(pInst, routeSolution_, nbVehicles_);
    MPBuildTime_->stop();
    MasterProblem_->routesToAdd_.clear();

    // set initial duals
    setInitialDuals(pInst, inputPaths, epoch);
    if (pInst->parameters_->initialDual_ == GREEDY_D) {
        routeSolution_.clear();
        for (auto & vehicleObj : pInst->vehicles_) {
            if (vehicleObj->currentRoute_->routeSize_ != vehicleObj->greedyRoute_->routeSize_)
                MasterProblem_->routesToAdd_.push_back(vehicleObj->greedyRoute_);

            routeSolution_.push_back(vehicleObj->greedyRoute_);
        }
        setCurrentRoutes(pInst);
        MasterProblem_->updateModel(pInst);
        MasterProblem_->routesToAdd_.clear();
        for (auto & vehicleObj : pInst->vehicles_) {
            vehicleObj->currentRoute_->calcMarginalCosts(pInst->parameters_->Wait_W1_, pInst->parameters_->Ride_W2_);
            vehicleObj->dual_ = 0;
        }

    }

    if (pInst->parameters_->routeRecycle_ && !availableRoutes_.empty()) {
        reFillRoutesToAdd(pInst, MasterProblem_->routesToAdd_);
        MasterProblem_->updateModel(pInst);
        MasterProblem_->routesToAdd_.clear();
    }
    if (pInst->parameters_->smoothDual_) {
        for (auto & requestObj : pInst->requests_) {
            requestObj->dual_ = 0.5 * requestObj->dual_ + 0.5 * requestObj->lastDual_;
            requestObj->lastDual_ = requestObj->dual_;
        }
        for (auto & vehicleObj : pInst->vehicles_)
            vehicleObj->dual_ = 0;
    }
    for (auto & requestObj : pInst->requests_) {
        requestObj->setMaxMinDual();
    }
    calcDualsStatistics(pInst);
    setObjValue();
    previousObj_ = objValue_;
    masterTime_->stop();
}


void CG_Algorithm::solveMP_LP(PInstance &pInst, const InputPaths &inputPaths, int epoch, float subProTime) {

    // solve RP with MIP solver
    while (true) {
        setAvailableTime();
        if (availableTime_ <= 1)
            break;

        MasterProblem_->routesToAdd_.clear();

        // select routes with negative reduced costs
        updateRoutesToAdd(NR, pInst, MasterProblem_->routesToAdd_);

        if (!MasterProblem_->routesToAdd_.empty()){
            MPBuildTime_->start();
            MasterProblem_->updateModel(pInst);
            MPBuildTime_->stop();
            MasterProblem_->solveModelLP(pInst, inputPaths);
            MPEpochSolveTime_ += MasterProblem_->solveTime_->dSinceStart().count();

            LPIter_++;
            epochTime_ += (masterTime_->dSinceStart().count() - iterTime_);
            iterTime_ = masterTime_->dSinceStart().count();
            objValue_ = MasterProblem_->objValue_;

            *pLogMPResultsStream_ << save_MPResults(epoch, "LMP", static_cast<int>(MasterProblem_->compRoutes_.size()),
                                                    masterTime_->dSinceStart().count(), subProTime, 0.0);

            RMPCounter_++;

            if (lpObjValue_ - MasterProblem_->objValue_ > 0.01) {
                lpObjValue_ = MasterProblem_->objValue_;
            } else
                break;
 //           if (LPIter_ == 2)
 //               break;
        }
        else
            break;
    }
    MasterProblem_->markColumnsToKeep();

}

void CG_Algorithm::solveMP_CG(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime) {
    masterTime_->start();
    previousObj_ = objValue_;

    if (SPIter_ == 0)
        lpObjValue_ = objValue_;

    iterTime_ = masterTime_->dSinceStart().count();

    /************************************************************************************************/
    //                                     MASTER PROBLEM
    /************************************************************************************************/
    setAvailableTime();
    if (pInst->parameters_->sortColumn_ == COMP_C) {
        updateScore(pInst);
    }
    if (availableTime_ > 1) {
        solveMP_LP(pInst, inputPaths, epoch, subProTime);
        objValue_ = previousObj_;

        setAvailableTime();
        if (availableTime_ < 3)
            availableTime_ = 3;

        MasterProblem_->solveModelInt(pInst, zSolution_, routeSolution_, inputPaths,
                                     availableTime_, previousObj_);
        setCurrentRoutes(pInst);
        MIPIter_++;
        MPEpochSolveTime_ += MasterProblem_->solveTime_->dSinceStart().count();
        setObjValue();

        epochTime_ += (masterTime_->dSinceStart().count() - iterTime_);
        (*pLogMPResultsStream_) << save_MPResults(epoch, "CG", static_cast<int>(MasterProblem_->compRoutes_.size()),
                                                        masterTime_->dSinceStart().count(), subProTime, 0.0);


        RMPCounter_++;

    }
    masterTime_->stop();
}
