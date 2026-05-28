//
// Created by Elahe Amiri on 2025-06-26.
//
// Solver-agnostic ISUD algorithm.  Backend objects are created once in the
// constructor via the PCPSolver / PReducedPro aliases defined in Types.h.
//

#include "ISUD_Algorithm.h"

#ifdef DARP_USE_CPLEX
#include "CplexSolver/CP_Cplex.h"
#include "CplexSolver/RP_Cplex.h"
#endif
#ifdef DARP_USE_GUROBI
#include "GurobiSolver/CPModeler.h"
#include "GurobiSolver/RP_Gurobi.h"
#endif

ISUD_Algorithm::ISUD_Algorithm(const InputPaths &inputPaths) : MasterAlgorithm(inputPaths) {
#ifdef DARP_USE_CPLEX
    ReducedPro_ = std::make_shared<RP_Cplex>();
    CPSolver_   = std::make_shared<CP_Cplex>();
#endif
#ifdef DARP_USE_GUROBI
    ReducedPro_ = std::make_shared<RP_Gurobi>(inputPaths.getOutputSolverLog());
    CPSolver_   = std::make_shared<CPModeler>(inputPaths.getOutputSolverLog());
#endif

    CPBuilt_ = false;
    CPEpochSolveTime_ = 0;

    RPIter_ = 0;
    CPIter_ = 0;

    maxIncDegree_ = 0;
    ZoomIter_ = 0;
    CPSuccess_ = 0;
    CPFails_ = 0;
}

//-----------------------------------------------------------------------------
void ISUD_Algorithm::initializations(PInstance &pInst, InputPaths &inputPaths, int epoch,
                                     const PGreedyModeler &GreedyModel) {
    CPEpochSolveTime_ = 0;
    maxIncDegree_ = pInst->parameters_->CP_IncDegree_;
    CPBuilt_ = false;

    initialization(pInst, inputPaths, GreedyModel);
    masterTime_->start();

    CPSolver_->initializeCP(pInst, pInst->parameters_->reducedCP_);

    for (auto &vehicleObj : pInst->vehicles_) {
        if (vehicleObj->currentRoute_->routeSize_ != vehicleObj->emptyRoute_->routeSize_)
            ReducedPro_->routesToAdd_.push_back(vehicleObj->emptyRoute_);
    }
    MPBuildTime_->start();
    ReducedPro_->buildModelRP(pInst, routeSolution_, nbVehicles_);
    MPBuildTime_->stop();

    setInitialDuals(pInst, inputPaths, epoch);

    if (pInst->parameters_->initialDual_ == GREEDY_D) {
        for (auto &requestObj : zSolution_) {
            if (requestObj->dual_ == 0)
                requestObj->dual_ = requestObj->penalty_;
        }
        for (auto &vehicleObj : pInst->vehicles_)
            vehicleObj->dual_ = 0;
        for (auto &vehicleObj : pInst->vehicles_) {
            if (vehicleObj->currentRoute_->routeSize_ != vehicleObj->greedyRoute_->routeSize_)
                ReducedPro_->routesToAdd_.push_back(vehicleObj->greedyRoute_);
        }
        ReducedPro_->updateRPModel(pInst);
        ReducedPro_->routesToAdd_.clear();
    }
    if (pInst->parameters_->routeRecycle_ && !availableRoutes_.empty()) {
        reFillRoutesToAdd(pInst, ReducedPro_->routesToAdd_);
        ReducedPro_->updateRPModel(pInst);
        ReducedPro_->routesToAdd_.clear();
    }
    if (pInst->parameters_->smoothDual_) {
        for (auto &requestObj : pInst->requests_) {
            requestObj->dual_ = 0.5f * requestObj->dual_ + 0.5f * requestObj->lastDual_;
            requestObj->lastDual_ = requestObj->dual_;
        }
        for (auto &vehicleObj : pInst->vehicles_)
            vehicleObj->dual_ = 0;
    }
    for (auto &requestObj : pInst->requests_)
        requestObj->setMaxMinDual();
    calcDualsStatistics(pInst);
    setObjValue();
    previousObj_ = objValue_;
    masterTime_->stop();
}

void ISUD_Algorithm::epochInitialization(PInstance &pInst, InputPaths &inputPaths, int epoch,
                                         const PGreedyModeler &GreedyModel) {
    initializations(pInst, inputPaths, epoch, GreedyModel);
    RMPCounter_++;
    MPnbRoutes_ = 0;
    lpObjValue_ = objValue_;

    if (availableRoutes_.empty())
        availableRoutes_.resize(pInst->nbVehicles_);
}

//-----------------------------------------------------------------------------
// Reduced Problem solve loop (common)
//-----------------------------------------------------------------------------
void ISUD_Algorithm::solveRP(PInstance &pInst, const InputPaths &inputPaths, int epoch, float subProTime) {
    RPTime_->start();
    iterTime_ = masterTime_->dSinceStart().count();
    int nbColumns = 0;
    while (true) {
        setAvailableTime();
        if (availableTime_ <= 1)
            break;
        CPSolver_->fractionalZ_.clear();
        nbColumns = solveReducedPro(pInst, 0, inputPaths);
        RPIter_++;
        epochTime_ += masterTime_->dSinceStart().count() - iterTime_;
        iterTime_ = masterTime_->dSinceStart().count();

        *pLogMPResultsStream_ << save_MPResults(epoch, "RP", nbColumns,
            masterTime_->dSinceStart().count(), subProTime, 0.0);
        RMPCounter_++;

        if (previousObj_ - objValue_ > 0.01)
            previousObj_ = objValue_;
        else
            break;

        if (nbColumns == 0)
            break;
    }
    RPTime_->stop();
}

//-----------------------------------------------------------------------------
// solveISUD_MIP  (MIP RP + outer restart loop)
//-----------------------------------------------------------------------------
void ISUD_Algorithm::solveISUD_MIP(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime) {
    try {
        masterTime_->start();
        setObjValue();
        previousObj_ = objValue_;
        bool restartAlgorithm = true;

        while (restartAlgorithm) {
            bool isCPImproved = true;

            //-----------------------------------------------------------------
            // Reduced Problem
            //-----------------------------------------------------------------
            solveRP(pInst, inputPaths, epoch, subProTime);

            //-----------------------------------------------------------------
            // Complementary Problem
            //-----------------------------------------------------------------
            CPTime_->start();
            setAvailableTime();

            if (minReducedCost_ > 0 || availableTime_ < 0.2) {
                restartAlgorithm = false;
                isCPImproved = false;
            }
            iterTime_ = masterTime_->dSinceStart().count();

            if (!pInst->parameters_->reducedCP_ && isCPImproved) {
                CPBuildTime_->start();
                CPSolver_->resetForNextIteration();
                CPSolver_->initializeCP(pInst, false);
                CPSolver_->buildModel_batch(pInst, routeSolution_);
                CPBuildTime_->stop();
            }

            while (isCPImproved) {
                isCPImproved = false;
                previousObj_ = objValue_;
                CPSolver_->routesToAdd_.clear();
                updateRoutesToAdd(CP, pInst, CPSolver_->routesToAdd_);
                for (auto &vehicleObj : pInst->vehicles_) {
                    if (!vehicleObj->currentRoute_->routeRequests_.empty())
                        CPSolver_->routesToAdd_.push_back(vehicleObj->emptyRoute_);
                }
                setAvailableTime();

                if (!CPSolver_->routesToAdd_.empty() && availableTime_ > 0.2) {
                    if (pInst->parameters_->reducedCP_) {
                        CPBuildTime_->start();
                        CPSolver_->resetForNextIteration();
                        CPSolver_->initializeCP(pInst, true);
                        CPSolver_->buildModel_batch(pInst);
                        CPBuildTime_->stop();
                        CPSolver_->solveCPModel(pInst, zSolution_, routeSolution_, inputPaths);
                    } else {
                        CPBuildTime_->start();
                        CPSolver_->updateModel(pInst);
                        CPBuildTime_->stop();
                        CPSolver_->solveCPModel(pInst, zSolution_, routeSolution_, inputPaths, false);
                        setCurrentRoutes(pInst);
                    }

                    CPIter_++;
                    (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, RMPCounter_, "Dual");
                    CPEpochSolveTime_ += CPSolver_->solveTime_->dSinceStart().count();
                    setObjValue();
                    epochTime_ += masterTime_->dSinceStart().count() - iterTime_;
                    iterTime_ = masterTime_->dSinceStart().count();
                    (*pLogMPResultsStream_) << save_MPResults(epoch, "CP",
                        static_cast<int>(CPSolver_->IncRoute_.size()),
                        masterTime_->dSinceStart().count(), subProTime, 0.0);

                    if (CPSolver_->status_ == FRACTIONAL) {
                        CPFails_++;
                        setAvailableTime();
                        if (pInst->parameters_->useZoom_ && availableTime_ > 0.2) {
                            iterTime_ = masterTime_->dSinceStart().count();
                            ZOOMTime_->start();
                            solveReducedPro(pInst, 1, inputPaths);
                            ZoomIter_++;
                            epochTime_ += masterTime_->dSinceStart().count() - iterTime_;
                            iterTime_ = masterTime_->dSinceStart().count();
                            (*pLogMPResultsStream_) << save_MPResults(epoch, "ZOOM",
                                static_cast<int>(ReducedPro_->compRoutes_.size()) - nbVehicles_,
                                masterTime_->dSinceStart().count(), subProTime, 0.0);
                            RMPCounter_++;
                            ZOOMTime_->stop();
                            if (previousObj_ - objValue_ > 0.01) {
                                previousObj_ = objValue_;
                            } else if (!isCPImproved) {
                                restartAlgorithm = false;
                                break;
                            }
                        } else if (!isCPImproved) {
                            restartAlgorithm = false;
                            break;
                        }
                    } else if ((CPSolver_->status_ == POSITIVE_VALUE || CPSolver_->status_ == INFEASIBLE)
                               && !isCPImproved) {
                        restartAlgorithm = false;
                        break;
                    } else {
                        CPSuccess_++;
                        previousObj_ = objValue_;
                        RMPCounter_++;
                        isCPImproved = true;
                        setAvailableTime();
                        restartAlgorithm = false;
                        if (availableTime_ <= 0.2) {
                            restartAlgorithm = false;
                            break;
                        }
                    }
                } else if (!isCPImproved) {
                    restartAlgorithm = false;
                    break;
                }
            }

            setAvailableTime();
            updateReducedCosts(pInst);
            if (minReducedCost_ > 0 || availableTime_ <= 0)
                restartAlgorithm = false;
            CPSolver_->resetForNextIteration();
            if (restartAlgorithm)
                CPSolver_->initializeCP(pInst, pInst->parameters_->reducedCP_);
            CPBuilt_ = false;
            CPTime_->stop();
        }

        (*pLogMPResultsStream_) << save_MPResults(epoch, "ISUD", MPnbRoutes_,
            masterTime_->dSinceStart().count(), subProTime, 0.0);
        masterTime_->stop();
    }
    catch (const std::exception &e) {
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e.what() << std::endl;
    }
}

//-----------------------------------------------------------------------------
// solveISUD_Pivot  (greedy pivot RP + single CP pass with warm-start model)
//-----------------------------------------------------------------------------
void ISUD_Algorithm::solveISUD_Pivot(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime) {
    try {
        masterTime_->start();
        setObjValue();
        previousObj_ = objValue_;

        bool isCPImproved = true;

        //-----------------------------------------------------------------
        // Reduced Problem (greedy pivot)
        //-----------------------------------------------------------------
        RPTime_->start();
        iterTime_ = masterTime_->dSinceStart().count();
        int nbColumns = solveRP_Pivot(pInst, 0, inputPaths);
        RPIter_++;
        epochTime_ += masterTime_->dSinceStart().count() - iterTime_;
        iterTime_ = masterTime_->dSinceStart().count();
        *pLogMPResultsStream_ << save_MPResults(epoch, "RP", nbColumns,
            masterTime_->dSinceStart().count(), subProTime, 0.0);
        RMPCounter_++;
        previousObj_ = objValue_;
        RPTime_->stop();

        //-----------------------------------------------------------------
        // Complementary Problem (single pass, in-place pivot update)
        //-----------------------------------------------------------------
        CPTime_->start();
        setAvailableTime();
        if (availableTime_ < 0.2)
            isCPImproved = false;

        iterTime_ = masterTime_->dSinceStart().count();

        CPBuildTime_->start();
        CPSolver_->resetForNextIteration();
        CPSolver_->initializeCP(pInst, pInst->parameters_->reducedCP_);
        CPSolver_->buildModel_batch(pInst, routeSolution_);

        CPSolver_->routesToAdd_.clear();
        updateRoutesToAdd(CP, pInst, CPSolver_->routesToAdd_);
        for (auto &vehicleObj : pInst->vehicles_) {
            if (!vehicleObj->currentRoute_->routeRequests_.empty())
                CPSolver_->routesToAdd_.push_back(vehicleObj->emptyRoute_);
        }
        CPSolver_->updateModel(pInst);
        CPBuildTime_->stop();

        while (isCPImproved) {
            isCPImproved = false;
            previousObj_ = objValue_;
            setAvailableTime();

            if (availableTime_ > 0.2) {
                CPSolver_->solveCPModel(pInst, zSolution_, routeSolution_, inputPaths, true);
                setCurrentRoutes(pInst);
                CPIter_++;
                CPEpochSolveTime_ += CPSolver_->solveTime_->dSinceStart().count();
                setObjValue();
                epochTime_ += masterTime_->dSinceStart().count() - iterTime_;
                iterTime_ = masterTime_->dSinceStart().count();
                (*pLogMPResultsStream_) << save_MPResults(epoch, "CP",
                    static_cast<int>(CPSolver_->IncRoute_.size()),
                    masterTime_->dSinceStart().count(), subProTime, 0.0);

                if (CPSolver_->status_ == FRACTIONAL) {
                    CPFails_++;
                    setAvailableTime();
                    if (pInst->parameters_->useZoom_ && availableTime_ > 0.2) {
                        iterTime_ = masterTime_->dSinceStart().count();
                        ZOOMTime_->start();
                        solveReducedPro(pInst, 1, inputPaths);
                        ZoomIter_++;
                        epochTime_ += masterTime_->dSinceStart().count() - iterTime_;
                        iterTime_ = masterTime_->dSinceStart().count();
                        (*pLogMPResultsStream_) << save_MPResults(epoch, "ZOOM",
                            static_cast<int>(ReducedPro_->compRoutes_.size()) - nbVehicles_,
                            masterTime_->dSinceStart().count(), subProTime, 0.0);
                        RMPCounter_++;
                        ZOOMTime_->stop();
                        if (previousObj_ - objValue_ > 0.01)
                            previousObj_ = objValue_;
                        break;
                    }
                } else if (CPSolver_->status_ == POSITIVE_VALUE || CPSolver_->status_ == INFEASIBLE) {
                    break;
                } else {
                    CPSuccess_++;
                    previousObj_ = objValue_;
                    RMPCounter_++;
                    isCPImproved = true;
                    setAvailableTime();
                }
            } else {
                break;
            }
        }

        CPBuilt_ = false;
        CPTime_->stop();

        (*pLogMPResultsStream_) << save_MPResults(epoch, "ISUD", MPnbRoutes_,
            masterTime_->dSinceStart().count(), subProTime, 0.0);
        masterTime_->stop();
    }
    catch (const std::exception &e) {
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e.what() << std::endl;
    }
}

void ISUD_Algorithm::solve(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime) {
    timeLimit_ = availableTime_;
    epochTime_ += subProTime;
    if (pInst->parameters_->isudVariant_ == ISUD_PIVOT_RP)
        solveISUD_Pivot(pInst, epoch, inputPaths, subProTime);
    else
        solveISUD_MIP(pInst, epoch, inputPaths, subProTime);
}

void ISUD_Algorithm::resetModels() {
    if (ReducedPro_)
        ReducedPro_->resetForNextIteration();
    if (CPSolver_)
        CPSolver_->resetForNextIteration();
}

bool ISUD_Algorithm::shouldTerminate(const PInstance &pInst, float previousObj, float previousLpObj, int iter) {
    if (pInst->parameters_->numIter_ == iter)
        return true;
    if (previousObj == objValue_) {
        CGSuccess_++;
        std::cout << "No changes in Objective" << std::endl;
        return true;
    }
    return false;
}

void ISUD_Algorithm::getIPSolution(const PInstance &pInst, int epoch, const InputPaths &inputPaths, float subProTime) {
    setObjValue();
}

int ISUD_Algorithm::solveReducedPro(PInstance &pInst, int compDegree, const InputPaths &inputPaths) {
    ReducedPro_->routesToAdd_.clear();
    if (compDegree == 0)
        updateRoutesToAdd(RP, pInst, ReducedPro_->routesToAdd_);
    else
        updateRoutesToAddZoom(ReducedPro_->routesToAdd_);

    if (!ReducedPro_->routesToAdd_.empty()) {
        MPBuildTime_->start();
        ReducedPro_->updateRPModel(pInst);
        MPBuildTime_->stop();
        ReducedPro_->solveModelRelaxInt(pInst, zSolution_, routeSolution_, inputPaths,
                                        availableTime_, objValue_);
        MPEpochSolveTime_ += ReducedPro_->solveTime_->dSinceStart().count();
        setCurrentRoutes(pInst);
        setObjValue();
    }
    return static_cast<int>(ReducedPro_->compRoutes_.size());
}

int ISUD_Algorithm::solveRP_Pivot(PInstance &pInst, int compDegree, const InputPaths &inputPaths) {
    ReducedPro_->routesToAdd_.clear();
    updateRoutesToAdd(RP, pInst, ReducedPro_->routesToAdd_);
    std::stable_sort(ReducedPro_->routesToAdd_.begin(), ReducedPro_->routesToAdd_.end(),
                     [](const PRoute &lhs, const PRoute &rhs) { return lhs->normal_RC_ < rhs->normal_RC_; });

    if (!ReducedPro_->routesToAdd_.empty()) {
        MPBuildTime_->start();
        boost::dynamic_bitset<> coveredRequests(pInst->nbRequests_);
        boost::dynamic_bitset<> usedVehicles(pInst->nbVehicles_);
        for (auto &routeObj : ReducedPro_->routesToAdd_) {
            if (!usedVehicles.test(routeObj->vehicleID_)) {
                bool isStillCompatible = true;
                for (auto &reqObj : routeObj->routeRequests_) {
                    if (coveredRequests.test(reqObj->taskIndex_)) {
                        isStillCompatible = false;
                        break;
                    }
                }
                if (isStillCompatible) {
                    usedVehicles.set(routeObj->vehicleID_);
                    coveredRequests |= routeObj->column_;
                    pInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
                }
            }
        }
        routeSolution_.clear();
        for (auto &veh : pInst->vehicles_)
            routeSolution_.push_back(veh->currentRoute_);

        setCurrentRoutes(pInst);
        setObjValue();
        MPBuildTime_->stop();
    }
    return static_cast<int>(ReducedPro_->compRoutes_.size());
}
