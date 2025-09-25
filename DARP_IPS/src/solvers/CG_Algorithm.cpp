//
// Created by Elahe Amiri on 2025-06-26.
//

#include "CG_Algorithm.h"

#include "CplexSolver/ComplementPro.h"
#include "CplexSolver/MIPMasterProblem.h"
#include "GurobiSolver/CP_Gurobi.h"
#include "GurobiSolver/MP_Gurobi.h"

CG_Algorithm::CG_Algorithm(const InputPaths &inputPaths, ModelSOLVER modelSolver) : MasterAlgorithm(inputPaths){
    if (modelSolver == CPLEX) {
        MasterPro_ = std::make_shared<MIPMasterProblem>();
    }
    else if (modelSolver == GUROBI) {
        MPGurobiPro_ = std::make_shared<MP_Gurobi>(inputPaths.getOutputSolverLog());
    }

    LPIter_ = 0;
    MIPIter_ = 0;
}

void CG_Algorithm::initializationCPLEX(PInstance &pInst, InputPaths &inputPaths, int epoch,
    const PGreedyModeler &GreedyModel) {

    initialization(pInst, inputPaths, GreedyModel);
    masterTime_->start();

    // build the model
    MasterPro_ = std::make_shared<MIPMasterProblem>();
    MasterPro_->routesToAdd_.clear();
    if (pInst->parameters_->dualMethod_ == AUX_D)
        DualAuxSolver_ = std::make_shared<DualAuxSolver>(pInst->nbTasks_, nbVehicles_);

    for (auto & vehicleObj : pInst->vehicles_) {
        if (vehicleObj->currentRoute_->routeSize_ != vehicleObj->emptyRoute_->routeSize_)
            MasterPro_->routesToAdd_.push_back(vehicleObj->emptyRoute_);
    }

    MPBuildTime_->start();
    MasterPro_->buildModelMP(pInst, routeSolution_, nbVehicles_);
    MPBuildTime_->stop();

    setInitialDuals(pInst, inputPaths, epoch);
    if (pInst->parameters_->initialDual_ == GREEDY_D) {
        for (auto & vehicleObj : pInst->vehicles_)
            MasterPro_->routesToAdd_.push_back(vehicleObj->greedyRoute_);
        MasterPro_->updateModel(pInst);
        for (auto & requestObj : zSolution_) {
            requestObj->dual_ = 0.5 * requestObj->marginalCost_ + 0.5 * requestObj->penalty_;
        }
 //       for (auto & vehicleObj : pInst->vehicles_)
 //           vehicleObj->dual_ = 0;
    }
    else if (availableRoutes_.size() > 0 && pInst->parameters_->routeRecycle_ &&
        (pInst->parameters_->initialDual_ == BARRIER || pInst->parameters_->initialDual_ == INITIAL_LP)){
        reFillRoutesToAdd(pInst, MasterPro_->routesToAdd_);
        MasterPro_->updateModel(pInst);
        MasterPro_->solveLPDual(pInst, inputPaths);
    }

    setObjValue();
    previousObj_ = objValue_;
    masterTime_->stop();
}

void CG_Algorithm::initializationGurobi(PInstance &pInst, InputPaths &inputPaths, int epoch,
    const PGreedyModeler &GreedyModel) {
    initialization(pInst, inputPaths, GreedyModel);
    masterTime_->start();

    // build the model
    MPGurobiPro_ = std::make_shared<MP_Gurobi>(inputPaths.getOutputSolverLog());
    MPGurobiPro_->routesToAdd_.clear();
    if (pInst->parameters_->dualMethod_ == AUX_D)
        DualAuxSolver_ = std::make_shared<DualAuxSolver>(pInst->nbTasks_, nbVehicles_);

    for (auto & vehicleObj : pInst->vehicles_) {
        if (vehicleObj->currentRoute_->routeSize_ != vehicleObj->emptyRoute_->routeSize_)
            MPGurobiPro_->routesToAdd_.push_back(vehicleObj->emptyRoute_);
    }

    MPBuildTime_->start();
    MPGurobiPro_->buildModelMP(pInst, routeSolution_, nbVehicles_);
    MPBuildTime_->stop();
    setInitialDuals(pInst, inputPaths, epoch);
    if (pInst->parameters_->initialDual_ == GREEDY_D) {
 //       if (epoch == 1) {
            for (auto & requestObj : zSolution_) {
                if (requestObj->dual_ == 0)
                    requestObj->dual_ = requestObj->penalty_;

            }
 //       }
        for (auto & vehicleObj : pInst->vehicles_)
            vehicleObj->dual_ = 0;
        for (auto & vehicleObj : pInst->vehicles_) {
            if (vehicleObj->currentRoute_->routeSize_ != vehicleObj->greedyRoute_->routeSize_)
                MPGurobiPro_->routesToAdd_.push_back(vehicleObj->greedyRoute_);
        }
        MPGurobiPro_->updateModel(pInst);
        MPGurobiPro_->routesToAdd_.clear();
    }
    if (availableRoutes_.size() > 0 && pInst->parameters_->routeRecycle_ &&
        (pInst->parameters_->initialDual_ == BARRIER || pInst->parameters_->initialDual_ == INITIAL_LP)) {
        MPGurobiPro_->routesToAdd_.clear();
        reFillRoutesToAdd(pInst, MPGurobiPro_->routesToAdd_);
        /*for (auto & vehicleObj : pInst->vehicles_)
            MPGurobiPro_->routesToAdd_.push_back(vehicleObj->greedyRoute_);*/

        MPGurobiPro_->updateModel(pInst);
        MPGurobiPro_->solveLPDual(pInst, inputPaths);
        /*for (auto & requestObj : pInst->requests_) {
            if (requestObj->dual_ == 0)
                requestObj->dual_ = requestObj->penalty_;
            else
                requestObj->dual_ = 0.8 * requestObj->dual_ + 0.2 * requestObj->penalty_;
        }*/
 //       for (auto & vehicleObj : pInst->vehicles_)
 //           vehicleObj->dual_ = 0;
 //       resetMPGurobi(pInst, inputPaths);
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
    setObjValue();
    previousObj_ = objValue_;
    masterTime_->stop();
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
        int nbColumns;
        if (pInst->parameters_->modelSolver_ == GUROBI) {
            MPGurobiPro_->solveModelInt(pInst, zSolution_, routeSolution_, inputPaths, availableTime_, previousObj_);
            MPEpochSolveTime_ += MPGurobiPro_->solveTime_->dSinceStart().count();
            nbColumns = MPGurobiPro_->compRoutes_.size();
        }
        else if (pInst->parameters_->modelSolver_ == CPLEX) {
            MasterPro_->solveModelInt(pInst, zSolution_, routeSolution_, inputPaths, availableTime_, previousObj_);
            MPEpochSolveTime_ += MasterPro_->solveTime_->dSinceStart().count();
            nbColumns = MasterPro_->compRoutes_.size();
        }

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

void CG_Algorithm::resetMPGurobi(PInstance &pInst, const InputPaths &inputPaths) {
    // build the model
    MPGurobiPro_.reset();
    MPGurobiPro_ = std::make_shared<MP_Gurobi>(inputPaths.getOutputSolverLog());
    MPGurobiPro_->routesToAdd_.clear();

    for (auto & vehicleObj : pInst->vehicles_) {
        if (vehicleObj->currentRoute_->routeSize_ != vehicleObj->emptyRoute_->routeSize_)
            MPGurobiPro_->routesToAdd_.push_back(vehicleObj->emptyRoute_);
    }

    MPBuildTime_->start();
    MPGurobiPro_->buildModelMP(pInst, routeSolution_, nbVehicles_);
    MPBuildTime_->stop();
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
        if (pInst->parameters_->modelSolver_ == CPLEX)
            solveMP_LP_CPLEX(pInst, inputPaths, epoch, subProTime);
        else
            solveMP_LP_Gurobi(pInst, inputPaths, epoch, subProTime);
    }
    masterTime_->stop();
}

void CG_Algorithm::solveMP_LP_CPLEX(PInstance &pInst, const InputPaths &inputPaths, int epoch, float subProTime) {

    while (true) {
        setAvailableTime();
        if (availableTime_ <= 1)
            break;

        MasterPro_->routesToAdd_.clear();

        // select routes with negative reduced costs
        updateRoutesToAdd(NR, pInst, MasterPro_->routesToAdd_);

        /*for (auto & vehicleObj : pInst->vehicles_) {
            for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]) {
                Tools::LogOutput routeStream(inputPaths.getOutputIncDegreeRdCost(), true);
                routeStream << routeObj->routeMetricsToString(epoch, RMPCounter_);
                routeStream.close();
            }
        }*/

        if (!MasterPro_->routesToAdd_.empty()){
            MPBuildTime_->start();
            MasterPro_->updateModel(pInst);
            MPBuildTime_->stop();
            MasterPro_->solveModelLP(pInst, inputPaths);
            MPEpochSolveTime_ += MasterPro_->solveTime_->dSinceStart().count();

            LPIter_++;
            epochTime_ += (masterTime_->dSinceStart().count() - iterTime_);
            iterTime_ = masterTime_->dSinceStart().count();
            objValue_ = MasterPro_->objValue_;

            *pLogMPResultsStream_ << save_MPResults(epoch, "LMP", static_cast<int>(MasterPro_->compRoutes_.size()),
                                                    masterTime_->dSinceStart().count(), subProTime, 0.0);

            RMPCounter_++;

            if (lpObjValue_ - MasterPro_->objValue_ > 0.01) {
                lpObjValue_ = MasterPro_->objValue_;
            } else
                break;
 //           if (LPIter_ == 2)
 //               break;
        }
        else
            break;
    }
}

void CG_Algorithm::solveMP_LP_Gurobi(PInstance &pInst, const InputPaths &inputPaths, int epoch, float subProTime) {

    // solve RP with MIP solver
    while (true) {
        setAvailableTime();
        if (availableTime_ <= 1)
            break;

        MPGurobiPro_->routesToAdd_.clear();
        /*if (RMPCounter_ == 1)
            updateRoutesToAddOne(NR, pInst, MPGurobiPro_->routesToAdd_);*/

        // select routes with negative reduced costs
        updateRoutesToAdd(NR, pInst, MPGurobiPro_->routesToAdd_);

        /*for (auto & vehicleObj : pInst->vehicles_) {
            for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]) {
                Tools::LogOutput routeStream(inputPaths.getOutputIncDegreeRdCost(), true);
                routeStream << routeObj->routeMetricsToString(epoch, RMPCounter_);
                routeStream.close();
            }
        }*/

        if (!MPGurobiPro_->routesToAdd_.empty()){
            MPBuildTime_->start();
            MPGurobiPro_->updateModel(pInst);
            MPBuildTime_->stop();
            MPGurobiPro_->solveModelLP(pInst, inputPaths);
            MPEpochSolveTime_ += MPGurobiPro_->solveTime_->dSinceStart().count();

            LPIter_++;
            epochTime_ += (masterTime_->dSinceStart().count() - iterTime_);
            iterTime_ = masterTime_->dSinceStart().count();
            objValue_ = MPGurobiPro_->objValue_;

            /**pLogMPResultsStream_ << save_MPResults(epoch, "LMP", static_cast<int>(MPGurobiPro_->compRoutes_.size()),
                                                    masterTime_->dSinceStart().count(), subProTime, 0.0);*/

            RMPCounter_++;

            if (lpObjValue_ - MPGurobiPro_->objValue_ > 0.01) {
                lpObjValue_ = MPGurobiPro_->objValue_;
            } else
                break;
 //           if (LPIter_ == 2)
 //               break;
        }
        else
            break;
    }

}

void CG_Algorithm::solveMP_CG_CPLEX(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime) {
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
        solveMP_LP_CPLEX(pInst, inputPaths, epoch, subProTime);
        objValue_ = previousObj_;

        setAvailableTime();
        if (availableTime_ < 3)
            availableTime_ = 3;
        if (pInst->parameters_->dualMethod_ == AUX_D) {
            DualAuxSolver_->initializeModel(pInst);
            DualAuxSolver_->buildModel(MasterPro_->compRoutes_, pInst->requests_);
            MasterPro_->solveModelIntAux_D(pInst, zSolution_, routeSolution_, inputPaths,
                                     availableTime_, previousObj_, DualAuxSolver_);
            setCurrentRoutes(pInst);
            MIPIter_++;
            MPEpochSolveTime_ += MasterPro_->solveTime_->dSinceStart().count();
            setObjValue();

            epochTime_ += masterTime_->dSinceStart().count() - iterTime_;
            (*pLogMPResultsStream_) << save_MPResults(epoch, "CG", static_cast<int>(MasterPro_->compRoutes_.size()),
                                                    masterTime_->dSinceStart().count(), subProTime, DualAuxSolver_->objValue_);
        }
        else if (pInst->parameters_->dualMethod_ == AUX_P) {
            MasterPro_->solveModelIntAux_P(pInst, zSolution_, routeSolution_, inputPaths,
                                     availableTime_, previousObj_, lpObjValue_);
            setCurrentRoutes(pInst);
            MIPIter_++;
            MPEpochSolveTime_ += MasterPro_->solveTime_->dSinceStart().count();
            setObjValue();

            epochTime_ += (masterTime_->dSinceStart().count() - iterTime_);
            (*pLogMPResultsStream_) << save_MPResults(epoch, "CG", static_cast<int>(MasterPro_->compRoutes_.size()),
                                                        masterTime_->dSinceStart().count(), subProTime, MasterPro_->auxObjValue_);
        }
        else if (pInst->parameters_->dualMethod_ == AUX_BOX) {
            MasterPro_->solveModelInt_box(pInst, zSolution_, routeSolution_, inputPaths,
                                     availableTime_, previousObj_, lpObjValue_);
            setCurrentRoutes(pInst);
            MIPIter_++;
            MPEpochSolveTime_ += MasterPro_->solveTime_->dSinceStart().count();
            setObjValue();

            epochTime_ += (masterTime_->dSinceStart().count() - iterTime_);
            (*pLogMPResultsStream_) << save_MPResults(epoch, "CG", static_cast<int>(MasterPro_->compRoutes_.size()),
                                                        masterTime_->dSinceStart().count(), subProTime, MasterPro_->auxObjValue_);
        }
        else {
            MasterPro_->solveModelInt(pInst, zSolution_, routeSolution_, inputPaths, availableTime_, previousObj_);
            setCurrentRoutes(pInst);
            MIPIter_++;
            MPEpochSolveTime_ += MasterPro_->solveTime_->dSinceStart().count();
            setObjValue();
            epochTime_ += (masterTime_->dSinceStart().count() - iterTime_);

            if (pInst->parameters_->dualMethod_ == LMP) {
                (*pLogMPResultsStream_) << save_MPResults(epoch, "CG", static_cast<int>(MasterPro_->compRoutes_.size()),
                                                        masterTime_->dSinceStart().count(), subProTime, 0.0);
            }
            else if (pInst->parameters_->dualMethod_ == LP_CP) {
                iterTime_ = masterTime_->dSinceStart().count();
                setAvailableTime();
                /************************************************************************************************/
                //                                     COMPLEMENTARY PROBLEM
                /************************************************************************************************/
                if (availableTime_ > 3) {
                    solveCP_CPLEX(pInst, epoch, inputPaths, subProTime);
                }
            }
            else if (pInst->parameters_->dualMethod_ == LAGRANGE) {
                lagSolver_ = std::make_unique<LagrangianSolver>(pInst, objValue_,routeSolution_, zSolution_,
                    availableRoutes_);
                lagSolver_->run(pInst);
                lagSolver_.reset();
            }
            else if (pInst->parameters_->dualMethod_ == INTERIOR) {
                MasterPro_->solveLPDual(pInst, inputPaths);
            }
        }
 //       (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, RMPCounter_, "Dual");
 //       (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, RMPCounter_, "Dual");

        RMPCounter_++;

    }
    masterTime_->stop();
}

void CG_Algorithm::solveMP_CG_Gurobi(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime) {
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
        solveMP_LP_Gurobi(pInst, inputPaths, epoch, subProTime);
        objValue_ = previousObj_;

        setAvailableTime();
        if (availableTime_ < 3)
            availableTime_ = 3;
        if (pInst->parameters_->dualMethod_ == AUX_P) {
            MPGurobiPro_->solveModelIntAux_P(pInst, zSolution_, routeSolution_, inputPaths,
                                     availableTime_, previousObj_, lpObjValue_);
            setCurrentRoutes(pInst);
            MIPIter_++;
            MPEpochSolveTime_ += MPGurobiPro_->solveTime_->dSinceStart().count();
            setObjValue();

            epochTime_ += (masterTime_->dSinceStart().count() - iterTime_);
            (*pLogMPResultsStream_) << save_MPResults(epoch, "CG", static_cast<int>(MPGurobiPro_->compRoutes_.size()),
                                                        masterTime_->dSinceStart().count(), subProTime, MPGurobiPro_->auxObjValue_);
        }
        else {
            MPGurobiPro_->solveModelInt(pInst, zSolution_, routeSolution_, inputPaths,
                                     availableTime_, previousObj_);
            setCurrentRoutes(pInst);
            MIPIter_++;
            MPEpochSolveTime_ += MPGurobiPro_->solveTime_->dSinceStart().count();
            setObjValue();

            epochTime_ += (masterTime_->dSinceStart().count() - iterTime_);
            (*pLogMPResultsStream_) << save_MPResults(epoch, "CG", static_cast<int>(MPGurobiPro_->compRoutes_.size()),
                                                            masterTime_->dSinceStart().count(), subProTime, 0.0);

            if (pInst->parameters_->dualMethod_ == LP_CP) {
                iterTime_ = masterTime_->dSinceStart().count();

                setAvailableTime();
                /************************************************************************************************/
                //                                     COMPLEMENTARY PROBLEM
                /************************************************************************************************/
                if (availableTime_ > 3) {
                    solveCP_Gurobi(pInst, epoch, inputPaths, subProTime);
                }
            }
            else if (pInst->parameters_->dualMethod_ == LAGRANGE) {
                lagSolver_ = std::make_unique<LagrangianSolver>(pInst, objValue_,routeSolution_, zSolution_,
                    availableRoutes_);
                lagSolver_->run(pInst);
                lagSolver_.reset();
            }
            else if (pInst->parameters_->dualMethod_ == INTERIOR) {
                MPGurobiPro_->solveLPDual(pInst, inputPaths);

            }
        }
/*
        for (auto & routeObj : routeSolution_) {
            for (int i = 1; i < routeObj->routeSize_; ++i) {
                if (routeObj->routeNodes_[i]->type_ == PICKUP) {
                    routeObj->routeNodes_[i]->related_Request_->avgDual_ = routeObj->totalDelay_ / routeObj->routeRequests_.size();
                    routeObj->routeNodes_[i]->related_Request_->minDual_ = routeObj->plannedReachTime_[i] - routeObj->routeNodes_[i]->related_Request_->initialEarlyPick_;
                }
            }
        }

        for (auto & requestObj : zSolution_) {
            requestObj->minDual_ = requestObj->penalty_;
            requestObj->avgDual_ = requestObj->penalty_;
        }
*/

 //       (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, RMPCounter_, "Dual");
 //       (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, RMPCounter_, "Dual");

        RMPCounter_++;

    }
 //   (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, RMPCounter_, "Dual");
    masterTime_->stop();
}

void CG_Algorithm::solveMP_Gurobi_tune(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime) {
    masterTime_->start();
    previousObj_ = objValue_;
    if (SPIter_ == 0)
        lpObjValue_ = objValue_;

    iterTime_ = masterTime_->dSinceStart().count();

    /************************************************************************************************/
    //                                     MASTER PROBLEM
    /************************************************************************************************/
    setAvailableTime();
    MPGurobiPro_->routesToAdd_.clear();

    // select routes with negative reduced costs
    updateRoutesToAdd(NR, pInst, MPGurobiPro_->routesToAdd_);

    MPBuildTime_->start();
    MPGurobiPro_->updateModel(pInst);
    MPBuildTime_->stop();
//    MPGurobiPro_->tuneModel(inputPaths, 1800.0f, 120.0f, 3);

    MPGurobiPro_->solveModelInt(pInst, zSolution_, routeSolution_, inputPaths,
                                         availableTime_, previousObj_);
    setCurrentRoutes(pInst);
    MIPIter_++;
    MPEpochSolveTime_ += MPGurobiPro_->solveTime_->dSinceStart().count();
    setObjValue();

    epochTime_ += (masterTime_->dSinceStart().count() - iterTime_);
    (*pLogMPResultsStream_) << save_MPResults(epoch, "CG", static_cast<int>(MPGurobiPro_->compRoutes_.size()),
                                                masterTime_->dSinceStart().count(), subProTime, 0.0);

    masterTime_->stop();
}

void CG_Algorithm::solveMP_CG(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime) {
//    solveMP_Gurobi_tune(pInst, epoch, inputPaths, subProTime);
    if (pInst->parameters_->modelSolver_ == GUROBI)
        solveMP_CG_Gurobi(pInst, epoch, inputPaths, subProTime);
    else if (pInst->parameters_->modelSolver_ == CPLEX)
        solveMP_CG_CPLEX(pInst, epoch, inputPaths, subProTime);
}
