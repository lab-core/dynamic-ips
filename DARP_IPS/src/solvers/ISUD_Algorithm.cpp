//
// Created by Elahe Amiri on 2025-06-26.
//

#include "ISUD_Algorithm.h"
#include "CplexSolver/ComplementPro.h"
#include "CplexSolver/ReducedProblem.h"
#include "GurobiSolver/CP_Gurobi.h"
#include "GurobiSolver/RP_Gurobi.h"

ISUD_Algorithm::ISUD_Algorithm(const InputPaths &inputPaths, ModelSOLVER modelSolver) : MasterAlgorithm(inputPaths) {
    if (modelSolver == CPLEX) {
        ReducedPro_ = std::make_shared<ReducedProblem>();
        CompPro_ = std::make_shared<ComplementPro>();
    }
    else if (modelSolver == GUROBI) {
        RPGurobiPro_ = std::make_shared<RP_Gurobi>(inputPaths.getOutputSolverLog());
        CPGurobiPro_ = std::make_shared<CP_Gurobi>(inputPaths.getOutputSolverLog());
    }

    CPBuilt_ = false;
    CPEpochSolveTime_ = 0;

    RPIter_ = 0;
    CPIter_ = 0;

    maxIncDegree_ = 0;
    ZoomIter_ = 0;
    CPSuccess_ = 0;
    CPFails_ = 0;
}

void ISUD_Algorithm::initializationCPLEX(PInstance &pInst, const InputPaths &inputPaths,
    const PGreedyModeler &GreedyModel) {

    initialization(pInst, inputPaths, GreedyModel);

    masterTime_->start();

    CPEpochSolveTime_ = 0;
    maxIncDegree_ = pInst->parameters_->CP_IncDegree_;
    CPBuilt_ = false;

    // Building models
    CompPro_ = std::make_shared<ComplementPro>();
    ReducedPro_ = std::make_shared<ReducedProblem>();

    ReducedPro_->routesToAdd_.clear();

    for (auto & vehicleObj : pInst->vehicles_) {
        if (vehicleObj->currentRoute_->routeSize_ != vehicleObj->emptyRoute_->routeSize_)
            ReducedPro_->routesToAdd_.push_back(vehicleObj->emptyRoute_);
    }

    MPBuildTime_->start();
    ReducedPro_->buildModel(pInst, routeSolution_, nbVehicles_);
    MPBuildTime_->stop();

    setObjValue();
    previousObj_ = objValue_;
    masterTime_->stop();
}

void ISUD_Algorithm::initializationGurobi(PInstance &pInst, const InputPaths &inputPaths,
    const PGreedyModeler &GreedyModel) {

    initialization(pInst, inputPaths, GreedyModel);

    masterTime_->start();

    CPEpochSolveTime_ = 0;
    maxIncDegree_ = pInst->parameters_->CP_IncDegree_;
    CPBuilt_ = false;

    // Building models
    CPGurobiPro_ = std::make_shared<CP_Gurobi>(inputPaths.getOutputSolverLog());
    CPGurobiPro_->initializeCPModel(pInst, pInst->nbVehicles_);
    RPGurobiPro_ = std::make_shared<RP_Gurobi>(inputPaths.getOutputSolverLog());

    RPGurobiPro_->routesToAdd_.clear();

    for (auto & vehicleObj : pInst->vehicles_) {
        if (vehicleObj->currentRoute_->routeSize_ != vehicleObj->emptyRoute_->routeSize_)
            RPGurobiPro_->routesToAdd_.push_back(vehicleObj->emptyRoute_);
    }

    MPBuildTime_->start();
    RPGurobiPro_->buildModelRP(pInst, routeSolution_, nbVehicles_);
    MPBuildTime_->stop();

    setObjValue();
    previousObj_ = objValue_;
    masterTime_->stop();
}

int ISUD_Algorithm::solveRP_CPLEX(PInstance &pInst, int compDegree, const InputPaths &inputPaths) {
    // improve by solving the Reduced problem
    ReducedPro_->routesToAdd_.clear();
    if (compDegree == 0) {
        // select compatible routes with negative reduced costs
        updateRoutesToAdd(RP, pInst, ReducedPro_->routesToAdd_);
    }
    else
        updateRoutesToAddZoom(ReducedPro_->routesToAdd_);

    if (!ReducedPro_->routesToAdd_.empty()){
        MPBuildTime_->start();
        ReducedPro_->updateRPModel(pInst);
        MPBuildTime_->stop();
        ReducedPro_->solveModelLPInt(pInst, zSolution_, routeSolution_, inputPaths,
                                     availableTime_, objValue_);
        MPEpochSolveTime_ += ReducedPro_->solveTime_->dSinceStart().count();
        setCurrentRoutes(pInst);
        setObjValue();
    }
    return static_cast<int>(ReducedPro_->compRoutes_.size());
}

int ISUD_Algorithm::solveRP_Gurobi(PInstance &pInst, int compDegree, const InputPaths &inputPaths) {
    // improve by solving the Reduced problem
    RPGurobiPro_->routesToAdd_.clear();
    if (compDegree == 0) {
        // select compatible routes with negative reduced costs
        updateRoutesToAdd(RP, pInst, RPGurobiPro_->routesToAdd_);
    }
    else
        updateRoutesToAddZoom(RPGurobiPro_->routesToAdd_);

    if (!RPGurobiPro_->routesToAdd_.empty()){
        MPBuildTime_->start();
        RPGurobiPro_->updateRPModel(pInst);
        MPBuildTime_->stop();
        RPGurobiPro_->solveModelRelaxInt(pInst, zSolution_, routeSolution_, inputPaths,
                                     availableTime_, objValue_);
        MPEpochSolveTime_ += RPGurobiPro_->solveTime_->dSinceStart().count();
        setCurrentRoutes(pInst);
        setObjValue();
    }
    return static_cast<int>(RPGurobiPro_->compRoutes_.size());
}

void ISUD_Algorithm::solveRP(PInstance &pInst, const InputPaths &inputPaths, int epoch, float subProTime) {
    RPTime_->start();
    iterTime_ = masterTime_->dSinceStart().count();
    int nbColumns;
    while (true) {
        setAvailableTime();
        if (availableTime_ <= 1)
            break;
        if (pInst->parameters_->modelSolver_ == CPLEX) {
            CompPro_->fractionalZ_.clear();
            nbColumns = solveRP_CPLEX( pInst, 0, inputPaths);
        }
        else if (pInst->parameters_->modelSolver_ == GUROBI) {
            CPGurobiPro_->fractionalZ_.clear();
            nbColumns = solveRP_Gurobi(pInst, 0, inputPaths);
        }

        RPIter_++;
        epochTime_ += masterTime_->dSinceStart().count() - iterTime_;
        iterTime_ = masterTime_->dSinceStart().count();

        *pLogMPResultsStream_ << save_MPResults(epoch, "RP", nbColumns,
            masterTime_->dSinceStart().count(), subProTime,0.0);
        RMPCounter_++;

        if (previousObj_ - objValue_ > 0.01) {
            previousObj_ = objValue_;
        } else
            break;
        if (RPIter_ == 2)
             break;

        if (nbColumns == 0)
            break;
    }
    RPTime_->stop();
}

void ISUD_Algorithm::solveISUD_CPLEX(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime) {
    try {
        masterTime_->start();
        setObjValue();
        previousObj_ = objValue_;
        bool restartAlgorithm = true;

        while (restartAlgorithm) {
            // move these here and initialize
            bool isCPImproved = true;
            /************************************************************************************************/
            //      SOLVE REDUCED PROBLEM
            /************************************************************************************************/
            solveRP(pInst, inputPaths, epoch, subProTime);

            /************************************************************************************************/
            //                                     COMPLEMENTARY PROBLEM
            /************************************************************************************************/
            // if the value of the objective function improves, the CP is built
            CPTime_->start();
            setAvailableTime();

            if (minReducedCost_ < 0 && availableTime_ > 3) {
                CPBuildTime_->start();
                CompPro_->routesToAdd_.clear();
                // the model is built only in the first iteration of ISUD then just the solution is updated and
                // incompatible columns remain the same
                if (!CPBuilt_)
                    CompPro_->buildModel(pInst, zSolution_, routeSolution_, nbVehicles_);
                else{
                    // Make sure that compatible columns have negative coefficient
                    CompPro_->repairModel(pInst, zSolution_, routeSolution_);
                }
                CPBuildTime_->stop();
                CPBuilt_ = true;
            }
            else {
                restartAlgorithm = false;
                isCPImproved = false;
            }
            iterTime_ = masterTime_->dSinceStart().count();
            while (isCPImproved) {
                isCPImproved = false;
                previousObj_ = objValue_;
                CompPro_->routesToAdd_.clear();
                updateRoutesToAdd(CP, pInst, CompPro_->routesToAdd_);
                setAvailableTime();
                if (!CompPro_->routesToAdd_.empty() && availableTime_ > 1) {
                    CPBuildTime_->start();
                    CompPro_->updateModel(pInst, zSolution_, routeSolution_);
                    CPBuildTime_->stop();
                    CompPro_->solveCPModel(pInst, zSolution_, routeSolution_, inputPaths);
                    setCurrentRoutes(pInst);
                    CPIter_++;

                    CPEpochSolveTime_ += CompPro_->solveTime_->dSinceStart().count();
                    setObjValue();
                    epochTime_ += masterTime_->dSinceStart().count() - iterTime_;
                    iterTime_ = masterTime_->dSinceStart().count();
                    (*pLogMPResultsStream_) << save_MPResults(epoch, "CP", static_cast<int>(CompPro_->IncRoute_.size()),
                                                                masterTime_->dSinceStart().count(), subProTime, 0.0);

                    if (CompPro_->status_ == FRACTIONAL) {
                        CPFails_++;
                        setAvailableTime();
                        if (pInst->parameters_->useZoom_ && availableTime_ > 1) {
                            iterTime_ = masterTime_->dSinceStart().count();
                            ZOOMTime_->start();
                            solveRP_CPLEX(pInst, 1, inputPaths);
                            ZoomIter_++;
                            epochTime_ += masterTime_->dSinceStart().count() - iterTime_;
                            iterTime_ = masterTime_->dSinceStart().count();

                            (*pLogMPResultsStream_)
                                    << save_MPResults(epoch, "ZOOM",
                                                      static_cast<int>(ReducedPro_->compRoutes_.size()) - nbVehicles_,
                                                      masterTime_->dSinceStart().count(), subProTime,
                                                      0.0);
                            RMPCounter_++;

                            ZOOMTime_->stop();
                            if (previousObj_ - objValue_ > 0.01) {
                                std::cout << previousObj_ << std::endl;
                                previousObj_ = objValue_;
                            }
                            else {
                                restartAlgorithm = false;
                                break;
                            }
                        }

                        else {
                            restartAlgorithm = false;
                            break;
                        }
                    }
                    else if (CompPro_->status_ == POSITIVE_VALUE || CompPro_->status_ == INFEASIBLE) {
                        restartAlgorithm = false;
                        break;
                    }
                    else {
                        CPSuccess_++;
                        previousObj_ = objValue_;
                        RMPCounter_++;
                        isCPImproved = false;
                        setAvailableTime();
                        restartAlgorithm = false;
                        if (availableTime_ <= 1) {
                            restartAlgorithm = false;
                            break;
                        }
                    }
                } else {
                    restartAlgorithm = false;
                    break;
                }
            }
            setAvailableTime();
            updateReducedCosts(pInst);
            if (minReducedCost_ > 0 || availableTime_ <= 0)
                restartAlgorithm = false;
            for (auto &routeObj: CompPro_->IncRoute_)
                routeObj->cpAdded_ = false;
            CompPro_.reset();
            CompPro_ = std::make_shared<ComplementPro>();
            CPBuilt_ = false;
            CPTime_->stop();
        }

        (*pLogMPResultsStream_)
                << save_MPResults(epoch, "ISUD", nbRoutes_, masterTime_->dSinceStart().count(), subProTime, 0.0);
        masterTime_->stop();
    }
    catch (const std::exception& e){
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e.what() << std::endl;
    }
}

void ISUD_Algorithm::solveISUD_Gurobi(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime) {
    try {
        masterTime_->start();
        setObjValue();
        previousObj_ = objValue_;
        bool restartAlgorithm = true;

        while (restartAlgorithm) {
            // move these here and initialize
            bool isCPImproved = true;
            /************************************************************************************************/
            //      SOLVE REDUCED PROBLEM
            /************************************************************************************************/
            solveRP(pInst, inputPaths, epoch, subProTime);

            /************************************************************************************************/
            //                                     COMPLEMENTARY PROBLEM
            /************************************************************************************************/
            // if the value of the objective function improves, the CP is built
            CPTime_->start();

            while (isCPImproved) {
                setAvailableTime();
                if (availableTime_ > 3) {

                    // Build CP model
                    CPBuildTime_->start();
                    CPGurobiPro_->routesToAdd_.clear();
                    CPGurobiPro_->buildModel(pInst, zSolution_, routeSolution_, nbVehicles_);
                    CPBuildTime_->stop();

                    iterTime_ = masterTime_->dSinceStart().count();
                    isCPImproved = false;

                    previousObj_ = objValue_;
                    updateRoutesToAdd(CP, pInst, CPGurobiPro_->routesToAdd_);
                    setAvailableTime();
                    if (!CPGurobiPro_->routesToAdd_.empty() && availableTime_ > 1) {
                        CPBuildTime_->start();
                        CPGurobiPro_->updateModel(pInst, zSolution_, routeSolution_);
                        CPBuildTime_->stop();
                        CPGurobiPro_->solveCPModelFresh(pInst, zSolution_, routeSolution_, inputPaths);
                        setCurrentRoutes(pInst);
                        CPIter_++;

                        CPEpochSolveTime_ += CPGurobiPro_->solveTime_->dSinceStart().count();
                        setObjValue();
                        epochTime_ += masterTime_->dSinceStart().count() - iterTime_;
                        iterTime_ = masterTime_->dSinceStart().count();
                        (*pLogMPResultsStream_) << save_MPResults(epoch, "CP", static_cast<int>(CPGurobiPro_->IncRoute_.size()),
                                                                    masterTime_->dSinceStart().count(), subProTime, 0.0);

                        if (CPGurobiPro_->status_ == FRACTIONAL) {
                            CPFails_++;
                            setAvailableTime();
                            if (pInst->parameters_->useZoom_ && availableTime_ > 1) {
                                iterTime_ = masterTime_->dSinceStart().count();
                                ZOOMTime_->start();
                                solveRP_Gurobi(pInst, 1, inputPaths);
                                ZoomIter_++;
                                epochTime_ += masterTime_->dSinceStart().count() - iterTime_;
                                iterTime_ = masterTime_->dSinceStart().count();

                                (*pLogMPResultsStream_)
                                        << save_MPResults(epoch, "ZOOM",
                                                          static_cast<int>(ReducedPro_->compRoutes_.size()) - nbVehicles_,
                                                          masterTime_->dSinceStart().count(), subProTime,
                                                          0.0);
                                RMPCounter_++;

                                ZOOMTime_->stop();
                                if (previousObj_ - objValue_ > 0.01) {
                                    std::cout << previousObj_ << std::endl;
                                    previousObj_ = objValue_;
                                }
                                else {
                                    restartAlgorithm = false;
                                    break;
                                }
                            }

                            else {
                                restartAlgorithm = false;
                                break;
                            }
                        }
                        else if (CPGurobiPro_->status_ == POSITIVE_VALUE || CPGurobiPro_->status_ == INFEASIBLE) {
                            restartAlgorithm = false;
                            break;
                        }
                        else {
                            CPSuccess_++;
                            previousObj_ = objValue_;
                            RMPCounter_++;
                            isCPImproved = true;
                            setAvailableTime();
                            restartAlgorithm = true;

                        }
                    } else {
                        restartAlgorithm = false;
                        break;
                    }
                }
                updateReducedCosts(pInst);
                if (minReducedCost_ > 0 || availableTime_ <= 1) {
                    restartAlgorithm = false;
                    break;
                }
                for (auto &routeObj: CPGurobiPro_->IncRoute_)
                    routeObj->cpAdded_ = false;
                CPGurobiPro_.reset();
                CPGurobiPro_ = std::make_shared<CP_Gurobi>(inputPaths.getOutputSolverLog());
            }
            setAvailableTime();
            if (minReducedCost_ > 0 || availableTime_ <= 1)
                restartAlgorithm = false;

            CPTime_->stop();
        }

        (*pLogMPResultsStream_)
                << save_MPResults(epoch, "ISUD", nbRoutes_, masterTime_->dSinceStart().count(), subProTime, 0.0);
        masterTime_->stop();
    }
    catch (const std::exception& e){
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e.what() << std::endl;
    }
}

void ISUD_Algorithm::solveISUD_Gurobi2(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime) {
    try {
        masterTime_->start();
        setObjValue();
        previousObj_ = objValue_;
        bool restartAlgorithm = true;

        while (restartAlgorithm) {
            // move these here and initialize
            bool isCPImproved = true;
            /************************************************************************************************/
            //      SOLVE REDUCED PROBLEM
            /************************************************************************************************/
            solveRP(pInst, inputPaths, epoch, subProTime);

            /************************************************************************************************/
            //                                     COMPLEMENTARY PROBLEM
            /************************************************************************************************/
            // if the value of the objective function improves, the CP is built
            CPTime_->start();
            setAvailableTime();

            if (minReducedCost_ < 0 && availableTime_ > 3) {
                CPBuildTime_->start();
                CPGurobiPro_->routesToAdd_.clear();
                // the model is built only in the first iteration of ISUD then just the solution is updated and
                // incompatible columns remain the same
                if (!CPBuilt_)
                    CPGurobiPro_->buildModel(pInst, zSolution_, routeSolution_, nbVehicles_);
                else{
                    // Make sure that compatible columns have negative coefficient
                    CPGurobiPro_->repairModel(pInst, zSolution_, routeSolution_);
                }
                CPBuildTime_->stop();
                CPBuilt_ = true;
            }
            else {
                restartAlgorithm = false;
                isCPImproved = false;
            }
            iterTime_ = masterTime_->dSinceStart().count();
            while (isCPImproved) {
                isCPImproved = false;
                previousObj_ = objValue_;
                CPGurobiPro_->routesToAdd_.clear();
                updateRoutesToAdd(CP, pInst, CPGurobiPro_->routesToAdd_);
                setAvailableTime();
                if (!CPGurobiPro_->routesToAdd_.empty() && availableTime_ > 1) {
                    CPBuildTime_->start();
                    CPGurobiPro_->updateModel(pInst, zSolution_, routeSolution_);
                    CPBuildTime_->stop();
                    CPGurobiPro_->solveCPModel(pInst, zSolution_, routeSolution_, inputPaths);
                    setCurrentRoutes(pInst);
                    CPIter_++;

                    CPEpochSolveTime_ += CPGurobiPro_->solveTime_->dSinceStart().count();
                    setObjValue();
                    epochTime_ += masterTime_->dSinceStart().count() - iterTime_;
                    iterTime_ = masterTime_->dSinceStart().count();
                    (*pLogMPResultsStream_) << save_MPResults(epoch, "CP", static_cast<int>(CPGurobiPro_->IncRoute_.size()),
                                                                masterTime_->dSinceStart().count(), subProTime, 0.0);

                    if (CPGurobiPro_->status_ == FRACTIONAL) {
                        CPFails_++;
                        setAvailableTime();
                        if (pInst->parameters_->useZoom_ && availableTime_ > 1) {
                            iterTime_ = masterTime_->dSinceStart().count();
                            ZOOMTime_->start();
                            solveRP_Gurobi(pInst, 1, inputPaths);
                            ZoomIter_++;
                            epochTime_ += masterTime_->dSinceStart().count() - iterTime_;
                            iterTime_ = masterTime_->dSinceStart().count();

                            (*pLogMPResultsStream_)
                                    << save_MPResults(epoch, "ZOOM",
                                                      static_cast<int>(RPGurobiPro_->compRoutes_.size()) - nbVehicles_,
                                                      masterTime_->dSinceStart().count(), subProTime,
                                                      0.0);
                            RMPCounter_++;

                            ZOOMTime_->stop();
                            if (previousObj_ - objValue_ > 0.01) {
                                std::cout << previousObj_ << std::endl;
                                previousObj_ = objValue_;
                            }
                            else {
                                restartAlgorithm = false;
                                break;
                            }
                        }

                        else {
                            restartAlgorithm = false;
                            break;
                        }
                    }
                    else if (CPGurobiPro_->status_ == POSITIVE_VALUE || CPGurobiPro_->status_ == INFEASIBLE) {
                        restartAlgorithm = false;
                        break;
                    }
                    else {
                        CPSuccess_++;
                        previousObj_ = objValue_;
                        RMPCounter_++;
                        isCPImproved = true;
                        setAvailableTime();
                        restartAlgorithm = true;
                        if (availableTime_ <= 1) {
                            restartAlgorithm = false;
                            break;
                        }
                    }
                } else {
                    restartAlgorithm = false;
                    break;
                }
            }
            setAvailableTime();
            updateReducedCosts(pInst);
            if (minReducedCost_ > 0 || availableTime_ <= 0)
                restartAlgorithm = false;

            CPGurobiPro_->resetForNextIteration();
 //           CPGurobiPro_.reset();
 //           CPGurobiPro_ = std::make_shared<CP_Gurobi>();
            CPBuilt_ = false;
            CPTime_->stop();
        }

        (*pLogMPResultsStream_)
                << save_MPResults(epoch, "ISUD", nbRoutes_, masterTime_->dSinceStart().count(), subProTime, 0.0);
        masterTime_->stop();
    }
    catch (const std::exception& e){
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e.what() << std::endl;
    }
}

void ISUD_Algorithm::solveISUD(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime) {
    if (pInst->parameters_->modelSolver_ == GUROBI)
        solveISUD_Gurobi2(pInst, epoch, inputPaths, subProTime);
    else if (pInst->parameters_->modelSolver_ == CPLEX)
        solveISUD_CPLEX(pInst, epoch, inputPaths, subProTime);
}
