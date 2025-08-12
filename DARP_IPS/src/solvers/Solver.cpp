//
// Created by Elahe Amiri on 2022-10-26.
//

#include "CG_Algorithm.h"
#include "ISUD_Algorithm.h"
#include "Solver.h"
#include "solvers/LabelingSubProblem.h"
#include "CplexSolver/CPLEXSubProblem.h"
#include "GurobiSolver/MP_Gurobi.h"

Solver::Solver(const PInstance & mainInst, InputPaths &inputPaths) {
    elapsedTime_ = 0;
    epoch_ = 0;
    nbOnePick_ = 0;
    nbTwoPick_ = 0;
    nbThreePick_ = 0;
    heuristicCG_ = 0;
    nbRecycle_ = 0;

    greedyRebalanceTime_ = 0;

    if (mainInst->parameters_->approach_ == ISUD)
        ISUD_Model_ = std::make_unique<ISUD_Algorithm>(inputPaths, mainInst->parameters_->modelSolver_);
    else if (mainInst->parameters_->approach_ == CG)
        CG_Model_ = std::make_unique<CG_Algorithm>(inputPaths, mainInst->parameters_->modelSolver_);


    GreedyModel_ = std::make_shared<GreedyModeler>();
    subProOptions_ = std::make_shared<solverOption>(mainInst->parameters_);

    simulationTime_ = new myTools::Timer(); simulationTime_->init();
    subProblemTime_ = new myTools::Timer(); subProblemTime_->init();
    preprocessTime_ = new myTools::Timer(); preprocessTime_->init();
    rebalancingTime_ = new myTools::Timer(); rebalancingTime_->init();

    runtimeMetrics_ = std::make_unique<RuntimeMetrics>();
    objValue_ = 0;
    avgEpochRuntime_ = 0;

    labelsPool_.defineSize(mainInst->parameters_->nbThreads_);

    if (mainInst->parameters_->approach_ != Greedy) {
        // this Stream defines the runtime outputs
        pLogRunTimesStream_ = new Tools::LogOutput(inputPaths.getOutputEpochRunTime());
        (*pLogRunTimesStream_) << "Epoch,nbRequests,nbNewRequests,nbNodes,EpochRuntime,ElapsedTime,MP_Runtime,"
                                  "RP_Runtime,MP_BuildRuntime,MP_SolveRuntime,CP_Runtime,CP_BuildRuntime,"
                                  "CP_SolveRuntime,ZoomISUD_Runtime,SubProbRuntime,"
                                  "#SP Iter,totalColumn,#LGenerated,#LDominated,#LEliminated,#nbPrunedArcs,"
                                  " #nbPrunedPath,nbNegative,#ColumnsAdded,#RecycledColumns,GreedyObj,Objective,"
                                  "LinearObjective,waitTime,destructTime,GreedyTime,#Return,#Idle,#passPerVehicle,"
                                  "#requestPerVehicle,#nodePerVehicle,"
                                  "#StateChanged,nbOnePick,nbTwoPick,nbThreePick,heuristicCG,upperBound,nbRecycle" << std::endl;
    }
    else {
        pLogRunTimesStream_ = new Tools::LogOutput(inputPaths.getOutputEpochRunTime());
        (*pLogRunTimesStream_) << "Epoch,nbRequests,nbNewRequests,nbNodes,EpochRuntime,ElapsedTime,"
                                  "GreedyObj,Objective,waitTime,GreedyTime,#Return,#Idle,#passPerVehicle,"
                                  "#requestPerVehicle,#nodePerVehicle" << std::endl;
    }


    pLogEpochSubRuntimeStream_ = new Tools::LogOutput(inputPaths.getOutputSubproSize());
    (*pLogEpochSubRuntimeStream_) << "Epoch,vehicleID,nbRequests,nbNodes,maxPick,#LGenerated,#LDominated,#LEliminated,"
                                     "#nbPrunedArcs,#nbPrunedPath,nbNegative,nbRoutes,BestRCost,runtime" << std::endl;

    /*pLogSolutionChange_ = new Tools::LogOutput(inputPaths.getOutputSolutionChange());
    (*pLogSolutionChange_) << "Epoch,#SubProblems,#Requests,#NewArrival,#PreScheduled, #ReScheduled, "
                              "#Inc(1 deg.),#Inc(2 deg.),#Inc(3 deg.),#Inc(4 deg.),#Inc(5 deg.),#Inc(total)" << std::endl;*/
}

Solver::~Solver() {
    delete simulationTime_;
    delete subProblemTime_;
    delete preprocessTime_;
    delete rebalancingTime_;
    pLogRunTimesStream_->close();
    pLogEpochSubRuntimeStream_->close();
//    pLogEpochSubRouteStream_->close();
//    pLogSolutionChange_->close();
    delete pLogRunTimesStream_;
    delete pLogEpochSubRuntimeStream_;
//    delete pLogEpochSubRouteStream_;
//    delete pLogSolutionChange_;
}

void Solver::selectVehiclesForSubproblem(const PInstance &EpochInst, int iter){
    if (!EpochInst->parameters_->vehiclePortion_ || iter == 1) {
        // The subproblems are solved for all the vehicles
        EpochInst->selectedVehicles_.assign(EpochInst->nbVehicles_, iter);
    }
    else if (iter == 2){
        // The subproblems are solved for the vehicles who have already some requests from the pre-iteration
        EpochInst->selectedVehicles_.assign(EpochInst->nbVehicles_, 0);
        for (const auto &vehicleObj : EpochInst->vehicles_) {
            if (!vehicleObj->currentRoute_->routeRequests_.empty()) {
                EpochInst->selectedVehicles_[vehicleObj->vehicleID_] = iter;
            }
        }
    }
}
void Solver::solveCG_Epoch(PInstance &EpochInst, PInstance & mainInst, InputPaths &inputPaths) {

    Tools::PThreadsPool pPool = Tools::ThreadsPool::newThreadsPool(EpochInst->parameters_->nbThreads_);
    bool repeat = true;

    // Set available time
    CG_Model_->setAvailableTime(EpochInst, simulationTime_->dSinceStart().count(), 1);
    EpochInst->updateTaskIndexLabeling();
    if (mainInst->parameters_->modelSolver_ == CPLEX)
        CG_Model_->initializationCPLEX(EpochInst, inputPaths, epoch_, GreedyModel_);
    else
        CG_Model_->initializationGurobi(EpochInst, inputPaths, epoch_, GreedyModel_);
    // Initialize variables
    int iter = 0;
    bool subProBreak = false;
    CG_Model_->RMPCounter_++;
    CG_Model_->nbRoutes_ = 0;

    CG_Model_->lpObjValue_ = CG_Model_->objValue_;
    EpochInst->selectedVehicles_.clear();
    EpochInst->selectedVehicles_.resize(EpochInst->nbVehicles_, 0);
    runtimeMetrics_->resetSubproblemMetrics();
    if (CG_Model_->availableRoutes_.empty())
        CG_Model_->availableRoutes_.resize(mainInst->nbVehicles_);

    // Main column generation loop
    while (true) {
        iter++;
        CG_Model_->setAvailableTime(EpochInst, simulationTime_->dSinceStart().count(), iter);
        int nbNegativeFound = 0;
        float previousObj = CG_Model_->objValue_;
        /*if (epoch_ == 0) {
            // request duals
            for (size_t i = 0; i < EpochInst->requests_.size(); ++i) {
                if (EpochInst->requests_[i]->solVehicleID_ != LARGE_CONSTANT)
                    EpochInst->requests_[i]->dual_ = EpochInst->requests_[i]->InitialDual_;
            }

            // vehicle duals
            for (size_t i = 0; i < EpochInst->vehicles_.size(); ++i)
                EpochInst->vehicles_[i]->dual_ = EpochInst->vehicles_[i]->InitialDual_;
        }
        *CG_Model_->pLogIterReqDualStream_ << EpochInst->saveReqDuals(epoch_, CG_Model_->RMPCounter_, "Dual");
        *CG_Model_->pLogIterVehDualStream_ << EpochInst->saveVehDuals(epoch_, CG_Model_->RMPCounter_, "Dual");*/

        //***********************************************************************************//
        //                    Solve subproblems using the extracted function
        //***********************************************************************************//
        if (mainInst->parameters_->subAlgorithm_ == LABEL_SETTING)
            subProBreak = solve_SP_Label(EpochInst, mainInst, iter, nbNegativeFound,
                CG_Model_->availableRoutes_, CG_Model_->availableTime_, CG_Model_->nbRoutes_);
        else
            subProBreak = solve_SP_CPLEX(EpochInst, mainInst, iter, nbNegativeFound,
                CG_Model_->availableRoutes_, CG_Model_->availableTime_, CG_Model_->nbRoutes_);

        if (subProBreak) {
            std::cout << "Terminate CG-> Not enough time to run the subproblems! " << std::endl;
            break;
        }

        CG_Model_->SPIters_++;
        CG_Model_->SPIter_++;

        if (nbNegativeFound == 0) {
            if (!repeat) {
                subProOptions_->disableHeuristics();
                EpochInst->parameters_->dynamicPricing_ = false;
                heuristicCG_ = CG_Model_->lpObjValue_;
                std::cout << "Solve the exact mode " << std::endl;
                repeat = true;
            }
            else {
                CG_Model_->CGSuccess_++;
                std::cout << "Terminate CG-> No negative column " << std::endl;
                break;
            }
        }

        else {
            // Update available time
            CG_Model_->setAvailableTime(EpochInst, simulationTime_->dSinceStart().count(), iter);
            if (CG_Model_->availableTime_ <= 0){
                break;
            }
            CG_Model_->timeLimit_ = CG_Model_->availableTime_;

            //***********************************************************************************//
            //                    Solve the Restricted Master Problem
            //***********************************************************************************//
            CG_Model_->epochTime_ += subProblemTime_->dSinceStart().count();
 //           if (iter == 1)
 //               EpochInst->resetDuals();
            if (EpochInst->parameters_->mainAlgorithm_ == RT_CG)
                CG_Model_->solveRMP_LP(EpochInst, epoch_, inputPaths, subProblemTime_->dSinceStart().count());
            else if (EpochInst->parameters_->mainAlgorithm_ == A_CG)
                CG_Model_->solveMP_CG(EpochInst, epoch_, inputPaths, subProblemTime_->dSinceStart().count());

            // Update available time
            CG_Model_->setAvailableTime(EpochInst, simulationTime_->dSinceStart().count(), iter);
            if (mainInst->parameters_->numIter_ == iter){
                break;
            }
        }
        if (CG_Model_->availableTime_ <= 0)
            break;

        if (EpochInst->parameters_->mainAlgorithm_ == A_CG){
            if (previousObj == CG_Model_->objValue_) {
                CG_Model_->CGSuccess_++;
                std::cout << "No changes in Objective" << std::endl;
                break;
            }
            /*else if (CG_Model_->objValue_ <= CG_Model_->upperbound_ * (1.0 - 0.01)) {
                std::cout << "Improve is sufficient!" << std::endl;
                break;
            }*/
        }

        std::cout << " simulation time: " << simulationTime_->dSinceStart().count() << std::endl;
    }  // end of CG while

    CG_Model_->setObjValue();

    if (EpochInst->parameters_->mainAlgorithm_ == RT_CG) {
        CG_Model_->availableTime_ = EpochInst->parameters_->epochLength_ - simulationTime_->dSinceStart().count();
        CG_Model_->timeLimit_ = std::max(CG_Model_->availableTime_, 10.0f);

        CG_Model_->solveRMP_IP(EpochInst, epoch_, inputPaths, subProblemTime_->dSinceStart().count());
    }

    CG_Model_->MasterPro_.reset();
    CG_Model_->MPGurobiPro_.reset();

    if (EpochInst->parameters_->dualMethod_ == AUX_D)
        CG_Model_->DualAuxSolver_.reset();

    if (EpochInst->parameters_->vehicleReturn_) {
        if (rebalancingTime_->dSinceStart().count() >= EpochInst->parameters_->epochLength_ || EpochInst->parameters_->solutionMode_ == DYNAMIC) {
            if (EpochInst->parameters_->returnPolicy_ == TO_SOURCE)
                returnVehicles(EpochInst);
            else if (EpochInst->parameters_->returnPolicy_ == ZONE)
                returnVehiclesZone(EpochInst);
            else
                returnVehiclesAlonso(mainInst);
        }
    }


    if (EpochInst->parameters_->solutionMode_ == ANYTIME){
        for (auto &vehicleObj: EpochInst->vehicles_){
            if (vehicleObj->preSolvePick_ >= 1 && vehicleObj->idle_){
                vehicleObj->updateCurrentRoute(EpochInst->simulationStartTime_ + elapsedTime_+ simulationTime_->dSinceStart().count());
            }
        }
    }
 //   subProOptions_->enableHeuristics(mainInst->parameters_);
//    EpochInst->parameters_->dynamicPricing_ = true;
    objValue_ = CG_Model_->objValue_;
    labelsPool_.clear();
    labelsPool_.defineSize(mainInst->parameters_->nbThreads_);
    CG_Model_->setLastDuals(EpochInst);
    std::cout << " end time: " << simulationTime_->dSinceStart().count() << std::endl;
}

void Solver::solveICG_Epoch(PInstance &EpochInst, PInstance &mainInst, InputPaths &inputPaths) {
    Tools::PThreadsPool pPool = Tools::ThreadsPool::newThreadsPool(EpochInst->parameters_->nbThreads_);

    // Set available time
    ISUD_Model_->setAvailableTime(EpochInst, simulationTime_->dSinceStart().count(), 1);
    EpochInst->updateTaskIndexLabeling();
    if (mainInst->parameters_->modelSolver_ == CPLEX)
        ISUD_Model_->initializationCPLEX(EpochInst, inputPaths, GreedyModel_);
    else
        ISUD_Model_->initializationGurobi(EpochInst, inputPaths, GreedyModel_);

    // Initialize variables
    int iter = 0;
    bool subProBreak = false;
    ISUD_Model_->RMPCounter_ ++;
    ISUD_Model_->nbRoutes_ = 0;

    ISUD_Model_->lpObjValue_ = ISUD_Model_->objValue_;
    EpochInst->selectedVehicles_.clear();
    EpochInst->selectedVehicles_.resize(EpochInst->nbVehicles_, 0);
    runtimeMetrics_->resetSubproblemMetrics();
    if (ISUD_Model_->availableRoutes_.empty())
        ISUD_Model_->availableRoutes_.resize(mainInst->nbVehicles_);


    while (true) {
        iter++;
        ISUD_Model_->setAvailableTime(EpochInst, simulationTime_->dSinceStart().count(), iter);
        int nbNegativeFound = 0;
        float previousObj = ISUD_Model_->objValue_;

        //***********************************************************************************//
        //                    Solve subproblems using the extracted function
        //***********************************************************************************//
        if (mainInst->parameters_->subAlgorithm_ == LABEL_SETTING)
            subProBreak = solve_SP_Label(EpochInst, mainInst, iter, nbNegativeFound,
                ISUD_Model_->availableRoutes_, ISUD_Model_->availableTime_, ISUD_Model_->nbRoutes_);
        else
            subProBreak = solve_SP_CPLEX(EpochInst, mainInst, iter, nbNegativeFound,
                ISUD_Model_->availableRoutes_, ISUD_Model_->availableTime_, ISUD_Model_->nbRoutes_);

        if (subProBreak) {
            std::cout << "Terminate CG-> Not enough time to run the subproblems! " << std::endl;
            break;
        }

        ISUD_Model_->SPIters_++;
        ISUD_Model_->SPIter_++;

        if (nbNegativeFound == 0) {
            ISUD_Model_->CGSuccess_++;
            std::cout << "Terminate CG-> No negative column " << std::endl;
            break;
        }else {
            // Update available time
            ISUD_Model_->setAvailableTime(EpochInst, simulationTime_->dSinceStart().count(), iter);
            if (ISUD_Model_->availableTime_ <= 0){
                break;
            }
            ISUD_Model_->timeLimit_ = ISUD_Model_->availableTime_;

            //***********************************************************************************//
            //                    Solve the Restricted Master Problem with ISUD
            //***********************************************************************************//
            ISUD_Model_->epochTime_ += subProblemTime_->dSinceStart().count();
 //           if (iter == 1)
 //               EpochInst->resetDuals();
            if(EpochInst->parameters_->mainAlgorithm_ == MP_ISUD)
                ISUD_Model_->solveISUD(EpochInst, epoch_, inputPaths, subProblemTime_->dSinceStart().count());

            // Update available time
            ISUD_Model_->setAvailableTime(EpochInst, simulationTime_->dSinceStart().count(), iter);
            if (mainInst->parameters_->numIter_ == iter){
                break;
            }
        }
        if (ISUD_Model_->availableTime_ <= 0)
            break;

        if (previousObj == ISUD_Model_->objValue_) {
            ISUD_Model_->CGSuccess_++;
            std::cout << "No changes in Objective" << std::endl;
            break;
        }

        std::cout << " simulation time: " << simulationTime_->dSinceStart().count() << std::endl;
    }  // end of CG while

    ISUD_Model_->setObjValue();

    // reset MP models
    ISUD_Model_->CompPro_.reset();
    ISUD_Model_->ReducedPro_.reset();
    ISUD_Model_->CPGurobiPro_.reset();
    ISUD_Model_->RPGurobiPro_.reset();

    if (EpochInst->parameters_->vehicleReturn_) {
        if (rebalancingTime_->dSinceStart().count() >= EpochInst->parameters_->epochLength_ || EpochInst->parameters_->solutionMode_ == DYNAMIC) {
            if (EpochInst->parameters_->returnPolicy_ == TO_SOURCE)
                returnVehicles(EpochInst);
            else if (EpochInst->parameters_->returnPolicy_ == ZONE)
                returnVehiclesZone(EpochInst);
            else
                returnVehiclesAlonso(mainInst);
        }
    }

    if (EpochInst->parameters_->solutionMode_ == ANYTIME){
        for (auto &vehicleObj: EpochInst->vehicles_){
            if (vehicleObj->preSolvePick_ >= 2 && vehicleObj->idle_){
                vehicleObj->updateCurrentRoute(EpochInst->simulationStartTime_ + elapsedTime_+ simulationTime_->dSinceStart().count());
            }
        }
    }
    labelsPool_.clear();
    labelsPool_.defineSize(mainInst->parameters_->nbThreads_);
    objValue_ = ISUD_Model_->objValue_;
    ISUD_Model_->setLastDuals(EpochInst);
    std::cout << " end time: " << simulationTime_->dSinceStart().count() << std::endl;
}

void Solver::solve_Epoch(PInstance &EpochInst, PInstance &mainInst, InputPaths &inputPaths) {
    if (EpochInst->parameters_->approach_ == ISUD)
        solveICG_Epoch(EpochInst, mainInst, inputPaths);
    else if (EpochInst->parameters_->approach_ == CG)
        solveCG_Epoch(EpochInst, mainInst, inputPaths);
}

bool Solver::solve_SP_Label(PInstance &EpochInst, PInstance &mainInst, int &iter, int &nbNegativeFound,
                            vector2D<PRoute> &availableRoutes, float availableTime, int &nbRoutes) {
    Tools::PThreadsPool pPool = Tools::ThreadsPool::newThreadsPool(EpochInst->parameters_->nbThreads_);

    bool subProBreak = false;
    nbOnePick_ = 0;
    nbTwoPick_ = 0;
    nbThreePick_ = 0;
    nbRecycle_ = 0;

    // Start the subproblems timer
    subProblemTime_->start();

    // Define subproblems
    std::vector<PLabelingSubPro> subProSolve;
    selectVehiclesForSubproblem(EpochInst, iter);

    // create subproblems
    int num = 0;

    for (auto &vehicleObj: EpochInst->vehicles_) {
        vehicleObj->bestReducedCost_ = INFINITY;

        if (EpochInst->selectedVehicles_[vehicleObj->vehicleID_] >= 1) {
            num++;
            subProSolve.emplace_back(std::make_shared<LabelingSubProblem>(vehicleObj, subProOptions_));

            // Handle partial pricing
            if (EpochInst->parameters_->partialPricing_) {
                if (vehicleObj->currentRoute_->routeRequests_.size() >= 2) {
                    vehicleObj->numPickup_ = std::min(3, EpochInst->parameters_->nbPick_);;
                    nbThreePick_++;
                } else if (!vehicleObj->currentRoute_->routeRequests_.empty()) {
                    vehicleObj->numPickup_ = 2;
                    nbTwoPick_++;
                } else {
                    vehicleObj->numPickup_ = 1;
                    nbOnePick_++;
                }
                subProSolve.back()->maxPickup_ = vehicleObj->numPickup_;
                vehicleObj->preSolvePick_ = subProSolve.back()->maxPickup_;
            }
            // Handle dynamic pricing
            else if (EpochInst->parameters_->dynamicPricing_) {
                subProSolve.back()->maxPickup_ = std::min(iter, EpochInst->parameters_->nbPick_);
            }
            // Default pricing
            else {
                subProSolve.back()->maxPickup_ = EpochInst->parameters_->nbPick_;
            }

            // Handle route recycling
            if (EpochInst->parameters_->LabelingStrategy_ == RE_PULLING && EpochInst->parameters_->routeRecycle_ &&
                !vehicleObj->currentRoute_->routeRequests_.empty() &&
                availableRoutes[vehicleObj->vehicleID_].size() > mainInst->parameters_->nbColumn_) {
                nbRecycle_ ++;
                subProSolve.back()->availableRoutes_ = availableRoutes[vehicleObj->vehicleID_];
                subProSolve.back()->availableRoutes_.push_back(vehicleObj->currentRoute_);
            }
            availableRoutes[vehicleObj->vehicleID_].clear();
        }
    }
    // solving subproblems
    for (auto &subProblem: subProSolve) {
        if (subProBreak) break;
        // Check time constraint for dynamic mode
        if (EpochInst->parameters_->solutionMode_ == DYNAMIC &&
            availableTime - subProblemTime_->dSinceStart().count() <= 0 && iter > 1) {
            subProBreak = true;
            break;
        }
        Tools::Job job([&]() {
            subProblem->initSubGraph(EpochInst);
            subProblem->labelPool_ = std::move(labelsPool_.pop_data());
            if (!subProblem->subRequests_.empty()) {
                if (subProblem->solveDynamic(
                        availableTime - subProblemTime_->dSinceStart().count())) {
                    subProblem->SolutionToRoutes(EpochInst->vehicles_[subProblem->Vehicle_->vehicleID_],
                                                 availableRoutes[(subProblem->Vehicle_)->vehicleID_],
                                                 mainInst, EpochInst->nbRequests_);
                } else {
                    subProblem->CollectLabels();
                    subProBreak = true;
                }
            }
            labelsPool_.push_data(std::move(subProblem->labelPool_));
        });
        pPool->run(job);
    }
    // Wait for all jobs to complete
    while (true) {
        if (!pPool->wait())
            break;
    }

    // Collect results from subproblems
    nbNegativeFound = 0;
    for (auto &subProblem: subProSolve) {
        nbRoutes += static_cast<int>(availableRoutes[(subProblem->Vehicle_)->vehicleID_].size());
        nbNegativeFound += subProblem->nbNegativeColumns_;
        runtimeMetrics_->updateSubproblemMetrics(subProblem);
 //       (*pLogEpochSubRuntimeStream_) << subProblem->toStringOut(epoch_);
    }


    // Clean up
    preprocessTime_->start();
    subProSolve.clear();
    preprocessTime_->stop();

    subProblemTime_->stop();
    runtimeMetrics_->subproEpochTime_ += subProblemTime_->dSinceStart().count();

    return subProBreak;
}

bool Solver::solve_SP_CPLEX(PInstance &EpochInst, PInstance &mainInst, int &iter, int &nbNegativeFound,
    vector2D<PRoute> &availableRoutes, float availableTime, int &nbRoutes) {
    Tools::PThreadsPool pPool = Tools::ThreadsPool::newThreadsPool(EpochInst->parameters_->nbThreads_);

    bool subProBreak = false;
    nbOnePick_ = 0;
    nbTwoPick_ = 0;
    nbThreePick_ = 0;

    // Start the subproblems timer
    subProblemTime_->start();

    // Define subproblems
    std::vector<PCplexSubPro> subProSolve;
    selectVehiclesForSubproblem(EpochInst, iter);

    // create subproblems
    int num = 0;

    for (auto &vehicleObj: EpochInst->vehicles_) {
        vehicleObj->bestReducedCost_ = INFINITY;

        if (EpochInst->selectedVehicles_[vehicleObj->vehicleID_] >= 1) {
            num++;
            subProSolve.emplace_back(std::make_shared<CPLEXSubProblem>(vehicleObj));
            availableRoutes[vehicleObj->vehicleID_].clear();
        }
    }

    // Solve CPLEX subproblems
    for (auto &subProblem: subProSolve) {
        if (subProBreak) break;
        // Check time constraint for dynamic mode
        if (EpochInst->parameters_->solutionMode_ == DYNAMIC &&
            availableTime - subProblemTime_->dSinceStart().count() <= 0 && iter > 1) {
            subProBreak = true;
            break;
        }
        Tools::Job job([&]() {
                try {
                    subProblem->initSubGraph(EpochInst);
                    // Initialize the CPLEX model
                    subProblem->initializeModel(EpochInst);

                    // Build the optimization model
                    subProblem->buildModel(EpochInst);

                    // Solve the subproblem
                    std::vector<PRoute> newRoutes;
                    subProblem->solveModel(EpochInst, newRoutes);

                    // Store generated routes
                    for (auto &route : newRoutes) {
                        if (route->reducedCost_ < -1e-6) { // Negative reduced cost threshold
                            availableRoutes[subProblem->Vehicle_->vehicleID_].push_back(route);
                            subProblem->nbNegativeColumns_++;
                        }
                        subProblem->nbGenerated_++;
                    }

                    // Update vehicle's best reduced cost
                    if (!newRoutes.empty()) {
                        float bestCost = newRoutes[0]->reducedCost_;
                        for (auto &route : newRoutes) {
                            bestCost = std::min(bestCost, route->reducedCost_);
                        }
                        subProblem->Vehicle_->bestReducedCost_ = bestCost;
                    }

                } catch (IloException& e) {
                    std::cerr << "CPLEX Exception in subproblem: " << e.getMessage() << std::endl;
                    subProBreak = true;
                } catch (...) {
                    std::cerr << "Unknown exception in CPLEX subproblem" << std::endl;
                    subProBreak = true;
                }
            });
        pPool->run(job);
    }
    // Wait for all jobs to complete
    while (true) {
        if (!pPool->wait())
            break;
    }

    // Collect results from subproblems
    nbNegativeFound = 0;
    for (auto &subProblem: subProSolve) {
        nbRoutes += static_cast<int>(availableRoutes[(subProblem->Vehicle_)->vehicleID_].size());
        nbNegativeFound += subProblem->nbNegativeColumns_;
        runtimeMetrics_->updateSubproblemMetrics(subProblem);
    }


    // Clean up
    preprocessTime_->start();
    subProSolve.clear();
    preprocessTime_->stop();

    subProblemTime_->stop();
    runtimeMetrics_->subproEpochTime_ += subProblemTime_->dSinceStart().count();
    return subProBreak;
}


void Solver::anyTimeSolver(PInstance &mainInst, InputPaths &inputPaths, bool middleSave,
                           float saveTime) {
    // define required variables
    epoch_ = 0;
    greedyRebalanceTime_ = 0;
    rebalancingTime_->start();
    std::vector<float> EpochTime = {1,1,1};

    // start simulation timer
    mainInst->setInitialTimes();
    for (int v = 0; v < mainInst->nbVehicles_; ++v) {
        mainInst->vehicles_[v]->setEmptyRoute(mainInst);
        mainInst->vehicles_[v]->setSolutionRoute();
        if (mainInst->vehicles_[v]->currentRoute_ == nullptr)
            mainInst->vehicles_[v]->setCurrentRoute(mainInst->vehicles_[v]->emptyRoute_);
    }

    int nbReceivedRequest = mainInst->nbOnboards_;
    PInstance EpochInst = std::make_shared<Instance>(*mainInst);
    float preObjective = objValue_;
    bool skip = false;
    while (nbReceivedRequest < mainInst->nbRequests_ ||
        (CG_Model_ && !CG_Model_->zSolution_.empty())||
        (ISUD_Model_ && !ISUD_Model_->zSolution_.empty())) {
        nextEpoch:
        simulationTime_->start();
        //       preprocessTime_->start();
        elapsedTime_ = simulationTime_->dSinceInit().count();
        std::cout << "---------------------"<< std::endl;
        std::cout << " ELAPSED TIME: " << elapsedTime_ << std::endl;
        std::cout << " PRE EPOCH TIME: " << runtimeMetrics_->epochRuntime_ << std::endl;
        std::cout << " EPOCH: " << epoch_ << std::endl;
        std::cout << "---------------------"<< std::endl;
        std::ofstream logFile(inputPaths.getOutputSolverLog(), std::ofstream::app);
        logFile << "---------------------------------------------------"<< std::endl;
        logFile << " EPOCH: " << epoch_ << std::endl;
        logFile.close();
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

//        mainInst->parameters_->committedTime_ = commitTime;

        // update vehicle status
        mainInst->nbOnboards_ = 0;
        boost::dynamic_bitset<> removedRequests;
        removedRequests.resize(EpochInst->nbRequests_);

        for (auto &vehicleObj: mainInst->vehicles_) {
//            vehicleObj->updateCurrentRoute(elapsedTime_);
            vehicleObj->updateStateTime(mainInst, mainInst->simulationStartTime_ + elapsedTime_, removedRequests);
            mainInst->nbOnboards_ += static_cast<int>(vehicleObj->onboards_.size());
        }

        if (mainInst->parameters_->routeRecycle_ ) {
            if (mainInst->parameters_->approach_ == ISUD && !ISUD_Model_->availableRoutes_.empty()) {
                for (auto &vehicleObj: mainInst->vehicles_) {
                    if (vehicleObj->stateChanged_)
                        ISUD_Model_->availableRoutes_[vehicleObj->vehicleID_].clear();
                }
                if (removedRequests.count()) {
                    updateAvailableRoutes(removedRequests, ISUD_Model_->availableRoutes_);
                }

            }
            else if (mainInst->parameters_->approach_ == CG && !CG_Model_->availableRoutes_.empty()) {
                for (auto &vehicleObj: mainInst->vehicles_) {
                    if (vehicleObj->stateChanged_)
                        CG_Model_->availableRoutes_[vehicleObj->vehicleID_].clear();
                }
                if (removedRequests.count()) {
                    updateAvailableRoutes(removedRequests, CG_Model_->availableRoutes_);
                }
            }
        }

        // resetting a subInstance
        EpochInst->resetInstance();
        // reading the data received in the previous epoch
        if (mainInst->parameters_->approach_ == ISUD) {
            EpochInst->buildPartialData(mainInst, ISUD_Model_->zSolution_ , elapsedTime_, nbReceivedRequest);
            if (EpochInst->parameters_->timeWindow_ == 0)
                EpochInst->updatePenalties(elapsedTime_);
            if (mainInst->parameters_->routeRecycle_ && !ISUD_Model_->availableRoutes_.empty())
                reconstructAvailableRoutes(mainInst, ISUD_Model_->availableRoutes_);
        }
        else if (mainInst->parameters_->approach_ == CG) {
            EpochInst->buildPartialData(mainInst, CG_Model_->zSolution_ , elapsedTime_, nbReceivedRequest);
            if (EpochInst->parameters_->timeWindow_ == 0)
                EpochInst->updatePenalties(elapsedTime_);
            if (mainInst->parameters_->routeRecycle_ && !CG_Model_->availableRoutes_.empty())
                reconstructAvailableRoutes(mainInst, CG_Model_->availableRoutes_);
        }
        else
            EpochInst->buildPartialData(mainInst, elapsedTime_, nbReceivedRequest);
        for (auto &vehicleObj: mainInst->vehicles_){
            if (EpochInst->nbRequests_ != 0) {
                vehicleObj->currentRoute_->createColumn(EpochInst->nbRequests_);
                vehicleObj->emptyRoute_->createColumn(EpochInst->nbRequests_);
            }
        }
        if (epoch_ == 0 && mainInst->nbOnboards_ > 0)
            nbReceivedRequest += EpochInst->nbRequests_;
        else
            nbReceivedRequest += EpochInst->nbNewRequests_;
        std::cout << "# TOTAL NUMBER OF REQUESTS: " << EpochInst->nbRequests_ << std::endl;
        if (elapsedTime_ >= saveTime && middleSave) {
            if (EpochInst->parameters_->mainAlgorithm_ == GREEDY){
                for (auto & requestObj: EpochInst->requests_)
                    requestObj->dual_ = requestObj->penalty_;
            }
            /*inputPaths.makeInstanceOutput(instNum);
            mainInst->saveStatus(inputPaths, EpochInst->simulationStartTime_ + elapsedTime_,1.5 * mainInst->parameters_->epochLength_);*/
            inputPaths.makeInstanceOutput("1");
            mainInst->saveStatus(inputPaths, EpochInst->simulationStartTime_ + elapsedTime_,3600*5);
            break;
        }
        std::cout << "# TOTAL NUMBER OF NEW REQUESTS: " << EpochInst->nbNewRequests_ << std::endl;
        if (EpochInst->nbRequests_ == 0 ||
            (EpochInst->nbNewRequests_ == 0 && (skip || mainInst->parameters_->mainAlgorithm_  == GREEDY))) {
            if (mainInst->parameters_->approach_ == CG)
                CG_Model_->availableRoutes_.clear();
            else if(mainInst->parameters_->approach_ == ISUD)
                ISUD_Model_->availableRoutes_.clear();
            simulationTime_->stop();
            simulationTime_->addTime(mainInst->requests_[nbReceivedRequest]->requestTime_ - mainInst->simulationStartTime_ - simulationTime_->dSinceInit().count());
            skip = false;
            goto nextEpoch;
        }
//        preprocessTime_->stop();
        if (EpochInst->parameters_->mainAlgorithm_ != GREEDY)
            solve_Epoch(EpochInst, mainInst, inputPaths);
        else if (EpochInst->parameters_->mainAlgorithm_ == GREEDY) {
            GreedyModel_->GreedySolver(EpochInst);
            if (EpochInst->parameters_->vehicleReturn_) {
                if (elapsedTime_ - greedyRebalanceTime_ >= EpochInst->parameters_->epochLength_) {
                    if (EpochInst->parameters_->returnPolicy_ == TO_SOURCE)
                        returnVehicles(EpochInst);
                    else if (EpochInst->parameters_->returnPolicy_ == ZONE)
                        returnVehiclesZone(EpochInst);
                    else {
                        greedyRebalanceTime_ = elapsedTime_;
                        returnVehiclesAlonso(mainInst);
                    }
                }
            }
            /*if (EpochInst->parameters_->solutionMode_ == ANYTIME){
                for (auto &vehicleObj: EpochInst->vehicles_){
                    if (vehicleObj->currentRoute_->routeSize_ > 1 && vehicleObj->idle_){
                        vehicleObj->updateCurrentRoute(EpochInst->simulationStartTime_ + elapsedTime_+ simulationTime_->dSinceStart().count());
                    }
                }
            }*/
        }
        if (preObjective != objValue_)
            skip = false;
        else
            skip = true;

        preObjective = objValue_;
        EpochInst->setAssignedEpochVehicles(EpochInst->simulationStartTime_ + elapsedTime_ + simulationTime_->dSinceInit().count());
        if (EpochInst->parameters_->mainAlgorithm_ != GREEDY)
            *pLogRunTimesStream_ << saveRuntimes(EpochInst);
        epoch_++;
        simulationTime_->stop();
    }
    elapsedTime_ = simulationTime_->dSinceInit().count();
    for (auto & vehicleObj : mainInst->vehicles_) {
        vehicleObj->finalizeSolutionRoutes(mainInst->simulationStartTime_ + elapsedTime_);
    }
    rebalancingTime_->stop();
}

void Solver::staticSolver(PInstance &mainInst, InputPaths &inputPaths, bool middleSave, float saveTime) {
    // define required variables
    epoch_ = 0;

    // start simulation timer
    mainInst->setInitialTimes();
    for (int v = 0; v < mainInst->nbVehicles_; ++v) {
        mainInst->vehicles_[v]->setEmptyRoute(mainInst);
        mainInst->vehicles_[v]->setSolutionRoute();
        if (mainInst->vehicles_[v]->currentRoute_ == nullptr)
            mainInst->vehicles_[v]->setCurrentRoute(mainInst->vehicles_[v]->emptyRoute_);
    }

    simulationTime_->start();
//    preprocessTime_->start();
    int nbReceivedRequest = mainInst->nbOnboards_;
    PInstance StaticInst = std::make_shared<Instance>(*mainInst);
    StaticInst->buildStaticData(mainInst, nbReceivedRequest);

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
        case MIP_CPLEX :
            MIPModel_ = std::make_unique<MIPSolver>();
            MIPModel_->SolveMIP(StaticInst, inputPaths);
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
            solve_Epoch(StaticInst, mainInst, inputPaths);
            break;
    }
    StaticInst->setAssignedEpochVehicles(simulationTime_->dSinceInit().count());
    simulationTime_->stop();
    if (!middleSave && StaticInst->parameters_->mainAlgorithm_ != GREEDY) {
        for (auto & vehicleObj : mainInst->vehicles_) {
            vehicleObj->departNode_->departTime_ = vehicleObj->currentRoute_->plannedDepartTime_[0];
            vehicleObj->finalizeSolutionRoutes(mainInst->simulationStartTime_ + saveTime);
        }
    }
}

void Solver::dynamicSolver(PInstance &mainInst, InputPaths &inputPaths, bool middleSave, float saveTime) {
    // define required variables
    epoch_ = 0;
    rebalancingTime_->start();
    int instance_count = mainInst->nbVehicles_;

    mainInst->setInitialTimes();
    for (int v = 0; v < mainInst->nbVehicles_; ++v) {
        mainInst->vehicles_[v]->setEmptyRoute(mainInst);
        mainInst->vehicles_[v]->setSolutionRoute();
        if (mainInst->vehicles_[v]->currentRoute_ == nullptr)
            mainInst->vehicles_[v]->setCurrentRoute(mainInst->vehicles_[v]->emptyRoute_);
    }

    int nbReceivedRequest = mainInst->nbOnboards_;
    PInstance EpochInst = std::make_shared<Instance>(*mainInst);
    float preObjective = objValue_;
    bool skip = false;
    while (nbReceivedRequest < mainInst->nbRequests_){
        /*while ((!solveEpoch && (nbReceivedRequest < mainInst->nbRequests_ || !masterModel_->zSolution_.empty())) ||
           (solveEpoch && nbReceivedRequest < mainInst->nbRequests_)) {*/
        nextEpoch:
        // start simulation timer
        simulationTime_->start();
//        preprocessTime_->start();

        elapsedTime_ = simulationTime_->dSinceInit().count();
        std::cout << "---------------------"<< std::endl;
        std::cout << " ELAPSED TIME: " << elapsedTime_ << std::endl;
        std::cout << " EPOCH: " << epoch_ << std::endl;
        std::cout << "---------------------"<< std::endl;

        std::ofstream logFile(inputPaths.getOutputSolverLog(), std::ofstream::app);
        logFile << "---------------------------------------------------"<< std::endl;
        logFile << " EPOCH: " << epoch_ << std::endl;
        logFile.close();
        // update vehicle status
        mainInst->nbOnboards_ = 0;
        boost::dynamic_bitset<> removedRequests;
        removedRequests.resize(EpochInst->nbRequests_);

        for (auto &vehicleObj: mainInst->vehicles_) {
            vehicleObj->updateStateTime(mainInst, static_cast<float>(mainInst->simulationStartTime_) +
                                        static_cast<float>(epoch_) * mainInst->parameters_->epochLength_, removedRequests);
            mainInst->nbOnboards_ += static_cast<int>(vehicleObj->onboards_.size());
        }
        if (mainInst->parameters_->routeRecycle_) {
            if (mainInst->parameters_->approach_ == ISUD && ISUD_Model_->availableRoutes_.size() > 0) {
                for (auto &vehicleObj: mainInst->vehicles_) {
                    if (vehicleObj->stateChanged_)
                        ISUD_Model_->availableRoutes_[vehicleObj->vehicleID_].clear();
                }
                if (removedRequests.count()) {
                    updateAvailableRoutes(removedRequests, ISUD_Model_->availableRoutes_);
                }

            }
            else if (mainInst->parameters_->approach_ == CG && CG_Model_->availableRoutes_.size() > 0) {
                for (auto &vehicleObj: mainInst->vehicles_) {
                    if (vehicleObj->stateChanged_)
                        CG_Model_->availableRoutes_[vehicleObj->vehicleID_].clear();
                }
                if (removedRequests.count()) {
                    updateAvailableRoutes(removedRequests, CG_Model_->availableRoutes_);
                }
            }
        }
        // resetting a subInstance
        EpochInst->resetInstance();
        // reading the data received in the previous epoch

        if (mainInst->parameters_->approach_ == ISUD) {
            EpochInst->buildPartialData(mainInst, ISUD_Model_->zSolution_ , static_cast<float>(epoch_) *
                mainInst->parameters_->epochLength_, nbReceivedRequest);
            if (EpochInst->parameters_->timeWindow_ == 0)
                EpochInst->updatePenalties(mainInst->parameters_->epochLength_ * static_cast<float>(epoch_));
            if (mainInst->parameters_->routeRecycle_ && !ISUD_Model_->availableRoutes_.empty())
                reconstructAvailableRoutes(mainInst, ISUD_Model_->availableRoutes_);
        }
        else if (mainInst->parameters_->approach_ == CG) {
            EpochInst->buildPartialData(mainInst, CG_Model_->zSolution_ , static_cast<float>(epoch_) *
            mainInst->parameters_->epochLength_, nbReceivedRequest);
            if (EpochInst->parameters_->timeWindow_ == 0)
                EpochInst->updatePenalties(mainInst->parameters_->epochLength_ * static_cast<float>(epoch_));
            if (mainInst->parameters_->routeRecycle_ && !CG_Model_->availableRoutes_.empty())
                reconstructAvailableRoutes(mainInst, CG_Model_->availableRoutes_);
        }
        else
            EpochInst->buildPartialData(mainInst, static_cast<float>(epoch_) * mainInst->parameters_->epochLength_, nbReceivedRequest);

        for (auto &vehicleObj: mainInst->vehicles_){
            if (EpochInst->nbRequests_ != 0) {
                vehicleObj->currentRoute_->createColumn(EpochInst->nbRequests_);
                vehicleObj->emptyRoute_->createColumn(EpochInst->nbRequests_);
            }
        }
        if (epoch_ == 0 && mainInst->nbOnboards_ > 0)
            nbReceivedRequest += EpochInst->nbRequests_;
        else
            nbReceivedRequest += EpochInst->nbNewRequests_;
        std::cout << "# TOTAL NUMBER OF RECEIVED REQUESTS: " << EpochInst->nbRequests_ << std::endl;
        std::cout << "# TOTAL NUMBER OF NEW REQUESTS: " << EpochInst->nbNewRequests_ << std::endl;
        // saving the status in the middle of running
        if ((static_cast<float> (epoch_) * EpochInst->parameters_->epochLength_ >= saveTime) && middleSave ) {
            inputPaths.makeInstanceOutput(std::to_string(instance_count));
            mainInst->saveStatus(inputPaths, EpochInst->simulationStartTime_ +
                                             static_cast<float>(epoch_) * EpochInst->parameters_->epochLength_,mainInst->parameters_->epochLength_);

 //           saveTime += 60;
            /*if (instance_count >= 30)
                break;*/
//            instance_count ++;


                      break;
        }

        if (EpochInst->nbRequests_ == 0 || (EpochInst->nbNewRequests_ == 0 && skip)) {
            if (mainInst->parameters_->approach_ == CG)
                CG_Model_->availableRoutes_.clear();
            else if(mainInst->parameters_->approach_ == ISUD)
                ISUD_Model_->availableRoutes_.clear();
            simulationTime_->stop();
            epoch_++;
            skip = false;
            goto nextEpoch;
        }
        //       preprocessTime_->stop();

        switch (EpochInst->parameters_->mainAlgorithm_) {
            case GREEDY:
                GreedyModel_->GreedySolver(EpochInst);
                if (EpochInst->parameters_->vehicleReturn_) {
                    if (EpochInst->parameters_->returnPolicy_ == TO_SOURCE)
                        returnVehicles(EpochInst);
                    else if (EpochInst->parameters_->returnPolicy_ == ZONE)
                        returnVehiclesZone(EpochInst);
                    else {
                        returnVehiclesAlonso(mainInst);
                    }
                }
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
                solve_Epoch(EpochInst, mainInst, inputPaths);
                break;
        }

        //       (*pLogEpochSolutionStream_) << EpochInst->saveEpochRoutes( epoch_);
        if (preObjective != objValue_)
            skip = false;
        else
            skip = true;

        preObjective = objValue_;
        EpochInst->setAssignedEpochVehicles(mainInst->simulationStartTime_
                                        + static_cast<float>(epoch_ + 1) * mainInst->parameters_->epochLength_);
        if (EpochInst->parameters_->mainAlgorithm_ != GREEDY)
            *pLogRunTimesStream_ << saveRuntimes(EpochInst);
        else
            *pLogRunTimesStream_ << saveRuntimesGreedy(EpochInst);
        epoch_++;
        simulationTime_->stop();
    }

    for (auto & vehicleObj : mainInst->vehicles_) {
        vehicleObj->finalizeSolutionRoutes(mainInst->simulationStartTime_
                                           + static_cast<float>(epoch_+1) * mainInst->parameters_->epochLength_);
    }
    rebalancingTime_->stop();

}

// function to print epoch runTime to file
std::string Solver::saveRuntimes(const PInstance &EpochInst) {
    std::stringstream repStr;
    runtimeMetrics_->epochRuntime_ = simulationTime_->dSinceStart().count();
    avgEpochRuntime_ = simulationTime_->dSinceInit().count() / static_cast<float>(epoch_ + 1);
    float upperbound = 0.0;
    repStr << epoch_ << ","
           << EpochInst->nbRequests_ << ","
           << EpochInst->nbNewRequests_ << ","
           << EpochInst->instGraph_->nbNodes_ - 2 * EpochInst->nbVehicles_ << ","
           << runtimeMetrics_->epochRuntime_ << ","
           << simulationTime_->dSinceInit().count() << ",";

    // master problem Times
    if (EpochInst->parameters_->approach_ == CG) {
        repStr << CG_Model_->runtimesToString(runtimeMetrics_);
        upperbound = CG_Model_->upperbound_;
    }
    else if (EpochInst->parameters_->approach_ == ISUD) {
        repStr << ISUD_Model_->runtimesToString(runtimeMetrics_);
        upperbound = ISUD_Model_->upperbound_;
    }
    EpochInst->calcVehicleMetric();

    repStr << preprocessTime_ ->dSinceStart().count()<< ","
           << GreedyModel_->greedyTime_->dSinceInit().count() - runtimeMetrics_->GreedyTime_ << ","
           << EpochInst->nbReturn_ << ","
           << EpochInst->nbIdle_ << ","
           << EpochInst->passPerVehicle_ << ","
           << EpochInst->requestPerVehicle_ << ","
           << EpochInst->nodePerVehicle_ << ","
           << EpochInst->nbStateChanged_ << ","
           << nbOnePick_ << ","
           << nbTwoPick_ << ","
           << nbThreePick_ << ","
           << heuristicCG_ << ","
           << upperbound << ","
           << nbRecycle_ <<"\n";
    runtimeMetrics_->GreedyTime_ = GreedyModel_->greedyTime_->dSinceInit().count();
    return repStr.str();
}

std::string Solver::saveRuntimesGreedy(const PInstance &EpochInst) {
    std::stringstream repStr;
    runtimeMetrics_->epochRuntime_ = simulationTime_->dSinceStart().count();
    avgEpochRuntime_ = simulationTime_->dSinceInit().count() / static_cast<float>(epoch_ + 1);
    repStr << epoch_ << ","
           << EpochInst->nbRequests_ << ","
           << EpochInst->nbNewRequests_ << ","
           << EpochInst->instGraph_->nbNodes_ - 2 * EpochInst->nbVehicles_ << ","
           << runtimeMetrics_->epochRuntime_ << ","
           << simulationTime_->dSinceInit().count() << ","
           << GreedyModel_->objValue_ << ","
           << GreedyModel_->objValue_ << ","
           << GreedyModel_->objValue_ << ","
           << GreedyModel_->greedyTime_->dSinceInit().count() - runtimeMetrics_->GreedyTime_ << ",";

    EpochInst->calcVehicleMetric();

    repStr << EpochInst->nbReturn_ << ","
           << EpochInst->nbIdle_ << ","
           << EpochInst->passPerVehicle_ << ","
           << EpochInst->requestPerVehicle_ << ","
           << EpochInst->nodePerVehicle_ <<"\n";
    runtimeMetrics_->GreedyTime_ = GreedyModel_->greedyTime_->dSinceInit().count();
    return repStr.str();
}

std::string Solver::toString(const PInstance & mainInst) const {
    std::stringstream repStr;
    repStr << "*************************************************************************************" << std::endl;
    repStr << "                        FINAL VEHICLE ROUTES AFTER " << std::setw(3) << epoch_ << " EPOCHS " << std::endl;
    repStr << "                                    " <<  eu::toString(mainInst->parameters_->solutionMode_) << " MODE" << std::endl;
    repStr << "*************************************************************************************" << std::endl;
    repStr << std::endl << std::endl;
    repStr << std::left << std::fixed << std::setprecision(2);
    repStr << std::setw(30) << "# NUMBER OF VEHICLES" << " = " << mainInst->nbVehicles_ << std::endl;
    repStr << "# " << std::endl;
    repStr << "############################   PARAMETERS AND OPTIONS   ############################" << std::endl;
    repStr << mainInst->parameters_->toString();
    repStr << "# " << std::endl;
    repStr << std::endl << std::endl;
    repStr << "################################   FINAL ROUTES   ################################" << std::endl;
    repStr << "#" << std::left << std::endl;
    for (auto & vehicleObj : mainInst->vehicles_) {
        if (vehicleObj->solutionRoute_->routeSize_ > 1) {
            repStr << "#" << std::left << std::endl;
            repStr << "#\t" << std::setw(24) << "- IDLE_TIME" << " : " << vehicleObj->idleTime_ << std::endl;
            repStr << vehicleObj->solutionRoute_->toString();
        }
    }
    repStr << std::endl << std::endl;
    repStr << "#################################   REQUESTS    #################################" << std::endl;
    repStr << "#" << std::endl;
    repStr << mainInst->solutionToString();
    repStr << std::left << std::fixed << std::setprecision(2);
    if (mainInst->parameters_->approach_ == CG)
        repStr << CG_Model_->toStringTimersTotal();
    else if (mainInst->parameters_->approach_ == ISUD)
        repStr << ISUD_Model_->toStringTimersTotal();
    repStr << std::setw(SENTENCE_SIZE) << "# TIME SPENT ON SOLVING SUB PROBLEMS" << " = " << subProblemTime_->dSinceInit().count() << " (s)" << std::endl;
    repStr << std::setw(SENTENCE_SIZE) << "# TIME SPENT ON GREEDY" << " = " << GreedyModel_->greedyTime_->dSinceInit().count() << " (s)" << std::endl;
    repStr << std::setw(SENTENCE_SIZE) << "# TIME SPENT ON ASSIGNMENT" << " = " << GreedyModel_->greedyAssignTime_->dSinceInit().count() << " (s)" << std::endl;
    if (mainInst->parameters_->approach_ == CG)
        repStr << CG_Model_->toStringTimersAvg(epoch_);
    else if (mainInst->parameters_->approach_ == ISUD)
        repStr << ISUD_Model_->toStringTimersAvg(epoch_);
    repStr << std::setw(SENTENCE_SIZE) << "# TIME SPENT ON SOLVING SUB PROBLEMS" << " = " << subProblemTime_->dSinceInit().count()/static_cast<float>(epoch_) << " (s)" << std::endl;
    repStr << std::setw(SENTENCE_SIZE) << "# TIME SPENT ON GREEDY" << " = " << GreedyModel_->greedyTime_->dSinceInit().count()/static_cast<float>(epoch_) << " (s)" << std::endl;
    repStr << std::setw(SENTENCE_SIZE) << "# TIME SPENT ON ASSIGNMENT" << " = " << GreedyModel_->greedyAssignTime_->dSinceInit().count()/static_cast<float>(epoch_) << " (s)" << std::endl;
    mainInst->instRepStr_ << epoch_-1 << "," ;
    if (mainInst->parameters_->approach_ == CG)
        CG_Model_->createFinalOutputString(mainInst, subProblemTime_->dSinceInit().count(),
            GreedyModel_->greedyTime_->dSinceInit().count());
    else if (mainInst->parameters_->approach_ == ISUD)
        ISUD_Model_->createFinalOutputString(mainInst, subProblemTime_->dSinceInit().count(),
            GreedyModel_->greedyTime_->dSinceInit().count());
    return repStr.str();
}

void Solver::solve(PInstance &mainInst, InputPaths &inputPaths, bool middleSave, float saveTime) {
    try {
        switch (mainInst->parameters_->solutionMode_) {
            case DYNAMIC:
                dynamicSolver(mainInst, inputPaths, middleSave, saveTime);
                break;

            case ANYTIME:
                anyTimeSolver(mainInst, inputPaths, middleSave, saveTime);
                break;

            case STATIC:
                staticSolver(mainInst, inputPaths, middleSave, saveTime);
                break;

            default:
                throw std::runtime_error("Unsupported solution mode: " +
                                       std::to_string(mainInst->parameters_->solutionMode_));
        }
    } catch (const std::exception& e) {
        std::cerr << "Error in " << eu::toString(mainInst->parameters_->solutionMode_)
                  << " solver: " << e.what() << std::endl;
        throw; // Re-throw to allow caller to handle
    } catch (...) {
        std::cerr << "Unknown error in " << eu::toString(mainInst->parameters_->solutionMode_)
                  << " solver" << std::endl;
        throw std::runtime_error("Unknown solver error");
    }
}

void RuntimeMetrics::resetSubproblemMetrics() {
    subproEpochTime_ = 0;
    nbNegativeFound_ = 0;
    nbGenerated_ = 0;
    nbDominated_ = 0;
    nbPrunedArcs_ = 0;
    nbPrunedPath_ = 0;
    nbEliminated_ = 0;
    nbRecycledColumns_ = 0;
}

void RuntimeMetrics::updateSubproblemMetrics(PLabelingSubPro &subProblem) {
    nbNegativeFound_ += subProblem->nbNegativeColumns_;
    nbGenerated_ += subProblem->nbGenerated_;
    nbDominated_ += subProblem->nbDominated_;
    nbEliminated_ += subProblem->nbEliminated_;
    nbPrunedPath_ += subProblem->nbPrunedPath_;
    nbPrunedArcs_ += subProblem->nbPrunedArcs_;
    nbRecycledColumns_ += subProblem->nbRecycledColumns_;
}

void RuntimeMetrics::updateSubproblemMetrics(PCplexSubPro &subProblem) {
    nbNegativeFound_ += subProblem->nbNegativeColumns_;
    nbGenerated_ += subProblem->nbGenerated_;
}

void Solver::CreateOneStopRoutes(const PVehicle &vehicle, vector<PRoute> &availableRoutes, const PInstance &pInst,
                                 const PInstance &EpochInst, int &nbNegative) {
    for (auto & requestObj : EpochInst->requests_) {
        PRoute newRoute = std::make_shared<Route>(vehicle->vehicleID_);
        newRoute->addSource(vehicle->departNode_, vehicle->departTime_, vehicle->numPassengers_);

        if (!vehicle->onboards_.empty()) {
            for (auto & nodeId : vehicle->onboards_){
                newRoute->addNode(pInst->instGraph_->nodes_[nodeId]);
            }
        }
        newRoute->addNode(pInst->instGraph_->pickNodes_[requestObj->getRequestId()]);
        newRoute->addNode(pInst->instGraph_->dropNodes_[requestObj->getRequestId()]);
        newRoute->reducedCost_ = newRoute->totalDelay_ - requestObj->dual_ - vehicle->dual_;
        newRoute->score_ = newRoute->reducedCost_ / static_cast<float>(newRoute->routeRequests_.size());
        newRoute->lambda_ = newRoute->totalDelay_/(newRoute->totalDelay_ - newRoute->reducedCost_ + 0.001f);
        newRoute->totalLength_ = newRoute->plannedDepartTime_.back() - vehicle->departTime_;
        if (newRoute->reducedCost_ < 0)
            nbNegative++;
        availableRoutes.emplace_back(std::move(newRoute));
        availableRoutes.back()->createColumn(EpochInst->nbRequests_);
    }
}


void Solver::updateAvailableRoutes(boost::dynamic_bitset<> &removedRequests, vector2D<PRoute> &availableRoutes) {
    for (auto &vehicleRoutes : availableRoutes) {
        // Remove routes with overlapping requests or empty route requests
        for (auto &route : vehicleRoutes) {
            if (route->column_.size() != removedRequests.size())
                std::cout << "error";
        }
        vehicleRoutes.erase(
                std::remove_if(vehicleRoutes.begin(), vehicleRoutes.end(),
                               [&removedRequests](const PRoute &routeObj) {
                                   return !(routeObj->column_ & removedRequests).none() || routeObj->routeRequests_.empty();
                               }),
                vehicleRoutes.end()
        );
    }
}

void Solver::reconstructAvailableRoutes(const PInstance &mainInst, vector2D<PRoute> &availableRoutes) {
    for (auto & vehicleObj : mainInst->vehicles_) {
        if (!availableRoutes[vehicleObj->vehicleID_].empty()) {
            for (int i = availableRoutes[vehicleObj->vehicleID_].size() - 1; i >= 0; --i){
                if (!availableRoutes[vehicleObj->vehicleID_][i]->reConstructRoute(vehicleObj))
                    availableRoutes[vehicleObj->vehicleID_].erase(availableRoutes[vehicleObj->vehicleID_].begin() + i);
            }
        }
    }
}
void Solver::returnVehicles(const PInstance & EpochInst) const {
    float lastEpoch = 0;
    if (EpochInst->parameters_->solutionMode_ == ANYTIME)
        lastEpoch = EpochInst->simulationStartTime_ + elapsedTime_ - EpochInst->parameters_->committedTime_;
    else
        lastEpoch = EpochInst->simulationStartTime_ + static_cast<float> (epoch_) * EpochInst->parameters_->epochLength_ - EpochInst->parameters_->epochLength_;

    // Return Idle Vehicles
    if (EpochInst->parameters_->vehicleReturn_) {
        for (auto &vehicleObj: EpochInst->vehicles_) {
            if (vehicleObj->currentRoute_->routeSize_ == 1 && vehicleObj->currentRoute_->plannedReachTime_[0]+
                vehicleObj->currentRoute_->routeNodes_.back()->serviceTime_ < lastEpoch) {
                if (vehicleObj->currentRoute_->routeNodes_.back()->locationID_ != vehicleObj->sinkNode_->locationID_){
                    PNode sinkNode = std::make_shared<Node>(vehicleObj->sinkNode_);
                    vehicleObj->currentRoute_->addSink(sinkNode);
                    if (EpochInst->parameters_->solutionMode_ == ANYTIME)
                        vehicleObj->updateCurrentRoute(EpochInst->simulationStartTime_ + elapsedTime_+ simulationTime_->dSinceStart().count());
                }
            }
        }
    }
}


void Solver::returnVehiclesZone(const PInstance & EpochInst) const {
    if (EpochInst->parameters_->vehicleReturn_) {
        float lastEpoch = 0;
        if (EpochInst->parameters_->solutionMode_ == ANYTIME)
            lastEpoch = EpochInst->simulationStartTime_ + elapsedTime_ - EpochInst->parameters_->WaitForReturn_;
        else
            lastEpoch = EpochInst->simulationStartTime_ + static_cast<float> (epoch_) * EpochInst->parameters_->epochLength_ - EpochInst->parameters_->WaitForReturn_;

        for (auto & zoneObj : EpochInst->zones_) {
            zoneObj.second->nbVehicles_ = 0;
        }

        for (auto &vehicleObj: EpochInst->vehicles_) {
            EpochInst->zones_[vehicleObj->currentRoute_->routeNodes_.back()->zoneID_]->nbVehicles_++;
        }
        for (auto &zoneObj : EpochInst->zones_) {
            if (zoneObj.second->nbVehicles_ < zoneObj.second->nbVehiclesRef_ * 0.95 && zoneObj.second->highDemandZone_) {
                zoneObj.second->underCapacity_ = true;
            }
            else
                zoneObj.second->underCapacity_ = false;
        }

        // Return Idle Vehicles
        std::vector<PVehicle> idleVehicles;

        for (auto &vehicleObj: EpochInst->vehicles_) {
            if (vehicleObj->currentRoute_->routeSize_ == 1 && vehicleObj->currentRoute_->plannedReachTime_[0]+
                vehicleObj->currentRoute_->routeNodes_.back()->serviceTime_ < lastEpoch) {
                if (!EpochInst->zones_[vehicleObj->departNode_->zoneID_]->underCapacity_ ) {
                    idleVehicles.push_back(vehicleObj);
                }
            }
        }

        if (!idleVehicles.empty()) {
            for (auto &vehicleObj : idleVehicles) {
                // Retrieve the current zone from the vehicle's last route node.
                int currentZoneID = vehicleObj->currentRoute_->routeNodes_.back()->zoneID_;
                PZone currentZone = EpochInst->zones_[currentZoneID];

                // Iterate over the current zone's successors (assumed to be ordered by proximity).
                for (Zone* successor : currentZone->successors_) {
                    // Check if the successor zone is under capacity (i.e., less than 90% of its reference).
                    if (successor->nbVehicles_ < successor->nbVehiclesRef_ * 0.95 && successor->highDemandZone_ &&
                        vehicleObj->sinkNode_->zoneID_ != successor->zoneID_) {
                        PNode sinkNode = std::make_shared<Node>(vehicleObj->sinkNode_);
                        sinkNode->zoneID_ = static_cast<int>(successor->zoneID_);
                        sinkNode->locationID_ = successor->centerLocationID_;
                        successor->nbVehicles_++; // Update vehicle count.
                        vehicleObj->currentRoute_->addSink(sinkNode);
                        if (EpochInst->parameters_->solutionMode_ == ANYTIME)
                            vehicleObj->updateCurrentRoute(EpochInst->simulationStartTime_
                                + elapsedTime_ + simulationTime_->dSinceStart().count());
                        break; // Assign to the nearest eligible successor.
                        }
                }
                // If no eligible successor is found, the vehicle remains unassigned.
            }
        }
    }
}


void Solver::returnVehiclesAssign(const PInstance & EpochInst) const {
    if (!EpochInst->parameters_->vehicleReturn_) return;
    /* ---------- 1. reference time for “idle” test ---------- */

    float epochStartTime = 0;
    if (EpochInst->parameters_->solutionMode_ == ANYTIME)
        epochStartTime = EpochInst->simulationStartTime_ + elapsedTime_;
    else
        epochStartTime = EpochInst->simulationStartTime_ + static_cast<float> (epoch_) * EpochInst->parameters_->epochLength_;
    float lastEpoch = epochStartTime - EpochInst->parameters_->WaitForReturn_;

    /* ---------- 2. collect idle vehicles ---------- */
    std::vector<PVehicle> idleVehicles;
    for (auto &vehicleObj: EpochInst->vehicles_) {
        if (vehicleObj->currentRoute_->routeSize_ == 1 && vehicleObj->currentRoute_->plannedReachTime_[0]+
            vehicleObj->currentRoute_->routeNodes_.back()->serviceTime_ < lastEpoch &&
            !EpochInst->zones_[vehicleObj->departNode_->zoneID_]->highDemandZone_) {
            idleVehicles.push_back(vehicleObj);
        }
    }
    if (idleVehicles.empty()) return;
    int totalNeed = 0;
    /* ---------- 3. compute zone weights ---------- */
    for (auto & zoneObj : EpochInst->zones_)
        zoneObj.second->nbUnserved_ = 0;

    for (auto &requestObj: EpochInst->requests_) {
        if (requestObj->committedPickTime_ == LARGE_CONSTANT)
            EpochInst->zones_[requestObj->pickZoneID_]->nbUnserved_++;
    }
    /* keep zones with positive demand */
    std::vector<int> zoneIDs;
    for (auto &zp : EpochInst->zones_) {
        if (zp.second->nbUnserved_ > 0) {
            zoneIDs.push_back(zp.first);
            totalNeed += zp.second->nbUnserved_;
            std::cout << "Zone: " << zp.first << " - " << zp.second->nbUnserved_ << std::endl;
        }
    }
    if (zoneIDs.empty()) return;

/* ---------- 4. build and solve assignment MIP ---------- */
    IloEnv   env;
    try {
        const std::size_t nV = idleVehicles.size();
        const std::size_t nZ = zoneIDs.size();

        IloModel model(env);
        IloArray<IloBoolVarArray> y(env, nV);   // y[v][z]

        for (std::size_t v = 0; v < nV; ++v) {
            y[v] = IloBoolVarArray(env, nZ);
            for (std::size_t z = 0; z < nZ; ++z)
                y[v][z] = IloBoolVar(env);
        }

        /* objective */
        IloExpr obj(env);
        for (std::size_t v = 0; v < nV; ++v) {
            int vehLoc = idleVehicles[v]->departNode_->locationID_;
            for (std::size_t z = 0; z < nZ; ++z) {
                float distance = durationMatrix_[vehLoc][EpochInst->zones_[zoneIDs[z]]->centerLocationID_];
                float cost = distance / EpochInst->zones_[zoneIDs[z]]->nbUnserved_;
                obj += cost * y[v][z];
            }
        }
        model.add(IloMinimize(env, obj)); obj.end();
        IloExpr total(env);
        /* constraints: (1) exactly one zone per vehicle */
        for (std::size_t v = 0; v < nV; ++v) {
            IloExpr sum(env);
            for (std::size_t z = 0; z < nZ; ++z) {
                sum += y[v][z];
                total += y[v][z];
            }
            model.add(sum <= 1); sum.end();
        }

        /* (2) capacity: do not oversupply a zone */
        for (std::size_t z = 0; z < nZ; ++z) {
            IloExpr cap(env);
            for (std::size_t v = 0; v < nV; ++v) cap += y[v][z];

            int capZ = EpochInst->zones_[zoneIDs[z]]->nbUnserved_;   // w_z
            model.add(cap <= capZ);
            cap.end();
        }

        /* (3) assign exactly min(|V|, total demand) vehicles */
        model.add(total == std::min(totalNeed, static_cast<int>(nV)));
        total.end();

        IloCplex cplex(model);
        cplex.setOut(env.getNullStream());
        cplex.setParam(IloCplex::Param::RootAlgorithm, 2);
        cplex.setParam(IloCplex::Param::Threads, EpochInst->parameters_->nbThreads_);
        cplex.solve();

        /* ---------- 5. write back assignments ---------- */
        for (std::size_t v = 0; v < nV; ++v)
            for (std::size_t z = 0; z < nZ; ++z)
                if (cplex.getValue(y[v][z]) > 0.5) {
  //                  std::cout << "Vehicle " << idleVehicles[v]->departNode_->zoneID_ << " to " << "Zone " << zoneIDs[z] << " with unserved: " << EpochInst->zones_[zoneIDs[z]]->nbUnserved_<< std::endl;

                    PNode sinkNode = std::make_shared<Node>(idleVehicles[v]->sinkNode_);
                    sinkNode->zoneID_ = zoneIDs[z];
                    sinkNode->locationID_ = EpochInst->zones_[zoneIDs[z]]->centerLocationID_;
                    idleVehicles[v]->currentRoute_->addSink(sinkNode);
                    if (EpochInst->parameters_->solutionMode_ == ANYTIME)
                        idleVehicles[v]->updateCurrentRoute(EpochInst->simulationStartTime_
                            + elapsedTime_ + simulationTime_->dSinceStart().count());
                    break;
                }
    }
    catch (const IloException &e) {
        std::cerr << "CPLEX error in returnVehiclesAssign: "
                  << e.getMessage() << std::endl;
    }
    env.end();
}

void Solver::returnVehiclesAlonsoCplex(const PInstance & EpochInst) const {
    rebalancingTime_->stop();
    if (!EpochInst->lastCommittedRequests_.empty()) {
        /* ---------- 1. reference time for “idle” test ---------- */

        float epochStartTime = 0;
        if (EpochInst->parameters_->solutionMode_ == ANYTIME)
            epochStartTime = EpochInst->simulationStartTime_ + elapsedTime_;
        else
            epochStartTime = EpochInst->simulationStartTime_ + static_cast<float> (epoch_) * EpochInst->parameters_->epochLength_;
        float lastEpoch = epochStartTime - EpochInst->parameters_->WaitForReturn_;

        /* ---------- 2. collect idle vehicles ---------- */
        std::vector<PVehicle> idleVehicles;
        for (auto &vehicleObj: EpochInst->vehicles_) {
            if (vehicleObj->currentRoute_->routeSize_ == 1 && vehicleObj->currentRoute_->plannedReachTime_[0]+
                vehicleObj->currentRoute_->routeNodes_.back()->serviceTime_ < lastEpoch) {
                idleVehicles.push_back(vehicleObj);
            }
        }
        if (!idleVehicles.empty()) {

            /* ---------- 4. build and solve assignment MIP ---------- */
            IloEnv   env;
            try {
                const std::size_t nV = idleVehicles.size();
                const std::size_t nR = EpochInst->lastCommittedRequests_.size();
                const int need = static_cast<int>(std::min(nV, nR));

                IloModel model(env);
                IloArray<IloBoolVarArray> y(env, nV);          // y[v][r]
                for (std::size_t v = 0; v < nV; ++v) {
                    y[v] = IloBoolVarArray(env, nR);
                    for (std::size_t r = 0; r < nR; ++r) y[v][r] = IloBoolVar(env);
                }

                /* objective */
                IloExpr obj(env);
                for (std::size_t v = 0; v < nV; ++v) {
                    int vehLoc = idleVehicles[v]->departNode_->locationID_;
                    for (std::size_t r = 0; r < nR; ++r) {
                        int reqLoc = EpochInst->lastCommittedRequests_[r]->PickUpID_;
                        float cost = durationMatrix_[vehLoc][reqLoc];
                        obj += cost * y[v][r];
                    }
                }
                model.add(IloMinimize(env, obj)); obj.end();
                IloExpr total(env);
                /* (1) at most one request per vehicle */
                for (std::size_t v = 0; v < nV; ++v) {
                    IloExpr sum(env);
                    for (std::size_t r = 0; r < nR; ++r) sum += y[v][r];
                    model.add(sum <= 1); sum.end();
                }

                /* (2) at most one vehicle per request */
                for (std::size_t r = 0; r < nR; ++r) {
                    IloExpr sum(env);
                    for (std::size_t v = 0; v < nV; ++v) sum += y[v][r];
                    model.add(sum <= 1); sum.end();
                }

                /* (3) use exactly min(|V_idle|, |R_late|) vehicles */
                IloExpr tot(env);
                for (std::size_t v = 0; v < nV; ++v)
                    for (std::size_t r = 0; r < nR; ++r) tot += y[v][r];
                model.add(tot == need); tot.end();

                IloCplex cplex(model);
                cplex.setOut(env.getNullStream());
                cplex.setParam(IloCplex::Param::RootAlgorithm, 2);
                cplex.setParam(IloCplex::Param::Threads, EpochInst->parameters_->nbThreads_);
                cplex.solve();

                /* ---------- 5. write back assignments ---------- */
                for (std::size_t v = 0; v < nV; ++v)
                    for (std::size_t r = 0; r < nR; ++r)
                        if (cplex.getValue(y[v][r]) > 0.5) {
                            std::cout << "Vehicle " << idleVehicles[v]->departNode_->zoneID_ << " to " << "Requests " << EpochInst->lastCommittedRequests_[r]->pickZoneID_ << std::endl;

                            PNode sinkNode = std::make_shared<Node>(idleVehicles[v]->sinkNode_);
                            sinkNode->zoneID_ = EpochInst->lastCommittedRequests_[r]->pickZoneID_;
                            sinkNode->locationID_ = EpochInst->lastCommittedRequests_[r]->PickUpID_;
                            idleVehicles[v]->currentRoute_->addSink(sinkNode);
                            if (EpochInst->parameters_->solutionMode_ == ANYTIME)
                                idleVehicles[v]->updateCurrentRoute(EpochInst->simulationStartTime_
                                    + elapsedTime_ + simulationTime_->dSinceStart().count());
                            break;
                        }
            }
            catch (const IloException &e) {
                std::cerr << "CPLEX error in returnVehiclesAssign: "
                          << e.getMessage() << std::endl;
            }
            env.end();
        }
    }
    EpochInst->lastCommittedRequests_.clear();
    rebalancingTime_->start();
}

void Solver::returnVehiclesAlonso(const PInstance & EpochInst) const {
    rebalancingTime_->stop();
    if (!EpochInst->lastCommittedRequests_.empty()) {
        /* ---------- 1. reference time for "idle" test ---------- */

        float epochStartTime = 0;
        if (EpochInst->parameters_->solutionMode_ == ANYTIME)
            epochStartTime = EpochInst->simulationStartTime_ + elapsedTime_;
        else
            epochStartTime = EpochInst->simulationStartTime_ + static_cast<float> (epoch_) * EpochInst->parameters_->epochLength_;
        float lastEpoch = epochStartTime - EpochInst->parameters_->WaitForReturn_;

        /* ---------- 2. collect idle vehicles ---------- */
        std::vector<PVehicle> idleVehicles;
        for (auto &vehicleObj: EpochInst->vehicles_) {
            if (vehicleObj->currentRoute_->routeSize_ == 1 && vehicleObj->currentRoute_->plannedReachTime_[0]+
                vehicleObj->currentRoute_->routeNodes_.back()->serviceTime_ < lastEpoch &&
                vehicleObj->currentRoute_->routeNodes_.back()->initialType_ != SINK) {
                idleVehicles.push_back(vehicleObj);
            }
        }
        if (!idleVehicles.empty()) {

            /* ---------- 4. build and solve assignment MIP ---------- */
            try {
                const std::size_t nV = idleVehicles.size();
                const std::size_t nR = EpochInst->lastCommittedRequests_.size();
                const int need = static_cast<int>(std::min(nV, nR));

                GRBEnv env = GRBEnv();
                env.set(GRB_IntParam_OutputFlag, 0); // Suppress output
                env.set(GRB_IntParam_Threads, EpochInst->parameters_->nbThreads_);

                GRBModel model = GRBModel(env);

                // Create decision variables y[v][r]
                std::vector<std::vector<GRBVar>> y(nV);
                for (std::size_t v = 0; v < nV; ++v) {
                    y[v].resize(nR);
                    for (std::size_t r = 0; r < nR; ++r) {
                        y[v][r] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS);
                    }
                }

                /* objective */
                GRBLinExpr obj = 0;
                for (std::size_t v = 0; v < nV; ++v) {
                    int vehLoc = idleVehicles[v]->departNode_->locationID_;
                    for (std::size_t r = 0; r < nR; ++r) {
                        int reqLoc = EpochInst->lastCommittedRequests_[r]->PickUpID_;
                        float cost = durationMatrix_[vehLoc][reqLoc];
                        obj += cost * y[v][r];
                    }
                }
                model.setObjective(obj, GRB_MINIMIZE);

                /* (1) at most one request per vehicle */
                for (std::size_t v = 0; v < nV; ++v) {
                    GRBLinExpr sum = 0;
                    for (std::size_t r = 0; r < nR; ++r) {
                        sum += y[v][r];
                    }
                    model.addConstr(sum <= 1);
                }

                /* (2) at most one vehicle per request */
                for (std::size_t r = 0; r < nR; ++r) {
                    GRBLinExpr sum = 0;
                    for (std::size_t v = 0; v < nV; ++v) {
                        sum += y[v][r];
                    }
                    model.addConstr(sum <= 1);
                }

                /* (3) use exactly min(|V_idle|, |R_late|) vehicles */
                GRBLinExpr tot = 0;
                for (std::size_t v = 0; v < nV; ++v) {
                    for (std::size_t r = 0; r < nR; ++r) {
                        tot += y[v][r];
                    }
                }
                model.addConstr(tot == need);

                // Solve the model
                model.optimize();

                /* ---------- 5. write back assignments ---------- */
                if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
                    for (std::size_t v = 0; v < nV; ++v) {
                        for (std::size_t r = 0; r < nR; ++r) {
                            if (y[v][r].get(GRB_DoubleAttr_X) > 0.5) {
                                std::cout << "Vehicle " << idleVehicles[v]->departNode_->zoneID_ << " to " << "Requests " << EpochInst->lastCommittedRequests_[r]->pickZoneID_ << std::endl;

                                PNode sinkNode = std::make_shared<Node>(idleVehicles[v]->sinkNode_);
                                sinkNode->zoneID_ = EpochInst->lastCommittedRequests_[r]->pickZoneID_;
                                sinkNode->locationID_ = EpochInst->lastCommittedRequests_[r]->PickUpID_;
                                idleVehicles[v]->currentRoute_->addSink(sinkNode);
                                if (EpochInst->parameters_->solutionMode_ == ANYTIME)
                                    idleVehicles[v]->updateCurrentRoute(EpochInst->simulationStartTime_
                                        + elapsedTime_ + simulationTime_->dSinceStart().count());
                                break;
                            }
                        }
                    }
                }
            }
            catch (const GRBException &e) {
                std::cerr << "Gurobi error in returnVehiclesAssign: "
                          << e.getMessage() << std::endl;
            }
        }
    }
    EpochInst->lastCommittedRequests_.clear();
    rebalancingTime_->start();
}

