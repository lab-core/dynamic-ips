//
// Created by Elahe Amiri on 2025-11-02.
//

#include "BaseSolver.h"


BaseSolver::BaseSolver(const PInstance &mainInst, const InputPaths &inputPaths) {
    labelsPool_.defineSize(mainInst->parameters_->nbThreads_);
    subProOptions_ = std::make_shared<solverOption>(mainInst->parameters_);
    if (mainInst->parameters_->approach_ == ISUD)
        MP_solver_ = std::make_unique<ISUD_Algorithm>(inputPaths);
    else if (mainInst->parameters_->approach_ == CG)
        MP_solver_ = std::make_unique<CG_Algorithm>(inputPaths);
    GreedyModel_ = std::make_shared<GreedyModeler>();


    // Shared state
    elapsedTime_ = 0.0;
    avgEpochRuntime_ = 0.0;
    epoch_ = 0;
    objValue_ = 0.0;
    rebalanceTime_ = 0;

    // Timers
    simulationTime_ = new myTools::Timer(); simulationTime_->init();
    subProblemTime_ = new myTools::Timer(); subProblemTime_->init();
    preprocessTime_ = new myTools::Timer(); preprocessTime_->init();
    rebalancingTime_ = new myTools::Timer(); rebalancingTime_->init();
    rebalancingProcessTime_ = new myTools::Timer(); rebalancingProcessTime_->init();

    // Subproblem tracking
    nbOnePick_ = 0;
    nbTwoPick_ = 0;
    nbThreePick_ = 0;
    nbRecycle_ = 0;
    heuristicEpochObj_ = 0.0;

    runtimeMetrics_ = std::make_unique<RuntimeMetrics>();


    // Logging
    pLogEpochSubRuntimeStream_ = new Tools::LogOutput(inputPaths.getOutputSubproSize());
    *pLogEpochSubRuntimeStream_ << "Epoch,vehicleID,nbRequests,nbNodes,nbOnboards,nbOnboards_nodes,plannedPick,"
                                     "possibleInsert,maxPick,#LGenerated,#LDominated,#LEliminated,#nbPrunedArcs,"
                                     "#nbPrunedPath,nbNegative,nbRoutes,BestRCost,runtime,stateChanged,removePick,"
                                     "removeDrop,#TwoPickRoute,#OnePickRoute,#newCovered,#PriorCover,#Committed,#avail" << std::endl;

    pLogEpochVehicleStream_= new Tools::LogOutput(inputPaths.getOutputVehicleEpoch());
    *pLogEpochVehicleStream_ << "Epoch,VehicleID,RouteID,nbOnboards,nbOnboards_nodes,nbCommitted,nbRequests,nbNodes,"
                                "length,avgPassPerStop,totalWait,totalTripDelay,objCoef,key" << std::endl;

    if (mainInst->parameters_->approach_ != Greedy) {
        pLogRunTimesStream_ = new Tools::LogOutput(inputPaths.getOutputEpochRunTime());
        *pLogRunTimesStream_ << "Epoch,nbRequests,nbNewRequests,nbNodes,EpochRuntime,ElapsedTime,MP_Runtime,"
                                  "RP_Runtime,MP_BuildRuntime,MP_SolveRuntime,CP_Runtime,CP_BuildRuntime,"
                                  "CP_SolveRuntime,ZoomISUD_Runtime,SubProbRuntime,#SP Iter,totalColumn,#LGenerated,"
                                  "#LDominated,#LEliminated,#nbPrunedArcs,#nbPrunedPath,nbNegative,#ColumnsAdded,"
                                  "#RecycledColumns,GreedyObj,Objective,LinearObjective,waitTime,TripDelay,"
                                  "maxDual,minDual,meanDual,maxSP_Size,meanSP_Size,"
                                  "maxRoute,meanRoute,destructTime,RebalancingRuntime,GreedyTime,"
                                  "#Return,#Idle,#passPerVehicle,#requestPerVehicle,#nodePerVehicle,#StateChanged,"
                                  "nbOnePick,nbTwoPick,nbThreePick,heuristicCG,upperBound,nbRecycle,nbCommitted,"
                                  "nbLess_50,nbLess_100,nbLess_200,totalRoutes,nbUnassigned" << std::endl;
    }
    else {
        pLogRunTimesStream_ = new Tools::LogOutput(inputPaths.getOutputEpochRunTime());
        (*pLogRunTimesStream_) << "Epoch,nbRequests,nbNewRequests,nbNodes,EpochRuntime,ElapsedTime,"
                                  "GreedyObj,Objective,waitTime,TripDelay,RebalancingRuntime,GreedyTime,#Return,#Idle,#passPerVehicle,"
                                  "#requestPerVehicle,#nodePerVehicle,#StateChanged,nbCommitted,nbUnassigned" << std::endl;
    }

}

BaseSolver::~BaseSolver() {
    delete simulationTime_;
    delete subProblemTime_;
    delete preprocessTime_;
    delete rebalancingTime_;
    delete rebalancingProcessTime_;
    pLogRunTimesStream_->close();
    pLogEpochSubRuntimeStream_->close();
    pLogEpochVehicleStream_->close();
    //    pLogEpochSubRouteStream_->close();
    //    pLogSolutionChange_->close();
    delete pLogRunTimesStream_;
    delete pLogEpochSubRuntimeStream_;
    delete pLogEpochVehicleStream_;
    //    delete pLogEpochSubRouteStream_;
    //    delete pLogSolutionChange_;
}


void BaseSolver::createFinalOutputString(const PInstance &pInst) const {
    pInst->instRepStr_ << 0 << "," << 0 << ",";
    pInst->instRepStr_ << 0 << "," << 0 << ",";
    pInst->instRepStr_ << 0 << ",";
    pInst->instRepStr_ << 0 << ",";
    pInst->instRepStr_ << 0 << ",";
    pInst->instRepStr_ << 0 << ",";
    pInst->instRepStr_<< 0 << ",";
    pInst->instRepStr_ << 0 << ",";
    pInst->instRepStr_ << 0 << "," << 0 << "," << rebalancingProcessTime_->dSinceInit().count() << ",";
    float TotalTime = GreedyModel_->greedyTime_->dSinceStart().count();
    pInst->instRepStr_ << TotalTime << ",";
    pInst->instRepStr_ << 0 << "," << 0 << ",";
    pInst->instRepStr_ << 0 << ",";
    pInst->instRepStr_ << 0 << ",";
    pInst->instRepStr_ << 0 << ",";
    pInst->instRepStr_ << 0 << ",";
    pInst->instRepStr_ << 0 << ",";
}

void BaseSolver::selectVehiclesForSubproblem(const PInstance &EpochInst, int iter) {
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

void BaseSolver::configureLabelingSubproblem(PLabelingSubPro &subProblem, PVehicle &vehicleObj, PInstance &EpochInst,
    int iter, vector2D<PRoute> &availableRoutes) {
    // Handle partial pricing
    if (EpochInst->parameters_->partialPricing_) {
        if (vehicleObj->onboards_.size() < 2) {
            vehicleObj->numPickup_ = 2;
            nbTwoPick_++;
        }
        else {
            vehicleObj->numPickup_ = 1;
            nbOnePick_++;
        }
        subProblem->maxPickup_ = vehicleObj->numPickup_;
        vehicleObj->preSolvePickLimit_ = subProblem->maxPickup_;
    }
    // Handle dynamic pricing
    else if (EpochInst->parameters_->dynamicPricing_) {
        subProblem->maxPickup_ = std::min(iter, EpochInst->parameters_->nbPick_);
    }
    // Default pricing
    else {
        subProblem->maxPickup_ = EpochInst->parameters_->nbPick_;
    }

    // Handle route recycling
    if (EpochInst->parameters_->reoptimizeSP_ && vehicleObj->currentRoute_->routeSize_ > 1) {
        if (EpochInst->parameters_->labelingReOptimizeStrategy_ == RE_INSERT) {
            if (availableRoutes[vehicleObj->vehicleID_].size() >= 50 && subProblem->maxPickup_ == 2 && EpochInst->nbNewRequests_ > 0) {
                subProblem->reOptimize_ = true;
                subProblem->availableRoutes_ = availableRoutes[vehicleObj->vehicleID_];
                nbRecycle_++;
            }
            else
                subProblem->reOptimize_ = false;
        }
        else if (!vehicleObj->removeDrop_) {
            subProblem->reOptimize_ = true;
            nbRecycle_++;
        }
        else
            subProblem->reOptimize_ = false;
    }
    else
        subProblem->reOptimize_ = false;
}

void BaseSolver::returnVehicles(const PInstance &EpochInst) const {
    rebalancingProcessTime_->start();
    rebalancingTime_->stop();

    // For ASSIGN, nothing to do if no committed requests to target
    if (EpochInst->parameters_->returnPolicy_ == ASSIGN &&
        EpochInst->lastCommittedRequests_.empty()) {
        rebalancingTime_->start();
        rebalancingProcessTime_->stop();
        return;
    }

    // Unified cutoff time using WaitForReturn_
    float epochStartTime;
    if (EpochInst->parameters_->solutionMode_ == ANYTIME)
        epochStartTime = EpochInst->simulationStartTime_ + elapsedTime_;
    else
        epochStartTime = EpochInst->simulationStartTime_ +
                         static_cast<float>(epoch_) * EpochInst->parameters_->epochLength_;
    const float lastEpoch = epochStartTime - EpochInst->parameters_->WaitForReturn_;

    // Collect idle vehicles
    std::vector<PVehicle> idleVehicles;
    for (auto &vehicleObj : EpochInst->vehicles_) {
        if (vehicleObj->currentRoute_->routeSize_ == 1 &&
            vehicleObj->currentRoute_->plannedReachTime_[0] +
            vehicleObj->currentRoute_->routeNodes_.back()->serviceTime_ < lastEpoch)
            idleVehicles.push_back(vehicleObj);
    }

    if (!idleVehicles.empty()) {
#ifdef DARP_USE_GUROBI
        AssignmentSolver assignSolver(solverEnv::gurobi());
#elif defined(DARP_USE_CPLEX)
        AssignmentSolver assignSolver(solverEnv::cplex());
#endif
        if (EpochInst->parameters_->returnPolicy_ == TO_SOURCE)
            assignSolver.returnToSource(EpochInst, idleVehicles, elapsedTime_, simulationTime_);
        else
            assignSolver.solve(EpochInst, idleVehicles, durationMatrix_,
                               elapsedTime_, simulationTime_);
    }

    EpochInst->lastCommittedRequests_.clear();
    rebalancingTime_->start();
    rebalancingProcessTime_->stop();
}

void BaseSolver::reconstructAvailableRoutes(const PInstance &mainInst, vector2D<PRoute> &availableRoutes) {
    for (auto & vehicleObj : mainInst->vehicles_) {
        if (!availableRoutes[vehicleObj->vehicleID_].empty()) {
            for (int i = availableRoutes[vehicleObj->vehicleID_].size() - 1; i >= 0; --i){
                if (!availableRoutes[vehicleObj->vehicleID_][i]->reConstruct(vehicleObj, mainInst->parameters_->Wait_W1_,
                    mainInst->parameters_->Ride_W2_))
                    availableRoutes[vehicleObj->vehicleID_].erase(availableRoutes[vehicleObj->vehicleID_].begin() + i);
                else {
                    availableRoutes[vehicleObj->vehicleID_][i]->calculateTripDelay(mainInst->parameters_->Wait_W1_,
                        mainInst->parameters_->Ride_W2_);
                }

            }
        }
    }
}

void BaseSolver::buildEpochInstance(PInstance &mainInst, PInstance &EpochInst, float elapsedTime, int &nbReceivedRequest) {
    EpochInst->resetInstance();

    if (mainInst->parameters_->approach_ == Greedy)
        EpochInst->buildPartialData(mainInst, GreedyModel_->zSolution_ , elapsedTime, nbReceivedRequest);
    else
        EpochInst->buildPartialData(mainInst, MP_solver_->zSolution_ , elapsedTime, nbReceivedRequest);
    if (EpochInst->parameters_->timeWindow_ == 0)
        EpochInst->updatePenalties(elapsedTime_);
    if (mainInst->parameters_->routeRecycle_ && !MP_solver_->availableRoutes_.empty())
        reconstructAvailableRoutes(mainInst, MP_solver_->availableRoutes_);

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
    EpochInst->lastCommittedRequests_ = mainInst->lastCommittedRequests_;
    std::cout << "# TOTAL NUMBER OF RECEIVED REQUESTS: " << EpochInst->nbRequests_ << std::endl;
    std::cout << "# TOTAL NUMBER OF NEW REQUESTS: " << EpochInst->nbNewRequests_ << std::endl;
}

void BaseSolver::logEpochInfo(int epoch, float elapsedTime, float simulationTime, float epochRuntime,
    InputPaths &inputPaths) {
    std::cout << "---------------------"<< std::endl;
    std::cout << " ELAPSED TIME: " << elapsedTime << std::endl;
    std::cout << " SIMULATION TIME: " << simulationTime << std::endl;
    std::cout << " PRE EPOCH TIME: " << epochRuntime << std::endl;
    std::cout << " EPOCH: " << epoch << std::endl;
    std::cout << "---------------------"<< std::endl;

    std::ofstream logFile(inputPaths.getOutputSolverLog(), std::ofstream::app);
    logFile << "---------------------------------------------------"<< std::endl;
    logFile << " EPOCH: " << epoch << std::endl;
    logFile.close();
}

void BaseSolver::handleVehicleReturn(PInstance &EpochInst) {
    if (EpochInst->parameters_->vehicleReturn_ &&
        elapsedTime_ - rebalanceTime_ >= EpochInst->parameters_->committedTime_) {
        returnVehicles(EpochInst);
        rebalanceTime_ = static_cast<int>(elapsedTime_);
    }
}

void BaseSolver::updateAvailableRoutes(boost::dynamic_bitset<> &removedRequests, vector2D<PRoute> &availableRoutes, PInstance &EpochInst) {

    for (auto& vehicleObj : EpochInst->vehicles_) {
        if (!vehicleObj->stateChanged_) {
            continue;
        }


        availableRoutes[vehicleObj->vehicleID_].erase(
            std::remove_if(
                availableRoutes[vehicleObj->vehicleID_].begin(),
                availableRoutes[vehicleObj->vehicleID_].end(),
                [&](const auto& route) {
                    // remove if route is invalid OR condition is true
                    if (!route || route->routeNodes_.size() < vehicleObj->removeNodes_.size()+2 || route->getRouteId() == vehicleObj->currentRoute_->getRouteId()) {
                        return true;
                    }

                    for (size_t i = 0; i < vehicleObj->removeNodes_.size(); i++) {
                        if (vehicleObj->removeNodes_[i] != route->routeNodes_[i+1]->nodeID_)
                            return true;
                    }
                    return false;
                }),
            availableRoutes[vehicleObj->vehicleID_].end());
    }


    for (auto &vehicleRoutes : availableRoutes) {
        // Remove routes with overlapping requests or empty route requests
        for (auto &route : vehicleRoutes) {
            if (route->column_.size() != removedRequests.size())
                std::cerr << "[ERROR] column size mismatch in route recycling\n";
        }
        vehicleRoutes.erase(
                std::remove_if(vehicleRoutes.begin(), vehicleRoutes.end(),
                               [&removedRequests](const PRoute &routeObj) {
                                   return !(routeObj->column_ & removedRequests).none() || routeObj->routeRequests_.empty();
                               }),
                vehicleRoutes.end()
        );
    }
    for (auto& vehicleObj : EpochInst->vehicles_)
        vehicleObj->routeAvail_ = availableRoutes[vehicleObj->vehicleID_].size();
}

void BaseSolver::CreateOneStopRoutes(const PVehicle &vehicle, std::vector<PRoute> &availableRoutes,
    const PInstance &pInst, const PInstance &EpochInst, int &nbNegative) {
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
        newRoute->calculateTripDelay(pInst->parameters_->Wait_W1_, pInst->parameters_->Ride_W2_);
        newRoute->reducedCost_ = newRoute->objCoef_ - requestObj->dual_ - vehicle->dual_;
        newRoute->normal_RC_ = newRoute->reducedCost_ / static_cast<float>(newRoute->routeRequests_.size());
        newRoute->lambda_ = newRoute->objCoef_/(requestObj->dual_ + vehicle->dual_ + 0.001f);
        newRoute->totalLength_ = newRoute->plannedDepartTime_.back() - vehicle->departTime_;
        if (newRoute->reducedCost_ < 0)
            nbNegative++;
        availableRoutes.emplace_back(std::move(newRoute));
        availableRoutes.back()->createColumn(EpochInst->nbRequests_);
    }
}

void BaseSolver::solveEpoch(PInstance &EpochInst, PInstance &mainInst, InputPaths &inputPaths) {
    Tools::PThreadsPool pPool = Tools::ThreadsPool::newThreadsPool(EpochInst->parameters_->nbThreads_);
    // Initialize variables
    int iter = 0;
    bool repeat = true;
    bool subProBreak = false;

    EpochInst->updateTaskIndexLabeling();
    EpochInst->selectedVehicles_.clear();
    EpochInst->selectedVehicles_.resize(EpochInst->nbVehicles_, 0);
    runtimeMetrics_->resetSubproblemMetrics();

    // Set available time
    MP_solver_->hasTimeRemaining(EpochInst, simulationTime_->dSinceStart().count(), 1);
    MP_solver_->epochInitialization(EpochInst, inputPaths, epoch_, GreedyModel_);


    // Main column generation loop
    while (true) {
        iter++;
        MP_solver_->hasTimeRemaining(EpochInst, simulationTime_->dSinceStart().count(), iter);
        int nbNegativeFound = 0;
        float previousObj = MP_solver_->objValue_;
        float previousLpObj = MP_solver_->lpObjValue_;

        //***********************************************************************************//
        //                    Solve subproblems using the extracted function
        //***********************************************************************************//
        if (mainInst->parameters_->subAlgorithm_ == LABEL_SETTING)
            subProBreak = solve_SP<PLabelingSubPro>(EpochInst, mainInst, iter, nbNegativeFound,
                MP_solver_->availableRoutes_, MP_solver_->availableTime_, MP_solver_->MPnbRoutes_,MP_solver_->duplicatesRoutes_);
        else {
#ifdef DARP_USE_GUROBI
            subProBreak = solve_SP<PGurobiSubPro>(EpochInst, mainInst, iter, nbNegativeFound,
                MP_solver_->availableRoutes_, MP_solver_->availableTime_, MP_solver_->MPnbRoutes_, MP_solver_->duplicatesRoutes_);

#elif DARP_USE_CPLEX
            subProBreak = solve_SP<PCplexSubPro>(EpochInst, mainInst, iter, nbNegativeFound,
                MP_solver_->availableRoutes_, MP_solver_->availableTime_, MP_solver_->MPnbRoutes_, MP_solver_->duplicatesRoutes_);
#endif
        }
        if (subProBreak) {
            std::cout << "Terminate CG-> Not enough time to run the subproblems! " << std::endl;
            break;
        }

        MP_solver_->SPIters_++;
        MP_solver_->SPIter_++;

        if (nbNegativeFound == 0) {
            if (!repeat) {
                subProOptions_->disableHeuristics();
                EpochInst->parameters_->dynamicPricing_ = false;
                heuristicEpochObj_ = MP_solver_->lpObjValue_;
                std::cout << "Solve the exact mode " << std::endl;
                repeat = true;
            }
            else {
                MP_solver_->CGSuccess_++;
                std::cout << "Terminate CG-> No negative column " << std::endl;
                break;
            }
        }

        else {
            //***********************************************************************************//
            //                    Solve the Master Problem
            //***********************************************************************************//
            // Check available time
            if (!MP_solver_->hasTimeRemaining(EpochInst, simulationTime_->dSinceStart().count(), iter))
                break;

            MP_solver_->solve(EpochInst, epoch_, inputPaths, subProblemTime_->dSinceStart().count());

        }

        if (!MP_solver_->hasTimeRemaining(EpochInst, simulationTime_->dSinceStart().count(), iter))
            break;
        if (MP_solver_->shouldTerminate(EpochInst, previousObj, previousLpObj, iter))
            break;

        std::cout << " simulation time: " << simulationTime_->dSinceStart().count() << std::endl;
    }  // end of CG while
    MP_solver_->availableTime_ = EpochInst->parameters_->epochLength_ - simulationTime_->dSinceStart().count();
    MP_solver_->getIPSolution(EpochInst, epoch_, inputPaths, subProblemTime_->dSinceStart().count());

    MP_solver_->resetModels();
    handleVehicleReturn(EpochInst);

//    subProOptions_->enableHeuristics(mainInst->parameters_);
//    EpochInst->parameters_->dynamicPricing_ = true;
    objValue_ = MP_solver_->objValue_;
    MP_solver_->setLastDuals(EpochInst);

    MP_solver_->checkCoveredVehicles(EpochInst);
 //   for (auto & vehicleObj : EpochInst->vehicles_)
 //       (*pLogEpochVehicleStream_) << vehicleObj->toStringOut(epoch_);

    labelsPool_.clear();
    labelsPool_.defineSize(mainInst->parameters_->nbThreads_);
    std::cout << " end time: " << simulationTime_->dSinceStart().count() << std::endl;
}

std::string BaseSolver::saveRuntimes(const PInstance &EpochInst) {
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
    repStr << MP_solver_->runtimesToString(runtimeMetrics_);
    upperbound = MP_solver_->upperbound_;

    EpochInst->calcVehicleMetric();
    EpochInst->countCommittedRequests();

    repStr << preprocessTime_->dSinceStart().count()<< ","
           << rebalancingProcessTime_->dSinceStart().count()<< ","
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
           << heuristicEpochObj_ << ","
           << upperbound << ","
           << nbRecycle_ <<","
           << EpochInst->nbCommitted_ <<","
           << runtimeMetrics_->nbColumnsLess_50_ <<","
           << runtimeMetrics_->nbColumnsLess_100_ <<","
           << runtimeMetrics_->nbColumnsLess_200_ <<","
           << runtimeMetrics_->nbRoutes_ << ","
           << MP_solver_->zSolution_.size() <<"\n";

    runtimeMetrics_->GreedyTime_ = GreedyModel_->greedyTime_->dSinceInit().count();
    return repStr.str();
}

std::string BaseSolver::saveRuntimesGreedy(const PInstance &EpochInst) {
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
           << GreedyModel_->totalWaitTime_ << ","
           << GreedyModel_->totalTripDelay_ << ","
           << rebalancingProcessTime_->dSinceStart().count()<< ","
           << GreedyModel_->greedyTime_->dSinceInit().count() - runtimeMetrics_->GreedyTime_ << ",";

    EpochInst->calcVehicleMetric();

    repStr << EpochInst->nbReturn_ << ","
           << EpochInst->nbIdle_ << ","
           << EpochInst->passPerVehicle_ << ","
           << EpochInst->requestPerVehicle_ << ","
           << EpochInst->nodePerVehicle_ << ","
           << EpochInst->nbStateChanged_ << ","
           << EpochInst->nbCommitted_ << ","
           << GreedyModel_->zSolution_.size() << "\n";
    runtimeMetrics_->GreedyTime_ = GreedyModel_->greedyTime_->dSinceInit().count();
    return repStr.str();
}

std::string BaseSolver::toString(const PInstance &mainInst) const {
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
    if (mainInst->parameters_->mainAlgorithm_ != GREEDY) {
        repStr << MP_solver_->toStringTimersTotal();
        repStr << std::setw(SENTENCE_SIZE) << "# TIME SPENT ON SOLVING SUB PROBLEMS" << " = " << subProblemTime_->dSinceInit().count() << " (s)" << std::endl;
    }
    repStr << std::setw(SENTENCE_SIZE) << "# TIME SPENT ON GREEDY" << " = " << GreedyModel_->greedyTime_->dSinceInit().count() << " (s)" << std::endl;
    if (mainInst->parameters_->mainAlgorithm_ != GREEDY)
        repStr << MP_solver_->toStringTimersAvg(epoch_);
    repStr << std::setw(SENTENCE_SIZE) << "# TIME SPENT ON SOLVING SUB PROBLEMS" << " = " << subProblemTime_->dSinceInit().count()/static_cast<float>(epoch_) << " (s)" << std::endl;
    repStr << std::setw(SENTENCE_SIZE) << "# TIME SPENT ON GREEDY" << " = " << GreedyModel_->greedyTime_->dSinceInit().count()/static_cast<float>(epoch_) << " (s)" << std::endl;
    mainInst->instRepStr_ << epoch_-1 << "," ;
    if (mainInst->parameters_->approach_ == Greedy)
        createFinalOutputString(mainInst);
    else
        MP_solver_->createFinalOutputString(mainInst, subProblemTime_->dSinceInit().count(),
            GreedyModel_->greedyTime_->dSinceInit().count(), rebalancingProcessTime_->dSinceInit().count());

    return repStr.str();
}

template<typename SubProblemType>
bool BaseSolver::solve_SP(PInstance &EpochInst, PInstance &mainInst, int &iter, int &nbNegativeFound,
                                vector2D<PRoute> &availableRoutes, float availableTime, int &nbRoutes,
                                std::vector<std::unordered_set<std::string>> &duplicatesRoutes) {
    Tools::PThreadsPool pPool = Tools::ThreadsPool::newThreadsPool(EpochInst->parameters_->nbThreads_);

    bool subProBreak = false;
    nbOnePick_ = 0;
    nbTwoPick_ = 0;
    nbThreePick_ = 0;
    nbRecycle_ = 0;
    std::vector<int> sizeValues;
    std::vector<int> routeValues;
    sizeValues.reserve(EpochInst->nbVehicles_);
    routeValues.reserve(EpochInst->nbVehicles_);

    // Start the subproblems timer
    subProblemTime_->start();

    // Define subproblems
    std::vector<SubProblemType> subProSolve;
    selectVehiclesForSubproblem(EpochInst, iter);

    // create subproblems
    int num = 0;
    for (auto &vehicleObj: EpochInst->vehicles_) {
        vehicleObj->bestReducedCost_ = INFINITY;

        if (EpochInst->selectedVehicles_[vehicleObj->vehicleID_] >= 1) {
            num++;

            // Create appropriate subproblem type
            if constexpr (std::is_same_v<SubProblemType, PLabelingSubPro>) {
                subProSolve.emplace_back(std::make_shared<LabelingSubProblem>(vehicleObj, subProOptions_));

                // Configure labeling-specific parameters
                configureLabelingSubproblem(subProSolve.back(), vehicleObj, EpochInst, iter, availableRoutes);

            }
            else {
#if defined(DARP_USE_GUROBI)
                subProSolve.emplace_back(std::make_shared<SubProblem_Gurobi>(vehicleObj));
#elif defined(DARP_USE_CPLEX)
                subProSolve.emplace_back(std::make_shared<SubProblem_Cplex>(vehicleObj));
#endif
            }

//            availableRoutes[vehicleObj->vehicleID_].clear();
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

            if constexpr (std::is_same_v<SubProblemType, PLabelingSubPro>) {
                // Label-based solving
                subProblem->labelPool_ = std::move(labelsPool_.pop_data());
                if (!subProblem->subRequests_.empty()) {
                    subProBreak = subProblem->solveSP(availableTime - subProblemTime_->dSinceStart().count(),
                    EpochInst->vehicles_[subProblem->Vehicle_->vehicleID_],
                                                 availableRoutes[subProblem->Vehicle_->vehicleID_],
                                                 mainInst, EpochInst->nbRequests_, duplicatesRoutes[subProblem->Vehicle_->vehicleID_]);
                }
                labelsPool_.push_data(std::move(subProblem->labelPool_));
            }
#if defined(DARP_USE_CPLEX)
            else {
                try {
                    subProblem->solveSP(EpochInst, availableRoutes[subProblem->Vehicle_->vehicleID_]);
                } catch (IloException& e) {
                    std::cerr << "CPLEX Exception in subproblem: " << e.getMessage() << std::endl;
                    subProBreak = true;
                }
            }
#elif defined(DARP_USE_GUROBI)
            else {
                try {
                    subProblem->solveSP(EpochInst, availableRoutes[subProblem->Vehicle_->vehicleID_]);
                } catch (GRBException& e) {
                    std::cerr << "Gurobi Exception in subproblem: " << e.getMessage() << std::endl;
                    subProBreak = true;
                }
            }
#endif
        });
        pPool->run(job);
    }
    // Wait for all jobs to complete
    pPool->wait();

    // Collect results from subproblems
    nbNegativeFound = 0;
    for (auto &subProblem: subProSolve) {
        nbRoutes += static_cast<int>(availableRoutes[subProblem->Vehicle_->vehicleID_].size());
        nbNegativeFound += subProblem->nbNegativeColumns_;
        runtimeMetrics_->updateSubproblemMetrics(subProblem);
        sizeValues.push_back(subProblem->subRequests_.size());
        routeValues.push_back(subProblem->SPnbOutputs_);
 //       if (epoch_ >= 720 && epoch_ < 1440 && (epoch_ - 720) % 18 == 0) {
 //           (*pLogEpochSubRuntimeStream_) << subProblem->toStringOut(epoch_);
 //       }
    }
    MP_solver_->subproSummary_.maxSize = *std::max_element(sizeValues.begin(), sizeValues.end());
    MP_solver_->subproSummary_.meanSize = std::accumulate(sizeValues.begin(), sizeValues.end(), 0.0) / sizeValues.size();

    MP_solver_->subproSummary_.maxRoute = *std::max_element(routeValues.begin(), routeValues.end());
    MP_solver_->subproSummary_.meanRoute = std::accumulate(routeValues.begin(), routeValues.end(), 0.0) / routeValues.size();

    // Clean up
    preprocessTime_->start();
    subProSolve.clear();
    preprocessTime_->stop();
    subProblemTime_->stop();
    runtimeMetrics_->subproEpochTime_ += subProblemTime_->dSinceStart().count();
    return subProBreak;
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
    nbColumnsLess_50_ = 0;
    nbColumnsLess_100_ = 0;
    nbColumnsLess_200_ = 0;
    nbRoutes_ = 0;
}

void RuntimeMetrics::updateSubproblemMetrics(const PLabelingSubPro &subProblem) {
    nbNegativeFound_ += subProblem->nbNegativeColumns_;
    nbGenerated_ += subProblem->nbGenerated_;
    nbDominated_ += subProblem->nbDominated_;
    nbEliminated_ += subProblem->nbEliminated_;
    nbPrunedPath_ += subProblem->nbPrunedPath_;
    nbPrunedArcs_ += subProblem->nbPrunedArcs_;
    nbRecycledColumns_ += subProblem->nbRecycledColumns_;
    nbRoutes_ += subProblem->SPnbOutputs_;
    if (subProblem->SPnbOutputs_ < 200) {
        nbColumnsLess_200_ ++;
        if (subProblem->SPnbOutputs_ < 100) {
            nbColumnsLess_100_ ++;
            if (subProblem->SPnbOutputs_ < 50)
                nbColumnsLess_50_ ++;
        }
    }
}

// RuntimeMetrics::updateSubproblemMetrics(const PCplexSubPro &) lives in
// src/CplexSolver/SubProblem_Cplex.cpp.
// RuntimeMetrics::updateSubproblemMetrics(const PGurobiSubPro &) lives in
// src/GurobiSolver/SubProblem_Gurobi.cpp.
