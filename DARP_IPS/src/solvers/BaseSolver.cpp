//
// Created by Elahe Amiri on 2025-11-02.
//

#include "BaseSolver.h"
#include "data/Instance.h"
#include "solvers/LabelingSubProblem.h"
#include "CplexSolver/CPLEXSubProblem.h"
#include "GurobiSolver/MP_Gurobi.h"
#include "CG_Algorithm.h"
#include "ISUD_Algorithm.h"

BaseSolver::BaseSolver(const PInstance &mainInst, const InputPaths &inputPaths) {
    labelsPool_.defineSize(mainInst->parameters_->nbThreads_);
    subProOptions_ = std::make_shared<solverOption>(mainInst->parameters_);
    if (mainInst->parameters_->approach_ == ISUD)
        MP_solver_ = std::make_unique<ISUD_Algorithm>(inputPaths, mainInst->parameters_->modelSolver_);
    else if (mainInst->parameters_->approach_ == CG)
        MP_solver_ = std::make_unique<CG_Algorithm>(inputPaths, mainInst->parameters_->modelSolver_);
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
                                  "nbLess_50,nbLess_100,nbLess_200,totalRoutes" << std::endl;
    }
    else {
        pLogRunTimesStream_ = new Tools::LogOutput(inputPaths.getOutputEpochRunTime());
        (*pLogRunTimesStream_) << "Epoch,nbRequests,nbNewRequests,nbNodes,EpochRuntime,ElapsedTime,"
                                  "GreedyObj,Objective,waitTime,TripDelay,RebalancingRuntime,GreedyTime,#Return,#Idle,#passPerVehicle,"
                                  "#requestPerVehicle,#nodePerVehicle,#StateChanged,nbCommitted" << std::endl;
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

void BaseSolver::returnVehiclesAssign(const PInstance &EpochInst) const {
    rebalancingProcessTime_->start();
    rebalancingTime_->stop();

    if (!EpochInst->lastCommittedRequests_.empty()) {
        /* ---------- 1. reference time for "idle" test ---------- */
        float epochStartTime = 0;
        if (EpochInst->parameters_->solutionMode_ == ANYTIME)
            epochStartTime = EpochInst->simulationStartTime_ + elapsedTime_;
        else
            epochStartTime = EpochInst->simulationStartTime_ + static_cast<float>(epoch_) * EpochInst->parameters_->epochLength_;
        float lastEpoch = epochStartTime - EpochInst->parameters_->WaitForReturn_;

        /* ---------- 2. collect idle vehicles ---------- */
        std::vector<PVehicle> idleVehicles;
        for (auto &vehicleObj: EpochInst->vehicles_) {
            if (vehicleObj->currentRoute_->routeSize_ == 1 &&
                vehicleObj->currentRoute_->plannedReachTime_[0] +
                vehicleObj->currentRoute_->routeNodes_.back()->serviceTime_ < lastEpoch) {
                idleVehicles.push_back(vehicleObj);
                }
        }

        if (!idleVehicles.empty()) {
            if (EpochInst->parameters_->modelSolver_ == GUROBI)
                solveGurobiAssignment(EpochInst, idleVehicles);
            else
                solveCplexAssignment(EpochInst, idleVehicles);
        }
    }

    EpochInst->lastCommittedRequests_.clear();
    rebalancingTime_->start();
    rebalancingProcessTime_->stop();
}

void BaseSolver::solveGurobiAssignment(const PInstance &EpochInst, std::vector<PVehicle> &idleVehicles) const {
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
                if (idleVehicles[v]->departNode_->zoneID_ != EpochInst->lastCommittedRequests_[r]->pickZoneID_)
                    y[v][r] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS);
                else
                    y[v][r] = model.addVar(0.0, 0.0, 0.0, GRB_CONTINUOUS);
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
                                + elapsedTime_ + simulationTime_->dSinceStart().count(),
                                EpochInst->parameters_->Wait_W1_, EpochInst->parameters_->Ride_W2_);
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

void BaseSolver::solveCplexAssignment(const PInstance &EpochInst, std::vector<PVehicle> &idleVehicles) const {
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
                            + elapsedTime_ + simulationTime_->dSinceStart().count(), EpochInst->parameters_->Wait_W1_,
                            EpochInst->parameters_->Ride_W2_);
                    break;
                }
    }
    catch (const IloException &e) {
        std::cerr << "CPLEX error in returnVehiclesAssign: "
                  << e.getMessage() << std::endl;
    }
    env.end();
}

void BaseSolver::returnVehiclesSource(const PInstance &EpochInst) const {
    rebalancingProcessTime_->start();
    rebalancingTime_->stop();
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
                        vehicleObj->updateCurrentRoute(EpochInst->simulationStartTime_ + elapsedTime_+
                        simulationTime_->dSinceStart().count(), EpochInst->parameters_->Wait_W1_, EpochInst->parameters_->Ride_W2_);
                }
                }
        }
    }
    rebalancingTime_->start();
    rebalancingProcessTime_->stop();
}

void BaseSolver::returnVehiclesZone(const PInstance &EpochInst) const {
    rebalancingProcessTime_->start();
    rebalancingTime_->stop();
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
                                + elapsedTime_ + simulationTime_->dSinceStart().count(),
                                EpochInst->parameters_->Wait_W1_, EpochInst->parameters_->Ride_W2_);
                        break; // Assign to the nearest eligible successor.
                        }
                }
                // If no eligible successor is found, the vehicle remains unassigned.
            }
        }
    }
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
                else
                    availableRoutes[vehicleObj->vehicleID_][i]->calculateTripDelay(mainInst->parameters_->Wait_W1_,
                        mainInst->parameters_->Ride_W2_);
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
    if (EpochInst->parameters_->vehicleReturn_ && elapsedTime_ - rebalanceTime_ >= EpochInst->parameters_->committedTime_) {
        if (EpochInst->parameters_->returnPolicy_ == TO_SOURCE)
            returnVehiclesSource(EpochInst);
        else if (EpochInst->parameters_->returnPolicy_ == ZONE)
            returnVehiclesZone(EpochInst);
        else
            returnVehiclesAssign(EpochInst);
        rebalanceTime_ = elapsedTime_;
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
                    if (!route || route->routeNodes_.size() < 2 || route->getRouteId() == vehicleObj->currentRoute_->getRouteId()) {
                        return true;
                    }

                    for (size_t i = 0; i < vehicleObj->removeNodes_.size(); i++) {
                        if (vehicleObj->removeNodes_[i] != route->routeNodes_[i+1]->nodeID_)
                            return true;
                        else
                            continue;
                    }
                }),
            availableRoutes[vehicleObj->vehicleID_].end());
    }


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
        else
            subProBreak = solve_SP<PCplexSubPro>(EpochInst, mainInst, iter, nbNegativeFound,
                MP_solver_->availableRoutes_, MP_solver_->availableTime_, MP_solver_->MPnbRoutes_, MP_solver_->duplicatesRoutes_);

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
           << runtimeMetrics_->nbRoutes_ <<"\n";

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
           << EpochInst->nbCommitted_ << "\n";
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

            } else { // CPLEX subproblem
                subProSolve.emplace_back(std::make_shared<CPLEXSubProblem>(vehicleObj));
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
            else {
                try {
                    subProblem->initSubGraph(EpochInst);
                    subProblem->solveSP(EpochInst,availableRoutes[subProblem->Vehicle_->vehicleID_]);
                } catch (IloException& e) {
                    std::cerr << "CPLEX Exception in subproblem: " << e.getMessage() << std::endl;
                    subProBreak = true;
                }
            }
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

void RuntimeMetrics::updateSubproblemMetrics(const PCplexSubPro &subProblem) {
    nbNegativeFound_ += subProblem->nbNegativeColumns_;
    nbGenerated_ += subProblem->nbGenerated_;
}
