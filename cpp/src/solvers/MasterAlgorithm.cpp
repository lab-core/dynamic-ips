//
// Created by Ella on 2021-10-09.
//

#include "MasterAlgorithm.h"

#include <unordered_set>
#include <cstdlib>

#include "BaseSolver.h"

// Solver-agnostic logic. CPLEX-specific bodies (solveCP_CPLEX) live in
// MasterAlgorithm_CPLEX.cpp; Gurobi-specific bodies (solveCP_Gurobi,
// solveCP_Dual_Gurobi) live in MasterAlgorithm_Gurobi.cpp. Each is compiled
// only when the matching USE_* option is enabled (see src/solvers/CMakeLists.txt).
#ifdef DARP_USE_CPLEX
#include "CplexSolver/CP_Cplex.h"
#endif
#ifdef DARP_USE_GUROBI
// calcDualsStatistics() catches GRBException; this transitively pulls in gurobi_c++.h.
#include "GurobiSolver/CPModeler.h"
#endif

//---------------------------------------------------------------------------------------------
//  Reduced Problem class
//  Build and solve the Reduced problem of the ISUD
//---------------------------------------------------------------------------------------------

MasterAlgorithm::MasterAlgorithm(const InputPaths &inputPaths) {

    objValue_ = 0;
    previousObj_ =0;
    lpObjValue_ = 0;
    totalWaitTime_ = 0;
    totalTripDelay_ = 0;
    masterTime_ = new myTools::Timer(); masterTime_->init();
    RPTime_ = new myTools::Timer(); RPTime_->init();
    CPTime_ = new myTools::Timer(); CPTime_->init();

    nbVehicles_ = 0;
    availableTime_ = 0;
    timeLimit_ = 0;
    MPEpochSolveTime_ = 0;
    upperbound_ = 0.0;
    iterTime_ = 0;

    MPBuildTime_ = new myTools::Timer(); MPBuildTime_->init();
    CPBuildTime_ = new myTools::Timer(); CPBuildTime_->init();

    ZOOMTime_ = new myTools::Timer(); ZOOMTime_->init();
    RMPCounter_ = 0;

    SPIter_ = 0;
    SPIters_ = 0;

    CGSuccess_ = 0;

    MPnbRoutes_ = 0;
    nbColumnsAdded_ = 0;
    GreedyObjValue_ = 0;

    minReducedCost_ = INFINITY;
    maxReducedCost_ = INFINITY;
    epochTime_ = 0;
    vehicleChange_ = 0;

    pLogMPResultsStream_ = new Tools::LogOutput(inputPaths.getOutputEpochResults());
    (*pLogMPResultsStream_) << "Epoch,MPIter,subProIter,TotalGenColumns,nbColumns,Model,ObjectiveValue,"
                                 "preObjective,RelativeImprove,MPTimeAcc.,SubProTime,IterTime,vehicleChange,AuxObj" << std::endl;

    pLogIterReqDualStream_ = new Tools::LogOutput(inputPaths.getOutputReqDuals());
    (*pLogIterReqDualStream_) << "Epoch,MPIter,RequestID,InitialDual,Dual,MinDual,MaxDual,AvgDual,Model,Penalty,diff" << std::endl;

    pLogIterVehDualStream_ = new Tools::LogOutput(inputPaths.getOutputVehDuals());
    (*pLogIterVehDualStream_) << "Epoch,MPIter,VehicleID,InitialDual,Dual,Model,delay,tripDelay,objCoef"
                                 "nbNodes,nbRequests,totalLength,stateChaneged" << std::endl;
}

MasterAlgorithm::~MasterAlgorithm() {
    delete masterTime_;
    delete RPTime_;
    delete CPTime_;
    delete MPBuildTime_;
    delete CPBuildTime_;
    delete ZOOMTime_;

    pLogMPResultsStream_->close();
    pLogIterReqDualStream_->close();
    pLogIterVehDualStream_->close();

    delete pLogMPResultsStream_;
    delete pLogIterReqDualStream_;
    delete pLogIterVehDualStream_;
}

void MasterAlgorithm::initializeVehicles(const PInstance &pInst){
    nbVehicles_ = 0;
    for (auto &vehicleObj: pInst->vehicles_) {
        vehicleObj->vehicleIndex_ = nbVehicles_;
        if (vehicleObj->departTime_ != vehicleObj->emptyRoute_->plannedDepartTime_[0]) {
            if (vehicleObj->currentRoute_->routeSize_ == 1)
                vehicleObj->emptyRoute_ = vehicleObj->currentRoute_;
            else {
                vehicleObj->setEmptyRoute(pInst);
            }
        }
        nbVehicles_++;
    }
}

void MasterAlgorithm::setInitialDuals(PInstance &pInst, InputPaths &inputPaths, int epoch) {
    if (pInst->parameters_->initialDual_ == PENALTIES){
        for (auto &requestObj : pInst->requests_) {
            requestObj->dual_ = requestObj->Req_W3_ * requestObj->penalty_;
        }
        for (auto &vehicleObj: pInst->vehicles_)
            vehicleObj->dual_ = 0;
    }
    else if (pInst->parameters_->initialDual_ == LAST_LP) {
        for (auto &requestObj : zSolution_) {
            requestObj->dual_ = requestObj->Req_W3_ * requestObj->penalty_;
        }

    }
}

void MasterAlgorithm::setLastDuals(PInstance &pInst) {
    // request duals
    for (size_t i = 0; i < pInst->requests_.size(); ++i)
        pInst->requests_[i]->InitialDual_ = pInst->requests_[i]->dual_;

    // vehicle duals
    for (size_t i = 0; i < pInst->vehicles_.size(); ++i)
        pInst->vehicles_[i]->InitialDual_ = pInst->vehicles_[i]->dual_;
}


void MasterAlgorithm::createInitialSolution(PInstance &pInst, const PGreedyModeler &GreedyModel){
    if (pInst->parameters_->initialStart_ == EMPTY_ROUTES){
        zSolution_.clear();
        routeSolution_.clear();
        for (auto &requestObj : pInst->requests_) {
            zSolution_.push_back(requestObj);
        }
        for (auto &vehicleObj: pInst->vehicles_) {
            routeSolution_.push_back(vehicleObj->emptyRoute_);
            vehicleObj->setCurrentRoute(vehicleObj->emptyRoute_);
        }
        setCurrentRoutes(pInst);
        setObjValue();
    }
    else if (pInst->parameters_->initialStart_ == PRE_SOLUTION){
        if (routeSolution_.empty()){
            for (auto &vehicleObj: pInst->vehicles_) {
                routeSolution_.push_back(vehicleObj->currentRoute_);
            }
        }
        for (int i = pInst->nbRequests_ - pInst->nbNewRequests_; i < pInst->nbRequests_; ++i){
            if (pInst->requests_[i]->solVehicleID_ == LARGE_CONSTANT)
                zSolution_.push_back(pInst->requests_[i]);
        }
        setObjValue();
    }

    else if (pInst->parameters_->initialStart_ == GREEDY_START){
        pInst->parameters_->greedyReOptimize_ = false;
        GreedyModel->GreedySolver(pInst);
        routeSolution_.clear();
        zSolution_.clear();
        for (auto &vehicleObj: pInst->vehicles_) {
            routeSolution_.push_back(vehicleObj->currentRoute_);
        }
        setObjValue();
        GreedyObjValue_ = objValue_;
        std::cout << "Objective value of Greedy Warm start: " << GreedyObjValue_ << std::endl;
    }

    // create upper bound
    upperbound_ = GreedyModel->GreedyUpperbound(pInst);
    std::cout << "Upper Bound by Greedy: " << upperbound_ << std::endl;
}

// this function creates initial routes serving only one request and fill zSolution_ with available requests
// Reduced problem is also solved to initialized dual costs
void MasterAlgorithm::initialization(PInstance &pInst, const InputPaths &inputPaths, const PGreedyModeler &GreedyModel) {
    masterTime_->start();
    MPEpochSolveTime_ = 0;
    nbColumnsAdded_ = 0;
    RMPCounter_ = 0;
    SPIter_ = 0;
    epochTime_ = 0;

    initializeVehicles(pInst);
    // create a feasible integer solution at the start of epoch or simulation
    createInitialSolution(pInst, GreedyModel);
    masterTime_->stop();
}

//This function updates the incompatibility degree of availableRoutes
void MasterAlgorithm::updateIncDegrees(PInstance &pInst, bool greedyBase) {
 //   calcAdjacenyPairs(pInst);
    for (auto & vehicleObj : pInst->vehicles_) {
        vehicleObj->currentRoute_->createColumn(pInst->nbRequests_);
        vehicleObj->greedyRoute_->createColumn(pInst->nbRequests_);
        if (!availableRoutes_[vehicleObj->vehicleID_].empty()) {
            for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]){
                routeObj->createColumn(pInst->nbRequests_);
                if (!greedyBase)
                    assessReqVehCompatibility(routeObj, pInst);
  //                  calcCompatibilityM1(routeObj, vehicleObj->currentRoute_);
                else
                    calcCompatibilityM1(routeObj, vehicleObj->greedyRoute_);
            }
        }
    }
}

void MasterAlgorithm::calcAdjacenyPairs(PInstance &pInst) {
    adjacencyPairs_.clear();
    vehiclePairs_.clear();
    for (auto & vehicleObj : pInst->vehicles_) {
        if (!vehicleObj->currentRoute_->routeRequests_.empty()) {
            vehiclePairs_.emplace_back(vehicleObj->vehicleID_, vehicleObj->currentRoute_->routeRequests_[0]->taskIndex_);
            if (vehicleObj->currentRoute_->routeRequests_.size() > 1) {
                for (size_t i=0; i + 1 < vehicleObj->currentRoute_->routeRequests_.size(); ++i) {
                    adjacencyPairs_.emplace_back(vehicleObj->currentRoute_->routeRequests_[i]->taskIndex_,
                        vehicleObj->currentRoute_->routeRequests_[i + 1]->taskIndex_);
                }
            }
        }
    }
}


// This function determines whether a given route is compatible with the current route of its assigned vehicle
// Just check all the requests are covered or not (max degree = 1)
void MasterAlgorithm::assessReqCompatibility(PRoute &route, PInstance &pInst) {
    route->isCompatible_ = true;
    route->incompatibilityDegree_ = 0;

    // If this column does not cover the requests of the current route in related vehicle
    if ((route->column_ & pInst->vehicles_[route->vehicleID_]->currentRoute_->column_)!=pInst->vehicles_[route->vehicleID_]->currentRoute_->column_){
        route->isCompatible_ = false;
        route->incompatibilityDegree_++;
    }
}

// This function determines whether a given route is compatible with the current route of its assigned vehicle
// Check all the current requests are covered or not, make sure other requests are not covered by other vehicles
// (max degree = (coverage = 1) + (number of vehicles - 1))

void MasterAlgorithm::assessReqVehCompatibility(PRoute &route, PInstance &pInst) {
    assessReqCompatibility(route,pInst);
    boost::dynamic_bitset<> vehicles;
    if ((route->column_ & pInst->vehicles_[route->vehicleID_]->currentRoute_->column_)!=pInst->vehicles_[route->vehicleID_]->currentRoute_->column_){
        route->isCompatible_ = false;
        route->incompatibilityDegree_++;
    }
    if (route->isCompatible_ ) {
        vehicles.resize(pInst->vehicles_.size());
        for (auto & requestObj : route->routeRequests_){
            if (requestObj->solVehicleID_ < LARGE_CONSTANT)
                vehicles.set(requestObj->solVehicleID_, true);
        }
        vehicles.set(route->vehicleID_, false);

        // Count incompatibility if there are any incompatible vehicles
        if (vehicles.count() > 0) {
            route->isCompatible_ = false;
            route->incompatibilityDegree_ += static_cast<int>(vehicles.count());
        }
    }
}

// This function calculates the symmetric difference between two routes  - it counts both:
// Requests that would be lost (current route has, new route doesn't)
// Requests that would be gained (new route has, current route doesn't)
void MasterAlgorithm::calcCompatibilityM1(PRoute &route, PRoute &currentVehicleRoute) {
    route->isCompatible_ = true;
    route->incompatibilityDegree_ = 0;
    // Add coefficients for request constraints
    for (auto& requestObj : currentVehicleRoute->routeRequests_) {
        if (!route->column_.test(requestObj->taskIndex_) ) {
            route->incompatibilityDegree_++;
        }
    }
    for (auto& requestObj : route->routeRequests_) {
        if (!currentVehicleRoute->column_.test(requestObj->taskIndex_) ) {
            route->incompatibilityDegree_++;
        }
    }

    if (route->incompatibilityDegree_ > 0)
        route->isCompatible_ = false;
}

// This function calculates the incompatibility degree based on M2 function just for the request pairs
void MasterAlgorithm::calcCompatibilityM2(PRoute &route, PInstance &pInst) {
    route->isCompatible_ = true;
    route->incompatibilityDegree_ = 0;
    if (!route->routeRequests_.empty()) {
        for (auto & e : adjacencyPairs_) {
            route->incompatibilityDegree_ += route->column_.test(e.first) ^ route->column_.test(e.second);
        }
    }
    if (route->incompatibilityDegree_ > 0)
        route->isCompatible_ = false;
}

void MasterAlgorithm::calcCompatibilityM2Full(PRoute &route, PInstance &pInst) {
    route->isCompatible_ = true;
    route->incompatibilityDegree_ = 0;
    if (!route->routeRequests_.empty()) {
        for (auto & e : adjacencyPairs_) {
            route->incompatibilityDegree_ += route->column_.test(e.first) ^ route->column_.test(e.second);
        }
    }
    if (route->incompatibilityDegree_ > 0) {
        route->isCompatible_ = false;
    }
    for (auto & e : vehiclePairs_) {
        if ((route->vehicleID_ == e.first) ^ route->column_.test(e.second))
         route->incompatibilityDegree_ ++;
    }
}

void MasterAlgorithm::updateScore1(PInstance &pInst) {
    auto start = std::chrono::high_resolution_clock::now(); // Start timer

    for (auto & vehicleObj : pInst->vehicles_) {
        if (!availableRoutes_[vehicleObj->vehicleID_].empty()) {
            for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]){
                routeObj->IncScoreRatio_ = 0.0;
                float m_j = static_cast<float>(routeObj->routeRequests_.size()) + 1.0f;
                if (m_j == 0.0) continue;
                for (auto & basisRouteObj: routeSolution_) {
                    float m_l = static_cast<float>(basisRouteObj->routeRequests_.size()) + 1.0f;
                    if (m_l == 0.0) continue;
                    float overlap = 0.0f;
                    for (const auto& req : routeObj->routeRequests_) {
                        if (basisRouteObj->column_.test(req->taskIndex_)) {
                            overlap += 1.0f;
                        }
                    }
                    if (basisRouteObj->vehicleID_ == routeObj->vehicleID_)
                        overlap += 1.0f;
                    routeObj->IncScoreRatio_ += (overlap * overlap) / (m_j * m_l);
                }
                for (auto & requestObj: routeObj->routeRequests_) {
                    if (requestObj->solVehicleID_ == LARGE_CONSTANT)
                        routeObj->IncScoreRatio_ += 1 / m_j;
                }
                routeObj->IncScoreRatio_ *= routeObj->reducedCost_;
            }
        }
    }

    auto end = std::chrono::high_resolution_clock::now(); // End timer
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "updateScore took " << elapsed.count() << " seconds\n";
}

void MasterAlgorithm::updateScore(PInstance &pInst) {
    // Precompute current solution structure for efficiency
    std::vector<boost::dynamic_bitset<>> currentRouteBitsets(pInst->nbVehicles_);
    std::vector<float> currentRouteSizes(pInst->nbVehicles_);

    for (size_t v = 0; v < pInst->nbVehicles_; ++v) {
        auto& currentRoute = pInst->vehicles_[v]->currentRoute_;
        currentRouteBitsets[v] = currentRoute->column_;
        currentRouteSizes[v] = static_cast<float>(currentRoute->routeRequests_.size()) + 1.0f;
    }

    // Update scores for all available routes
    for (auto& vehicleObj : pInst->vehicles_) {
        int vID = vehicleObj->vehicleID_;

        for (auto& routeObj : availableRoutes_[vID]) {
            routeObj->IncScoreRatio_ = 0.0;

            float m_j = static_cast<float>(routeObj->routeRequests_.size()) + 1.0f;
            if (m_j <= 1.0f) continue; // Skip empty routes

            // Compute compatibility with each basis column (current routes)
            for (size_t basisVehicle = 0; basisVehicle < pInst->nbVehicles_; ++basisVehicle) {
                float m_l = currentRouteSizes[basisVehicle];
                if (m_l <= 1.0f) continue; // Skip empty basis routes

                // Count overlapping requests
                float overlap = static_cast<float>((routeObj->column_ & currentRouteBitsets[basisVehicle]).count());

                // Add 1 if both routes use the same vehicle (vehicle assignment constraint)
                if (basisVehicle == vID) {
                    overlap += 1.0f;
                }

                // Add contribution to score
                routeObj->IncScoreRatio_ += (overlap * overlap) / (m_j * m_l);
            }

            // Handle unserved requests (if they contribute to compatibility)
            int unservedCount = 0;
            for (const auto& req : routeObj->routeRequests_) {
                if (req->solVehicleID_ == LARGE_CONSTANT) {
                    unservedCount++;
                }
            }
            if (unservedCount > 0) {
                // Each unserved request forms its own "basis column" with z_i = 1
                // Contributing 1/m_j per unserved request
                routeObj->IncScoreRatio_ += static_cast<float>(unservedCount) / m_j;
            }

            routeObj->IncScoreRatio_ *= routeObj->reducedCost_;

        }
    }
}

// Display function
std::string MasterAlgorithm::toString() const {

    std::stringstream repStr;
    repStr << "#" << std::endl;
    repStr << std::left << std::fixed << std::setprecision(2);
    repStr << "# TOTAL OBJECTIVE (WAITING TIMES + PENALTIES) = " << objValue_ << std::endl;
    repStr << "# TOTAL WAITING TIMES                         = " << totalWaitTime_ << std::endl;
    repStr << "# TOTAL TRIP DELAY                            = " << totalTripDelay_ << std::endl;
    repStr << "# NUMBER OF UN-SERVED REQUESTS                 = " << zSolution_.size() << std::endl;
    repStr << "#" << std::endl;

    repStr << "# TIME SPENT ON MP IMPROVEMENT              = " << masterTime_->dSinceStart().count() << " (s)" << std::endl;
    repStr << "# TIME SPENT ON RP IMPROVEMENT                = " << RPTime_->dSinceStart().count() << " (s)" << std::endl;
    repStr << "# TIME SPENT ON CP IMPROVEMENT                = " << CPTime_->dSinceStart().count() << " (s)" << std::endl;
    repStr << "# TIME SPENT ON ZOOM ISUD                     = " << ZOOMTime_->dSinceStart().count() << " (s)" << std::endl;
    return repStr.str();
}

std::string MasterAlgorithm::toStringTimersTotal() const {
    std::stringstream repStr;
    repStr << std::left << std::fixed << std::setprecision(2);
    repStr << "#" << std::endl;
    repStr << "# -------------------   TOTAL MP RUN TIMES   -------------------" << std::endl;
    repStr << "#" << std::endl;
    repStr << std::setw(SENTENCE_SIZE) << "# TIME SPENT ON MP IMPROVEMENT" << " = " << masterTime_->dSinceInit().count() << " (s)" << std::endl;
    repStr << std::setw(SENTENCE_SIZE) << "# TIME SPENT ON RP IMPROVEMENT" << " = " << RPTime_->dSinceInit().count() << " (s)" << std::endl;
    repStr << std::setw(SENTENCE_SIZE) << "# TIME SPENT ON CP IMPROVEMENT" << " = " << CPTime_->dSinceInit().count() << " (s)" << std::endl;
    repStr << std::setw(SENTENCE_SIZE) << "# TIME SPENT ON ZOOM ISUD" << " = " << ZOOMTime_->dSinceInit().count() << " (s)" << std::endl;

    return repStr.str();
}

std::string MasterAlgorithm::toStringTimersAvg(int epoch) const {
    std::stringstream repStr;
    repStr << std::left << std::fixed << std::setprecision(2);
    repStr << "#" << std::endl;
    repStr << "# -------------   AVERAGE MP RUN TIMES PER EPOCH   -------------" << std::endl;
    repStr << "#" << std::endl;
    repStr << std::setw(SENTENCE_SIZE) << "# TIME SPENT ON MP IMPROVEMENT" << " = " << masterTime_->dSinceInit().count() / static_cast<float>(epoch) << " (s)" << std::endl;
    repStr << std::setw(SENTENCE_SIZE) << "# TIME SPENT ON RP IMPROVEMENT" << " = " << RPTime_->dSinceInit().count()/static_cast<float>(epoch) << " (s)" << std::endl;
    repStr << std::setw(SENTENCE_SIZE) << "# TIME SPENT ON CP IMPROVEMENT" << " = " << CPTime_->dSinceInit().count()/static_cast<float>(epoch) << " (s)" << std::endl;
    repStr << std::setw(SENTENCE_SIZE) << "# TIME SPENT ON ZOOM ISUD" << " = " << ZOOMTime_->dSinceInit().count() / static_cast<float>(epoch) << " (s)" << std::endl;

    return repStr.str();
}

void MasterAlgorithm::updateReducedCosts(const PInstance &pInst) {
    minReducedCost_ = INFINITY;
    maxReducedCost_ = INFINITY;
    for (auto & vehicleObj : pInst->vehicles_){
        for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]){
            routeObj->reducedCost_ = routeObj->objCoef_ - vehicleObj->dual_;
            for (auto & nodeObj: routeObj->routeNodes_){
                if (nodeObj->type_ == PICKUP){
                    routeObj->reducedCost_ -= nodeObj->related_Request_->dual_;
                }
            }
            if (minReducedCost_ > routeObj->reducedCost_ )
                minReducedCost_ = routeObj->reducedCost_;
            if (!routeObj->routeRequests_.empty()) {
                routeObj->normal_RC_ = routeObj->reducedCost_ / static_cast<float>(routeObj->routeRequests_.size());
                routeObj->lambda_ = routeObj->objCoef_ / (routeObj->objCoef_ - routeObj->reducedCost_ + 0.0001f);
  //              routeObj->waitScore_ = 0;
                /*for (int i = 0; i < routeObj->routeRequests_.size(); ++i) {
                    if (routeObj->routeRequests_[i]->plannedDelay_ != LARGE_CONSTANT)
                        routeObj->waitScore_ += routeObj->plannedDelay_[i] - routeObj->routeRequests_[i]->plannedDelay_;
                }*/
            }
            else {
                routeObj->normal_RC_ = 0;
                routeObj->lambda_ = 0;
            }
            routeObj->IncScore_ = routeObj->reducedCost_ * routeObj->IncScoreRatio_;
        }
        vehicleObj->bestReducedCost_ = minReducedCost_;
    }
    if (minReducedCost_ < 0)
        maxReducedCost_ = -1.0f * minReducedCost_;
}

void MasterAlgorithm::updateRoutesToAdd(SelectionMode selectMode, PInstance &pInst, std::vector<PRoute> &routesToAdd){
    updateReducedCosts(pInst);
    if (selectMode == CP || selectMode == RP) {
        updateIncDegrees(pInst, false);
        if (pInst->parameters_->sortColumn_ == COMP_C) {
            updateScore(pInst);
        }
        /*if (selectMode == CP) {
            for (int v = 0; v < pInst->nbVehicles_; ++v) {
                if (!pInst->vehicles_[v]->emptyRoute_->cpAdded_ &&
                    !pInst->vehicles_[v]->currentRoute_->routeRequests_.empty()) {
                    routesToAdd.push_back(pInst->vehicles_[v]->emptyRoute_);
                    pInst->vehicles_[v]->emptyRoute_->cpAdded_ = true;
                    }
            }
        }*/
    }

    if (minReducedCost_ <= 0 || selectMode == CP) {
        for (auto & vehicleObj : pInst->vehicles_) {
            if (pInst->parameters_->sortColumn_ == NORMAL_RC)
                std::stable_sort(availableRoutes_[vehicleObj->vehicleID_].begin(),
                                availableRoutes_[vehicleObj->vehicleID_].end(),
                                [](const PRoute &lhs, const PRoute &rhs) { return lhs->normal_RC_ < rhs->normal_RC_; });

            else if (pInst->parameters_->sortColumn_ == LAMBDA_S)
                std::stable_sort(availableRoutes_[vehicleObj->vehicleID_].begin(),
                             availableRoutes_[vehicleObj->vehicleID_].end(),
                             [](const PRoute &lhs, const PRoute &rhs) { return lhs->lambda_ < rhs->lambda_; });

            else if (pInst->parameters_->sortColumn_ == COMP_C) {
                std::stable_sort(availableRoutes_[vehicleObj->vehicleID_].begin(),
                             availableRoutes_[vehicleObj->vehicleID_].end(),
                             [](const PRoute &lhs, const PRoute &rhs) { return lhs->IncScore_ < rhs->IncScore_; });
//                vehicleObj->emptyRoute_->createColumn(pInst->nbRequests_);
//                routesToAdd.push_back(vehicleObj->emptyRoute_);
            }
            else
                std::stable_sort(availableRoutes_[vehicleObj->vehicleID_].begin(),
                                 availableRoutes_[vehicleObj->vehicleID_].end(),
                                 [](const PRoute &lhs, const PRoute &rhs) { return lhs->reducedCost_ < rhs->reducedCost_; });

            int numAdded = 0;
            for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]) {
                switch(selectMode){
                    case CP:
                         if (!routeObj->cpAdded_ && !routeObj->isCompatible_ ) {
                             routesToAdd.push_back(routeObj);
  //                           numAdded++;
                         }
                        break;
                    case RP:
                        if (!routeObj->mpAdded_ && routeObj->isCompatible_  && routeObj->reducedCost_ <= 0) {
                            routesToAdd.push_back(routeObj);
                            numAdded++;
                        }
                        break;
                    default: // CG and MIP:
                        if (!routeObj->mpAdded_ && routeObj->reducedCost_ <= 0) {
                            routesToAdd.push_back(routeObj);
                            numAdded++;
                        }
                        break;
                }
                if (numAdded > pInst->parameters_->nbColumn_)
                    break;
            }
            nbColumnsAdded_ += numAdded;
        }
    }
}

void MasterAlgorithm::updateRoutesToAddOne(SelectionMode selectMode, PInstance &pInst, std::vector<PRoute> &routesToAdd) {
    for (auto & vehicleObj : pInst->vehicles_) {
        for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]) {
            if (!routeObj->mpAdded_ && routeObj->routeRequests_.size() == 1)
                routesToAdd.push_back(routeObj);
        }
    }
}

void MasterAlgorithm::reFillRoutesToAdd(PInstance &pInst, std::vector<PRoute> &routesToAdd) {

    for (auto & vehicleObj : pInst->vehicles_) {
        for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]) {
            if (routeObj->getRouteId() == vehicleObj->currentRoute_->getRouteId())
                continue;
            // Insert returns {iterator, inserted}; inserted==true means new
            if (routeObj->keepMP_) {
                // first time we see this column pattern → keep it
                routeObj->createColumn(pInst->nbRequests_);
                routesToAdd.push_back(routeObj);
                nbColumnsAdded_++;
            }
        }
    }
}

void MasterAlgorithm::reFillRoutesToAddCP(PInstance &pInst, std::vector<PRoute> &routesToAdd) {
    updateIncDegrees(pInst, false);
    nbColumnsAdded_ = 0;
    for (int v = 0; v < pInst->nbVehicles_; ++v) {
        pInst->vehicles_[v]->emptyRoute_->createColumn(pInst->nbRequests_);
        routesToAdd.push_back(pInst->vehicles_[v]->emptyRoute_);
    }
    for (auto & vehicleObj : pInst->vehicles_) {
        std::unordered_set<std::string> seen;
        for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]) {
            if (routeObj->getRouteId() == vehicleObj->currentRoute_->getRouteId() || routeObj->routeRequests_.empty())
                continue;
            std::string key = makeKey(*routeObj, vehicleObj->vehicleID_);
            if (seen.insert(key).second) {
                if (routeObj->incompatibilityDegree_ > 1) {
                    routesToAdd.push_back(routeObj);
                    nbColumnsAdded_++;
                }
            }
        }
    }
}

void MasterAlgorithm::updateRoutesToAddZoom(std::vector<PRoute> &routesToAdd) const {
    if (!CPSolver_) return;
    for (auto &routeObj : CPSolver_->fractionalRoutes_) {
        if (!routeObj->mpAdded_)
            routesToAdd.push_back(routeObj);
    }
    for (auto &routeObj : CPSolver_->IncRoute_) {
        if (!routeObj->mpAdded_ && routeObj->reducedCost_ <= 0)
            routesToAdd.push_back(routeObj);
    }
}

// function to save the reduced costs and incompatibility degree of the created routes
void MasterAlgorithm::save_IncDegree_RDCost(const InputPaths &inputPaths, int epoch, int isudIter) const {
    std::ofstream myFile;
    myFile.open (inputPaths.getOutputIncDegreeRdCost(), std::ofstream::app);

    for (int i = 0; i < availableRoutes_.size(); i++){
        for (auto & routeObj : availableRoutes_[i]) {
            myFile << epoch << ",";
            myFile << isudIter << ",";
            myFile << i << ",";
            myFile << routeObj->incompatibilityDegree_ << ",";
            myFile << routeObj->reducedCost_ << ",";
            myFile << routeObj->createTime_ << ",";
            myFile << routeObj->getRouteId() << "\n";
        }
    }
    myFile.close();
}


std::string MasterAlgorithm::save_MPResults(int epoch, const std::string& model, int nbColumns, float reachTime,
                                            float subProTime, float auxObj) const {
    std::stringstream repStr;
    repStr << epoch << ",";
    repStr << RMPCounter_ << ",";
    repStr << SPIter_ << ",";
    repStr << MPnbRoutes_ << ",";
    repStr << nbColumns << ",";
    repStr << model << ",";
    repStr << objValue_ << ",";
    repStr << previousObj_ << ",";
    repStr << 100*((previousObj_ - objValue_)/previousObj_) << ",";
    repStr << reachTime << ",";
    repStr << subProTime << ",";
    repStr << epochTime_ << ",";
    repStr << vehicleChange_ << ",";
    repStr << auxObj << "\n";
    return repStr.str();
}

void MasterAlgorithm::setCurrentRoutes(const PInstance &pInst) {
    pInst->resetAssignedVehicles();
    for (auto &routeObj : routeSolution_) {
        pInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
    }
    zSolution_.clear();
    for (auto &requestObj: pInst->requests_) {
        if (requestObj->solVehicleID_ == LARGE_CONSTANT)
            zSolution_.push_back(requestObj);
    }
}

void MasterAlgorithm::setObjValue() {
    objValue_ = 0.0;
    totalWaitTime_ = 0.0;
    totalTripDelay_ = 0.0;
    for (auto & routeObj : routeSolution_) {
        objValue_ += routeObj->objCoef_;
        totalWaitTime_ += routeObj->totalWait_;
        totalTripDelay_ += routeObj->totalTripDelay_;
    }
    for (auto & zRequest : zSolution_) {
        objValue_+= zRequest->Req_W3_ * zRequest->penalty_;
    }
}

void MasterAlgorithm::setAvailableTime() {
    availableTime_ = timeLimit_ - masterTime_->dSinceStart().count();
}

bool MasterAlgorithm::hasTimeRemaining(const PInstance &pInst, float elapsedTime, int iteration) {
    if (iteration > 1) {
        switch (pInst->parameters_->solutionMode_) {
            case ANYTIME:
                availableTime_ = pInst->parameters_->committedTime_ - elapsedTime;
                break;
            case DYNAMIC:
                if (pInst->parameters_->mainAlgorithm_ == RT_CG)
                    availableTime_ = pInst->parameters_->epochLength_ - elapsedTime - 2;
                else
                    availableTime_ = pInst->parameters_->epochLength_ - elapsedTime;
                break;
            case STATIC:
                availableTime_ = LARGE_CONSTANT;
                break;
        }
    }
    else
        availableTime_ = LARGE_CONSTANT;
    if (availableTime_ <= 0)
        return false;
    return true;
}

void MasterAlgorithm::updateEpochTimers(PRuntimeMetrics &runtimeMetrics) {
    runtimeMetrics->masterEpochTime_ = masterTime_->dSinceInit().count();
    runtimeMetrics->RPEpochTime_ = RPTime_->dSinceInit().count();
    runtimeMetrics->RPEpochBuildTime_ = MPBuildTime_->dSinceInit().count();
    runtimeMetrics->CPEpochTime_ = CPTime_->dSinceInit().count();
    runtimeMetrics->CPEpochBuildTime_ = CPBuildTime_->dSinceInit().count();
    runtimeMetrics->isudMIPEpochTime_ = ZOOMTime_->dSinceInit().count();
}

std::string MasterAlgorithm::runtimesToString(PRuntimeMetrics &runtimeMetrics) {
    std::stringstream repStr;

    // master problem Times
    repStr << masterTime_->dSinceInit().count() - runtimeMetrics->masterEpochTime_ << ","
           << RPTime_->dSinceInit().count() - runtimeMetrics->RPEpochTime_ << ","
           << MPBuildTime_->dSinceInit().count() - runtimeMetrics->RPEpochBuildTime_ << ","
           << MPEpochSolveTime_ << ","
           << CPTime_->dSinceInit().count() - runtimeMetrics->CPEpochTime_ << ","
           << CPBuildTime_->dSinceInit().count() - runtimeMetrics->CPEpochBuildTime_ << ","
           << CPEpochSolveTime_ << ","
           << ZOOMTime_->dSinceInit().count() - runtimeMetrics->isudMIPEpochTime_ << ",";

    updateEpochTimers(runtimeMetrics);

    repStr << runtimeMetrics->subproEpochTime_ << ","
           << SPIter_ << ","
           << MPnbRoutes_ << ","
           << runtimeMetrics->nbGenerated_ << ","
           << runtimeMetrics->nbDominated_ << ","
           << runtimeMetrics->nbEliminated_ << ","
           << runtimeMetrics->nbPrunedArcs_ << ","
           << runtimeMetrics->nbPrunedPath_ << ","
           << runtimeMetrics->nbNegativeFound_ << ","
           << nbColumnsAdded_ << ","
           << runtimeMetrics->nbRecycledColumns_ << ","
           << GreedyObjValue_ << ","
           << objValue_ << ","
           << lpObjValue_ << ","
           << totalWaitTime_ << ","
           << totalTripDelay_ << ","
            // add the dual stats here:
           << summaryDuals_.maxDual << ","
           << summaryDuals_.minDual << ","
           << summaryDuals_.meanDual << ","
           << subproSummary_.maxSize << ","
           << subproSummary_.meanSize << ","
           << subproSummary_.maxRoute << ","
           << subproSummary_.meanRoute << ",";
    return repStr.str();
}

void MasterAlgorithm::createFinalOutputString(const PInstance &pInst, float subproblemTime, float greedyRuntime,
    float rebalancingRuntime) {
    pInst->instRepStr_ << LPIter_ << "," << MIPIter_ << ",";
    pInst->instRepStr_ << RPIter_ << "," << CPIter_ << ",";
    pInst->instRepStr_ << ZoomIter_ << ",";
    pInst->instRepStr_ << SPIters_ << ",";
    pInst->instRepStr_ << masterTime_->dSinceInit().count() << ",";
    pInst->instRepStr_ << RPTime_->dSinceInit().count() << ",";
    pInst->instRepStr_<< CPTime_->dSinceInit().count() << ",";
    pInst->instRepStr_ << ZOOMTime_->dSinceInit().count() << ",";
    pInst->instRepStr_ << subproblemTime << "," << greedyRuntime << "," << rebalancingRuntime << ",";
    float TotalTime = masterTime_->dSinceInit().count() + subproblemTime + greedyRuntime;
    pInst->instRepStr_ << TotalTime << ",";
    if (masterTime_->dSinceInit().count() > 0 ){
        pInst->instRepStr_ << RPTime_->dSinceInit().count() / masterTime_->dSinceInit().count() << ",";
        pInst->instRepStr_ << CPTime_->dSinceInit().count() / masterTime_->dSinceInit().count() << ",";
    }
    else{
        pInst->instRepStr_ << 0 << "," << 0 << ",";
    }
    pInst->instRepStr_ << masterTime_->dSinceInit().count() / TotalTime << ",";
    pInst->instRepStr_ << subproblemTime/TotalTime << ",";
    pInst->instRepStr_ << CPSuccess_ << ",";
    pInst->instRepStr_ << CPFails_ << ",";
    pInst->instRepStr_ << CGSuccess_ << ",";
}

void MasterAlgorithm::checkCoveredVehicles(PInstance &pInst) {
    const double reducedCostThreshold = pInst->parameters_->reducedCostThreshold_;

    // Reset coverage / insertion flags
    for (auto & requestObj: pInst->requests_) {
        if (pInst->parameters_->labelingReOptimizeStrategy_ != BY_BASIS) {
            requestObj->coveredVehicles_.reset();
            requestObj->coveredVehicles_.resize(nbVehicles_);
        }
        requestObj->insertedVehicles_.reset();
        requestObj->insertedVehicles_.resize(nbVehicles_);
    }
    // Cover requests already present in current routes
    for (auto & vehicleObj : pInst->vehicles_) {
        for (auto & requestObj: vehicleObj->currentRoute_->routeRequests_) {
            requestObj->coveredVehicles_.set(vehicleObj->vehicleID_, true);
        }
    }
    if (pInst->parameters_->labelingReOptimizeStrategy_ == BY_BASIS) {
        for (auto & vehicleObj : pInst->vehicles_) {
            auto & routes = availableRoutes_[vehicleObj->vehicleID_];

            // Coverage check only on remaining routes
            for (auto & routeObj: routes) {
                if (routeObj->keepMP_) {
                    for (auto & requestObj: routeObj->routeRequests_) {
                        requestObj->coveredVehicles_.set(vehicleObj->vehicleID_, true);
                    }
                }
            }
        }
    }
    else if (pInst->parameters_->labelingReOptimizeStrategy_ == BY_POOL) {
        for (auto & vehicleObj : pInst->vehicles_) {
            auto & routes = availableRoutes_[vehicleObj->vehicleID_];

            // Remove routes with high reduced cost and keepMP_ == false
            routes.erase(
                std::remove_if(routes.begin(), routes.end(),
                    [reducedCostThreshold](const auto &routeObj) {
                        return (routeObj->reducedCost_ > reducedCostThreshold &&
                                routeObj->keepMP_ == false);
                    }),
                routes.end()
            );

            // Coverage check only on remaining routes
            for (auto & routeObj: routes) {
                if (routeObj->keepMP_) {
                    for (auto & requestObj: routeObj->routeRequests_) {
                        requestObj->coveredVehicles_.set(vehicleObj->vehicleID_, true);
                    }
                }
            }
        }
    }
}

void MasterAlgorithm::calcDualsStatistics(const PInstance &pInst) {
    std::vector<double> pi_values;
    pi_values.reserve(pInst->requests_.size());

    // Get request constraint duals
    for (size_t i = 0; i < pInst->requests_.size(); ++i) {
        pi_values.push_back(pInst->requests_[i]->dual_);
    }

    // Compute stats if we have requests
    summaryDuals_.maxDual = *std::max_element(pi_values.begin(), pi_values.end());
    summaryDuals_.minDual = *std::min_element(pi_values.begin(), pi_values.end());
    summaryDuals_.meanDual = std::accumulate(pi_values.begin(), pi_values.end(), 0.0) / pi_values.size();
}


