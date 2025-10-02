//
// Created by Ella on 2021-10-09.
//

#include "MasterAlgorithm.h"

#include "Solver.h"
#include "GurobiSolver/MP_Gurobi.h"
#include "CplexSolver/MIPMasterProblem.h"
#include "CplexSolver/DualAuxSolver.h"
#include "CplexSolver/ComplementPro.h"
#include "GurobiSolver/CP_Gurobi.h"
#include "GurobiSolver/CP_Reduced.h"
#include <unordered_set>
#include "GurobiSolver/CPModeler.h"
#include <cstdlib>

//---------------------------------------------------------------------------------------------
//  Reduced Problem class
//  Build and solve the Reduced problem of the ISUD
//---------------------------------------------------------------------------------------------

MasterAlgorithm::MasterAlgorithm(const InputPaths &inputPaths) {

    objValue_ = 0;
    previousObj_ =0;
    lpObjValue_ = 0;
    totalWaitTime_ = 0;
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

    nbRoutes_ = 0;
    nbColumnsAdded_ = 0;
    nbColumnsLess_50_ = 0;
    nbColumnsLess_100_ = 0;
    nbColumnsLess_200_ = 0;
    nbCoveredTasks_ = 0;
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
    (*pLogIterVehDualStream_) << "Epoch,MPIter,VehicleID,InitialDual,Dual,Model,delay,nbNodes,nbRequests,totalLength,stateChaneged" << std::endl;
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
            requestObj->dual_ = requestObj->penalty_;
        }
        for (auto &vehicleObj: pInst->vehicles_)
            vehicleObj->dual_ = 0;
    }
    else if (pInst->parameters_->initialDual_ == LAST_LP) {
        for (auto &requestObj : zSolution_) {
            requestObj->dual_ = requestObj->penalty_;
        }
        /*for (auto & requestObj : pInst->requests_) {
            requestObj->dual_ = 0.5 * requestObj->dual_ + 0.5 * requestObj->penalty_;
        }
        for (auto & vehicleObj : pInst->vehicles_)
            vehicleObj->dual_ = 0;*/
    }
    else if (pInst->parameters_->initialDual_ == ADJUSTED) {
        for (auto &requestObj : zSolution_) {
            requestObj->dual_ = requestObj->penalty_;
        }
        for (auto &vehicleObj: pInst->vehicles_)
            vehicleObj->adjustDuals();
    }
    else if (availableRoutes_.size() > 0 && pInst->parameters_->routeRecycle_) {
        if (pInst->parameters_->initialDual_ == LAGRANGIAN) {
            lagSolver_ = std::make_unique<LagrangianSolver>(pInst, objValue_,routeSolution_, zSolution_,
                availableRoutes_);
            lagSolver_->run(pInst);
            lagSolver_.reset();
        }
        else if (pInst->parameters_->initialDual_ == INIT_CP) {
  //          solveCP_Dual_Gurobi(pInst, epoch, inputPaths, 0.0);
            for (auto &requestObj : zSolution_) {
                if (requestObj->dual_ == 0)
                    requestObj->dual_ = requestObj->penalty_;
            }
        }
    }
    if (pInst->parameters_->dualMethod_ == AUX_BOX) {
        float box = 0.8;
        for (auto &requestObj : zSolution_) {
            requestObj->dual_ = box * requestObj->penalty_;
        }
    }
    if (pInst->parameters_->initialDual_ == ZERO) {
        float box = 0.8;
        for (auto &requestObj : pInst->requests_) {
            requestObj->dual_ = 0;
        }
        for (auto & vehicleObj : pInst->vehicles_)
            vehicleObj->dual_ = 0;
    }
    if (pInst->parameters_->initialDual_ == RANDOM) {
        for (auto &requestObj : pInst->requests_) {
            requestObj->dual_ = (rand() % (int) (0.5 * requestObj->penalty_))+ 1;
        }
        for (auto & vehicleObj : pInst->vehicles_)
            vehicleObj->dual_ = 0;
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
        zSolution_.capacity();
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
    pInst->parameters_->greedyReOptimize_ = false;
    GreedyModel->greedyTime_->start();
    GreedyModel->initialization(pInst);
    GreedyModel->solveInsertion(pInst);
    upperbound_ = GreedyModel->createUpperbound();
    GreedyModel->greedyTime_->stop();
    std::cout << "Upper Bound by Greedy: " << upperbound_ << std::endl;
}

// this function creates initial routes serving only one request and fill zSolution_ with available requests
// Reduced problem is also solved to initialized dual costs
void MasterAlgorithm::initialization(PInstance &pInst, const InputPaths &inputPaths, const PGreedyModeler &GreedyModel) {
    masterTime_->start();
    MPEpochSolveTime_ = 0;
    nbColumnsAdded_ = 0;
    nbColumnsLess_50_ = 0;
    nbColumnsLess_100_ = 0;
    nbColumnsLess_200_ = 0;
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
                    calcCompatibilityM1(routeObj, vehicleObj->currentRoute_);
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
    route->isCompatible_ = true;
    route->incompatibilityDegree_ = 0;

    // If this column does not cover the requests of the current route in related vehicle
    if ((route->column_ & pInst->vehicles_[route->vehicleID_]->currentRoute_->column_)!=pInst->vehicles_[route->vehicleID_]->currentRoute_->column_){
        route->isCompatible_ = false;
        route->incompatibilityDegree_++;
    }
    boost::dynamic_bitset<> vehicles;
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

    // XOR operation to find differences (differences.count() should be added to incompatibilityDegree_)
    // auto differences = route->column_ ^ pInst->vehicles_[route->vehicleID_]->currentRoute_->column_;

    if (route->incompatibilityDegree_ > 0)
        route->isCompatible_ = false;
}

// This function calculates calculate the incompatibility degree based on M2 function just for the request pairs
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

    // route->IncScore_ = route->reducedCost_/ std::max(route->incompatibilityDegree_, 1);
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
 //               std::cout << "score: " << routeObj->IncScore_ << " - " << routeObj->incompatibilityDegree_ << std::endl;
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
            routeObj->reducedCost_ = routeObj->totalDelay_ - vehicleObj->dual_;
            for (auto & nodeObj: routeObj->routeNodes_){
                if (nodeObj->type_ == PICKUP){
                    routeObj->reducedCost_ -= nodeObj->related_Request_->dual_;
                }
            }
            if (minReducedCost_ > routeObj->reducedCost_ )
                minReducedCost_ = routeObj->reducedCost_;
            if (!routeObj->routeRequests_.empty()) {
                routeObj->score_ = routeObj->reducedCost_ / static_cast<float>(routeObj->routeRequests_.size());
                routeObj->lambda_ = routeObj->totalDelay_ / (routeObj->totalDelay_ - routeObj->reducedCost_ + 0.0001f);
                routeObj->waitScore_ = 0;
                /*for (int i = 0; i < routeObj->routeRequests_.size(); ++i) {
                    if (routeObj->routeRequests_[i]->plannedDelay_ != LARGE_CONSTANT)
                        routeObj->waitScore_ += routeObj->plannedDelay_[i] - routeObj->routeRequests_[i]->plannedDelay_;
                }*/
            }
            else {
                routeObj->score_ = 0;
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
        for (int v = 0; v < pInst->nbVehicles_; ++v) {
            if (!pInst->vehicles_[v]->emptyRoute_->cpAdded_ &&
                !pInst->vehicles_[v]->currentRoute_->routeRequests_.empty()) {
                routesToAdd.push_back(pInst->vehicles_[v]->emptyRoute_);
                pInst->vehicles_[v]->emptyRoute_->cpAdded_ = true;
            }
        }
    }

    if (minReducedCost_ <= 0 || selectMode == CP) {
        for (auto & vehicleObj : pInst->vehicles_) {
            if (pInst->parameters_->sortColumn_ == NORMAL_RC)
                std::stable_sort(availableRoutes_[vehicleObj->vehicleID_].begin(),
                                availableRoutes_[vehicleObj->vehicleID_].end(),
                                [](const PRoute &lhs, const PRoute &rhs) { return lhs->score_ < rhs->score_; });

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
                         if (routeObj->incompatibilityDegree_ > 1) {
                        routesToAdd.push_back(routeObj);
                        //  numAdded++;
                           }
                        break;
                    case RP:
                        if (!routeObj->mpAdded_ && routeObj->incompatibilityDegree_ < 2  && routeObj->reducedCost_ < 0) {
                            routesToAdd.push_back(routeObj);
                            numAdded++;
                        }
                        break;
                    default: // CG and MIP:
                        if (!routeObj->mpAdded_ && routeObj->reducedCost_ < 0) {
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
        std::unordered_set<std::string> seen;
        for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]) {
            if (routeObj->getRouteId() == vehicleObj->currentRoute_->getRouteId())
                continue;
            // Build a simple signature for this column
            std::string key = makeKey(*routeObj, vehicleObj->vehicleID_);
            // Insert returns {iterator, inserted}; inserted==true means new
            if (seen.insert(key).second) {
                // first time we see this column pattern → keep it
                routeObj->createColumn(pInst->nbRequests_);
                routesToAdd.push_back(routeObj);
                nbColumnsAdded_++;
            }
        }
    }
}

void MasterAlgorithm::reFillRoutesToAddCP(PInstance &pInst, std::vector<PRoute> &routesToAdd) {
    updateIncDegrees(pInst, true);
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
    // add fractional routes
    for (auto & routeObj: CompPro_->fractionalRoutes_) {
        if (!routeObj->mpAdded_)
            routesToAdd.push_back(routeObj);
    }

    for (auto & routeObj : CompPro_->IncRoute_) {
        if (!routeObj->mpAdded_ && routeObj->reducedCost_ <= 0)
            routesToAdd.push_back(routeObj);
    }
}


// function to save the reduced costs and incompatibility degree of the created routes
void MasterAlgorithm::save_IncDegree_RDCost(const InputPaths &inputPaths, int epoch, int isudIter) const {
    std::ofstream myFile;
    myFile.open (inputPaths.getOutputIncDegreeRdCost(), std::ofstream::app);

    for (int i = 0; i < availableRoutes_.size(); i++){
        //   for (auto & routeListObj : availableRoutes_) {
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
    repStr << nbRoutes_ << ",";
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
    for (auto & routeObj : routeSolution_) {
        objValue_ += routeObj->totalDelay_;
        totalWaitTime_ += routeObj->totalDelay_;
    }
    for (auto & zRequest : zSolution_) {
        objValue_+= zRequest->penalty_;
    }
}

void MasterAlgorithm::setAvailableTime() {
    availableTime_ = timeLimit_ - masterTime_->dSinceStart().count();
}

void MasterAlgorithm::setAvailableTime(const PInstance &pInst, float elapsedTime, int iteration) {
    if (iteration > 1) {
        switch (pInst->parameters_->solutionMode_) {
            case ANYTIME:
                availableTime_ = pInst->parameters_->committedTime_ - elapsedTime;
                break;
            case DYNAMIC:
                if (pInst->parameters_->mainAlgorithm_ == RT_CG)
                    availableTime_ = pInst->parameters_->epochLength_ - elapsedTime - 5;
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
    availableTime_ = LARGE_CONSTANT;
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
           << nbRoutes_ << ","
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
            // add the dual stats here:
           << summaryDuals_.maxDual << ","
           << summaryDuals_.minDual << ","
           << summaryDuals_.meanDual << ","
           << summaryDuals_.medianDual << ",";
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

void MasterAlgorithm::solveCP_CPLEX(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime) {
    /************************************************************************************************/
    //                                     COMPLEMENTARY PROBLEM
    /************************************************************************************************/
    CompPro_ = std::make_shared<ComplementPro>();
    CompPro_->routesToAdd_.clear();
    updateRoutesToAdd(CP, pInst, CompPro_->routesToAdd_);
    if (minReducedCost_ < 0 && CompPro_->routesToAdd_.size() > 0) {
        CPTime_->start();
        CPBuildTime_->start();
        CompPro_->buildModel(pInst, zSolution_, routeSolution_, nbVehicles_);
        CompPro_->updateModel(pInst, zSolution_, routeSolution_);
        CPBuildTime_->stop();

        while (true) {
            previousObj_ = objValue_;
            CompPro_->solveCPModel(pInst, zSolution_, routeSolution_, inputPaths);
            CPIter_++;
            setCurrentRoutes(pInst);

            CPEpochSolveTime_ += CompPro_->solveTime_->dSinceStart().count();
            setObjValue();
            epochTime_ += (masterTime_->dSinceStart().count() - iterTime_);
            iterTime_ = masterTime_->dSinceStart().count();
            (*pLogMPResultsStream_) << save_MPResults(epoch, "CP", static_cast<int>(CompPro_->IncRoute_.size()),
                                                        masterTime_->dSinceStart().count(), subProTime, 0.0);

            if (CompPro_->status_ == NEGATIVE_VALUE) {

                CPSuccess_++;
                previousObj_ = objValue_;
                RMPCounter_++;
                setAvailableTime();
                if (availableTime_ < 3) {
                    break;
                }
            }
            else {
                break;
            }
        }

        CompPro_.reset();
        CompPro_ = std::make_shared<ComplementPro>();
        CPTime_->stop();
    }
}

void MasterAlgorithm::solveCP_Gurobi(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime) {
    /************************************************************************************************/
    //                                     COMPLEMENTARY PROBLEM
    /************************************************************************************************/
    CPGurobiPro_ = std::make_shared<CPModeler>(inputPaths.getOutputSolverLog());
    CPGurobiPro_->initializeCP(pInst, pInst->parameters_->reducedCP_);
    CPGurobiPro_->routesToAdd_.clear();
    updateRoutesToAdd(CP, pInst, CPGurobiPro_->routesToAdd_);
    CPTime_->start();
    CPBuildTime_->start();
    if (pInst->parameters_->reducedCP_) {
        CPGurobiPro_->buildModel_batch(pInst);
        CPBuildTime_->stop();
        CPGurobiPro_->solveCPModel(pInst, zSolution_, routeSolution_, inputPaths);
    }
    else {
        CPGurobiPro_->buildModel_batch(pInst, routeSolution_);
        CPGurobiPro_->updateModel();
        CPBuildTime_->stop();
        CPGurobiPro_->solveCPModel(pInst, zSolution_, routeSolution_, inputPaths, false);
        setCurrentRoutes(pInst);
    }
    CPIter_++;
    CPEpochSolveTime_ += CPGurobiPro_->solveTime_->dSinceStart().count();
    setObjValue();
    epochTime_ += (masterTime_->dSinceStart().count() - iterTime_);
    iterTime_ = masterTime_->dSinceStart().count();
    (*pLogMPResultsStream_) << save_MPResults(epoch, "CP", static_cast<int>(CPGurobiPro_->IncRoute_.size()),
                                                masterTime_->dSinceStart().count(), subProTime, 0.0);

    CPGurobiPro_.reset();
    CPTime_->stop();
}

void MasterAlgorithm::solveCP_Dual_Gurobi(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime) {
    /************************************************************************************************/
    //                                     COMPLEMENTARY PROBLEM
    /************************************************************************************************/
    /*std:: vector<PRoute> routeBack = routeSolution_;
    routeSolution_.clear();
    for (auto & vehicleObj : pInst->vehicles_) {
        availableRoutes_[vehicleObj->vehicleID_].push_back(vehicleObj->currentRoute_);
        availableRoutes_[vehicleObj->vehicleID_].push_back(vehicleObj->emptyRoute_);
        routeSolution_.push_back(vehicleObj->greedyRoute_);
    }
    setCurrentRoutes(pInst);*/
    CPGurobiPro_ = std::make_shared<CPModeler>(inputPaths.getOutputSolverLog());
    CPGurobiPro_->initializeCP(pInst, pInst->parameters_->reducedCP_);
    CPGurobiPro_->routesToAdd_.clear();
    reFillRoutesToAddCP(pInst, CPGurobiPro_->routesToAdd_);
    //   updateRoutesToAdd(CP, pInst, CPGurobiPro_->routesToAdd_);
    /*for (auto & vehicleObj : pInst->vehicles_) {
        CPGurobiPro_->routesToAdd_.push_back(vehicleObj->emptyRoute_);
    }*/
    if (!CPGurobiPro_->routesToAdd_.empty()) {
        CPTime_->start();
        CPBuildTime_->start();
        if (pInst->parameters_->reducedCP_) {
            CPGurobiPro_->buildModel_batch(pInst);
        }
        else {
            CPGurobiPro_->buildModel_batch(pInst, routeSolution_);
            CPGurobiPro_->updateModel();
        }
        CPBuildTime_->stop();
        CPGurobiPro_->solveCPDual(pInst, inputPaths);
        CPIter_++;
        CPEpochSolveTime_ += CPGurobiPro_->solveTime_->dSinceStart().count();
        CPGurobiPro_.reset();
        CPTime_->stop();
    }
}

void MasterAlgorithm::buildBasis(const PInstance &pInst) {
    // 1. Triplet buffer
    std::vector<Triplet> nz;
    std::size_t nnz_estimate = pInst->nbVehicles_ + 3 * pInst->nbRequests_;
    nz.reserve(nnz_estimate);

    int currentCol = 0;
    std::sort(routeSolution_.begin(), routeSolution_.end(),
                  [](const PRoute& a, const PRoute& b) {
                      return a->vehicleID_ < b->vehicleID_;});

    // BLOCK 1: Current solution routes (y_r variables)
    // These go in columns 0 to |routeSolution_|-1
    for (const auto& routeObj : routeSolution_) {
        // Vehicle constraint: exactly one route per vehicle
        nz.emplace_back(routeObj->vehicleID_, currentCol, 1.0);

        // Request constraints: route covers these requests
        for (const auto& requestObj : routeObj->routeRequests_) {
            nz.emplace_back(pInst->nbVehicles_ + requestObj->taskIndex_, currentCol, 1.0);
        }
        currentCol++;
    }

    // BLOCK 2: Unserved request variables (z_i variables)
    // These go in columns |routeSolution_| to |routeSolution_|+|zSolution_|-1
    for (size_t i = 0; i < zSolution_.size(); i++) {
        // Only request constraint: z_i = 1 means request i is unserved
        nz.emplace_back(pInst->nbVehicles_ + zSolution_[i]->taskIndex_, currentCol, 1.0);
        currentCol++;
    }

    // BLOCK 3: Additional routes to complete basis
    int columnsNeeded = pInst->nbVehicles_ + pInst->nbRequests_ - currentCol;

    if (columnsNeeded > 0) {
        pickRoutesRoundRobin(columnsNeeded, pInst);

        std::cout << "Adding " << addedRouteToBase_.size() << " additional routes to complete basis" << std::endl;

        for (const auto& route : addedRouteToBase_) {
            // Vehicle constraint: route uses this vehicle
            nz.emplace_back(route->vehicleID_, currentCol, 1.0);

            // Request constraints: route serves these requests
            for (const auto& requestObj : route->routeRequests_) {
                nz.emplace_back(pInst->nbVehicles_ + requestObj->taskIndex_, currentCol, 1.0);
            }
            currentCol++;
        }

    }

    // Build the square basis matrix
    int nRows = pInst->nbVehicles_ + pInst->nbRequests_;
    Basis_ = SpMat(nRows, nRows);  // Now it's square!
    Basis_.setFromTriplets(nz.begin(), nz.end());
    Basis_.makeCompressed();

    std::cout << "Basis constructed: " << nRows << "x" << nRows
              << " with " << Basis_.nonZeros() << " non-zeros" << std::endl;
}

void MasterAlgorithm::pickRoutesRoundRobin1(int columnsNeeded) {
    addedRouteToBase_.clear();
    std::vector<size_t> idx(availableRoutes_.size(), 0); // index of next candidate route per vehicle

    bool progressed = true;
    while (columnsNeeded > 0 && progressed) {
        progressed = false;
        for (size_t v = 0; v < availableRoutes_.size() && columnsNeeded > 0; ++v) {
            // Keep trying routes for vehicle v until we find one with incompatibilityDegree_ > 0
            while (idx[v] < availableRoutes_[v].size()) {
                auto& route = availableRoutes_[v][idx[v]++];
                if (route->incompatibilityDegree_ > 0) {
                    addedRouteToBase_.push_back(route);
                    --columnsNeeded;
                    progressed = true;
                    break; // done with vehicle v, go to next vehicle
                }
                // else: try next route for the same vehicle
            }
        }
    }
}

void MasterAlgorithm::pickRoutesRoundRobin(int columnsNeeded, const PInstance &pInst) {
    addedRouteToBase_.clear();
    // Collect all incompatible routes by vehicle
    std::vector<std::vector<PRoute>> incompatibleByVehicle(availableRoutes_.size());

    for (size_t v = 0; v < availableRoutes_.size(); ++v) {
        if (!pInst->vehicles_[v]->currentRoute_->routeRequests_.empty()) {
            for (auto& route : availableRoutes_[v]) {
                // incompatibilityDegree_ > 0 means route is NOT in current solution
                // and serves some requests (otherwise incompatibility would be 0)
                if (route->incompatibilityDegree_ > 0 && !route->routeRequests_.empty()) {
                    incompatibleByVehicle[v].push_back(route);
                }
            }


            // Sort by incompatibility degree (descending) for each vehicle
            std::sort(incompatibleByVehicle[v].begin(), incompatibleByVehicle[v].end(),
                      [](const PRoute& a, const PRoute& b) {
                          return a->incompatibilityDegree_ > b->incompatibilityDegree_;
                      });
        }
    }

    // Round-robin selection from sorted lists
    std::vector<size_t> vehicleIdx(availableRoutes_.size(), 0);
    bool progress = true;

    while (columnsNeeded > 0 && progress) {
        progress = false;

        for (size_t v = 0; v < incompatibleByVehicle.size() && columnsNeeded > 0; ++v) {
            if (vehicleIdx[v] < incompatibleByVehicle[v].size()) {
                addedRouteToBase_.push_back(incompatibleByVehicle[v][vehicleIdx[v]]);
                vehicleIdx[v]++;
                columnsNeeded--;
                progress = true;

                std::cout << "Picked route from vehicle " << v
                          << " with incompatibility " << addedRouteToBase_.back()->incompatibilityDegree_
                          << " serving " << addedRouteToBase_.back()->routeRequests_.size() << " requests" << std::endl;
            }
        }
    }

    if (columnsNeeded > 0) {
        std::cerr << "Warning: Could only find " << addedRouteToBase_.size()
                  << " incompatible routes, still need " << columnsNeeded << " more columns" << std::endl;
    }
}
void MasterAlgorithm::computeBasisInverse() {
    const int n = static_cast<int>(Basis_.rows());

    // Factorisation -----------------------------------------------------
    Eigen::SparseLU<SpMat> solver;
    solver.analyzePattern(Basis_);     // symbolic phase
    solver.factorize(Basis_);          // numeric phase
    if (solver.info() != Eigen::Success)
        throw std::runtime_error("LU factorisation failed");

    // Identity matrix as right‑hand side -------------------------------
    SpMat I(n, n);
    std::vector<Triplet> eye;
    eye.reserve(n);
    for (int i = 0; i < n; ++i) eye.emplace_back(i, i, 1.0);
    I.setFromTriplets(eye.begin(), eye.end(), [](double, double){ return 1.0; });

    // Solve B * X = I  =>  X = B^{-1} -----------------------------------
    BasisInv_ = solver.solve(I);
    if (solver.info() != Eigen::Success)
        throw std::runtime_error("Solve for inverse failed");

    BasisInv_.makeCompressed();            // optional
}

void MasterAlgorithm::computePsudoInverse(const PInstance &pInst) {
    int nCols = pInst->nbVehicles_ + static_cast<int>(zSolution_.size());
    Eigen::VectorXd cB(nCols);
    for (int v = 0; v < pInst->nbVehicles_; ++v)
        cB[v] = routeSolution_[v]->totalDelay_;
    for (int i = 0; i < (int)zSolution_.size(); ++i)
        cB[pInst->nbVehicles_ + i] = zSolution_[i]->penalty_;

    // --- π = B * (BᵀB)⁻¹ c_B, all sparse ------------------------------------
    SpMat BtB = Basis_.transpose() * Basis_;              // stays sparse
    Eigen::SimplicialLDLT<SpMat> chol;
    chol.compute(BtB);                                    // symbolic+numeric fact.
    Eigen::VectorXd y = chol.solve(cB);                   // y = (BᵀB)⁻¹ c_B
    Eigen::VectorXd pi = Basis_ * y;                      // π = B y   (sparse*dense)

    // 8. Print π (duals)
    std::cout << "Duals π (one per row in Basis_):\n";
    for (int i = 0; i < pi.size(); ++i)
        std::cout << "  π[" << i << "] = " << pi[i] << '\n';
}

// Updated cost vector construction
void MasterAlgorithm::buildCostVector(const PInstance &pInst) {
    int nRows = pInst->nbVehicles_ + pInst->nbRequests_;
    costVector_.resize(nRows);
    costVector_.setZero();

    int currentCol = 0;

    // Costs for current solution routes
    for (const auto& route : routeSolution_) {
        costVector_(currentCol) = route->totalDelay_;
        currentCol++;
    }

    // Penalty costs for unserved requests
    for (const auto& unservedReq : zSolution_) {
        costVector_(currentCol) = unservedReq->penalty_;
        currentCol++;
    }

    // Costs for additional routes in basis
    for (const auto& route : addedRouteToBase_) {
        costVector_(currentCol) = route->totalDelay_;
        currentCol++;
    }
}

// Validation function to check basis structure
bool MasterAlgorithm::validateBasisStructure(const PInstance &pInst) {
    int nRows = pInst->nbVehicles_ + pInst->nbRequests_;

    // Check if matrix is square
    if (Basis_.rows() != nRows || Basis_.cols() != nRows) {
        std::cerr << "Error: Basis is not square! " << Basis_.rows() << "x" << Basis_.cols() << std::endl;
        return false;
    }

    // Check request constraints (last |P| rows)
    std::vector<int> requestCoverage(pInst->nbRequests_, 0);
    for (int k = 0; k < Basis_.outerSize(); ++k) {
        for (SpMat::InnerIterator it(Basis_, k); it; ++it) {
            int row = it.row();
            if (row >= pInst->nbVehicles_) {
                int reqIdx = row - pInst->nbVehicles_;
                if (std::abs(it.value() - 1.0) < 1e-10) {
                    requestCoverage[reqIdx]++;
                }
            }
        }
    }

    for (int i = 0; i < pInst->nbRequests_; ++i) {
        if (requestCoverage[i] != 1) {
            std::cerr << "Warning: Request " << i << " has coverage " << requestCoverage[i] << std::endl;
        }
    }

    return true;
}


void MasterAlgorithm::buildBasis2(const PInstance &pInst) {
    std::vector<Triplet> nz;
    nz.reserve(5 * (pInst->nbVehicles_ + pInst->nbRequests_));

    int currentCol = 0;
    std::vector<boost::dynamic_bitset<>> usedPatternsPerVehicle[pInst->nbVehicles_]; // Track patterns per vehicle
    std::sort(routeSolution_.begin(), routeSolution_.end(),
                  [](const PRoute& a, const PRoute& b) {
                      return a->vehicleID_ < b->vehicleID_;});
    // BLOCK 1: Current solution routes
    for (const auto& routeObj : routeSolution_) {
        // Vehicle constraint
        nz.emplace_back(routeObj->vehicleID_, currentCol, 1.0);

        // Request constraints
        for (const auto& requestObj : routeObj->routeRequests_) {
            nz.emplace_back(pInst->nbVehicles_ + requestObj->taskIndex_, currentCol, 1.0);
        }

        usedPatternsPerVehicle[routeObj->vehicleID_].push_back(routeObj->column_);
        currentCol++;
    }

    // BLOCK 2: Unserved request variables (z_i)
    for (const auto& zReq : zSolution_) {
        nz.emplace_back(pInst->nbVehicles_ + zReq->taskIndex_, currentCol, 1.0);
        currentCol++;
    }

    // BLOCK 3: Add incompatible routes until basis is complete
    int totalSize = pInst->nbVehicles_ + pInst->nbRequests_;

    addedRouteToBase_.clear();
    std::vector<size_t> idx(availableRoutes_.size(), 0); // index of next candidate route per vehicle

    bool progressed = true;
    int columnsNeeded =  (pInst->nbVehicles_ + pInst->nbRequests_ - currentCol);
    while (columnsNeeded > 0 && progressed) {
        progressed = false;
        for (size_t v = 0; v < availableRoutes_.size() && columnsNeeded > 0; ++v) {
            // Keep trying routes for vehicle v until we find one with incompatibilityDegree_ > 0
            while (idx[v] < availableRoutes_[v].size()) {
                auto& route = availableRoutes_[v][idx[v]++];
                if (route->incompatibilityDegree_ > 0 &&
                    !route->routeRequests_.empty() &&
                    !isDuplicatePatternForVehicle(route->column_, usedPatternsPerVehicle[v])) {
                    addedRouteToBase_.push_back(route);
                    --columnsNeeded;
                    progressed = true;
                    // Add vehicle constraint
                    nz.emplace_back(route->vehicleID_, currentCol, 1.0);

                    // Add request constraints
                    for (const auto& req : route->routeRequests_) {
                        nz.emplace_back(pInst->nbVehicles_ + req->taskIndex_, currentCol, 1.0);
                    }

                    usedPatternsPerVehicle[v].push_back(route->column_);
                    currentCol++;
                    break; // done with vehicle v, go to next vehicle
                }
                // else: try next route for the same vehicle
            }
        }
    }

    // Build matrix
    Basis_ = SpMat(totalSize, totalSize);
    Basis_.setFromTriplets(nz.begin(), nz.end());
    Basis_.makeCompressed();


    std::cout << "Basis built: " << totalSize << "x" << totalSize << std::endl;
    exportBasisToCSV("basis_matrix.csv");
}

bool MasterAlgorithm::isDuplicatePatternForVehicle(const boost::dynamic_bitset<>& pattern,
                                                   const std::vector<boost::dynamic_bitset<>>& usedPatterns) {
    for (const auto& used : usedPatterns) {
        if (pattern == used) return true;
    }
    return false;
}

void MasterAlgorithm::exportBasisToCSV(const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open " << filename << " for writing" << std::endl;
        return;
    }

    std::cout << "Exporting basis matrix to " << filename << "..." << std::endl;

    // Write header (optional - column indices)
    for (int col = 0; col < Basis_.cols(); ++col) {
        file << "Col_" << col;
        if (col < Basis_.cols() - 1) file << ",";
    }
    file << "\n";

    // Write matrix row by row
    for (int row = 0; row < Basis_.rows(); ++row) {
        for (int col = 0; col < Basis_.cols(); ++col) {
            // Get value at (row, col)
            double value = Basis_.coeff(row, col);
            file << value;
            if (col < Basis_.cols() - 1) file << ",";
        }
        file << "\n";
    }

    file.close();
    std::cout << "Matrix exported successfully!" << std::endl;
    std::cout << "Matrix properties:" << std::endl;
    std::cout << "- Size: " << Basis_.rows() << "x" << Basis_.cols() << std::endl;
    std::cout << "- Non-zeros: " << Basis_.nonZeros() << std::endl;
    std::cout << "- Density: " << (100.0 * Basis_.nonZeros()) / (Basis_.rows() * Basis_.cols()) << "%" << std::endl;
}

void MasterAlgorithm::checkCoveredVehicles(PInstance &pInst) {
    for (auto & requestObj: pInst->requests_) {
        requestObj->coveredVehicles_.reset();
        requestObj->coveredVehicles_.resize(nbVehicles_);
    }
    if (pInst->parameters_->LabelingStrategy_ == PULLING) {
        for (auto & requestObj: pInst->requests_) {
            requestObj->coveredVehicles_.flip();
        }
    }
    else {
        for (auto & vehicleObj : pInst->vehicles_) {
            for (auto & routeObj: availableRoutes_[vehicleObj->vehicleID_]) {
                for (auto & requestObj: routeObj->routeRequests_) {
                    requestObj->coveredVehicles_.set(vehicleObj->vehicleID_);
                }
            }
        }
        if (pInst->parameters_->labelingReOptimizeStrategy_ != BY_ROUTE) {
            for (auto & vehicleObj : pInst->vehicles_) {
                for (auto & requestObj: vehicleObj->currentRoute_->routeRequests_) {
                    requestObj->coveredVehicles_.set(vehicleObj->vehicleID_);
                }
            }
        }
    }
}

void MasterAlgorithm::calcDualsStatistics(const PInstance &pInst) {
    try {
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

        std::sort(pi_values.begin(), pi_values.end());
        size_t n = pi_values.size();
        if (n % 2 == 0)
            summaryDuals_.medianDual = 0.5 * (pi_values[n/2 - 1] + pi_values[n/2]);
        else
            summaryDuals_.medianDual = pi_values[n/2];

    } catch (GRBException& e) {
        std::cerr << "Error in getDuals: " << e.getMessage() << std::endl;
        throw;
    }
}

// save initial duals
/*if (pInst->parameters_->solutionMode_ != ANYTIME) {
    (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, RMPCounter_, "initial");
    (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, RMPCounter_, "initial");
}*/


