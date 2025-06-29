//
// Created by Ella on 2021-10-09.
//

#include "MasterAlgorithm.h"

#include "solver.h"
#include "../GurobiSolver/MP_Gurobi.h"
#include "../CplexSolver/MIPMasterProblem.h"
#include "../CplexSolver/DualAuxSolver.h"
#include "../CplexSolver/ComplementPro.h"

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
    (*pLogIterVehDualStream_) << "Epoch,MPIter,VehicleID,InitialDual,Dual,Model,diff" << std::endl;
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

void MasterAlgorithm::createInitialSolution(PInstance &pInst, const PGreedyModeler &GreedyModel){
    if (pInst->parameters_->initialStart_ == EMPTY_ROUTES){
        for (auto &requestObj : pInst->requests_) {
            zSolution_.push_back(requestObj);
            requestObj->dual_ = requestObj->penalty_;
            requestObj->InitialDual_ = requestObj->penalty_;
        }
        routeSolution_.clear();
        for (auto &vehicleObj: pInst->vehicles_) {
            routeSolution_.push_back(vehicleObj->emptyRoute_);
            vehicleObj->setCurrentRoute(vehicleObj->emptyRoute_);
        }
        for (auto &vehicleObj: pInst->vehicles_)
            vehicleObj->dual_ = 0;
        setObjValue();
    }
    else if (pInst->parameters_->initialStart_ == PRE_SOLUTION){
        if (routeSolution_.empty()){
            for (auto &vehicleObj: pInst->vehicles_) {
                routeSolution_.push_back(vehicleObj->currentRoute_);
            }
        }
        float box = 1;
        if (pInst->parameters_->initialDual_ == AUX_BOX)
            box = 0.8;
        for (int i = pInst->nbRequests_ - pInst->nbNewRequests_; i < pInst->nbRequests_; ++i){
            if (pInst->requests_[i]->solVehicleID_ == LARGE_CONSTANT) {
                zSolution_.push_back(pInst->requests_[i]);
                pInst->requests_[i]->dual_ = box * pInst->requests_[i]->penalty_;
                pInst->requests_[i]->InitialDual_ = box * pInst->requests_[i]->penalty_;
            }
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
    else if (pInst->parameters_->initialStart_ == IP_SOLUTION){
        if (routeSolution_.empty()){
            for (auto &vehicleObj: pInst->vehicles_) {
                routeSolution_.push_back(vehicleObj->currentRoute_);
                if (!vehicleObj->currentRoute_->routeRequests_.empty()) {
                    for (int i =0; i < vehicleObj->currentRoute_->routeNodes_.size(); ++i){
                        if (vehicleObj->currentRoute_->routeNodes_[i]->type_ == PICKUP) {
                            vehicleObj->currentRoute_->routeNodes_[i]->related_Request_->dual_ =
                                    vehicleObj->currentRoute_->plannedReachTime_[i] -
                                    vehicleObj->currentRoute_->routeNodes_[i]->initialReadyTime_;
                            vehicleObj->currentRoute_->routeNodes_[i]->related_Request_->InitialDual_ = vehicleObj->currentRoute_->routeNodes_[i]->related_Request_->dual_;
                        }
                    }
                }
                vehicleObj->dual_ = 0;
                vehicleObj->InitialDual_ = 0;
            }
        }
        for (int i = pInst->nbRequests_ - pInst->nbNewRequests_; i < pInst->nbRequests_; ++i){
            if (pInst->requests_[i]->solVehicleID_ == LARGE_CONSTANT) {
                zSolution_.push_back(pInst->requests_[i]);
                pInst->requests_[i]->dual_ = pInst->requests_[i]->penalty_;
                pInst->requests_[i]->InitialDual_ = pInst->requests_[i]->penalty_;
            }
        }
        setObjValue();
    }
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

// function to calculate incompatibility degree of a route
void MasterAlgorithm::calcIncompatibilityBit(const PRoute &route, const PInstance &pInst) {
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

// this function updates the incompatibility degree of availableRoutes
void MasterAlgorithm::updateIncDegreesBit(const PInstance &pInst) const {
    for (auto & vehicleObj : pInst->vehicles_) {
        if (!availableRoutes_[vehicleObj->vehicleID_].empty()) {
            for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]){
                calcIncompatibilityBit(routeObj, pInst);
            }
        }
    }
}

void MasterAlgorithm::calcIncompatibilityM(const PRoute &route) const {
    route->isCompatible_ = true;
    route->incompatibilityDegree_ = 0;
    if (!route->routeRequests_.empty()) {
        for (auto & e : adjacencyPairs_) {
            route->incompatibilityDegree_ += route->column_.test(e.first) ^ route->column_.test(e.second);
        }
    }

    if (route->incompatibilityDegree_ > 0) {
        route->isCompatible_ = false;
//        route->IncScore_ = route->reducedCost_/ route->incompatibilityDegree_;
    }
//    else {
//        route->IncScore_ = route->reducedCost_;
//    }
}

void MasterAlgorithm::calcIncompatibilityMFull(const PRoute &route) const {
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

void MasterAlgorithm::updateIncDegreesM(const PInstance &pInst) {
    adjacencyPairs_.clear();
    vehiclePairs_.clear();
    for (auto & vehicleObj : pInst->vehicles_) {
        if (!vehicleObj->currentRoute_->routeRequests_.empty()) {
            vehiclePairs_.emplace_back(vehicleObj->vehicleID_, vehicleObj->currentRoute_->routeRequests_[0]->taskIndex_);
            if (vehicleObj->currentRoute_->routeRequests_.size() > 1) {
                for (size_t i=0; i + 1 < vehicleObj->currentRoute_->routeRequests_.size(); ++i) {
                    adjacencyPairs_.emplace_back(vehicleObj->currentRoute_->routeRequests_[i]->taskIndex_, vehicleObj->currentRoute_->routeRequests_[i + 1]->taskIndex_);
                }
            }
        }
    }
    for (auto & vehicleObj : pInst->vehicles_) {
        if (!availableRoutes_[vehicleObj->vehicleID_].empty()) {
            for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]){
                calcIncompatibilityMFull(routeObj);
            }
        }
    }
}

void MasterAlgorithm::updateScore1(const PInstance &pInst) {
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

void MasterAlgorithm::updateScore(const PInstance &pInst) {
    for (auto & vehicleObj : pInst->vehicles_) {
        if (!availableRoutes_[vehicleObj->vehicleID_].empty()) {
            for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]){
                routeObj->IncScoreRatio_ = 0.0;
                float m_j = static_cast<float>(routeObj->routeRequests_.size()) + 1.0f;
                if (m_j == 0.0) continue;
                boost::dynamic_bitset<> vehicles(pInst->nbVehicles_);
                for (auto & requestObj: routeObj->routeRequests_) {
                    if (requestObj->solVehicleID_ == LARGE_CONSTANT)
                        routeObj->IncScoreRatio_ += 1 / m_j;
                    else if (!vehicles.test(requestObj->solVehicleID_)){
                        vehicles.set(requestObj->solVehicleID_, true);
                        float m_l = static_cast<float>(pInst->vehicles_[requestObj->solVehicleID_]->currentRoute_->routeRequests_.size()) + 1.0f;
                        if (m_l == 0.0) continue;
                        float overlap = 0.0f;
                        for (const auto& req : routeObj->routeRequests_) {
                            if (pInst->vehicles_[requestObj->solVehicleID_]->currentRoute_->column_.test(req->taskIndex_)) {
                                overlap += 1.0f;
                            }
                        }
                        if (pInst->vehicles_[requestObj->solVehicleID_]->currentRoute_->vehicleID_ == routeObj->vehicleID_)
                            overlap += 1.0f;
                        routeObj->IncScoreRatio_ += (overlap * overlap) / (m_j * m_l);
                    }
                }
                if (!vehicles.test(routeObj->vehicleID_)){
                    float m_l = static_cast<float>(pInst->vehicles_[routeObj->vehicleID_]->currentRoute_->routeRequests_.size()) + 1.0f;
                    if (m_l == 0.0) continue;
                    float overlap = 0.0f;
                    for (const auto& req : routeObj->routeRequests_) {
                        if (pInst->vehicles_[routeObj->vehicleID_]->currentRoute_->column_.test(req->taskIndex_)) {
                            overlap += 1.0f;
                        }
                    }
                    if (pInst->vehicles_[routeObj->vehicleID_]->currentRoute_->vehicleID_ == routeObj->vehicleID_)
                        overlap += 1.0f;
                    routeObj->IncScoreRatio_ += (overlap * overlap) / (m_j * m_l);
                }
     //           std::cout << "score: " << routeObj->IncScore_ << " - " << routeObj->incompatibilityDegree_ << std::endl;
 //               routeObj->IncScoreRatio_ *= routeObj->reducedCost_;
            }
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
                for (int i = 0; i < routeObj->routeRequests_.size(); ++i) {
                    if (routeObj->routeRequests_[i]->plannedDelay_ != LARGE_CONSTANT)
                        routeObj->waitScore_ += routeObj->plannedDelay_[i] - routeObj->routeRequests_[i]->plannedDelay_;
                }
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

void MasterAlgorithm::updateRoutesToAdd(SelectionMode selectMode, const PInstance &pInst, std::vector<PRoute> &routesToAdd){
    updateReducedCosts(pInst);
    if (selectMode == CP || selectMode == RP) {
        updateIncDegreesM(pInst);
        if (pInst->parameters_->sortColumn_ == COMP_C) {
            updateScore(pInst);
        }
    }
    if (minReducedCost_ <= 0) {
        for (auto & vehicleObj : pInst->vehicles_) {
            if (pInst->parameters_->sortColumn_ == NORMAL_RC)
                std::stable_sort(availableRoutes_[vehicleObj->vehicleID_].begin(),
                                availableRoutes_[vehicleObj->vehicleID_].end(),
                                [](const PRoute &lhs, const PRoute &rhs) { return lhs->score_ < rhs->score_; });

            else if (pInst->parameters_->sortColumn_ == LAMBDA_S)
                    std::stable_sort(availableRoutes_[vehicleObj->vehicleID_].begin(),
                                 availableRoutes_[vehicleObj->vehicleID_].end(),
                                 [](const PRoute &lhs, const PRoute &rhs) { return lhs->lambda_ < rhs->lambda_; });

            else if (pInst->parameters_->sortColumn_ == COMP_C || selectMode == CP)
                    std::stable_sort(availableRoutes_[vehicleObj->vehicleID_].begin(),
                                 availableRoutes_[vehicleObj->vehicleID_].end(),
                                 [](const PRoute &lhs, const PRoute &rhs) { return lhs->IncScore_ < rhs->IncScore_; });
            else
                std::stable_sort(availableRoutes_[vehicleObj->vehicleID_].begin(),
                                 availableRoutes_[vehicleObj->vehicleID_].end(),
                                 [](const PRoute &lhs, const PRoute &rhs) { return lhs->reducedCost_ < rhs->reducedCost_; });

            int numAdded = 0;
            for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]) {
                switch(selectMode){
                    case CP:
                        if (!routeObj->cpAdded_ && routeObj->incompatibilityDegree_ > 0 && routeObj->reducedCost_ < maxReducedCost_) {
                            routesToAdd.push_back(routeObj);
                            numAdded++;
                        }
                        break;
                    case RP:
                        if (!routeObj->mpAdded_ && routeObj->isCompatible_ && routeObj->reducedCost_ < 0) {
                            routesToAdd.push_back(routeObj);
                            numAdded++;
                        }
                        break;
                    default: // CG and MIP:
                        if (!routeObj->mpAdded_  && routeObj->reducedCost_ < 0) {
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

void MasterAlgorithm::setCurrentRoutes(const PInstance &pInst) const {
    pInst->resetAssignedVehicles();
    for (auto &routeObj : routeSolution_) {
        pInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
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
           << totalWaitTime_ << ",";
    return repStr.str();
}

void MasterAlgorithm::createFinalOutputString(const PInstance &pInst, float subproblemTime, float greedyRuntime) {
    pInst->instRepStr_ << LPIter_ << "," << MIPIter_ << ",";
    pInst->instRepStr_ << RPIter_ << "," << CPIter_ << ",";
    pInst->instRepStr_ << ZoomIter_ << ",";
    pInst->instRepStr_ << SPIters_ << ",";
    pInst->instRepStr_ << masterTime_->dSinceInit().count() << ",";
    pInst->instRepStr_ << RPTime_->dSinceInit().count() << ",";
    pInst->instRepStr_<< CPTime_->dSinceInit().count() << ",";
    pInst->instRepStr_ << ZOOMTime_->dSinceInit().count() << ",";
    pInst->instRepStr_ << subproblemTime << "," << greedyRuntime << ",";
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


// save initial duals
/*if (pInst->parameters_->solutionMode_ != ANYTIME) {
    (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, RMPCounter_, "initial");
    (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, RMPCounter_, "initial");
}*/