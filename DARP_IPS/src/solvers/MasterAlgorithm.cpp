//
// Created by Ella on 2021-10-09.
//

#include "MasterAlgorithm.h"
#include "solvers/MIPMasterProblem.h"
#include "solvers/DualAuxSolver.h"
#include "solvers/ComplementPro.h"

//---------------------------------------------------------------------------------------------
//  Reduced Problem class
//  Build and solve the Reduced problem of the ISUD
//---------------------------------------------------------------------------------------------

MasterAlgorithm::MasterAlgorithm(const InputPaths &inputPaths) {
//    ReducedPro_ = std::make_shared<ReducedProblem>();
    CompPro_ = std::make_shared<ComplementPro>();
    ReducedPro_ = std::make_shared<ReducedProblem>();
    MasterPro_ = std::make_shared<MIPMasterProblem>();
    objValue_ = 0;
    previousObj_ =0;
    lpObjValue_ = 0;
    totalWaitTime_ = 0;
    masterTime_ = new myTools::Timer(); masterTime_->init();
    RPTime_ = new myTools::Timer(); RPTime_->init();
    CPTime_ = new myTools::Timer(); CPTime_->init();
    CPBuilt_ = false;
    nbVehicles_ = 0;
    availableTime_ = 0;
    timeLimit_ = 0;
    MPEpochSolveTime_ = 0;
    CPEpochSolveTime_ = 0;
    iterTime_ = 0;

    MPBuildTime_ = new myTools::Timer(); MPBuildTime_->init();
    CPBuildTime_ = new myTools::Timer(); CPBuildTime_->init();

    ZOOMTime_ = new myTools::Timer(); ZOOMTime_->init();
    RMPCounter_ = 0;
    LPIter_ = 0;
    MIPIter_ = 0;
    RPIter_ = 0;
    CPIter_ = 0;
    SPIter_ = 0;
    SPIters_ = 0;
    ZoomIter_ = 0;
    CPSuccess_ = 0;
    CGSuccess_ = 0;
    CPFails_ = 0;
    nbRoutes_ = 0;
    nbColumnsAdded_ = 0;
    nbCoveredTasks_ = 0;
    GreedyObjValue_ = 0;
    maxIncDegree_ = 0;
    minReducedCost_ = INFINITY;
    maxReducedCost_ = INFINITY;
    epochTime_ = 0;
    vehicleChange_ = 0;

    pLogIsudResultsStream_ = new Tools::LogOutput(inputPaths.getOutputEpochResults());
    (*pLogIsudResultsStream_) << "Epoch,MPIter,subProIter,TotalGenColumns,nbColumns,Model,ObjectiveValue,"
                                 "preObjective,RelativeImprove,MPTimeAcc.,SubProTime,IterTime,vehicleChange,AuxObj" << std::endl;

    pLogIterReqDualStream_ = new Tools::LogOutput(inputPaths.getOutputReqDuals());
    (*pLogIterReqDualStream_) << "Epoch,ISUDIter,RequestID,InitialDual,Dual,MinDual,MaxDual,AvgDual,Model,Penalty,diff" << std::endl;

    pLogIterVehDualStream_ = new Tools::LogOutput(inputPaths.getOutputVehDuals());
    (*pLogIterVehDualStream_) << "Epoch,ISUDIter,VehicleID,InitialDual,Dual,Model,diff" << std::endl;
}

MasterAlgorithm::~MasterAlgorithm() {
    delete masterTime_;
    delete RPTime_;
    delete CPTime_;

    delete MPBuildTime_;
    delete CPBuildTime_;

    delete ZOOMTime_;
    pLogIsudResultsStream_->close();
    pLogIterReqDualStream_->close();
    pLogIterVehDualStream_->close();
    delete pLogIsudResultsStream_;
    delete pLogIterReqDualStream_;
    delete pLogIterVehDualStream_;
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
        if (pInst->parameters_->initialDual_ == AUX_box)
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
    MPEpochSolveTime_ = 0;
    CPEpochSolveTime_ = 0;
    masterTime_->start();
    maxIncDegree_ = pInst->parameters_->CP_IncDegree_;
    nbColumnsAdded_ = 0;
    RMPCounter_ = 0;
    SPIter_ = 0;
    CPBuilt_ = false;
    epochTime_ = 0;

    initializeVehicles(pInst);
    // create a feasible integer solution at the start of epoch or simulation
    createInitialSolution(pInst, GreedyModel);

    if (pInst->parameters_->mainAlgorithm_ == MP_ISUD){
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

    }
    else {
        // build the model
        MasterPro_ = std::make_shared<MIPMasterProblem>();
        MasterPro_->routesToAdd_.clear();
        if (pInst->parameters_->initialDual_ == AUX_D)
            DualAuxSolver_ = std::make_shared<DualAuxSolver>(pInst->nbTasks_, nbVehicles_);

        for (auto & vehicleObj : pInst->vehicles_) {
            if (vehicleObj->currentRoute_->routeSize_ != vehicleObj->emptyRoute_->routeSize_)
                MasterPro_->routesToAdd_.push_back(vehicleObj->emptyRoute_);
        }

        MPBuildTime_->start();
        MasterPro_->buildModelMP(pInst, routeSolution_, nbVehicles_);
        MPBuildTime_->stop();
        // set duals based on greedy
        if (pInst->parameters_->initialStart_ == GREEDY_START)
            MasterPro_->solveModelLP(pInst, inputPaths);
    }

    setObjValue();
    previousObj_ = objValue_;
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

void MasterAlgorithm::solveISUD(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime) {
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
                updateRoutesToAdd(CP, pInst);
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
                    (*pLogIsudResultsStream_) << save_MPResults(epoch, "CP", static_cast<int>(CompPro_->IncRoute_.size()),
                                                                masterTime_->dSinceStart().count(), subProTime, 0.0);

                    if (CompPro_->status_ == FRACTIONAL) {
                        CPFails_++;
                        setAvailableTime();
                        if (pInst->parameters_->useZoom_ && availableTime_ > 1) {
                            iterTime_ = masterTime_->dSinceStart().count();
                            ZOOMTime_->start();
                            solveRP_LPINT(pInst, 1, inputPaths);
                            ZoomIter_++;
                            epochTime_ += masterTime_->dSinceStart().count() - iterTime_;
                            iterTime_ = masterTime_->dSinceStart().count();

                            (*pLogIsudResultsStream_)
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

        (*pLogIsudResultsStream_)
                << save_MPResults(epoch, "ISUD", nbRoutes_, masterTime_->dSinceStart().count(), subProTime, 0.0);
        masterTime_->stop();
    }
    catch (const std::exception& e){
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e.what() << std::endl;
    }
}

void MasterAlgorithm::solveISUD_improved(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime) {
    try {
        masterTime_->start();
        float tiLim = availableTime_;
        setObjValue();
        previousObj_ = objValue_;
        bool restartAlgorithm = true;

        // update reduced costs if needed only at the start of epoch (only if we use penalties to create routes)
        if ((pInst->parameters_->initialStart_ == PRE_SOLUTION) && (pInst->parameters_->initialDual_ == PENALTIES) &&
            (RMPCounter_ == 1)) {
            for (auto &requestObj: pInst->requests_)
                requestObj->dual_ = requestObj->InitialDual_;
            for (auto &vehicleObj: pInst->vehicles_)
                vehicleObj->dual_ = vehicleObj->InitialDual_;
        }

        MPBuildTime_->start();
        ReducedPro_->buildModel(pInst, routeSolution_, nbVehicles_);
        MPBuildTime_->stop();

        /*if (pInst->parameters_->solutionMode_ != ANYTIME) {
            (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, RMPCounter_, "initial");
            (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, RMPCounter_, "initial");
        }*/
        while (restartAlgorithm) {

            bool isCPImproved = true;
            restartAlgorithm = false;
            /************************************************************************************************/
            //                                     REDUCED PROBLEM
            /************************************************************************************************/
            RPTime_->start();

            // solve RP with MIP solver
            while (true) {
                CompPro_->fractionalZ_.clear();
                updateReducedCosts(pInst);
                availableTime_ = tiLim - masterTime_->dSinceStart().count();
                if (minReducedCost_ >= 0 || availableTime_ < 0) {
                    break;
                }
                setObjValue();
                solveRP_LPINT(pInst, 0, inputPaths);
                setCurrentRoutes(pInst);
                /*if (pInst->parameters_->solutionMode_ != ANYTIME) {
                    (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, RMPCounter_, "LRP");
                    (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, RMPCounter_, "LRP");
                }*/

                RPIter_++;
                //               std::cout << "RP improve: " << objValue_ << std::endl;
//                if (epoch == 4)
                (*pLogIsudResultsStream_) << save_MPResults(epoch, "RP",
                                                            static_cast<int>(ReducedPro_->compRoutes_.size()),
                                                            masterTime_->dSinceStart().count(), subProTime,
                                                            ReducedPro_->auxObjValue_);
                RMPCounter_++;
                if (previousObj_ > objValue_) {
                    previousObj_ = objValue_;
                } else
                    break;
            }
            RPTime_->stop();

            // update reduced costs
            /************************************************************************************************/
            //                                     COMPLEMENTARY PROBLEM
            /************************************************************************************************/
            int CPCounter = 0;
            updateReducedCosts(pInst);
            // if the value of the objective function improves, the CP is build
            CPTime_->start();
            availableTime_ = tiLim - masterTime_->dSinceStart().count();

            if (minReducedCost_ <= 0 && availableTime_ > 0) {
                CPBuildTime_->start();
                updateIncDegreesBit(pInst);
                CompPro_->routesToAdd_.clear();
                for (auto &vehicleObj: pInst->vehicles_){
                    if (vehicleObj->vehicleIndex_ > -1){
                        for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]) {
                            CompPro_->routesToAdd_.push_back(routeObj);
                        }
                    }
                }
                //               updateRoutesToAdd(false, pInst);

                CompPro_->buildModelCP_improved(pInst, zSolution_, routeSolution_, nbVehicles_, objValue_);
                CPBuildTime_->stop();
            }
            else {
                restartAlgorithm = false;
                isCPImproved = false;
            }

            while (isCPImproved) {
                isCPImproved = false;
                previousObj_ = objValue_;

                availableTime_ = tiLim - masterTime_->dSinceStart().count();
                if (availableTime_ > 0) {
                    CompPro_->solveCP2Model(pInst, zSolution_, routeSolution_, inputPaths);
                    CPIter_++;
                    CPEpochSolveTime_ += CompPro_->solveTime_->dSinceStart().count();
                    setObjValue();
                    (*pLogIsudResultsStream_) << save_MPResults(epoch, "CP", static_cast<int>(CompPro_->IncRoute_.size()),
                                                                masterTime_->dSinceStart().count(), subProTime, 0.0);

                    if (CompPro_->status_ == FRACTIONAL) {
                        CPFails_++;
                        std::cout << "# The Algorithm needs modification to find integer direction" << std::endl;
                        restartAlgorithm = false;
                        break;
                    }
                    else if (CompPro_->status_ == POSITIVE_VALUE) {
                        isCPImproved = false;
                        if (CPCounter == 0) {
                            restartAlgorithm = false;
                            break;
                        }
                    }
                    else if (CompPro_->status_ == INFEASIBLE) {
                        isCPImproved = false;
                        if (CPCounter == 0) {
                            restartAlgorithm = false;
                            break;
                        }
                    }
                    else {
                        setCurrentRoutes(pInst);
                        CPSuccess_++;
                        previousObj_ = objValue_;
                        RMPCounter_++;
                        isCPImproved = true;
                        CPCounter++;
                        updateIncDegreesBit(pInst);
                        CompPro_->objFunction_.setLinearCoef(CompPro_->auxVar_ , -objValue_);
                        for (int r = static_cast<int>(CompPro_->routeIncVar_.getSize()) - 1; r >= 0; --r) {
                            CompPro_->normalConst_.setLinearCoef(CompPro_->routeIncVar_[r], CompPro_->IncRoute_[r]->incompatibilityDegree_);
                        }
                        availableTime_ = tiLim - masterTime_->dSinceStart().count();
                        if (availableTime_ <= 0) {
                            restartAlgorithm = false;
                            break;
                        }
                    }
                } else
                    restartAlgorithm = false;
            }
            availableTime_ = tiLim - masterTime_->dSinceStart().count();
            if ((minReducedCost_ > 0) || (availableTime_ <= 0))
                restartAlgorithm = false;
            for (auto &routeObj: CompPro_->IncRoute_)
                routeObj->cpAdded_ = false;
            CompPro_.reset();
            CompPro_ = std::make_shared<ComplementPro>();

            CPTime_->stop();
        }
        /*for (auto & routeObj: ReducedPro_->compRoutes_)
            routeObj->mpAdded_ = false;*/
        ReducedPro_.reset();
        ReducedPro_ = std::make_shared<ReducedProblem>();
        /*std::cout << "# number of un-served requests: " << zSolution_.size() << std::endl;
        std::cout << "# Time spent on ISUD iteration  = " << masterTime_->dSinceStart().count() << " (seconds)"
                  << std::endl;
        for (auto &requestObj: zSolution_)
            std::cout << "request " << requestObj->getRequestId() << " : " << requestObj->penalty_ << std::endl;*/
        (*pLogIsudResultsStream_)
                << save_MPResults(epoch, "ISUD", nbRoutes_, masterTime_->dSinceStart().count(), subProTime, 0.0);
        masterTime_->stop();
    }
    catch (const std::exception& e){
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e.what() << std::endl;
    }
}

void MasterAlgorithm::solveMP_MIP(PInstance &pInst, int epoch, const InputPaths &inputPaths, float subProTime) {
    masterTime_->start();
    previousObj_ = objValue_;

    // update reduced costs if needed only at the start of epoch if we used penalties to create routes
    if  ((pInst->parameters_->initialStart_ == PRE_SOLUTION)&&(pInst->parameters_->initialDual_ == PENALTIES) && (RMPCounter_ == 1)){
        for(auto & requestObj: pInst->requests_)
            requestObj->dual_ = requestObj->InitialDual_;
        for(auto & vehicleObj : pInst->vehicles_)
            vehicleObj->dual_ = vehicleObj->InitialDual_;
    }

    // save initial duals
    /*if (pInst->parameters_->solutionMode_ != ANYTIME) {
        (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, RMPCounter_, "initial");
        (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, RMPCounter_, "initial");
    }*/

    /************************************************************************************************/
    //                                     MASTER PROBLEM
    /************************************************************************************************/

    // solve RP with MIP solver
    while (true) {
        setAvailableTime();
        if (availableTime_ < 1)
            break;
        solveMP_INT(pInst, inputPaths);
        setCurrentRoutes(pInst);
        MIPIter_++;
        (*pLogIsudResultsStream_) << save_MPResults(epoch, "RMP", static_cast<int>(MasterPro_->compRoutes_.size()),
                                                    masterTime_->dSinceStart().count(), subProTime,
                                                    MasterPro_->auxObjValue_);
        RMPCounter_++;
        setObjValue();
        if (previousObj_ > objValue_) {
            previousObj_ = objValue_;
        }
        else
            break;
    }

    setObjValue();
    (*pLogIsudResultsStream_) << save_MPResults(epoch, "MIP", static_cast<int>(MasterPro_->compRoutes_.size()),
                                                masterTime_->dSinceStart().count(), subProTime,
                                                MasterPro_->auxObjValue_);
    RMPCounter_++;
    masterTime_->stop();
}

// Solve Reduced Problem (RP) and log results
void MasterAlgorithm::solveRP(PInstance &pInst, const InputPaths &inputPaths, int epoch, float subProTime) {
    RPTime_->start();
    CompPro_->fractionalZ_.clear();
    iterTime_ = masterTime_->dSinceStart().count();
    while (true) {
        setAvailableTime();
        if (availableTime_ <= 1)
            break;

        ReducedPro_->routesToAdd_.clear();

        // select compatible routes with negative reduced costs
        updateRoutesToAdd(RP, pInst);
        /*for (auto & vehicleObj : pInst->vehicles_) {
            for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]) {
                Tools::LogOutput routeStream(inputPaths.getOutputIncDegreeRdCost(), true);
                routeStream << routeObj->getRouteId() << "," << epoch << "," << RMPCounter_ << "," << routeObj->vehicleID_ ;
                routeStream << "," << routeObj->totalDelay_ << "," << routeObj->incompatibilityDegree_ ;
                routeStream << "," << routeObj->reducedCost_ << "," << routeObj->lambda_ << "," ;
                routeStream << routeObj->score_ << "," << routeObj->IncScoreRatio_ << "," << routeObj->waitScore_ ;
                routeStream << "," << routeObj->nbCommitted_ << "," << routeObj->createTime_ << "," << routeObj->routeRequests_.size() <<"\n";
                routeStream.close();

            }
        }*/

        if (!ReducedPro_->routesToAdd_.empty()){
            MPBuildTime_->start();
            ReducedPro_->updateRPModel(pInst);
            MPBuildTime_->stop();
            ReducedPro_->solveModelLPInt(pInst, zSolution_, routeSolution_, inputPaths,
                                         availableTime_, objValue_);
            MPEpochSolveTime_ += ReducedPro_->solveTime_->dSinceStart().count();
            setCurrentRoutes(pInst);
            setObjValue();

            RPIter_++;
            epochTime_ += masterTime_->dSinceStart().count() - iterTime_;
            iterTime_ = masterTime_->dSinceStart().count();

            *pLogIsudResultsStream_ << save_MPResults(epoch, "RP", static_cast<int>(ReducedPro_->compRoutes_.size()),
                                                    masterTime_->dSinceStart().count(), subProTime,0.0);
            RMPCounter_++;

            if (previousObj_ - objValue_ > 0.01) {
                previousObj_ = objValue_;
            } else
                break;
            if (RPIter_ == 2)
                 break;
        }
        else
            break;
    }
    RPTime_->stop();
}

void MasterAlgorithm::setCurrentRoutes(const PInstance &pInst) const {
    pInst->resetAssignedVehicles();
    for (auto &routeObj : routeSolution_) {
        pInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
    }
}

void MasterAlgorithm::solveMP_CG(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime) {
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
        if (pInst->parameters_->initialDual_ == AUX_D) {
            DualAuxSolver_->initializeModel(pInst);
            DualAuxSolver_->buildModel(MasterPro_->compRoutes_, pInst->requests_);
            MasterPro_->solveModelIntAux_D(pInst, zSolution_, routeSolution_, inputPaths,
                                     availableTime_, previousObj_, DualAuxSolver_);
            setCurrentRoutes(pInst);
            MIPIter_++;
            MPEpochSolveTime_ += MasterPro_->solveTime_->dSinceStart().count();
            setObjValue();

            epochTime_ += masterTime_->dSinceStart().count() - iterTime_;
            (*pLogIsudResultsStream_) << save_MPResults(epoch, "CG", static_cast<int>(MasterPro_->compRoutes_.size()),
                                                    masterTime_->dSinceStart().count(), subProTime, DualAuxSolver_->objValue_);
        }
        else if (pInst->parameters_->initialDual_ == AUX_P) {
            MasterPro_->solveModelIntAux_P(pInst, zSolution_, routeSolution_, inputPaths,
                                     availableTime_, previousObj_, lpObjValue_);
            setCurrentRoutes(pInst);
            MIPIter_++;
            MPEpochSolveTime_ += MasterPro_->solveTime_->dSinceStart().count();
            setObjValue();

            epochTime_ += (masterTime_->dSinceStart().count() - iterTime_);
            (*pLogIsudResultsStream_) << save_MPResults(epoch, "CG", static_cast<int>(MasterPro_->compRoutes_.size()),
                                                        masterTime_->dSinceStart().count(), subProTime, MasterPro_->auxObjValue_);
        }
        else if (pInst->parameters_->initialDual_ == AUX_box) {
            MasterPro_->solveModelInt_box(pInst, zSolution_, routeSolution_, inputPaths,
                                     availableTime_, previousObj_, lpObjValue_);
            setCurrentRoutes(pInst);
            MIPIter_++;
            MPEpochSolveTime_ += MasterPro_->solveTime_->dSinceStart().count();
            setObjValue();

            epochTime_ += (masterTime_->dSinceStart().count() - iterTime_);
            (*pLogIsudResultsStream_) << save_MPResults(epoch, "CG", static_cast<int>(MasterPro_->compRoutes_.size()),
                                                        masterTime_->dSinceStart().count(), subProTime, MasterPro_->auxObjValue_);
        }
        else if (pInst->parameters_->initialDual_ == LP_CP) {
            MasterPro_->solveModelInt(pInst, zSolution_, routeSolution_, inputPaths,
                                     availableTime_, previousObj_);
            setCurrentRoutes(pInst);
            MIPIter_++;
            MPEpochSolveTime_ += MasterPro_->solveTime_->dSinceStart().count();
            setObjValue();
            epochTime_ += (masterTime_->dSinceStart().count() - iterTime_);
            iterTime_ = masterTime_->dSinceStart().count();

            (*pLogIsudResultsStream_) << save_MPResults(epoch, "MIP", static_cast<int>(MasterPro_->compRoutes_.size()),
                                                        masterTime_->dSinceStart().count(), subProTime, MasterPro_->auxObjValue_);

            setAvailableTime();
            /************************************************************************************************/
            //                                     COMPLEMENTARY PROBLEM
            /************************************************************************************************/
            if (availableTime_ > 3) {
                solveCP(pInst, epoch, inputPaths, subProTime);
            }
        }
        else {
            MasterPro_->solveModelInt(pInst, zSolution_, routeSolution_, inputPaths,
                                     availableTime_, previousObj_);
            setCurrentRoutes(pInst);
            MIPIter_++;
            MPEpochSolveTime_ += MasterPro_->solveTime_->dSinceStart().count();
            setObjValue();

            epochTime_ += (masterTime_->dSinceStart().count() - iterTime_);
            (*pLogIsudResultsStream_) << save_MPResults(epoch, "CG", static_cast<int>(MasterPro_->compRoutes_.size()),
                                                        masterTime_->dSinceStart().count(), subProTime, 0.0);
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
    /*if (!pInst->parameters_->vehiclePortion_){
        if (SPIter_ == 1){
            // Update selectedVehicles_ for comparison
            pInst->firstSelectedVehicles_.assign(pInst->nbVehicles_, 0);
            for (const auto &vehicleObj : pInst->vehicles_) {
                if (!vehicleObj->currentRoute_->routeRequests_.empty()) {
                    pInst->firstSelectedVehicles_[vehicleObj->vehicleID_] = 1;
                }
            }
        }
        else {
            // Prepare a solution vector for the current iteration
            int diffCount = 0;
            for (const auto &vehicleObj : pInst->vehicles_) {
                if (!vehicleObj->currentRoute_->routeRequests_.empty()){
                    if (pInst->firstSelectedVehicles_[vehicleObj->vehicleID_] == 0)
                        diffCount ++;
                }
            }

            // Calculate the percentage change in the number of zeros
            vehicleChange_ = (static_cast<float>(diffCount) / pInst->firstSelectedVehicles_.size()) * 100.0;
        }
    }*/
    masterTime_->stop();
}

void MasterAlgorithm::solveMP_CP(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime) {
    masterTime_->start();
    previousObj_ = objValue_;
    if (SPIter_ == 0)
        lpObjValue_ = objValue_;

    iterTime_ = masterTime_->dSinceStart().count();

    /************************************************************************************************/
    //                                     MASTER PROBLEM
    /************************************************************************************************/
    setAvailableTime();
    if (availableTime_ > 1) {
        solveMP_LP(pInst, inputPaths, epoch, subProTime);
        objValue_ = previousObj_;

        setAvailableTime();
        if (availableTime_ < 3)
            availableTime_ = 3;
        MasterPro_->solveModelInt(pInst, zSolution_, routeSolution_, inputPaths,
                                     availableTime_, previousObj_);
        setCurrentRoutes(pInst);
        MIPIter_++;
        MPEpochSolveTime_ += MasterPro_->solveTime_->dSinceStart().count();
        setObjValue();

        epochTime_ += (masterTime_->dSinceStart().count() - iterTime_);
        iterTime_ = masterTime_->dSinceStart().count();
        (*pLogIsudResultsStream_) << save_MPResults(epoch, "CG", static_cast<int>(MasterPro_->compRoutes_.size()),
                                                         masterTime_->dSinceStart().count(), subProTime, 0.0);

        RMPCounter_++;
        setAvailableTime();
        /************************************************************************************************/
        //                                     COMPLEMENTARY PROBLEM
        /************************************************************************************************/
        if (availableTime_ > 3) {
            solveCP(pInst, epoch, inputPaths, subProTime);
        }
    }
    masterTime_->stop();
}

void MasterAlgorithm::solveCP(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime) {
    /************************************************************************************************/
    //                                     COMPLEMENTARY PROBLEM
    /************************************************************************************************/
    CompPro_->routesToAdd_.clear();
    updateRoutesToAdd(CP, pInst);
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
            (*pLogIsudResultsStream_) << save_MPResults(epoch, "CP", static_cast<int>(CompPro_->IncRoute_.size()),
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

void MasterAlgorithm::solveRLMP(PInstance &pInst, int epoch, const InputPaths &inputPaths, float subProTime) {
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

void MasterAlgorithm::solveRMP(const PInstance &pInst, int epoch, const InputPaths &inputPaths, float subProTime) {
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
        MasterPro_->solveModelInt(pInst, zSolution_, routeSolution_, inputPaths, availableTime_, previousObj_);
        setCurrentRoutes(pInst);
        MIPIter_++;
        MPEpochSolveTime_ += MasterPro_->solveTime_->dSinceStart().count();
        setObjValue();
        epochTime_ += masterTime_->dSinceStart().count();

        (*pLogIsudResultsStream_) << save_MPResults(epoch, "CG", static_cast<int>(MasterPro_->compRoutes_.size()),
                                                    masterTime_->dSinceStart().count(), subProTime, 0.0);
        RMPCounter_++;
    }

    masterTime_->stop();
}
void MasterAlgorithm::solveRP_LPINT(PInstance &pInst, int compDegree, const InputPaths &inputPaths) {
    // improve by solving the Reduced problem
    ReducedPro_->routesToAdd_.clear();
    if (compDegree == 0) {
        // select compatible routes with negative reduced costs
        updateRoutesToAdd(RP, pInst);
    }
    else
        updateRoutesToAddZoom();

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
}

void MasterAlgorithm::solveMP_LP(PInstance &pInst, const InputPaths &inputPaths, int epoch, float subProTime) {

    // solve RP with MIP solver
    while (true) {
        setAvailableTime();
        if (availableTime_ <= 1)
            break;

        MasterPro_->routesToAdd_.clear();

        // select routes with negative reduced costs
        updateRoutesToAdd(NR, pInst);

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
    *pLogIsudResultsStream_ << save_MPResults(epoch, "LMP", static_cast<int>(MasterPro_->compRoutes_.size()),
                                                    masterTime_->dSinceStart().count(), subProTime, 0.0);
}


void MasterAlgorithm::solveMP_INT(PInstance &pInst, const InputPaths &inputPaths) {
    MasterPro_->routesToAdd_.clear();

    // select routes with negative reduced costs
    updateRoutesToAdd(NR, pInst);

    if (!MasterPro_->routesToAdd_.empty()){

        MPBuildTime_->start();
        MasterPro_->updateModel(pInst);
        MPBuildTime_->stop();
        MasterPro_->solveModelIntAux_P(pInst, zSolution_, routeSolution_, inputPaths,
                                     availableTime_, objValue_, lpObjValue_);
        MPEpochSolveTime_ += MasterPro_->solveTime_->dSinceStart().count();
        objValue_ = MasterPro_->objValue_;
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
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON MP IMPROVEMENT" << " = " << masterTime_->dSinceInit().count() << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON RP IMPROVEMENT" << " = " << RPTime_->dSinceInit().count() << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON CP IMPROVEMENT" << " = " << CPTime_->dSinceInit().count() << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON ZOOM ISUD" << " = " << ZOOMTime_->dSinceInit().count() << " (s)" << std::endl;

    return repStr.str();
}

std::string MasterAlgorithm::toStringTimersAvg(int epoch) const {
    std::stringstream repStr;
    repStr << std::left << std::fixed << std::setprecision(2);
    repStr << "#" << std::endl;
    repStr << "# -------------   AVERAGE MP RUN TIMES PER EPOCH   -------------" << std::endl;
    repStr << "#" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON MP IMPROVEMENT" << " = " << masterTime_->dSinceInit().count() / static_cast<float>(epoch) << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON RP IMPROVEMENT" << " = " << RPTime_->dSinceInit().count()/static_cast<float>(epoch) << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON CP IMPROVEMENT" << " = " << CPTime_->dSinceInit().count()/static_cast<float>(epoch) << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON ZOOM ISUD" << " = " << ZOOMTime_->dSinceInit().count() / static_cast<float>(epoch) << " (s)" << std::endl;

    return repStr.str();
}

void MasterAlgorithm::updateRoutesToAdd(selectionMode selectMode, const PInstance &pInst){
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
                            CompPro_->routesToAdd_.push_back(routeObj);
                            numAdded++;
                        }
                        break;
                    case RP:
                        if (!routeObj->mpAdded_ && routeObj->isCompatible_ && routeObj->reducedCost_ < 0) {
                            ReducedPro_->routesToAdd_.push_back(routeObj);
                            numAdded++;
                        }
                        break;
                    default: // CG and MIP:
                        if (!routeObj->mpAdded_ && routeObj->reducedCost_ < 0) {
                            MasterPro_->routesToAdd_.push_back(routeObj);
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

void MasterAlgorithm::updateRoutesToAddZoom() const {
    // add fractional routes
    for (auto & routeObj: CompPro_->fractionalRoutes_) {
        if (!routeObj->mpAdded_)
            ReducedPro_->routesToAdd_.push_back(routeObj);
    }

    for (auto & routeObj : CompPro_->IncRoute_) {
        if (!routeObj->mpAdded_ && routeObj->reducedCost_ <= 0)
            ReducedPro_->routesToAdd_.push_back(routeObj);
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
}


// save initial duals
/*if (pInst->parameters_->solutionMode_ != ANYTIME) {
    (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, RMPCounter_, "initial");
    (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, RMPCounter_, "initial");
}*/