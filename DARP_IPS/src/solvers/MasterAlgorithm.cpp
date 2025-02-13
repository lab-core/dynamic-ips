//
// Created by Ella on 2021-10-09.
//

#include "MasterAlgorithm.h"

//---------------------------------------------------------------------------------------------
//  Reduced Problem class
//  Build and solve the Reduced problem of the ISUD
//---------------------------------------------------------------------------------------------

MasterAlgorithm::MasterAlgorithm(InputPaths &inputPaths) {
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
    cpIncDegree_ = 2;
    GreedyObjValue_ = 0;
    maxIncDegree_ = 0;
    minReducedCost_ = INFINITY;
    maxReducedCost_ = INFINITY;
    epochTime_ = 0;
    vehicleChange_ = 0;

    pLogIsudResultsStream_ = new Tools::LogOutput(inputPaths.getOutputEpochResults());
    (*pLogIsudResultsStream_) << "Epoch,MPIter,subProIter,TotalGenColumns,nbColumns,Model,ObjectiveValue,"
                                 "preObjective,RelativeImprove,MPTimeAcc.,SubProTime,IterTime,vehicleChange,AuxObj" << std::endl;

    /*pLogIterReqDualStream_ = new Tools::LogOutput(inputPaths.getOutputReqDuals());
    (*pLogIterReqDualStream_) << "Epoch,ISUDIter,RequestID,Dual,Model,Penalty," << std::endl;

    pLogIterVehDualStream_ = new Tools::LogOutput(inputPaths.getOutputVehDuals());
    (*pLogIterVehDualStream_) << "Epoch,ISUDIter,VehicleID, Dual, Model" << std::endl;*/
}

MasterAlgorithm::~MasterAlgorithm() {
    delete masterTime_;
    delete RPTime_;
    delete CPTime_;

    delete MPBuildTime_;
    delete CPBuildTime_;

    delete ZOOMTime_;
    pLogIsudResultsStream_->close();
    /*pLogIterReqDualStream_->close();
    pLogIterVehDualStream_->close();*/
    delete pLogIsudResultsStream_;
    /*delete pLogIterReqDualStream_;
    delete pLogIterVehDualStream_;*/
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

void MasterAlgorithm::initializeVehicles(PInstance &pInst){
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

void MasterAlgorithm::createInitialSolution(PInstance &pInst, PGreedyModeler &GreedyModel){
    if (pInst->parameters_->initialStart_ == EMPTY_ROUTES){
        for (auto &requestObj : pInst->requests_) {
            zSolution_.push_back(requestObj);
            requestObj->dual_ = requestObj->penalty_;
            requestObj->InitialDual_ = requestObj->penalty_;
        }
        routeSolution_.clear();
        for (auto &vehicleObj: pInst->vehicles_) {
            routeSolution_.push_back(vehicleObj->emptyRoute_);
            vehicleObj->currentRoute_ = vehicleObj->emptyRoute_;
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
        for (int i = pInst->nbRequests_ - pInst->nbNewRequests_; i < pInst->nbRequests_; ++i){
            if (pInst->requests_[i]->solVehicleID_ == LARGE_CONSTANT) {
                zSolution_.push_back(pInst->requests_[i]);
                pInst->requests_[i]->dual_ = pInst->requests_[i]->penalty_;
                pInst->requests_[i]->InitialDual_ = pInst->requests_[i]->penalty_;
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
                if (vehicleObj->currentRoute_->routeRequests_.size() > 0) {
                    for (int i =0; i < vehicleObj->currentRoute_->routeNodes_.size(); ++i){
                        if (vehicleObj->currentRoute_->routeNodes_[i]->type_ == PICKUP) {
                            vehicleObj->currentRoute_->routeNodes_[i]->related_Request_->dual_ =
                                    vehicleObj->currentRoute_->plannedReachTime_[i] -
                                    vehicleObj->currentRoute_->routeNodes_[i]->readyTime_;
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

// this function create initial routes serving only one request and fill zSolution_ with available requests
// Reduced problem is also solved to initialized dual costs
void MasterAlgorithm::initialization(PInstance &pInst, InputPaths &inputPaths, PGreedyModeler &GreedyModel) {
    MPEpochSolveTime_ = 0;
    CPEpochSolveTime_ = 0;
    masterTime_->start();
    cpIncDegree_ = 2;
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
void MasterAlgorithm::calcIncompatibilityBit(PRoute &route, PInstance &pInst) {
    route->isCompatible_ = true;
    route->incompatibilityDegree_ = 0;

    // If this column does not cover the requests of the current route in related vehicle
    if ((route->column_ & pInst->vehicles_[route->vehicleID_]->currentRoute_->column_)!=pInst->vehicles_[route->vehicleID_]->currentRoute_->column_){
        route->isCompatible_ = false;
        route->incompatibilityDegree_++;
    }
    std::bitset<MAX_BIT_SIZE> vehicles;
    for (auto & requestObj : route->routeRequests_){
        if (requestObj->solVehicleID_ < LARGE_CONSTANT)
            vehicles.set(requestObj->solVehicleID_, true);
    }
    vehicles.set(route->vehicleID_, false);

    // Count incompatibility if there are any incompatible vehicles
    if (vehicles.count() > 0) {
        route->isCompatible_ = false;
        route->incompatibilityDegree_ += vehicles.count();
    }
}

// this function update the incompatibility degree of availableRoutes and
void MasterAlgorithm::updateIncDegreesBit(PInstance &pInst) {
    for (auto & vehicleObj : pInst->vehicles_) {
        if (!availableRoutes_[vehicleObj->vehicleID_].empty()) {
            for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]){
                calcIncompatibilityBit(routeObj, pInst);
            }
        }
    }
}


void MasterAlgorithm::updateReducedCosts(PInstance &pInst) {
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
            if (minReducedCost_ > routeObj->reducedCost_ && (!routeObj->mpAdded_ && !routeObj->cpAdded_))
                minReducedCost_ = routeObj->reducedCost_;
            if (!routeObj->routeRequests_.empty()) {
                routeObj->score_ = routeObj->reducedCost_ / routeObj->routeRequests_.size();
                routeObj->lambda_ = routeObj->totalDelay_ / (routeObj->totalDelay_ - routeObj->reducedCost_ + 0.0001);
            }
            else {
                routeObj->score_ = 0;
                routeObj->lambda_ = 0;
            }
        }
        vehicleObj->bestReducedCost_ = minReducedCost_;
    }
    if (minReducedCost_ < 0)
        maxReducedCost_ = ((-0.5)*minReducedCost_);
}

void MasterAlgorithm::solveISUD(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime) {
    try {
        masterTime_->start();
        setObjValue();
        previousObj_ = objValue_;
        bool restartAlgorithm = true;
        bool isCPImproved;
        int CPCounter;
        float iterTime;

        /*if (pInst->parameters_->solutionMode_ != ANYTIME) {
            (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, RMPCounter_, "initial");
            (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, RMPCounter_, "initial");
        }*/
        while (restartAlgorithm) {
            isCPImproved = true;
            restartAlgorithm = true;
            /************************************************************************************************/
            //      SOLVE REDUCED PROBLEM
            /************************************************************************************************/
            solveRP(pInst, inputPaths, epoch, subProTime);

            /************************************************************************************************/
            //                                     COMPLEMENTARY PROBLEM
            /************************************************************************************************/
            CPCounter = 0;
            // if objective improves, the CP is build
            CPTime_->start();
            setAvailableTime();

            if (minReducedCost_ <= 0 && availableTime_ > 0) {
                CPBuildTime_->start();
                CompPro_->routesToAdd_.clear();
                // the model is built only in first iteration of ISUD then just the solution is updated and
                // incompatible columns remains the same
                if (!CPBuilt_)
                    CompPro_->buildModel(pInst, zSolution_, routeSolution_, nbVehicles_);
                else{
                    // just make sure that compatible columns has negative coefficient
                    CompPro_->repairModel(pInst, zSolution_, routeSolution_, nbVehicles_);
                }
                CPBuildTime_->stop();
                CPBuilt_ = true;
            }
            else {
                restartAlgorithm = false;
                isCPImproved = false;
            }
            iterTime = masterTime_->dSinceStart().count();
            while (isCPImproved) {
                isCPImproved = false;
                previousObj_ = objValue_;
                updateReducedCosts(pInst);
                updateIncDegreesBit(pInst);
                CompPro_->routesToAdd_.clear();
                updateRoutesToAdd(CP, pInst);
                setAvailableTime();
                if (!CompPro_->routesToAdd_.empty() && availableTime_ > 1) {
                    // add empty routes
                    for (int v = 0; v < pInst->nbVehicles_; ++v) {
                        if (pInst->vehicles_[v]->vehicleIndex_>-1 && !pInst->vehicles_[v]->emptyRoute_->cpAdded_ &&
                            !pInst->vehicles_[v]->currentRoute_->routeRequests_.empty())
                            CompPro_->routesToAdd_.push_back(pInst->vehicles_[v]->emptyRoute_);
                    }
                    CPBuildTime_->start();
                    CompPro_->updateModel(pInst, zSolution_, routeSolution_);
                    CPBuildTime_->stop();
                    CompPro_->solveCPModel(pInst, zSolution_, routeSolution_, inputPaths);
                    CPIter_++;
                    // save duals
                    /*if (pInst->parameters_->solutionMode_ != ANYTIME) {
                        (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, RMPCounter_, "CP");
                        (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, RMPCounter_, "CP");
                    }*/

                    CPEpochSolveTime_ += CompPro_->solveTime_->dSinceStart().count();
                    setObjValue();
                    epochTime_ += (masterTime_->dSinceStart().count() - iterTime);
                    iterTime = masterTime_->dSinceStart().count();
                    (*pLogIsudResultsStream_) << save_MPResults(epoch, "CP", (int) (CompPro_->IncRoute_.size()),
                                                                masterTime_->dSinceStart().count(), subProTime, 0.0);

                    if (CompPro_->status_ == FRACTIONAL) {
                        CPFails_++;
                        setAvailableTime();
                        if (pInst->parameters_->useZoom_ && availableTime_ > 1) {
                            ZOOMTime_->start();
                            solveRP_LPINT(pInst, 1, inputPaths);
                            ZoomIter_++;
                            (*pLogIsudResultsStream_)
                                    << save_MPResults(epoch, "ZOOM",
                                                      (int) ReducedPro_->compRoutes_.size() - nbVehicles_,
                                                      masterTime_->dSinceStart().count(), subProTime,
                                                      ReducedPro_->auxObjValue_);
                            RMPCounter_++;

                            /*if (pInst->parameters_->solutionMode_ != ANYTIME) {
                                (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, RMPCounter_, "zoom");
                                (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, RMPCounter_, "zoom");
                            }*/
                            ZOOMTime_->stop();
                            if (previousObj_ > objValue_) {
                                previousObj_ = objValue_;
                            }
                            else {
                          //      restartAlgorithm = false;
                                break;
                            }
                        }

                        else {
                            restartAlgorithm = false;
                            break;
                        }
                    }
                    else if (CompPro_->status_ == POSITIVE_VALUE) {
                        restartAlgorithm = false;
                        break;
                    }
                    else if (CompPro_->status_ == INFEASIBLE) {
                        restartAlgorithm = false;
                        break;
                    }
                    else {
                        setCurrentRoutes(pInst);
                        CPSuccess_++;
                        previousObj_ = objValue_;
                        RMPCounter_++;
                        isCPImproved = true;
                        CPCounter++;
                        setAvailableTime();
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
            if ((minReducedCost_ > 0) || (availableTime_ <= 0))
                restartAlgorithm = false;
            for (auto &routeObj: CompPro_->IncRoute_)
                routeObj->cpAdded_ = false;
            CompPro_.reset();
            CompPro_ = std::make_shared<ComplementPro>();
            CPBuilt_ = false;
            CPTime_->stop();
        }
        /*for (auto & routeObj: ReducedPro_->compRoutes_)
            routeObj->mpAdded_ = false;
        ReducedPro_.reset();
        ReducedPro_ = std::make_shared<ZoomReducedProblem>();*/
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
void MasterAlgorithm::solveISUD_improved(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime) {
    try {
        masterTime_->start();
        float tiLim = availableTime_;
        setObjValue();
        previousObj_ = objValue_;
        bool restartAlgorithm = true;
        bool isCPImproved;
        int CPCounter;

        // update reduced costs if needed only at the start of epoch, if we used penalties to create routes
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

            isCPImproved = true;
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
                                                            (int) ReducedPro_->compRoutes_.size() - nbVehicles_,
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
            CPCounter = 0;
            updateReducedCosts(pInst);
            // if objective improves, the CP is build
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
                    /*if (pInst->parameters_->solutionMode_ != ANYTIME) {
                        (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, RMPCounter_, "CP");
                        (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, RMPCounter_, "CP");
                    }*/
                    CPIter_++;
                    CPEpochSolveTime_ += CompPro_->solveTime_->dSinceStart().count();
                    setObjValue();
                    (*pLogIsudResultsStream_) << save_MPResults(epoch, "CP", (int) (CompPro_->IncRoute_.size()),
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
                        for (int r = (int) CompPro_->routeIncVar_.getSize() - 1; r >= 0; --r) {
                            CompPro_->normalConst_[0].setLinearCoef(CompPro_->routeIncVar_[r], CompPro_->IncRoute_[r]->incompatibilityDegree_);
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

void MasterAlgorithm::solveMP_MIP(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime) {
    masterTime_->start();
    previousObj_ = objValue_;

    // update reduced costs if needed only at the start of epoch, if we used penalties to create routes
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
        updateReducedCosts(pInst);
        if (minReducedCost_ >= 0) {
            break;
        }
        setAvailableTime();
        if (availableTime_ < 1)
            break;
        solveMP_INT(pInst, inputPaths);
        MIPIter_++;
        (*pLogIsudResultsStream_) << save_MPResults(epoch, "RMP", (int) MasterPro_->compRoutes_.size() - nbVehicles_,
                                                    masterTime_->dSinceStart().count(), subProTime,
                                                    MasterPro_->auxObjValue_);

        RMPCounter_++;
        // save duals
        /*if (pInst->parameters_->solutionMode_ != ANYTIME) {
            (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, RMPCounter_, "RMP");
            (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, RMPCounter_, "RMP");
        }*/
        setObjValue();
        if (previousObj_ > objValue_) {
            previousObj_ = objValue_;
        }
        else
            break;
    }

    setObjValue();
    (*pLogIsudResultsStream_) << save_MPResults(epoch, "MIP", (int) MasterPro_->compRoutes_.size() - nbVehicles_,
                                                masterTime_->dSinceStart().count(), subProTime,
                                                MasterPro_->auxObjValue_);
    RMPCounter_++;


    /*std::cout << "# number of un-served requests: " << zSolution_.size() << std::endl;
    std::cout << "# Time spent on ISUD iteration  = " << masterTime_->dSinceStart().count() << " (seconds)"
              << std::endl;
    for (auto &requestObj: zSolution_)
        std::cout << "request " << requestObj->getRequestId() << " : " << requestObj->penalty_ << std::endl;*/

    setCurrentRoutes(pInst);

    masterTime_->stop();
}

// Solve Reduced Problem (RP) and log results
void MasterAlgorithm::solveRP(PInstance &pInst, InputPaths &inputPaths, int epoch, float subProTime) {
    RPTime_->start();
    CompPro_->fractionalZ_.clear();
    float iterTime = masterTime_->dSinceStart().count();
    while (true) {
        updateReducedCosts(pInst);
        setAvailableTime();

        if (availableTime_ <= 1)
            break;

        setObjValue();
        solveRP_LPINT(pInst, 0, inputPaths);
        setCurrentRoutes(pInst);
        epochTime_ += (masterTime_->dSinceStart().count() - iterTime);
        iterTime = masterTime_->dSinceStart().count();

        RPIter_++;
        (*pLogIsudResultsStream_) << save_MPResults(epoch, "RP", (int) ReducedPro_->compRoutes_.size() - nbVehicles_,
                                                    masterTime_->dSinceStart().count(), subProTime,
                                                    ReducedPro_->auxObjValue_);
        RMPCounter_++;

        updateReducedCosts(pInst);
        if (previousObj_ - objValue_ > 0.01) {
            previousObj_ = objValue_;
        } else
            break;

        if (minReducedCost_ >= -0.1)
            break;
    }
    RPTime_->stop();
}

void MasterAlgorithm::setCurrentRoutes(PInstance &pInst) {
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
        }
        else if (pInst->parameters_->initialDual_ == AUX_P)
            MasterPro_->solveModelIntAux_P(pInst, zSolution_, routeSolution_, inputPaths,
                                     availableTime_, previousObj_);
        else
            MasterPro_->solveModelInt(pInst, zSolution_, routeSolution_, inputPaths,
                                     availableTime_, previousObj_);

        MIPIter_++;
        MPEpochSolveTime_ += MasterPro_->solveTime_->dSinceStart().count();
        setObjValue();

        setCurrentRoutes(pInst);


        epochTime_ += (masterTime_->dSinceStart().count() - iterTime_);
        if (pInst->parameters_->initialDual_ == AUX_D)
            (*pLogIsudResultsStream_) << save_MPResults(epoch, "CG", (int) MasterPro_->compRoutes_.size() - nbVehicles_,
                                                    masterTime_->dSinceStart().count(), subProTime, DualAuxSolver_->objValue_);
        else if (pInst->parameters_->initialDual_ == AUX_P)
            (*pLogIsudResultsStream_) << save_MPResults(epoch, "CG", (int) MasterPro_->compRoutes_.size() - nbVehicles_,
                                                        masterTime_->dSinceStart().count(), subProTime, MasterPro_->auxObjValue_);
        else
            (*pLogIsudResultsStream_) << save_MPResults(epoch, "CG", (int) MasterPro_->compRoutes_.size() - nbVehicles_,
                                                        masterTime_->dSinceStart().count(), subProTime, 0.0);

        RMPCounter_++;

    }
    if (!pInst->parameters_->vehiclePortion_){
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
    }
    masterTime_->stop();
}

void MasterAlgorithm::solveRLMP(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime) {
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

void MasterAlgorithm::solveRMP(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime) {
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
        MIPIter_++;
        MPEpochSolveTime_ += MasterPro_->solveTime_->dSinceStart().count();
        setObjValue();
        epochTime_ += masterTime_->dSinceStart().count();

        (*pLogIsudResultsStream_) << save_MPResults(epoch, "CG", (int) MasterPro_->compRoutes_.size() - nbVehicles_,
                                                    masterTime_->dSinceStart().count(), subProTime, 0.0);
        RMPCounter_++;
    }
    setCurrentRoutes(pInst);

    masterTime_->stop();
}
void MasterAlgorithm::solveRP_LPINT(PInstance &pInst, int compDegree, InputPaths &inputPaths) {
    // improve by solving the Reduced problem
    ReducedPro_->routesToAdd_.clear();
    if (compDegree == 0) {
        // select compatible routes with negative reduced costs
        updateIncDegreesBit(pInst);
        updateRoutesToAdd(RP, pInst);
    }
    else
    {
        updateRoutesToAddZoom();
    }

    if (!ReducedPro_->routesToAdd_.empty()){
        /*for (int v = 0; v < pInst->nbVehicles_; ++v) {
            if (pInst->vehicles_[v]->vehicleIndex_>-1 && !pInst->vehicles_[v]->emptyRoute_->mpAdded_ &&
                !pInst->vehicles_[v]->currentRoute_->routeRequests_.empty())
                ReducedPro_->routesToAdd_.push_back(pInst->vehicles_[v]->emptyRoute_);
        }*/
        MPBuildTime_->start();
        ReducedPro_->updateModel(pInst);
        MPBuildTime_->stop();
        ReducedPro_->solveModelLPInt(pInst, zSolution_, routeSolution_, inputPaths,
                                     availableTime_, objValue_);
        MPEpochSolveTime_ += ReducedPro_->solveTime_->dSinceStart().count();
        setObjValue();
    }
}

void MasterAlgorithm::solveMP_LP(PInstance &pInst, InputPaths &inputPaths, int epoch, float subProTime) {

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
        }

        LPIter_++;
        epochTime_ += (masterTime_->dSinceStart().count() - iterTime_);
        iterTime_ = masterTime_->dSinceStart().count();
        objValue_ = MasterPro_->objValue_;
        (*pLogIsudResultsStream_) << save_MPResults(epoch, "LMP", MasterPro_->compRoutes_.size() - nbVehicles_,
                                                    masterTime_->dSinceStart().count(), subProTime, 0.0);

        RMPCounter_++;

        if (lpObjValue_ - MasterPro_->objValue_ > 0.01) {
            lpObjValue_ = MasterPro_->objValue_;
        } else
            break;

        updateReducedCosts(pInst);
        if (minReducedCost_ >= -0.1)
            break;
    }
}


void MasterAlgorithm::solveMP_INT(PInstance &pInst, InputPaths &inputPaths) {
    MasterPro_->routesToAdd_.clear();

    // select routes with negative reduced costs
    updateRoutesToAdd(NR, pInst);

    if (!MasterPro_->routesToAdd_.empty()){

        MPBuildTime_->start();
        MasterPro_->updateModel(pInst);
        MPBuildTime_->stop();
        MasterPro_->solveModelIntAux_P(pInst, zSolution_, routeSolution_, inputPaths,
                                     availableTime_, objValue_);
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
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON MP IMPROVEMENT" << " = " << masterTime_->dSinceInit().count() / epoch << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON RP IMPROVEMENT" << " = " << RPTime_->dSinceInit().count()/epoch << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON CP IMPROVEMENT" << " = " << CPTime_->dSinceInit().count()/epoch << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON ZOOM ISUD" << " = " << ZOOMTime_->dSinceInit().count() / epoch << " (s)" << std::endl;

    return repStr.str();
}

void MasterAlgorithm::updateRoutesToAdd(selectionMode selectMode, PInstance &pInst){
    for (auto & vehicleObj : pInst->vehicles_) {
        if (pInst->parameters_->sortColumn_ == C_SCORE) {
            std::stable_sort(availableRoutes_[vehicleObj->vehicleID_].begin(),
                             availableRoutes_[vehicleObj->vehicleID_].end(),
                             [](const PRoute &lhs, const PRoute &rhs) { return lhs->score_ < rhs->score_; });
        }
        else if (pInst->parameters_->sortColumn_ == CLAMBDA){
            std::stable_sort(availableRoutes_[vehicleObj->vehicleID_].begin(),
                             availableRoutes_[vehicleObj->vehicleID_].end(),
                             [](const PRoute &lhs, const PRoute &rhs) { return lhs->lambda_ < rhs->lambda_; });
        }

        else {
            std::stable_sort(availableRoutes_[vehicleObj->vehicleID_].begin(),
                             availableRoutes_[vehicleObj->vehicleID_].end(),
                             [](const PRoute &lhs, const PRoute &rhs) { return lhs->reducedCost_ < rhs->reducedCost_; });
        }


        int numAdded = 0;
        for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]) {
            switch(selectMode){
                case CP:
                    if (!routeObj->cpAdded_ && (!routeObj->isCompatible_) && !routeObj->routeRequests_.empty()) {
                        CompPro_->routesToAdd_.push_back(routeObj);
                        numAdded++;
                    }
                    break;
                case RP:
                    if (!routeObj->mpAdded_ && (routeObj->incompatibilityDegree_ < 2) && routeObj->reducedCost_ < -0.1) {
                        ReducedPro_->routesToAdd_.push_back(routeObj);
                        numAdded++;
                    }
                    break;
                default: // CG and MIP:
                    if (!routeObj->mpAdded_ && routeObj->reducedCost_ < -0.1) {
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

void MasterAlgorithm::updateRoutesToAddZoom() {
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
void MasterAlgorithm::save_IncDegree_RDCost(InputPaths &inputPaths, int epoch, int isudIter) {
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
    availableTime_ = (int)(timeLimit_ - masterTime_->dSinceStart().count());
}

void MasterAlgorithm::setAvailableTime(PInstance &pInst, float elapsedTime, int iteration) {
    if (iteration > 1) {
        switch (pInst->parameters_->solutionMode_) {
            case ANYTIME:
                availableTime_ = static_cast<int>(pInst->parameters_->committedTime_ - elapsedTime);
                break;
            case DYNAMIC:
                if (pInst->parameters_->mainAlgorithm_ == RT_CG)
                    availableTime_ = static_cast<int>(pInst->parameters_->epochLength_ - elapsedTime) - 5;
                else
                    availableTime_ = static_cast<int>(pInst->parameters_->epochLength_ - elapsedTime);
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