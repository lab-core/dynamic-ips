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
    CPFails_ = 0;
    nbRoutes_ = 0;
    nbCoveredTasks_ = 0;
    cpIncDegree_ = 2;
    GreedyObjValue_ = 0;
    maxIncDegree_ = 0;
    minReducedCost_ = INFINITY;
    maxReducedCost_ = INFINITY;

    pLogIsudResultsStream_ = new Tools::LogOutput(inputPaths.getOutputEpochResults());
    (*pLogIsudResultsStream_) << "Epoch,MPIter,subProIter,TotalGenColumns,nbColumns,Model,ObjectiveValue,MPTimeAcc.,SubProTime,AuxObj" << std::endl;

    pLogIterReqDualStream_ = new Tools::LogOutput(inputPaths.getOutputReqDuals());
    (*pLogIterReqDualStream_) << "Epoch,ISUDIter,RequestID,Dual,Model,Penalty," << std::endl;

    pLogIterVehDualStream_ = new Tools::LogOutput(inputPaths.getOutputVehDuals());
    (*pLogIterVehDualStream_) << "Epoch,ISUDIter,VehicleID, Dual, Model" << std::endl;
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

// this function create initial routes serving only one request and fill zSolution_ with available requests
// Reduced problem is also solved to initialized dual costs
void MasterAlgorithm::initialization(PInstance &pInst, InputPaths &inputPaths) {
    MPEpochSolveTime_ = 0;
    CPEpochSolveTime_ = 0;
    cpIncDegree_ = 2;
    maxIncDegree_ = pInst->parameters_->CP_IncDegree_;
    masterTime_->start();

    // Building models
    CompPro_.reset();
    ReducedPro_.reset();
    MasterPro_.reset();
    CompPro_ = std::make_shared<ComplementPro>();
    ReducedPro_ = std::make_shared<ReducedProblem>();
    MasterPro_ = std::make_shared<MIPMasterProblem>();
    ReducedPro_->routesToAdd_.clear();

    RMPCounter_ = 0;
    SPIter_ = 0;
    CPBuilt_ = false;

    for (auto &vehicleObj: pInst->vehicles_) {
        if (vehicleObj->departTime_ != vehicleObj->emptyRoute_->plannedDepartTime_[0]) {
            if (vehicleObj->currentRoute_->routeSize_ == 1)
                vehicleObj->emptyRoute_ = vehicleObj->currentRoute_;
            else {
                // define new empty route and remove the old one
                vehicleObj->setEmptyRoute(pInst);
            }
        }
        ReducedPro_->routesToAdd_.push_back(vehicleObj->emptyRoute_);
    }

    // create a feasible integer solution at the start of epoch or simulation
    if (pInst->parameters_->initialStart_ == EMPTY_ROUTES){
        for (auto &requestObj : pInst->requests_)
            zSolution_.push_back(requestObj);
        routeSolution_.clear();
        for (auto &vehicleObj: pInst->vehicles_) {
            routeSolution_.push_back(vehicleObj->emptyRoute_);
        }
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
        routeSolution_.clear();
        zSolution_.clear();
        // it has been solved before in solver
        for (auto &vehicleObj: pInst->vehicles_) {
        //    ReducedPro_->routesToAdd_.push_back(vehicleObj->currentRoute_);
            routeSolution_.push_back(vehicleObj->currentRoute_);
        }
        setObjValue();
        GreedyObjValue_ = objValue_;
        std::cout << "Objective value of Greedy Warm start: " << GreedyObjValue_ << std::endl;
    }

    if ((pInst->parameters_->addOneRequestColumn_)&&(pInst->nbOnboards_ == 0)){
        // adding new arrival requests to zSolutions
        for (int i = pInst->nbRequests_ - pInst->nbNewRequests_; i < pInst->nbRequests_; ++i) {
            // creating routes with only one request
            for (int v = 0; v < pInst->nbVehicles_; ++v) {
                // creating an empty route
                PRoute newRoute = std::make_shared<Route>(pInst->vehicles_[v]->vehicleID_);

                newRoute->addSource(pInst->vehicles_[v]->departNode_,pInst->vehicles_[v]->departTime_,
                                    pInst->vehicles_[v]->numPassengers_);
                newRoute->addNode(pInst->instGraph_->pickNodes_[i]);
                newRoute->addNode(pInst->instGraph_->dropNodes_[i]);
                availableRoutes_[pInst->vehicles_[v]->vehicleID_].push_back(newRoute);
                ReducedPro_->routesToAdd_.push_back(newRoute);
            }
        }
    }

    // set the duals of un-served requests based on penalties
    if (pInst->parameters_->initialDual_ == PENALTIES){
        for (auto &requestObj: pInst->requests_)
            requestObj->dual_ = requestObj->penalty_;
        for (auto &vehicleObj: pInst->vehicles_)
            vehicleObj->dual_ = 0;

    } else {
        for (auto &requestObj: zSolution_)
            requestObj->dual_ = requestObj->penalty_;
    }
    for (auto &requestObj: zSolution_)
        requestObj->InitialDual_ = requestObj->penalty_;

    /*RPTime_->start();

    if ((pInst->parameters_->addOneRequestColumn_)&&(pInst->nbOnboards_ == 0)) {
        MPBuildTime_->start();
        ReducedPro_->buildModel(pInst, routeSolution_, nbVehicles_);
        MPBuildTime_->stop();
        ReducedPro_->solveModelLPInt(pInst, zSolution_, routeSolution_, inputPaths,
                                     (int)availableTime_, objValue_);
        setObjValue();
    }


    RPTime_->stop();*/
    masterTime_->stop();
}


// function to calculate incompatibility degree of a route
void MasterAlgorithm::calcIncompatibilityBit(PRoute &route, PInstance &pInst) {
    route->isCompatible_ = true;
    route->incompatibilityDegree_ = 0;
    if ((route->column_ & pInst->vehicles_[route->vehicleID_]->currentRoute_->column_)!=pInst->vehicles_[route->vehicleID_]->currentRoute_->column_){
        route->isCompatible_ = false;
        route->incompatibilityDegree_++;
    }
    std::bitset<MAX_BIT_SIZE>  vehicles;
    for (auto & requestObj : route->routeRequests_){
        if (requestObj->solVehicleID_ < LARGE_CONSTANT)
            vehicles.set(requestObj->solVehicleID_, true);
    }
    vehicles.set(route->vehicleID_, false);
    if (vehicles.count()>0){
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
            if (minReducedCost_ > routeObj->reducedCost_)
                minReducedCost_ = routeObj->reducedCost_;
            if (!routeObj->routeRequests_.empty())
                routeObj->score_ = routeObj->reducedCost_/routeObj->routeRequests_.size();
            else
                routeObj->score_ = 0;
        }
        vehicleObj->bestReducedCost_ = minReducedCost_;
    }
    if (minReducedCost_ < 0)
        maxReducedCost_ = ((-0.5)*minReducedCost_);
}

void MasterAlgorithm::solveISUD(PInstance &pInst, int epoch, InputPaths &inputPaths, double subProTime) {
    try {
        masterTime_->start();
        setObjValue();
        double previousObj = objValue_;
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

        if (RMPCounter_ == 1) {
            MPBuildTime_->start();
            ReducedPro_->routesToAdd_.clear();
            ReducedPro_->buildModel(pInst,  routeSolution_, nbVehicles_);
            MPBuildTime_->stop();
/*            if (availableTime_ < 1 && pInst->parameters_->solutionMode_ != ANYTIME)
                tiLim = 3;*/
        }

        if (pInst->parameters_->solutionMode_ != ANYTIME) {
            (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, RMPCounter_, "initial");
            (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, RMPCounter_, "initial");
        }
        while (restartAlgorithm) {
            isCPImproved = true;
            restartAlgorithm = true;
            /************************************************************************************************/
            //                                     REDUCED PROBLEM
            /************************************************************************************************/
            RPTime_->start();
            // solve RP with MIP solver
            CompPro_->fractionalZ_.clear();
            while (true) {
                updateReducedCosts(pInst);
                setAvailableTime();

                if (minReducedCost_ >= 0 || availableTime_ < 0)
                    break;

                setObjValue();
                solveRP_LPINT(pInst, 0, inputPaths);
                for (auto & routeObj : routeSolution_) {
                    pInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
                }

                RPIter_++;
                (*pLogIsudResultsStream_) << save_ISUDResults(epoch, "RP", (int) ReducedPro_->compRoutes_.size() - nbVehicles_,
                                                              masterTime_->dSinceStart().count(), subProTime,
                                                              ReducedPro_->auxObjValue_);
                RMPCounter_++;
                // save duals
                if (pInst->parameters_->solutionMode_ != ANYTIME) {
                    (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, RMPCounter_, "LRP");
                    (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, RMPCounter_, "LRP");
                }
                if (previousObj > objValue_) {
                    previousObj = objValue_;
                } else
                    break;
            }
            RPTime_->stop();

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

            while (isCPImproved) {
                isCPImproved = false;
                previousObj = objValue_;
                updateReducedCosts(pInst);
                updateIncDegreesBit(pInst);
                CompPro_->routesToAdd_.clear();
                updateRoutesToAdd(CP, pInst);
                setAvailableTime();
                if (!CompPro_->routesToAdd_.empty() && availableTime_ > 0) {
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
                    if (pInst->parameters_->solutionMode_ != ANYTIME) {
                        (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, RMPCounter_, "CP");
                        (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, RMPCounter_, "CP");
                    }

                    CPEpochSolveTime_ += CompPro_->solveTime_->dSinceStart().count();
                    setObjValue();
                    (*pLogIsudResultsStream_) << save_ISUDResults(epoch, "CP", (int) (CompPro_->IncRoute_.size()),
                                                                  masterTime_->dSinceStart().count(), subProTime, 0.0);

                    if (CompPro_->status_ == FRACTIONAL) {
                        CPFails_++;
                        setAvailableTime();
                        if (pInst->parameters_->useZoom_ && availableTime_ > 1) {
                            ZOOMTime_->start();
                            solveRP_LPINT(pInst, 1, inputPaths);
                            ZoomIter_++;
                            (*pLogIsudResultsStream_)
                                    << save_ISUDResults(epoch, "ZOOM", (int) ReducedPro_->compRoutes_.size() - nbVehicles_,
                                                        masterTime_->dSinceStart().count(), subProTime,
                                                        ReducedPro_->auxObjValue_);
                            RMPCounter_++;

                            if (pInst->parameters_->solutionMode_ != ANYTIME) {
                                (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, RMPCounter_, "zoom");
                                (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, RMPCounter_, "zoom");
                            }
                            ZOOMTime_->stop();
                            if (previousObj > objValue_) {
                                previousObj = objValue_;
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
                    else if (CompPro_->status_ == POSITIVE_VALUE) {
                        restartAlgorithm = false;
                        break;
                    }
                    else if (CompPro_->status_ == INFEASIBLE) {
                        restartAlgorithm = false;
                        break;
                    }
                    else {
                        for (auto & routeObj : routeSolution_) {
                            pInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
                        }
                        CPSuccess_++;
                        previousObj = objValue_;
                        RMPCounter_++;
                        isCPImproved = true;
                        CPCounter++;
                        setAvailableTime();
                        if (availableTime_ <= 0) {
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
                << save_ISUDResults(epoch, "ISUD", nbRoutes_, masterTime_->dSinceStart().count(), subProTime, 0.0);
        masterTime_->stop();
    }
    catch (const std::exception& e){
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e.what() << std::endl;
    }
}
void MasterAlgorithm::solveISUD_improved(PInstance &pInst, int epoch, InputPaths &inputPaths, double subProTime) {
    try {
        masterTime_->start();
        float tiLim = availableTime_;
        setObjValue();
        double previousObj = objValue_;
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

        if (pInst->parameters_->solutionMode_ != ANYTIME) {
            (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, RMPCounter_, "initial");
            (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, RMPCounter_, "initial");
        }
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
                for (auto & routeObj : routeSolution_) {
                    pInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
                }
                if (pInst->parameters_->solutionMode_ != ANYTIME) {
                    (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, RMPCounter_, "LRP");
                    (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, RMPCounter_, "LRP");
                }

                RPIter_++;
                //               std::cout << "RP improve: " << objValue_ << std::endl;
//                if (epoch == 4)
                (*pLogIsudResultsStream_) << save_ISUDResults(epoch, "RP", (int) ReducedPro_->compRoutes_.size() - nbVehicles_,
                                                              masterTime_->dSinceStart().count(), subProTime,
                                                              ReducedPro_->auxObjValue_);
                RMPCounter_++;
                if (previousObj > objValue_) {
                    previousObj = objValue_;
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
                previousObj = objValue_;

                availableTime_ = tiLim - masterTime_->dSinceStart().count();
                if (availableTime_ > 0) {
                    CompPro_->solveCP2Model(pInst, zSolution_, routeSolution_, inputPaths);
                    if (pInst->parameters_->solutionMode_ != ANYTIME) {
                        (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, RMPCounter_, "CP");
                        (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, RMPCounter_, "CP");
                    }
                    CPIter_++;
                    CPEpochSolveTime_ += CompPro_->solveTime_->dSinceStart().count();
                    setObjValue();
                    (*pLogIsudResultsStream_) << save_ISUDResults(epoch, "CP", (int) (CompPro_->IncRoute_.size()),
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
                        for (auto & routeObj : routeSolution_) {
                            pInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
                        }
                        CPSuccess_++;
                        previousObj = objValue_;
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
                << save_ISUDResults(epoch, "ISUD", nbRoutes_, masterTime_->dSinceStart().count(), subProTime, 0.0);
        masterTime_->stop();
    }
    catch (const std::exception& e){
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e.what() << std::endl;
    }
}

void MasterAlgorithm::solveMP_MIP(PInstance &pInst, int epoch, InputPaths &inputPaths, double subProTime) {
    masterTime_->start();
    double previousObj = objValue_;

    // update reduced costs if needed only at the start of epoch, if we used penalties to create routes
    if  ((pInst->parameters_->initialStart_ == PRE_SOLUTION)&&(pInst->parameters_->initialDual_ == PENALTIES) && (RMPCounter_ == 1)){
        for(auto & requestObj: pInst->requests_)
            requestObj->dual_ = requestObj->InitialDual_;
        for(auto & vehicleObj : pInst->vehicles_)
            vehicleObj->dual_ = vehicleObj->InitialDual_;
    }
    if (RMPCounter_ == 1) {
        MPBuildTime_->start();
        MasterPro_->routesToAdd_.clear();
        MasterPro_->buildModelMP(pInst, routeSolution_, nbVehicles_);
        MPBuildTime_->stop();
    }

    // save initial duals
    if (pInst->parameters_->solutionMode_ != ANYTIME) {
        (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, RMPCounter_, "initial");
        (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, RMPCounter_, "initial");
    }

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
        (*pLogIsudResultsStream_) << save_ISUDResults(epoch, "RMP", (int) MasterPro_->compRoutes_.size()-nbVehicles_,
                                                      masterTime_->dSinceStart().count(), subProTime,
                                                      MasterPro_->auxObjValue_);

        RMPCounter_++;
        // save duals
        if (pInst->parameters_->solutionMode_ != ANYTIME) {
            (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, RMPCounter_, "RMP");
            (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, RMPCounter_, "RMP");
        }
        setObjValue();
        if (previousObj > objValue_) {
            previousObj = objValue_;
        }
        else
            break;
    }

    setObjValue();
    (*pLogIsudResultsStream_) << save_ISUDResults(epoch, "MIP", (int) MasterPro_->compRoutes_.size()-nbVehicles_,
                                                  masterTime_->dSinceStart().count(), subProTime,
                                                  MasterPro_->auxObjValue_);
    RMPCounter_++;


    /*std::cout << "# number of un-served requests: " << zSolution_.size() << std::endl;
    std::cout << "# Time spent on ISUD iteration  = " << masterTime_->dSinceStart().count() << " (seconds)"
              << std::endl;
    for (auto &requestObj: zSolution_)
        std::cout << "request " << requestObj->getRequestId() << " : " << requestObj->penalty_ << std::endl;*/


    for (auto & routeObj : routeSolution_) {
        pInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
    }

    masterTime_->stop();
}

void MasterAlgorithm::solveMP_CG(PInstance &pInst, int epoch, InputPaths &inputPaths, double subProTime) {
    masterTime_->start();
    setObjValue();
    double previousObj = objValue_;


    // update reduced costs if needed only at the start of epoch, if we used penalties to create routes
    if  ((pInst->parameters_->initialStart_ == PRE_SOLUTION)&&(pInst->parameters_->initialDual_ == PENALTIES) && (RMPCounter_ == 1)){
        for(auto & requestObj: pInst->requests_)
            requestObj->dual_ = requestObj->InitialDual_;
        for(auto & vehicleObj : pInst->vehicles_)
            vehicleObj->dual_ = vehicleObj->InitialDual_;
    }
    if (RMPCounter_ == 1) {
        MPBuildTime_->start();
        MasterPro_->routesToAdd_.clear();
        MasterPro_->buildModelMP(pInst, routeSolution_, nbVehicles_);
        MPBuildTime_->stop();
    }

    // save initial duals
    /*if (pInst->parameters_->solutionMode_ != ANYTIME) {
        (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, RMPCounter_, "initial");
        (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, RMPCounter_, "initial");
    }*/
    /************************************************************************************************/
    //                                     MASTER PROBLEM
    /************************************************************************************************/
    setAvailableTime();
    if (availableTime_ > 1) {
        double lpObj = previousObj;
        // solve RP with MIP solver
        while (true) {
            updateReducedCosts(pInst);
            setAvailableTime();
            if (minReducedCost_ >= 0 || availableTime_ < 0)
                break;

            solveMP_LP(pInst, inputPaths);
            LPIter_++;
            (*pLogIsudResultsStream_) << save_ISUDResults(epoch, "LMP", MasterPro_->compRoutes_.size()-nbVehicles_,
                                                          masterTime_->dSinceStart().count(), subProTime, 0.0);

            RMPCounter_++;
            // save duals
            /*if (pInst->parameters_->solutionMode_ != ANYTIME) {
                (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, RMPCounter_, "LMP");
                (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, RMPCounter_, "LMP");
            }*/
            if (lpObj > objValue_) {
                lpObj = objValue_;
            } else
                break;
        }


        setAvailableTime();
        if (availableTime_ < 1)
            availableTime_ = 2;
        // solve the model in Integer mode
        lpObjValue_ = lpObj;
        MasterPro_->solveModelInt(pInst, zSolution_, routeSolution_, inputPaths, availableTime_, previousObj);
        MIPIter_++;
        MPEpochSolveTime_ += MasterPro_->solveTime_->dSinceStart().count();
        setObjValue();


        (*pLogIsudResultsStream_) << save_ISUDResults(epoch, "CG", (int) MasterPro_->compRoutes_.size()-nbVehicles_,
                                                      masterTime_->dSinceStart().count(), subProTime, 0.0);
        RMPCounter_++;
    }

    for (auto & routeObj : routeSolution_) {
        pInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
    }

    masterTime_->stop();
}

void MasterAlgorithm::solveRLMP(PInstance &pInst, int epoch, InputPaths &inputPaths, double subProTime) {
    masterTime_->start();
    setObjValue();
    double previousObj = objValue_;

    if (RMPCounter_ == 1) {
        MPBuildTime_->start();
        MasterPro_->routesToAdd_.clear();
        MasterPro_->buildModelMP(pInst, routeSolution_, nbVehicles_);
        MPBuildTime_->stop();
    }

    // save initial duals
    /*if (pInst->parameters_->solutionMode_ != ANYTIME) {
        (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, RMPCounter_, "initial");
        (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, RMPCounter_, "initial");
    }*/
    /************************************************************************************************/
    //                                     MASTER PROBLEM
    /************************************************************************************************/
    setAvailableTime();
    if (availableTime_ > 1) {
        double lpObj = previousObj;
        // solve RP with MIP solver
        while (true) {
            updateReducedCosts(pInst);
            setAvailableTime();
            if (minReducedCost_ >= 0 || availableTime_ < 0)
                break;

            solveMP_LP(pInst, inputPaths);
            LPIter_++;
            (*pLogIsudResultsStream_) << save_ISUDResults(epoch, "LMP", MasterPro_->compRoutes_.size()-nbVehicles_,
                                                          masterTime_->dSinceStart().count(), subProTime, 0.0);

            RMPCounter_++;
            // save duals
            if (lpObj > objValue_) {
                lpObj = objValue_;
            } else
                break;
        }
    }
    masterTime_->stop();
}

void MasterAlgorithm::solveRMP(PInstance &pInst, int epoch, InputPaths &inputPaths, double subProTime) {
    masterTime_->start();
    double previousObj = objValue_;

    /************************************************************************************************/
    //                                     MASTER PROBLEM
    /************************************************************************************************/
    setAvailableTime();
    if (availableTime_ > 1) {
        double lpObj = previousObj;
        // solve RP with MIP solver
        if (availableTime_ < 1)
            availableTime_ = 2;
        // solve the model in Integer mode
        lpObjValue_ = lpObj;
        MasterPro_->solveModelInt(pInst, zSolution_, routeSolution_, inputPaths, availableTime_, previousObj);
        MIPIter_++;
        MPEpochSolveTime_ += MasterPro_->solveTime_->dSinceStart().count();
        setObjValue();


        (*pLogIsudResultsStream_) << save_ISUDResults(epoch, "CG", (int) MasterPro_->compRoutes_.size()-nbVehicles_,
                                                      masterTime_->dSinceStart().count(), subProTime, 0.0);
        RMPCounter_++;
    }

    for (auto & routeObj : routeSolution_) {
        pInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
    }

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
        for (int v = 0; v < pInst->nbVehicles_; ++v) {
            if (pInst->vehicles_[v]->vehicleIndex_>-1 && !pInst->vehicles_[v]->emptyRoute_->mpAdded_ &&
                !pInst->vehicles_[v]->currentRoute_->routeRequests_.empty())
                ReducedPro_->routesToAdd_.push_back(pInst->vehicles_[v]->emptyRoute_);
        }
        MPBuildTime_->start();
        ReducedPro_->updateModel(pInst, CompPro_->fractionalZ_);
        MPBuildTime_->stop();
        ReducedPro_->solveModelLPInt(pInst, zSolution_, routeSolution_, inputPaths,
                                     availableTime_, objValue_);
        MPEpochSolveTime_ += ReducedPro_->solveTime_->dSinceStart().count();
        setObjValue();
    }
}

void MasterAlgorithm::solveMP_LP(PInstance &pInst, InputPaths &inputPaths) {
    MasterPro_->routesToAdd_.clear();

    // select routes with negative reduced costs
    updateRoutesToAdd(NR, pInst);

    if (!MasterPro_->routesToAdd_.empty()){
        for (int v = 0; v < pInst->nbVehicles_; ++v) {
            if (pInst->vehicles_[v]->vehicleIndex_>-1 && !pInst->vehicles_[v]->emptyRoute_->mpAdded_&&
                !pInst->vehicles_[v]->currentRoute_->routeRequests_.empty())
                MasterPro_->routesToAdd_.push_back(pInst->vehicles_[v]->emptyRoute_);
        }
        MPBuildTime_->start();
        MasterPro_->updateModel(pInst);
        MPBuildTime_->stop();
        MasterPro_->solveModelLP(pInst, inputPaths);
        MPEpochSolveTime_ += MasterPro_->solveTime_->dSinceStart().count();
        objValue_ = MasterPro_->objValue_;
    }
}

void MasterAlgorithm::solveMP_INT(PInstance &pInst, InputPaths &inputPaths) {
    MasterPro_->routesToAdd_.clear();

    // select routes with negative reduced costs
    updateRoutesToAdd(NR, pInst);

    if (!MasterPro_->routesToAdd_.empty()){
        for (int v = 0; v < pInst->nbVehicles_; ++v) {
            if (pInst->vehicles_[v]->vehicleIndex_>-1 && !pInst->vehicles_[v]->emptyRoute_->mpAdded_ &&
                    pInst->vehicles_[v]->emptyRoute_->routeSize_ != pInst->vehicles_[v]->currentRoute_->routeSize_) {
                MasterPro_->routesToAdd_.push_back(pInst->vehicles_[v]->emptyRoute_);
            }
        }
        MPBuildTime_->start();
        MasterPro_->updateModel(pInst);
        MPBuildTime_->stop();
        MasterPro_->solveModelIntAux(pInst, zSolution_, routeSolution_, inputPaths,
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

    repStr << "# TIME SPENT ON ISUD IMPROVEMENT              = " << masterTime_->dSinceStart().count() << " (s)" << std::endl;
    repStr << "# TIME SPENT ON RP IMPROVEMENT                = " << RPTime_->dSinceStart().count() << " (s)" << std::endl;
    repStr << "# TIME SPENT ON CP IMPROVEMENT                = " << CPTime_->dSinceStart().count() << " (s)" << std::endl;
    repStr << "# TIME SPENT ON ZOOM ISUD                     = " << ZOOMTime_->dSinceStart().count() << " (s)" << std::endl;
    return repStr.str();
}

std::string MasterAlgorithm::toStringTimersTotal() const {
    std::stringstream repStr;
    repStr << std::left << std::fixed << std::setprecision(2);
    repStr << "#" << std::endl;
    repStr << "# -------------------   TOTAL ISUD RUN TIMES   -------------------" << std::endl;
    repStr << "#" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON ISUD IMPROVEMENT" << " = " << masterTime_->dSinceInit().count() << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON RP IMPROVEMENT" << " = " << RPTime_->dSinceInit().count() << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON CP IMPROVEMENT" << " = " << CPTime_->dSinceInit().count() << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON ZOOM ISUD" << " = " << ZOOMTime_->dSinceInit().count() << " (s)" << std::endl;

    return repStr.str();
}

std::string MasterAlgorithm::toStringTimersAvg(int epoch) const {
    std::stringstream repStr;
    repStr << std::left << std::fixed << std::setprecision(2);
    repStr << "#" << std::endl;
    repStr << "# -------------   AVERAGE ISUD RUN TIMES PER EPOCH   -------------" << std::endl;
    repStr << "#" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON ISUD IMPROVEMENT" << " = " << masterTime_->dSinceInit().count() / epoch << " (s)" << std::endl;
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
                    if (!routeObj->mpAdded_ && (routeObj->incompatibilityDegree_ < 2) && routeObj->reducedCost_ < 0) {
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


std::string MasterAlgorithm::save_ISUDResults(int epoch, const std::string& model, int nbColumns, float reachTime,
                                              double subProTime, double auxObj) const {
    std::stringstream repStr;

    repStr << epoch << ",";
    repStr << RMPCounter_ << ",";
    repStr << SPIter_ << ",";
    repStr << nbRoutes_ << ",";
    repStr << nbColumns << ",";
    repStr << model << ",";
    repStr << objValue_ << ",";
    repStr << reachTime << ",";
    repStr << subProTime << ",";
    repStr << auxObj << "\n";
    return repStr.str();
}

void MasterAlgorithm::setAvailableTime() {
    availableTime_ = (int)(timeLimit_ - masterTime_->dSinceStart().count());
}
























