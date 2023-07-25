//
// Created by Ella on 2021-10-09.
//

#include "ISUDAlgorithm.h"

//---------------------------------------------------------------------------------------------
//  Reduced Problem class
//  Build and solve the Reduced problem of the ISUD
//---------------------------------------------------------------------------------------------

ISUDAlgorithm::ISUDAlgorithm(InputPaths &inputPaths) {
//    ReducedPro_ = std::make_shared<ReducedProblem>();
    CompPro_ = std::make_shared<ComplementPro>();
    MIPReducedPro_ = std::make_shared<ZoomReducedProblem>();
    MasterPro_ = std::make_shared<MasterPro>();
    objValue_ = 0;
    totalWaitTime_ = 0;
    isudTime_ = new myTools::Timer(); isudTime_->init();
    RPTime_ = new myTools::Timer(); RPTime_->init();
    CPTime_ = new myTools::Timer(); CPTime_->init();


    RPBuildTime_ = new myTools::Timer(); RPBuildTime_->init();
    CPBuildTime_ = new myTools::Timer(); CPBuildTime_->init();

    isudMIPTime_ = new myTools::Timer(); isudMIPTime_->init();
    isudIter_ = 0;
    TisudIter_ = 0;
    LPIter_ = 0;
    MPIter_ = 0;
    RPIter_ = 0;
    CPIter_ = 0;
    SPIter_ = 0;
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

    pLogIterSolutionStream_ = new Tools::LogOutput(inputPaths.getOutputEpochIsud());
    (*pLogIterSolutionStream_) << "Epoch,ISUDIter,VehicleID,NodeID,RequestTime,ReachTime,NodeType,LocationID,RouteID,IncDegree" << std::endl;

    pLogIterReqDualStream_ = new Tools::LogOutput(inputPaths.getOutputReqDuals());
    (*pLogIterReqDualStream_) << "Epoch,ISUDIter,RequestID,Dual,Model,Penalty," << std::endl;

    pLogIterVehDualStream_ = new Tools::LogOutput(inputPaths.getOutputVehDuals());
    (*pLogIterVehDualStream_) << "Epoch,ISUDIter,VehicleID, Dual, Model" << std::endl;
}

ISUDAlgorithm::~ISUDAlgorithm() {
    delete isudTime_;
    delete RPTime_;
    delete CPTime_;

    delete RPBuildTime_;
    delete CPBuildTime_;

    delete isudMIPTime_;
    pLogIsudResultsStream_->close();
    pLogIterSolutionStream_->close();
    pLogIterReqDualStream_->close();
    pLogIterVehDualStream_->close();
    delete pLogIsudResultsStream_;
    delete pLogIterSolutionStream_;
    delete pLogIterReqDualStream_;
    delete pLogIterVehDualStream_;
}


void ISUDAlgorithm::setObjValue() {
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
void ISUDAlgorithm::initialization(PInstance &pInst, InputPaths &inputPaths) {
    RPEpochSolveTime_ = 0;
    CPEpochSolveTime_ = 0;
    cpIncDegree_ = 2;
    maxIncDegree_ = pInst->parameters_->CP_IncDegree_;
    isudTime_->start();
    CompPro_.reset();
    MIPReducedPro_.reset();
    CompPro_ = std::make_shared<ComplementPro>();
    MIPReducedPro_ = std::make_shared<ZoomReducedProblem>();
    MIPReducedPro_->routesToAdd_.clear();
    isudIter_ = 0;
    SPIter_ = 0;

    for (auto &vehicleObj: pInst->vehicles_) {
        if (vehicleObj->departTime_ != vehicleObj->emptyRoute_->plannedDepartTime_[0]) {
            if (vehicleObj->currentRoute_->routeSize_ == 1)
                vehicleObj->emptyRoute_ = vehicleObj->currentRoute_;
            else {
                // define new empty route and remove the old one
                vehicleObj->setEmptyRoute(pInst);
            }
        }
        MIPReducedPro_->routesToAdd_.push_back(vehicleObj->emptyRoute_);
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
        for (int i = pInst->nbRequests_ - pInst->nbNewRequests_; i < pInst->nbRequests_; ++i)
            zSolution_.push_back(pInst->requests_[i]);
        if (objValue_ == 0){
            routeSolution_.clear();
            for (auto &vehicleObj: pInst->vehicles_) {
                routeSolution_.push_back(vehicleObj->emptyRoute_);
            }
        }
        setObjValue();
    }
    else if (pInst->parameters_->initialStart_ == GREEDY_START){
        routeSolution_.clear();
        zSolution_.clear();
        // it has been solved before in solver
        for (auto &vehicleObj: pInst->vehicles_) {
        //    MIPReducedPro_->routesToAdd_.push_back(vehicleObj->currentRoute_);
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
                newRoute->addNode1(pInst->instGraph_->pickNodes_[i]);
                newRoute->addNode1(pInst->instGraph_->dropNodes_[i]);
                availableRoutes_[pInst->vehicles_[v]->vehicleID_].push_back(newRoute);
                MIPReducedPro_->routesToAdd_.push_back(newRoute);
            }
        }
    }

    // set the duals of unserved requests based on penalties
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
        requestObj->CPDual_ = requestObj->penalty_;

 //   std::cout << "# -----SOLVING THE REDUCED PROBLEM AT THE START OF EPOCH-------" << std::endl;
    RPTime_->start();

    if ((pInst->parameters_->addOneRequestColumn_)&&(pInst->nbOnboards_ == 0)) {
        RPBuildTime_->start();
        MIPReducedPro_->buildModel(pInst, zSolution_, routeSolution_, nbVehicles_);
        RPBuildTime_->stop();
        MIPReducedPro_->solveModelLPInt(pInst, zSolution_, routeSolution_, inputPaths,
                                        (int)availableTime_, objValue_);
        setObjValue();
    }


    RPTime_->stop();
    isudTime_->stop();
}

// function to create M2 matrix for each column in the current solution
Eigen::MatrixXd ISUDAlgorithm::calcM2Matrix(PRoute &solColumn) {
    unsigned int nbRows = solColumn->routeRequests_.size() - 1;
    Eigen::MatrixXd M2 = Eigen::MatrixXd::Zero(nbRows, nbRows+1);
    for (int i = 0; i < nbRows; ++i) {
        M2(i,i) = 1;
        M2(i,i+1) = -1;
    }
    return M2;
}

Eigen::MatrixXd ISUDAlgorithm::calcM2Matrix(int nbRows) {
    Eigen::MatrixXd M2 = Eigen::MatrixXd::Zero(nbRows, nbRows+1);
    for (int i = 0; i < nbRows; ++i) {
        M2(i,i) = 1;
        M2(i,i+1) = -1;
    }
    return M2;
}

// function to calculate incompatibility matrix
void ISUDAlgorithm::calcIncMatrix() {
//    incRequestToOrder_.clear();
    /*for (auto & requestObj : zSolution_)
        requestObj->taskIncIndex_  =-1;*/
    nbCoveredTasks_ = 0;
    int orderCount = 0;
    std::stable_sort(routeSolution_.begin(),routeSolution_.end(),[](const PRoute &lhs, const PRoute &rhs){
        return lhs->routeRequests_.size() < rhs->routeRequests_.size();});
    if (routeSolution_.back()->routeRequests_.size() <= 1) {
        incMatrix_ = Eigen::MatrixXd::Zero(0,0);
    }
    else {
        incMatrix_ = Eigen::MatrixXd::Zero(0,0);
        for (auto & routeObj : routeSolution_) {
            if (routeObj->routeRequests_.size() > 1) {
                for (auto & requestObj : routeObj->routeRequests_) {
 //                   incRequestToOrder_[requestObj->getRequestId()] = orderCount;
                    requestObj->taskIncIndex_ = orderCount;
                    orderCount++;
                    nbCoveredTasks_++;
                }
                unsigned int nbRows = routeObj->routeRequests_.size() - 1;
                unsigned int rowSize = incMatrix_.rows();
                unsigned int colSize = incMatrix_.cols();
                if (nbRows == 0) {
                    incMatrix_.conservativeResize(rowSize, colSize + 1);
                    incMatrix_.bottomRightCorner(rowSize, 1) = Eigen::MatrixXd::Zero(rowSize, 1);
                } else {

                    incMatrix_.conservativeResize(rowSize + nbRows, colSize + nbRows + 1);
                    incMatrix_.bottomRightCorner(nbRows, nbRows + 1) = calcM2Matrix(routeObj);
                    incMatrix_.bottomLeftCorner(nbRows, colSize) = Eigen::MatrixXd::Zero(nbRows, colSize);
                    incMatrix_.topRightCorner(rowSize, nbRows + 1) = Eigen::MatrixXd::Zero(rowSize, nbRows + 1);
                }
            }
        }
    }
}

void ISUDAlgorithm::calcIncMatrixFull() {
//    incRequestToOrder_.clear();
    for (auto & requestObj : zSolution_)
        requestObj->taskIncIndex_  = -1;
    nbCoveredTasks_ = 0;
    incVehicleToOrder_.clear();
    int orderCount = 0;
    std::stable_sort(routeSolution_.begin(),routeSolution_.end(),[](const PRoute &lhs, const PRoute &rhs){
        return lhs->routeRequests_.size() < rhs->routeRequests_.size();});
    if (routeSolution_.back()->routeRequests_.empty()) {
        incMatrix_ = Eigen::MatrixXd::Zero(0,0);
    }
    else {
        incMatrix_ = Eigen::MatrixXd::Zero(0,0);
        for (auto & routeObj : routeSolution_) {
            if (routeObj->routeRequests_.size() >= 1) {
                for (auto & requestObj : routeObj->routeRequests_) {
                    requestObj->taskIncIndex_ = orderCount;
                    nbCoveredTasks_ ++;
                    orderCount++;
                }
                incVehicleToOrder_[routeObj->vehicleID_] = orderCount;
                orderCount++;
                unsigned int nbRows = routeObj->routeRequests_.size();
                unsigned int rowSize = incMatrix_.rows();
                unsigned int colSize = incMatrix_.cols();

                incMatrix_.conservativeResize(rowSize + nbRows, colSize + nbRows + 1);
                incMatrix_.bottomRightCorner(nbRows, nbRows + 1) = calcM2Matrix((int)routeObj->routeRequests_.size());
                incMatrix_.bottomLeftCorner(nbRows, colSize) = Eigen::MatrixXd::Zero(nbRows, colSize);
                incMatrix_.topRightCorner(rowSize, nbRows + 1) = Eigen::MatrixXd::Zero(rowSize, nbRows + 1);
            }
        }
    }
}
// function to calculate incompatibility degree of a route
void ISUDAlgorithm::calcIncompatibility(PRoute &route) {
    if (incMatrix_.rows() > 0) {
        Eigen::MatrixXd pattern = Eigen::MatrixXd::Zero(nbCoveredTasks_,1);

        for (auto & requestObj : route->routeRequests_) {
            if (requestObj->taskIncIndex_ !=  - 1)
                pattern(requestObj->taskIncIndex_, 0) = 1;
        }
        Eigen::MatrixXd multiplication = incMatrix_ * pattern;
        route->incompatibilityDegree_ = multiplication.cwiseAbs().colwise().sum()[0];
    }
    else
        route->incompatibilityDegree_ = 0;
}

void ISUDAlgorithm::calcIncompatibilityFull(PRoute &route) {
    if (incMatrix_.rows() > 0) {
        Eigen::MatrixXd pattern = Eigen::MatrixXd::Zero(nbCoveredTasks_ + (int) incVehicleToOrder_.size(),1);
        for (auto & requestObj : route->routeRequests_) {
            if (requestObj->taskIncIndex_ != -1)
                pattern(requestObj->taskIncIndex_, 0) = 1;
        }
        if (incVehicleToOrder_.count(route->vehicleID_)>0)
            pattern(incVehicleToOrder_[route->vehicleID_],0) = 1;
        Eigen::MatrixXd multiplication = incMatrix_ * pattern;
        route->incompatibilityDegree_ = multiplication.cwiseAbs().colwise().sum()[0];
    }
    else
        route->incompatibilityDegree_ = 0;
}
void ISUDAlgorithm::calcIncompatibilityBit(PRoute &route, PInstance &pInst) {
    route->isCompatible_ = true;
    route->incompatibilityDegree_ = 0;
    if ((route->column_ & pInst->vehicles_[route->vehicleID_]->currentRoute_->column_)!=pInst->vehicles_[route->vehicleID_]->currentRoute_->column_){
        route->isCompatible_ = false;
        route->incompatibilityDegree_++;
    }
    std::bitset<3000>  vehicles;
    for (auto & requestObj : route->routeRequests_){
        if (requestObj->solVehicleID_ < 999999)
            vehicles.set(requestObj->solVehicleID_, true);
    }
    vehicles.set(route->vehicleID_, false);
    if (vehicles.count()>0){
        route->isCompatible_ = false;
        route->incompatibilityDegree_ += vehicles.count();
    }
}

void ISUDAlgorithm::calcIncompatibilityMatrix() {
   /* if (incMatrix_.rows() > 0){
//        Eigen::MatrixXd pattern = Eigen::MatrixXd::Zero((int) incRequestToOrder_.size() + (int) incVehicleToOrder_.size(),generatedRoutes_.size());
        Eigen::MatrixXd pattern = Eigen::MatrixXd::Zero(nbCoveredTasks_+ (int) incVehicleToOrder_.size(),generatedRoutes_.size());
        int i = 0;
        for (auto & routeObj: generatedRoutes_){
            for (auto & requestObj : routeObj.second->routeRequests_) {
                *//*if (incRequestToOrder_.count(requestObj->getRequestId()) > 0)
                pattern(incRequestToOrder_[requestObj->getRequestId()], 0) = 1;*//*
                if (requestObj->taskIncIndex_ != -1)
                    pattern(requestObj->taskIncIndex_, 0) = 1;
            }
            if (incVehicleToOrder_.count(routeObj.second->VehicleID_)>0)
                pattern(incVehicleToOrder_[routeObj.second->VehicleID_],i) = 1;
            i++;
        }
        Eigen::MatrixXd incValues = (incMatrix_ * pattern).cwiseAbs().colwise().sum();
        i = 0;
        for (auto & routeObj: generatedRoutes_){
            routeObj.second->incompatibilityDegree_ = incValues(0,i);
            i++;
        }
    }
    else {
        for (auto & routeObj: generatedRoutes_){
            routeObj.second->incompatibilityDegree_ = 0;
        }
    }*/
}

// this function update the incompatibility degree of availableRoutes and
// order them based on the incompatibility degree and reduced cost
void ISUDAlgorithm::updateIncDegrees(PInstance &pInst) {
    for (auto & requestObj : pInst->requests_)
        requestObj->taskIncIndex_ = -1;
//    calcIncMatrix();
    calcIncMatrixFull();
    maxIncDegree_ = 0;
    Tools::PThreadsPool pPool = Tools::ThreadsPool::newThreadsPool(pInst->parameters_->nbThreads_);

    for (auto & vehicleObj : pInst->vehicles_) {
        if (!availableRoutes_[vehicleObj->vehicleID_].empty()) {
            Tools::Job job([&]() {
                updateRoutesIncDegree(vehicleObj->vehicleID_);
            });
            pPool->run(job);
//            updateRoutesIncDegree(vehicleObj->VehicleID_);
        }
    }
    pPool->wait();
   /* while(true){
        if (!pPool->wait())
            break;
    }*/
}
void ISUDAlgorithm::updateIncompatState(PInstance &pInst) {
    /*std::shared_ptr<myTools::BitVector>  zList = std::make_shared<myTools::BitVector>(pInst->nbRequests_);
    std::shared_ptr<myTools::BitVector>  coveredList = std::make_shared<myTools::BitVector>(pInst->nbRequests_);*/

    std::bitset<MAX_SIZE>  zList;
    std::bitset<MAX_SIZE>  coveredList;
    for (auto & requestObj : zSolution_)
        zList.set(requestObj->taskIndex_, true);

    for (auto & vehicleObj : pInst->vehicles_) {
        if (!availableRoutes_[vehicleObj->vehicleID_].empty()) {
            coveredList = zList;
 //           std::shared_ptr<myTools::BitVector>  currentSolution = std::make_shared<myTools::BitVector>(pInst->nbRequests_);
            std::bitset<MAX_SIZE> currentSolution;
            for (auto & requestObj : vehicleObj->currentRoute_->routeRequests_) {
                coveredList.set(requestObj->taskIndex_, true);
                currentSolution.set(requestObj->taskIndex_, true);
            }
            for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]){
                if (((routeObj->column_ & coveredList)==routeObj->column_) && ((currentSolution & routeObj->column_) == currentSolution))
                    routeObj->isCompatible_ = true;
                else
                    routeObj->isCompatible_ = false;
            }
            /*std::stable_sort(availableRoutes_[vehicleObj->VehicleID_].begin(),
                             availableRoutes_[vehicleObj->VehicleID_].end(),
                             [](const PRoute &lhs, const PRoute &rhs){return lhs->score_ < rhs->score_;});*/
        }
    }
}
void ISUDAlgorithm::updateIncDegreesBit(PInstance &pInst) {

    for (auto & vehicleObj : pInst->vehicles_) {
        if (!availableRoutes_[vehicleObj->vehicleID_].empty()) {
            for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]){
                calcIncompatibilityBit(routeObj, pInst);
            }
        }
    }
}
void ISUDAlgorithm::updateRoutesIncDegree(int &vehicleID) {

    for (auto & routeObj : availableRoutes_[vehicleID]) {
 //       calcIncompatibility(routeObj);
        calcIncompatibilityFull(routeObj);
        if (routeObj->incompatibilityDegree_ > maxIncDegree_)
            maxIncDegree_  =routeObj->incompatibilityDegree_;
    }

    // sort the routes based on their incompatibility degree
    std::stable_sort(availableRoutes_[vehicleID].begin(),availableRoutes_[vehicleID].end(),[](const PRoute &lhs, const PRoute &rhs){
        return std::tie(lhs->incompatibilityDegree_, rhs->score_) < std::tie(rhs->incompatibilityDegree_, lhs->score_);
    });

    /*std::stable_sort(availableRoutes_[vehicleID].begin(),availableRoutes_[vehicleID].end(),[](const PRoute &lhs, const PRoute &rhs){
        return lhs->score_ > rhs->score_;});*/
}


void ISUDAlgorithm::updateReducedCosts(PInstance &pInst) {
    minReducedCost_ = INFINITY;
    maxReducedCost_ = INFINITY;
    for (auto & vehicleObj : pInst->vehicles_){
    //    vehicleObj->resetBestReducedCost();


        for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]){
            routeObj->reducedCost_ = routeObj->totalDelay_ - vehicleObj->dual_;
            for (auto & nodeObj: routeObj->routeNodes_){
                if (nodeObj->type_ == PICKUP){
                    routeObj->reducedCost_ -= nodeObj->related_Request_->dual_;
                }
            }
            /*if (routeObj->reducedCost_ < pInst->vehicles_[vehicleID]->bestReducedCost_) {
                pInst->vehicles_[vehicleID]->bestReducedCost_ = routeObj->reducedCost_;
                if (minReducedCost_ > routeObj->reducedCost_)
                    minReducedCost_ = routeObj->reducedCost_;
            }*/
            if (minReducedCost_ > routeObj->reducedCost_)
                minReducedCost_ = routeObj->reducedCost_;
            if (!routeObj->routeRequests_.empty())
                routeObj->score_ = routeObj->reducedCost_/routeObj->routeRequests_.size();
            else
                routeObj->score_ = 0;
        }
    }
    if (minReducedCost_ < 0)
        maxReducedCost_ = ((-0.5)*minReducedCost_);
}

void ISUDAlgorithm::solveISUD(PInstance &pInst, int epoch, InputPaths &inputPaths, double subProTime) {
    try {
        isudTime_->start();
        float tiLim = availableTime_;
        if (pInst->parameters_->initialStart_ == GREEDY_START) {
            routeSolution_.clear();
            zSolution_.clear();
            // it has been solved before in solver
            for (auto &vehicleObj: pInst->vehicles_) {
                routeSolution_.push_back(vehicleObj->currentRoute_);
            }
            setObjValue();
            GreedyObjValue_ = objValue_;
            std::cout << "Greedy Warm start: " << GreedyObjValue_ << std::endl;
        }
        setObjValue();
        double previousObj = objValue_;
        bool restartAlgorithm = true;
        bool isCPImproved;
        int CPCounter;

        // update reduced costs if needed only at the start of epoch, if we used penalties to create routes
        if ((pInst->parameters_->initialStart_ == PRE_SOLUTION) && (pInst->parameters_->initialDual_ == PENALTIES) &&
            (isudIter_ == 1)) {
            for (auto &requestObj: pInst->requests_)
                requestObj->dual_ = requestObj->CPDual_;
            for (auto &vehicleObj: pInst->vehicles_)
                vehicleObj->dual_ = vehicleObj->CPDual_;
        }

        RPBuildTime_->start();
        MIPReducedPro_->buildModel(pInst, zSolution_, routeSolution_, nbVehicles_);
        RPBuildTime_->stop();

        if (pInst->parameters_->solutionMode_ != ANYTIME) {
            (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, isudIter_, "initial");
            (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, isudIter_, "initial");
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
                availableTime_ = tiLim - isudTime_->dSinceStart().count();
                if (minReducedCost_ >= 0 || availableTime_ < 0) {
                    break;
                }
                setObjValue();
                solveRP_LPINT(pInst, 0, inputPaths);
                for (auto & routeObj : routeSolution_) {
                    pInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
                }
                if (pInst->parameters_->solutionMode_ != ANYTIME) {
                    (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, isudIter_, "LRP");
                    (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, isudIter_, "LRP");
                }

                RPIter_++;
                std::cout << "RP improve: " << objValue_ << std::endl;
//                if (epoch == 4)
                (*pLogIsudResultsStream_) << save_ISUDResults(epoch, "RP", (int) MIPReducedPro_->compRoutes_.size()-nbVehicles_,
                                                              isudTime_->dSinceStart().count(), subProTime,
                                                              pInst->calculateL1NormReq(), pInst->calculateL1NormVeh(),MIPReducedPro_->auxObjValue_);
                isudIter_++;
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
            availableTime_ = tiLim - isudTime_->dSinceStart().count();

            if (minReducedCost_ <= 0 && availableTime_ > 0) {
                CPBuildTime_->start();
                CompPro_->routesToAdd_.clear();
                CompPro_->buildModel(pInst, zSolution_, routeSolution_, nbVehicles_);
                CPBuildTime_->stop();
            }
            else {
                restartAlgorithm = false;
                isCPImproved = false;
            }

            while (isCPImproved) {
                isCPImproved = false;
                previousObj = objValue_;
                updateIncDegreesBit(pInst);
                CompPro_->routesToAdd_.clear();
                updateRoutesToAdd(false, pInst);
                availableTime_ = tiLim - isudTime_->dSinceStart().count();
                if (!CompPro_->routesToAdd_.empty() && availableTime_ > 0) {
                    CPBuildTime_->start();
                    CompPro_->updateModel(pInst, zSolution_, routeSolution_);
                    CPBuildTime_->stop();
                    CompPro_->solveCPModel(pInst, zSolution_, routeSolution_, inputPaths);
                    updateReducedCosts(pInst);
                    if (pInst->parameters_->solutionMode_ != ANYTIME) {
                        (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, isudIter_, "CP");
                        (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, isudIter_, "CP");
                    }
                    CPIter_++;
                    CPEpochSolveTime_ += CompPro_->solveTime_->dSinceStart().count();
                    setObjValue();
                    std::cout << "CP improve: " << objValue_ << std::endl;
//                    if (epoch == 4)
                    (*pLogIsudResultsStream_) << save_ISUDResults(epoch, "CP", (int) (CompPro_->IncRoute_.size()),
                                                                  isudTime_->dSinceStart().count(), subProTime,
                                                                  pInst->calculateL1NormReq(), pInst->calculateL1NormVeh(),0.0);

                    if (CompPro_->status_ == FRACTIONAL) {
                        CPFails_++;
                        std::cout << "# The Algorithm needs modification to find integer direction" << std::endl;
                        if (pInst->parameters_->useZoom_) {
                            isudMIPTime_->start();
                            availableTime_ = tiLim - isudTime_->dSinceStart().count();
                            solveRP_LPINT(pInst, 1, inputPaths);
                            RPIter_++;
                            isudMIPTime_->stop();
                            if (pInst->parameters_->solutionMode_ != ANYTIME) {
                                (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, isudIter_, "zoom");
                                (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, isudIter_, "zoom");
                            }
                            if (previousObj > objValue_) {
                                previousObj = objValue_;
//                                if (epoch == 4)
                                (*pLogIsudResultsStream_)
                                        << save_ISUDResults(epoch, "ZOOM", (int) MIPReducedPro_->compRoutes_.size()-nbVehicles_,
                                                            isudTime_->dSinceStart().count(), subProTime,
                                                            pInst->calculateL1NormReq(), pInst->calculateL1NormVeh(), MIPReducedPro_->auxObjValue_);
                                isudIter_++;
                                isCPImproved = true;
                                updateReducedCosts(pInst);
                                std::cout << "ZOOM improve: " << objValue_ << std::endl;
                        //        break;
                            }
                            else {
                                if (CPCounter == 0) {
                                    restartAlgorithm = false;
                                    break;
                                }
                            }
                        }
                        /*else if (pInst->parameters_->useMultiStage_) {
                            if (cpIncDegree_ < maxIncDegree_)
                                cpIncDegree_++;
                            else {
                                restartAlgorithm = false;
                                break;
                            }
                        }*/
                        else {
  //                          if (CPCounter == 0) {
                                restartAlgorithm = false;
                                break;
   //                         }
                        }
                    }
                    else if (CompPro_->status_ == POSITIVE_VALUE) {
                        if (pInst->parameters_->useMultiStage_)
                            if (cpIncDegree_ < maxIncDegree_)
                                cpIncDegree_++;
                            else {
                                isCPImproved = false;
                                if (CPCounter == 0) {
                                    restartAlgorithm = false;
                                    break;
                                }
                            }
                        else {
                            isCPImproved = false;
                            if (CPCounter == 0) {
                                restartAlgorithm = false;
                                break;
                            }
                        }
                    }
                    else if (CompPro_->status_ == INFEASIBLE) {
                        if (pInst->parameters_->useMultiStage_)
                            if (cpIncDegree_ < maxIncDegree_)
                                cpIncDegree_++;
                            else {
                                isCPImproved = false;
                                if (CPCounter == 0) {
                                    restartAlgorithm = false;
                                    break;
                                }
                            }
                        else {
                            isCPImproved = false;
                            if (CPCounter == 0) {
                                restartAlgorithm = false;
                                break;
                            }
                        }
                    }
                    else {
                        for (auto & routeObj : routeSolution_) {
                            pInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
                        }
                        CPSuccess_++;
                        previousObj = objValue_;
                        isudIter_++;
                        isCPImproved = true;
                        CPCounter++;
                        availableTime_ = tiLim - isudTime_->dSinceStart().count();
                        if (availableTime_ <= 0) {
                            restartAlgorithm = false;
                            break;
                        }
                    }
                } else
                    restartAlgorithm = false;
            }
            availableTime_ = tiLim - isudTime_->dSinceStart().count();
            if ((minReducedCost_ > 0) || (availableTime_ <= 0))
                restartAlgorithm = false;
            for (auto &routeObj: CompPro_->IncRoute_)
                routeObj->cpAdded_ = false;
            CompPro_.reset();
            CompPro_ = std::make_shared<ComplementPro>();

            CPTime_->stop();
        }
        /*for (auto & routeObj: MIPReducedPro_->compRoutes_)
            routeObj->mpAdded_ = false;*/
        MIPReducedPro_.reset();
        MIPReducedPro_ = std::make_shared<ZoomReducedProblem>();
        std::cout << "# number of unserved requests: " << zSolution_.size() << std::endl;
        std::cout << "# Time spent on ISUD iteration  = " << isudTime_->dSinceStart().count() << " (seconds)"
                  << std::endl;
        for (auto &requestObj: zSolution_)
            std::cout << "request " << requestObj->getRequestId() << " : " << requestObj->penalty_ << std::endl;
 //       if (epoch == 4)
        (*pLogIsudResultsStream_)
                << save_ISUDResults(epoch, "ISUD", nbRoutes_, isudTime_->dSinceStart().count(), subProTime,
                                    pInst->calculateL1NormReq(), pInst->calculateL1NormVeh(), 0.0);
        isudTime_->stop();
    }
    catch (const std::exception& e){
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e.what() << std::endl;
    }
}

void ISUDAlgorithm::solveMP_MIP(PInstance &pInst, int epoch, InputPaths &inputPaths, double subProTime) {
    isudTime_->start();
    RPTime_->start();
    int tilim = availableTime_;
    double previousObj = objValue_;

    // update reduced costs if needed only at the start of epoch, if we used penalties to create routes
    if  ((pInst->parameters_->initialStart_ == PRE_SOLUTION)&&(pInst->parameters_->initialDual_ == PENALTIES) && (isudIter_ == 1)){
        for(auto & requestObj: pInst->requests_)
            requestObj->dual_ = requestObj->CPDual_;
        for(auto & vehicleObj : pInst->vehicles_)
            vehicleObj->dual_ = vehicleObj->CPDual_;
    }
    RPBuildTime_->start();
    MasterPro_->buildModelMP(pInst, zSolution_, routeSolution_, nbVehicles_);
    RPBuildTime_->stop();

    // save initial duals
    if (pInst->parameters_->solutionMode_ != ANYTIME) {
        (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, isudIter_, "initial");
        (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, isudIter_, "initial");
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
        availableTime_ = (int) (tilim - isudTime_->dSinceStart().count());
        if (availableTime_ < 1)
            break;
        int proSize = MasterPro_->compRoutes_.size();
        solveMP_INT(pInst, inputPaths);
        std::cout << "MP improve: " << objValue_ << std::endl;
        MPIter_++;
//        if (epoch == 4)
        (*pLogIsudResultsStream_) << save_ISUDResults(epoch, "RMP", (int) MasterPro_->compRoutes_.size()-nbVehicles_,
                                                      isudTime_->dSinceStart().count(), subProTime,
                                                      pInst->calculateL1NormReq(), pInst->calculateL1NormVeh(),
                                                      MasterPro_->auxObjValue_);
        // save initial duals
        isudIter_++;
        if (pInst->parameters_->solutionMode_ != ANYTIME) {
            (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, isudIter_, "RMP");
            (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, isudIter_, "RMP");
        }
        setObjValue();
        if (previousObj > objValue_) {
            previousObj = objValue_;
        }
        else
//        else if (proSize == MasterPro_->compRoutes_.size())
            break;
    }

    setObjValue();
    /*(*pLogIsudResultsStream_) << save_ISUDResults(epoch, "MP", (int) MasterPro_->compRoutes_.size(),
                                                  isudTime_->dSinceStart().count(), subProTime,
                                                  pInst->calculateL1NormReq(), pInst->calculateL1NormVeh(),
                                                  MasterPro_->auxObjValue_);*/
    /*if (epoch == 3) {
        MasterPro_->solveModelLPInt(pInst, zSolution_, routeSolution_, inputPaths,
                                    availableTime_, objValue_);
    }*/
    for (auto &routeObj: MasterPro_->compRoutes_)
        routeObj->mpAdded_ = false;

    std::cout << "# number of unserved requests: " << zSolution_.size() << std::endl;
    std::cout << "# Time spent on ISUD iteration  = " << isudTime_->dSinceStart().count() << " (seconds)"
              << std::endl;
    for (auto &requestObj: zSolution_)
        std::cout << "request " << requestObj->getRequestId() << " : " << requestObj->penalty_ << std::endl;

    MasterPro_.reset();
    MasterPro_ = std::make_shared<MasterPro>();
    for (auto & routeObj : routeSolution_) {
        pInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
    }

    RPTime_->stop();
    isudTime_->stop();
}
void ISUDAlgorithm::solveMP_MIPCP(PInstance &pInst, int epoch, InputPaths &inputPaths, double subProTime) {
    isudTime_->start();
    RPTime_->start();
    int tilim = availableTime_;
    if (pInst->parameters_->initialStart_ == GREEDY_START) {
        routeSolution_.clear();
        zSolution_.clear();
        // it has been solved before in solver
        for (auto &vehicleObj: pInst->vehicles_) {
            routeSolution_.push_back(vehicleObj->currentRoute_);
        }
        setObjValue();
        GreedyObjValue_ = objValue_;
        std::cout << "Greedy Warm start: " << GreedyObjValue_ << std::endl;
    }
    setObjValue();
    double previousObj = objValue_;
    bool restartAlgorithm = true;
    bool isCPImproved;
    int CPCounter;

    // update reduced costs if needed only at the start of epoch, if we used penalties to create routes
    if  ((pInst->parameters_->initialStart_ == PRE_SOLUTION)&&(pInst->parameters_->initialDual_ == PENALTIES) && (isudIter_ == 1)){
        for(auto & requestObj: pInst->requests_)
            requestObj->dual_ = requestObj->CPDual_;
        for(auto & vehicleObj : pInst->vehicles_)
            vehicleObj->dual_ = vehicleObj->CPDual_;
    }
    RPBuildTime_->start();
    MasterPro_->buildModelMP(pInst, zSolution_, routeSolution_, nbVehicles_);
    RPBuildTime_->stop();

    // save initial duals
    if (pInst->parameters_->solutionMode_ != ANYTIME) {
        (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, isudIter_, "initial");
        (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, isudIter_, "initial");
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
        availableTime_ = (int) (tilim - isudTime_->dSinceStart().count());
        if (availableTime_ < 1)
            break;
        int proSize = MasterPro_->compRoutes_.size();
        solveMP_INT(pInst, inputPaths);
        for (auto & routeObj : routeSolution_) {
            pInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
        }
        std::cout << "MP improve: " << objValue_ << std::endl;
        MPIter_++;
//        if (epoch == 4)
        (*pLogIsudResultsStream_) << save_ISUDResults(epoch, "RMP", (int) MasterPro_->compRoutes_.size()-nbVehicles_,
                                                      isudTime_->dSinceStart().count(), subProTime,
                                                      pInst->calculateL1NormReq(), pInst->calculateL1NormVeh(),
                                                      MasterPro_->auxObjValue_);
        // save initial duals
        isudIter_++;
        if (pInst->parameters_->solutionMode_ != ANYTIME) {
            (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, isudIter_, "RMP");
            (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, isudIter_, "RMP");
        }
        setObjValue();
        if (previousObj > objValue_) {
            previousObj = objValue_;
        } else
            break;
    }

    setObjValue();
    RPTime_->stop();

    // update reduced costs
    /************************************************************************************************/
    //                                     COMPLEMENTARY PROBLEM
    /************************************************************************************************/
    CPCounter = 0;
    updateReducedCosts(pInst);
    // if objective improves, the CP is build
    CPTime_->start();
    availableTime_ = tilim - isudTime_->dSinceStart().count();
    isCPImproved = true;
    if (minReducedCost_ <= 0 && availableTime_ > 0) {
        CPBuildTime_->start();
        CompPro_->routesToAdd_.clear();
        CompPro_->buildModel(pInst, zSolution_, routeSolution_, nbVehicles_);
        CPBuildTime_->stop();
    }
    else {
        restartAlgorithm = false;
        isCPImproved = false;
    }

    while (isCPImproved) {
        isCPImproved = false;
        previousObj = objValue_;
        updateIncDegreesBit(pInst);
        CompPro_->routesToAdd_.clear();
        updateRoutesToAdd(false, pInst);
        availableTime_ = tilim - isudTime_->dSinceStart().count();
        if (!CompPro_->routesToAdd_.empty() && availableTime_ > 0) {
            CPBuildTime_->start();
            CompPro_->updateModel(pInst, zSolution_, routeSolution_);
            CPBuildTime_->stop();
            CompPro_->solveCPModel(pInst, zSolution_, routeSolution_, inputPaths);
            updateReducedCosts(pInst);
            if (pInst->parameters_->solutionMode_ != ANYTIME) {
                (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, isudIter_, "CP");
                (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, isudIter_, "CP");
            }
            CPIter_++;
            CPEpochSolveTime_ += CompPro_->solveTime_->dSinceStart().count();
            setObjValue();
            std::cout << "CP improve: " << objValue_ << std::endl;

            if (CompPro_->status_ == FRACTIONAL) {
                CPFails_++;
                std::cout << "# The Algorithm needs modification to find integer direction" << std::endl;
                if (pInst->parameters_->useZoom_) {
                    isudMIPTime_->start();

                    for (auto & routeObj : CompPro_->IncRoute_) {
                        if (!routeObj->mpAdded_)
                            MasterPro_->routesToAdd_.push_back(routeObj);
                    }

                    availableTime_ = tilim - isudTime_->dSinceStart().count();
                    RPBuildTime_->start();
                    MasterPro_->updateModel(pInst);
                    RPBuildTime_->stop();
                    MasterPro_->solveModelIntAux(pInst, zSolution_, routeSolution_, inputPaths,
                                                 availableTime_, objValue_);
                    RPEpochSolveTime_ += MasterPro_->solveTime_->dSinceStart().count();
                    objValue_ = MasterPro_->objValue_;

                    MPIter_++;
                    isudMIPTime_->stop();
                    if (pInst->parameters_->solutionMode_ != ANYTIME) {
                        (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, isudIter_, "zoom");
                        (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, isudIter_, "zoom");
                    }
                    if (previousObj > objValue_) {
                        previousObj = objValue_;
 //                       if (epoch == 4)
                        (*pLogIsudResultsStream_)
                                << save_ISUDResults(epoch, "ZOOM", (int) MasterPro_->compRoutes_.size()-nbVehicles_,
                                                    isudTime_->dSinceStart().count(), subProTime,
                                                    pInst->calculateL1NormReq(), pInst->calculateL1NormVeh(), MasterPro_->auxObjValue_);
                        isudIter_++;
                        isCPImproved = true;
                        updateReducedCosts(pInst);
                        /*CompPro_->routesToAdd_.clear();
                        updateIncDegrees(pInst);
                        updateRoutesToAdd(maxIncDegree_, pInst);*/
                        std::cout << "ZOOM improve: " << objValue_ << std::endl;
                        break;
                    }
                    else {
                        if (CPCounter == 0) {
                            restartAlgorithm = false;
                            break;
                        }
                    }
                }
                else if (pInst->parameters_->useMultiStage_) {
                    if (cpIncDegree_ < maxIncDegree_)
                        cpIncDegree_++;
                    else {
                        restartAlgorithm = false;
                        break;
                    }
                }
                else {
                    //                          if (CPCounter == 0) {
                    restartAlgorithm = false;
                    break;
                    //                         }
                }
            }
            else if (CompPro_->status_ == POSITIVE_VALUE) {
                if (pInst->parameters_->useMultiStage_)
                    if (cpIncDegree_ < maxIncDegree_)
                        cpIncDegree_++;
                    else {
                        isCPImproved = false;
                        if (CPCounter == 0) {
                            restartAlgorithm = false;
                            break;
                        }
                    }
                else {
                    isCPImproved = false;
                    if (CPCounter == 0) {
                        restartAlgorithm = false;
                        break;
                    }
                }
            }
            else if (CompPro_->status_ == INFEASIBLE) {
                if (pInst->parameters_->useMultiStage_)
                    if (cpIncDegree_ < maxIncDegree_)
                        cpIncDegree_++;
                    else {
                        isCPImproved = false;
                        if (CPCounter == 0) {
                            restartAlgorithm = false;
                            break;
                        }
                    }
                else {
                    isCPImproved = false;
                    if (CPCounter == 0) {
                        restartAlgorithm = false;
                        break;
                    }
                }
            }
            else {
                CPSuccess_++;
                for (auto & routeObj : routeSolution_) {
                    pInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
                }
 //               if (epoch == 4)
                (*pLogIsudResultsStream_) << save_ISUDResults(epoch, "CP", (int) (CompPro_->IncRoute_.size()),
                                                              isudTime_->dSinceStart().count(), subProTime,
                                                              pInst->calculateL1NormReq(), pInst->calculateL1NormVeh(),0.0);
                previousObj = objValue_;
                isudIter_++;
                isCPImproved = true;
                CPCounter++;
                availableTime_ = tilim - isudTime_->dSinceStart().count();
                if (availableTime_ <= 0) {
                    restartAlgorithm = false;
                    break;
                }
            }
        } else
            restartAlgorithm = false;
    }

    CompPro_.reset();
    CompPro_ = std::make_shared<ComplementPro>();

    CPTime_->stop();
    std::cout << "# number of unserved requests: " << zSolution_.size() << std::endl;
    std::cout << "# Time spent on ISUD iteration  = " << isudTime_->dSinceStart().count() << " (seconds)"
              << std::endl;
    for (auto &requestObj: zSolution_)
        std::cout << "request " << requestObj->getRequestId() << " : " << requestObj->penalty_ << std::endl;

    MasterPro_.reset();
    MasterPro_ = std::make_shared<MasterPro>();


    isudTime_->stop();
}
void ISUDAlgorithm::solveMP_CG(PInstance &pInst, int epoch, InputPaths &inputPaths, double subProTime) {
    isudTime_->start();
    RPTime_->start();
    int tilim = availableTime_;
    double previousObj = objValue_;

    // update reduced costs if needed only at the start of epoch, if we used penalties to create routes
    if  ((pInst->parameters_->initialStart_ == PRE_SOLUTION)&&(pInst->parameters_->initialDual_ == PENALTIES) && (isudIter_ == 1)){
        for(auto & requestObj: pInst->requests_)
            requestObj->dual_ = requestObj->CPDual_;
        for(auto & vehicleObj : pInst->vehicles_)
            vehicleObj->dual_ = vehicleObj->CPDual_;
    }
    RPBuildTime_->start();
    MasterPro_->buildModelMP(pInst, zSolution_, routeSolution_, nbVehicles_);
    RPBuildTime_->stop();

    // save initial duals
    if (pInst->parameters_->solutionMode_ != ANYTIME) {
        (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, isudIter_, "initial");
        (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, isudIter_, "initial");
    }
    /************************************************************************************************/
    //                                     MASTER PROBLEM
    /************************************************************************************************/
    availableTime_ = (int) (tilim - isudTime_->dSinceStart().count());
    if (availableTime_ > 1) {
        double lpObj = previousObj;
        // solve RP with MIP solver
        while (true) {
            updateReducedCosts(pInst);
            if (minReducedCost_ >= 0) {
                break;
            }
            solveMP_LP(pInst, inputPaths);
            std::cout << "LMP improve: " << objValue_ << std::endl;
            LPIter_++;
  //          if (epoch == 4)
            (*pLogIsudResultsStream_) << save_ISUDResults(epoch, "LMP", MasterPro_->compRoutes_.size()-nbVehicles_,
                                                          isudTime_->dSinceStart().count(), subProTime,
                                                          pInst->calculateL1NormReq(), pInst->calculateL1NormVeh(),
                                                          0.0);
            isudIter_++;
            if (pInst->parameters_->solutionMode_ != ANYTIME) {
                (*pLogIterReqDualStream_) << pInst->saveReqDuals(epoch, isudIter_, "LMP");
                (*pLogIterVehDualStream_) << pInst->saveVehDuals(epoch, isudIter_, "LMP");
            }
            if (lpObj > objValue_) {
                lpObj = objValue_;
            } else
                break;
        }


        availableTime_ = tilim - isudTime_->dSinceStart().count();
        if (availableTime_ < 1)
            availableTime_ = 2;
        // solve the model in Integer mode
        MasterPro_->solveModelInt(pInst, zSolution_, routeSolution_, inputPaths, availableTime_, previousObj);
        RPEpochSolveTime_ += MasterPro_->solveTime_->dSinceStart().count();
        setObjValue();

        std::cout << "MP improve: " << objValue_ << std::endl;

        MPIter_++;
//        if (epoch == 4)
        (*pLogIsudResultsStream_) << save_ISUDResults(epoch, "CG", (int) MasterPro_->compRoutes_.size()-nbVehicles_,
                                                      isudTime_->dSinceStart().count(), subProTime,
                                                      pInst->calculateL1NormReq(), pInst->calculateL1NormVeh(),
                                                      0.0);
        isudIter_++;

        for (auto &routeObj: MasterPro_->compRoutes_)
            routeObj->mpAdded_ = false;


        std::cout << "# number of unserved requests: " << zSolution_.size() << std::endl;
        std::cout << "# Time spent on ISUD iteration  = " << isudTime_->dSinceStart().count() << " (seconds)"
                  << std::endl;
        for (auto &requestObj: zSolution_)
            std::cout << "request " << requestObj->getRequestId() << " : " << requestObj->penalty_ << std::endl;
    }

    MasterPro_.reset();
    MasterPro_ = std::make_shared<MasterPro>();
    for (auto & routeObj : routeSolution_) {
        pInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
    }

    RPTime_->stop();
    isudTime_->stop();
}
void ISUDAlgorithm::solveRP_LPINT(PInstance &pInst, int compDegree, InputPaths &inputPaths) {
//    isudMIPTime_->start();
    // improve by solving the Reduced problem
    MIPReducedPro_->routesToAdd_.clear();
    /*RPBuildTime_->start();
    MIPReducedPro_->buildModel(pInst, zSolution_, routeSolution_);
    RPBuildTime_->stop();*/
    if (compDegree == 0) {
        updateIncDegreesBit(pInst);
        updateRoutesToAdd(true, pInst);
    }
    else
    {
        for (auto & routeObj : CompPro_->IncRoute_) {
            if (!routeObj->mpAdded_)
                MIPReducedPro_->routesToAdd_.push_back(routeObj);
        }
        /*for (auto & vehicleObj : pInst->vehicles_) {
            for (auto & routeObj : availableRoutes_[vehicleObj->VehicleID_]) {
                if (routeObj->incompatibilityDegree_ <= pInst->parameters_->MIP_maxIncDegree_ && !routeObj->mpAdded_)
                    MIPReducedPro_->routesToAdd_.push_back(routeObj);
            }
        }*/
    }

    if (!MIPReducedPro_->routesToAdd_.empty()){
        /*for (auto & routeObj : routeSolution_){
            if (!routeObj->mpAdded_)
                MIPReducedPro_->routesToAdd_.push_back(routeObj);
        }*/
        for (int v = 0; v < pInst->nbVehicles_; ++v) {
            if (pInst->vehicles_[v]->vehicleIndex_>-1 && !pInst->vehicles_[v]->emptyRoute_->mpAdded_ &&
            !pInst->vehicles_[v]->currentRoute_->routeRequests_.empty())
                MIPReducedPro_->routesToAdd_.push_back(pInst->vehicles_[v]->emptyRoute_);
        }
        RPBuildTime_->start();
        MIPReducedPro_->updateModel(pInst, CompPro_->fractionalZ_);
        RPBuildTime_->stop();
        MIPReducedPro_->solveModelLPInt(pInst, zSolution_, routeSolution_,inputPaths,
                                       availableTime_, objValue_);
        RPEpochSolveTime_ += MIPReducedPro_->solveTime_->dSinceStart().count();
        setObjValue();
    }
//    isudMIPTime_->stop();
}

void ISUDAlgorithm::solveMP_LP(PInstance &pInst, InputPaths &inputPaths) {
    MasterPro_->routesToAdd_.clear();

    // select routes with negative reduced costs
    for (auto & vehicleObj : pInst->vehicles_) {
        int nbAdded = 0;
        std::stable_sort(availableRoutes_[vehicleObj->vehicleID_].begin(),availableRoutes_[vehicleObj->vehicleID_].end(),[](const PRoute &lhs, const PRoute &rhs){
            return lhs->score_ < rhs->score_;});
        std::bitset<MAX_SIZE> coveredList;
        for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]) {
            if (routeObj->reducedCost_ < 0 && !routeObj->mpAdded_) {
                MasterPro_->routesToAdd_.push_back(routeObj);
                nbAdded++;
            }
            if (nbAdded > 60)
                break;
        }
        /*for (auto & routeObj : availableRoutes_[vehicleObj->VehicleID_]) {
            if (routeObj->reducedCost_ <= 0 && !routeObj->mpAdded_) {
                if ((routeObj->column_ | coveredList) != coveredList) {
                    MasterPro_->routesToAdd_.push_back(routeObj);
                    nbAdded++;
                    coveredList = coveredList | routeObj->column_;
                }
            }
            if (((vehicleObj->graphRequests_ & coveredList) == vehicleObj->graphRequests_) && (nbAdded > 80))
                break;
        }*/
    }

    if (!MasterPro_->routesToAdd_.empty()){
        for (int v = 0; v < pInst->nbVehicles_; ++v) {
            if (pInst->vehicles_[v]->vehicleIndex_>-1 && !pInst->vehicles_[v]->emptyRoute_->mpAdded_)
                MasterPro_->routesToAdd_.push_back(pInst->vehicles_[v]->emptyRoute_);
        }
        RPBuildTime_->start();
        MasterPro_->updateModel(pInst);
        RPBuildTime_->stop();
        MasterPro_->solveModelLP(pInst, inputPaths);
        RPEpochSolveTime_ += MasterPro_->solveTime_->dSinceStart().count();
        objValue_ = MasterPro_->objValue_;
    }
}

void ISUDAlgorithm::solveMP_INT(PInstance &pInst, InputPaths &inputPaths) {
    MasterPro_->routesToAdd_.clear();

    // select routes with negative reduced costs
    for (auto & vehicleObj : pInst->vehicles_) {
        int nbAdded = 0;
        std::stable_sort(availableRoutes_[vehicleObj->vehicleID_].begin(),availableRoutes_[vehicleObj->vehicleID_].end(),[](const PRoute &lhs, const PRoute &rhs){
            return lhs->score_ < rhs->score_;});
        std::bitset<MAX_SIZE> coveredList;
        for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]) {
            if (routeObj->reducedCost_ < 0 && !routeObj->mpAdded_) {
                MasterPro_->routesToAdd_.push_back(routeObj);
                nbAdded++;
            }
            if (nbAdded > 60)
                break;
        }

        /*for (auto & routeObj : availableRoutes_[vehicleObj->VehicleID_]) {
            if (routeObj->reducedCost_ <= 0 && !routeObj->mpAdded_) {
                if ((routeObj->column_ | coveredList) != coveredList) {
                    MasterPro_->routesToAdd_.push_back(routeObj);
                    nbAdded++;
                    coveredList = coveredList | routeObj->column_;
                }
            }
            if (((vehicleObj->graphRequests_ & coveredList) == vehicleObj->graphRequests_) && (nbAdded > 80)) {
                break;
            }
        }*/
    }

    if (!MasterPro_->routesToAdd_.empty()){
        for (int v = 0; v < pInst->nbVehicles_; ++v) {
            if (pInst->vehicles_[v]->vehicleIndex_>-1 && !pInst->vehicles_[v]->emptyRoute_->mpAdded_)
                MasterPro_->routesToAdd_.push_back(pInst->vehicles_[v]->emptyRoute_);
        }
        RPBuildTime_->start();
        MasterPro_->updateModel(pInst);
        RPBuildTime_->stop();
//        MasterPro_->solveModelLP(pInst, inputPaths);
        MasterPro_->solveModelIntAux(pInst, zSolution_, routeSolution_, inputPaths,
                                     availableTime_, objValue_);
        RPEpochSolveTime_ += MasterPro_->solveTime_->dSinceStart().count();
        objValue_ = MasterPro_->objValue_;
    }
}


// Display function
std::string ISUDAlgorithm::toString() const {

    std::stringstream repStr;
    repStr << "#" << std::endl;
    repStr << std::left << std::fixed << std::setprecision(2);
    repStr << "# TOTAL OBJECTIVE (WAITING TIMES + PENALTIES) = " << objValue_ << std::endl;
    repStr << "# TOTAL WAITING TIMES                         = " << totalWaitTime_ << std::endl;
    repStr << "# NUMBER OF UNSERVED REQUESTS                 = " << zSolution_.size() << std::endl;
    repStr << "#" << std::endl;

    repStr << "# TIME SPENT ON ISUD IMPROVEMENT              = " << isudTime_->dSinceStart().count() << " (s)" << std::endl;
    repStr << "# TIME SPENT ON RP IMPROVEMENT                = " << RPTime_->dSinceStart().count() << " (s)" << std::endl;
    repStr << "# TIME SPENT ON CP IMPROVEMENT                = " << CPTime_->dSinceStart().count() << " (s)" << std::endl;
    repStr << "# TIME SPENT ON MIP ISUD                      = " << isudMIPTime_->dSinceStart().count() << " (s)" << std::endl;

    /*for (auto & routeObj : routeSolution_) {
        repStr << routeObj->toString();
    }*/
    return repStr.str();
}

std::string ISUDAlgorithm::toStringTimersTotal() const {
    std::stringstream repStr;
    repStr << std::left << std::fixed << std::setprecision(2);
    repStr << "#" << std::endl;
    repStr << "# -------------------   TOTAL ISUD RUN TIMES   -------------------" << std::endl;
    repStr << "#" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON ISUD IMPROVEMENT" << " = " << isudTime_->dSinceInit().count() << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON RP IMPROVEMENT" << " = " << RPTime_->dSinceInit().count() << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON CP IMPROVEMENT" << " = " << CPTime_->dSinceInit().count() << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON MIP ISUD" << " = " << isudMIPTime_->dSinceInit().count() << " (s)" << std::endl;

    return repStr.str();
}

std::string ISUDAlgorithm::toStringTimersAvg(int epoch) const {
    std::stringstream repStr;
    repStr << std::left << std::fixed << std::setprecision(2);
    repStr << "#" << std::endl;
    repStr << "# -------------   AVERAGE ISUD RUN TIMES PER EPOCH   -------------" << std::endl;
    repStr << "#" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON ISUD IMPROVEMENT" << " = " << isudTime_->dSinceInit().count()/epoch << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON RP IMPROVEMENT" << " = " << RPTime_->dSinceInit().count()/epoch << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON CP IMPROVEMENT" << " = " << CPTime_->dSinceInit().count()/epoch << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON MIP ISUD" << " = " << isudMIPTime_->dSinceInit().count()/epoch << " (s)" << std::endl;

    return repStr.str();
}

void ISUDAlgorithm::updateRoutesToAdd(bool compatible, PInstance &pInst) {
//    pInst->sortVehicles(BEST_REDUCE_COST);
    for (auto & vehicleObj : pInst->vehicles_) {
        std::stable_sort(availableRoutes_[vehicleObj->vehicleID_].begin(),
                         availableRoutes_[vehicleObj->vehicleID_].end(),
                         [](const PRoute &lhs, const PRoute &rhs){return lhs->score_ < rhs->score_;});
        int numAdded = 0;
        std::bitset<MAX_SIZE>  coveredList;
        if (compatible) {
            for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]) {
                if ((routeObj->isCompatible_)&&(routeObj->reducedCost_ <= 0) && !routeObj->mpAdded_) {
                    MIPReducedPro_->routesToAdd_.push_back(routeObj);
                    numAdded++;
                }
                if (numAdded > 60)
                    break;
            }

            /*for (auto & routeObj : availableRoutes_[vehicleObj->VehicleID_]) {
                if ((routeObj->isCompatible_) && (routeObj->reducedCost_ <= 0) && !routeObj->mpAdded_) {
                    if ((routeObj->column_ | coveredList) != coveredList) {
                        MIPReducedPro_->routesToAdd_.push_back(routeObj);
                        numAdded++;
                        coveredList = coveredList | routeObj->column_;
                    }
                }
                if (((vehicleObj->graphRequests_ & coveredList) == vehicleObj->graphRequests_) && (numAdded > 80))
                    break;
            }*/
        }
        else {
            for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]) {
                if ((!routeObj->isCompatible_)&& !routeObj->routeRequests_.empty() && !routeObj->cpAdded_) {
                    CompPro_->routesToAdd_.push_back(routeObj);
                    numAdded++;
                }
                if (numAdded > 60)
                    break;
            }

            /*for (auto & routeObj : availableRoutes_[vehicleObj->VehicleID_]) {
                if (!routeObj->routeRequests_.empty() && !routeObj->mpAdded_) {
 //                   if ((routeObj->column_ & coveredList) == 0) {
                    if ((routeObj->column_ | coveredList) != coveredList) {
                        CompPro_->routesToAdd_.push_back(routeObj);
                        numAdded++;
                        coveredList = coveredList | routeObj->column_;
                    }
                }
 //               if (numAdded > 80)
                if (((vehicleObj->graphRequests_ & coveredList) == vehicleObj->graphRequests_) || (numAdded > 80))
                    break;
            }*/
        }
    }
//    pInst->restVehicleOrder();
}

void ISUDAlgorithm::updateRoutesToAddZoom(PInstance &pInst) {
    // add compatible columns to the current solution
    for (auto & vehicleObj : pInst->vehicles_) {
        for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]) {
            MIPReducedPro_->routesToAdd_.push_back(routeObj);
  //          ReducedPro_->routesToAdd_.push_back(routeObj);
            /*bool routeIsAdded = false;
            for (auto & fracRouteObj: CompPro_->fractionalRoutes_) {
                if (fracRouteObj->getRouteId() == routeObj->getRouteId()) {
                    routeIsAdded = true;
                    break;
                }
            }
            if (!routeIsAdded) {
                if ((routeObj != vehicleObj->currentRoute_)&& (routeObj->reducedCost_ < -0.001)) {
                    if (routeObj->incompatibilityDegree_ == 0) {
                        ReducedPro_->routesToAdd_.push_back(routeObj);
                    }
                    else {
                        // add compatible columns with fractional routes
                        for (auto & fracRouteObj: CompPro_->fractionalRoutes_) {
                            if (isCompatible(fracRouteObj, routeObj, ReducedPro_->requestToOrder_)) {
                                ReducedPro_->routesToAdd_.push_back(routeObj);
                                routeIsAdded = true;
                                break;
                            }
                        }

                        // add compatible columns with fractional requests
                        if (!routeIsAdded) {
                            for (auto &requestObj: CompPro_->fractionalZ_) {
                                for (auto &requestID: routeObj->routeRequests_) {
                                    if (requestObj->getRequestId() == requestID) {
                                        ReducedPro_->routesToAdd_.push_back(routeObj);
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
            }*/


        }
    }
}


// function to save the reduced costs and incompatibility degree of the created routes
void ISUDAlgorithm::save_IncDegree_RDCost(InputPaths &inputPaths, int epoch, int isudIter) {
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


std::string ISUDAlgorithm::save_ISUDResults(int epoch, const std::string& model, int nbColumns, float reachTime,
                                            double subProTime, double dualNormReq, double dualNormVeh, double auxObj) const {
    std::stringstream repStr;

    repStr << epoch << ",";
    repStr << isudIter_ << ",";
    repStr << SPIter_ << ",";
//    repStr << dualNormVeh << ",";
    repStr << nbRoutes_ << ",";
    repStr << nbColumns << ",";
    repStr << model << ",";
    repStr << objValue_ << ",";
    repStr << reachTime << ",";
    repStr << subProTime << ",";
    repStr << auxObj << "\n";
    return repStr.str();
}










/*void ISUDAlgorithm::updatePatterns(PInstance &pInst) {
    for (auto & routeObj: generatedRoutes_){
        routeObj.second->createPattern(pInst->requestToOrder_);
    }
}*/

/*void ISUDAlgorithm::updateFullPattern() {
    for (auto & routeObj: generatedRoutes_){
        routeObj.second->createFullPattern(incRequestToOrder_, incVehicleToOrder_);
    }
}*/





















