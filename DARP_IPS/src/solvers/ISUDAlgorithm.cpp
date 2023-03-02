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
    objValue_ = 0;
    isudTime_ = new myTools::Timer(); isudTime_->init();
    RPTime_ = new myTools::Timer(); RPTime_->init();
    CPTime_ = new myTools::Timer(); CPTime_->init();

    RPBuildTime_ = new myTools::Timer(); RPBuildTime_->init();
    CPBuildTime_ = new myTools::Timer(); CPBuildTime_->init();

    isudMIPTime_ = new myTools::Timer(); isudMIPTime_->init();
    isudIter_ = 0;
    nbRoutes_ = 0;
    nbCoveredTasks_ = 0;
    cpIncDegree_ = 2;
    GreedyObjValue_ = 0;
    maxIncDegree_ = 0;
    minReducedCost_ = INFINITY;
    maxReducedCost_ = INFINITY;

    pLogIsudResultsStream_ = new Tools::LogOutput(inputPaths.getOutputEpochResults());
    (*pLogIsudResultsStream_) << "Epoch, ISUDIter, TotalGenColumns, nbColumns, Model, ObjectiveValue" << std::endl;

    pLogIterSolutionStream_ = new Tools::LogOutput(inputPaths.getOutputEpochIsud());
    (*pLogIterSolutionStream_) << "Epoch, ISUDIter,VehicleID,NodeID,RequestTime,ReachTime,NodeType,LocationID,RouteID" << std::endl;
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
    delete pLogIsudResultsStream_;
    delete pLogIterSolutionStream_;
}


void ISUDAlgorithm::setObjValue() {
    objValue_ = 0.0;
    for (auto & routeObj : routeSolution_) {
        objValue_ += routeObj->totalDelay_;
    }
    for (auto & zRequest : zSolution_) {
        objValue_+= zRequest->penalty_;
    }
}

// this function create initial routes serving only one request and fill zSolution_ with available requests
// Reduced problem is also solved to initialized dual costs
void ISUDAlgorithm::initialization(PInstance &pInst) {
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
    }
    /*else if (pInst->parameters_->initialStart_ == GREEDY_START){
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
    }*/

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
        MIPReducedPro_->buildModel(pInst, zSolution_, routeSolution_);
        RPBuildTime_->stop();
        MIPReducedPro_->solveModelDual(pInst, zSolution_, routeSolution_);
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
            /*if (incRequestToOrder_.count(requestObj->getRequestId()) > 0)
                pattern(incRequestToOrder_[requestObj->getRequestId()], 0) = 1;*/
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
 //       Eigen::MatrixXd pattern = Eigen::MatrixXd::Zero((int) incRequestToOrder_.size() + (int) incVehicleToOrder_.size(),1);
        Eigen::MatrixXd pattern = Eigen::MatrixXd::Zero(nbCoveredTasks_ + (int) incVehicleToOrder_.size(),1);
        for (auto & requestObj : route->routeRequests_) {
            /*if (incRequestToOrder_.count(requestObj->getRequestId()) > 0)
                pattern(incRequestToOrder_[requestObj->getRequestId()], 0) = 1;*/
            if (requestObj->taskIncIndex_ != -1)
                pattern(requestObj->taskIncIndex_, 0) = 1;
        }
        if (incVehicleToOrder_.count(route->vehicleID_)>0)
            pattern(incVehicleToOrder_[route->vehicleID_],0) = 1;
        Eigen::MatrixXd multiplication = incMatrix_ * pattern;
        /*route->incompatibilityDegree_ = 0;
        for (int i = 0; i < multiplication.rows(); ++i) {
            if (multiplication(i,0) != 0)
                route->incompatibilityDegree_ ++;
        }*/
        route->incompatibilityDegree_ = multiplication.cwiseAbs().colwise().sum()[0];
    }
    else
        route->incompatibilityDegree_ = 0;
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
            if (incVehicleToOrder_.count(routeObj.second->vehicleID_)>0)
                pattern(incVehicleToOrder_[routeObj.second->vehicleID_],i) = 1;
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
    calcIncMatrix();
//    calcIncMatrixFull();
    maxIncDegree_ = 0;
    Tools::PThreadsPool pPool = Tools::ThreadsPool::newThreadsPool(pInst->parameters_->nbThreads_);

    for (auto & vehicleObj : pInst->vehicles_) {
        if (!availableRoutes_[vehicleObj->vehicleID_].empty()) {
            Tools::Job job([&]() {
                updateRoutesIncDegree(vehicleObj->vehicleID_);
            });
            pPool->run(job);
//            updateRoutesIncDegree(vehicleObj->vehicleID_);
        }
    }
    pPool->wait();
   /* while(true){
        if (!pPool->wait())
            break;
    }*/
}
void ISUDAlgorithm::updateIncDegreesBit(PInstance &pInst) {
    std::shared_ptr<myTools::BitVector>  zList = std::make_shared<myTools::BitVector>(pInst->nbRequests_);
    std::shared_ptr<myTools::BitVector>  coveredList = std::make_shared<myTools::BitVector>(pInst->nbRequests_);
    for (auto & requestObj : zSolution_)
        zList->add(requestObj->taskIndex_);

    for (auto & vehicleObj : pInst->vehicles_) {
        if (!availableRoutes_[vehicleObj->vehicleID_].empty()) {
            coveredList->copyValues(*zList);
            std::shared_ptr<myTools::BitVector>  currentSolution = std::make_shared<myTools::BitVector>(pInst->nbRequests_);
            for (auto & requestObj : vehicleObj->currentRoute_->routeRequests_) {
                coveredList->add(requestObj->taskIndex_);
                currentSolution->add(requestObj->taskIndex_);
            }
            for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]){
                if (routeObj->column_->isSubset(*coveredList) && currentSolution->isSubset(*routeObj->column_))
                    routeObj->isCompatible_ = true;
                else
                    routeObj->isCompatible_ = false;
            }
        }
    }
}
void ISUDAlgorithm::updateRoutesIncDegree(int &vehicleID) {

    for (auto & routeObj : availableRoutes_[vehicleID]) {
        calcIncompatibility(routeObj);
//        calcIncompatibilityFull(routeObj);
        if (routeObj->incompatibilityDegree_ > maxIncDegree_)
            maxIncDegree_  =routeObj->incompatibilityDegree_;
    }

    // sort the routes based on their incompatibility degree
    std::stable_sort(availableRoutes_[vehicleID].begin(),availableRoutes_[vehicleID].end(),[](const PRoute &lhs, const PRoute &rhs){
        return std::tie(lhs->incompatibilityDegree_, lhs->reducedCost_) < std::tie(rhs->incompatibilityDegree_, rhs->reducedCost_);
    });
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
        }
    }
    if (minReducedCost_ < 0)
        maxReducedCost_ = ((-1)*minReducedCost_);
}

void ISUDAlgorithm::solveISUD(PInstance &pInst, int epoch, InputPaths &inputPaths) {
    isudTime_->start();
    if (pInst->parameters_->initialStart_ == GREEDY_START){
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
    double previousObj = objValue_;
    bool restartAlgorithm = true;
    bool isCPImproved;
    bool isCPBuilt;
    int CPCounter;

    while (restartAlgorithm){
        isCPImproved = true;
        restartAlgorithm = true;
        isCPBuilt = false;
        /************************************************************************************************/
        //                                     REDUCED PROBLEM
        /************************************************************************************************/
        RPTime_->start();
        CPCounter = 0;
        MIPReducedPro_->routesToAdd_.clear();

        // update reduced costs if needed only at the start of epoch, there is another update after CP
        if  ((pInst->parameters_->initialStart_ == PRE_SOLUTION)&&(pInst->parameters_->initialDual_ == PENALTIES) && (isudIter_ == 1)){
            minReducedCost_ = INFINITY;
            maxReducedCost_ = INFINITY;
            for(auto & requestObj: pInst->requests_)
                requestObj->dual_ = requestObj->CPDual_;
            updateReducedCosts(pInst);
 //           save_IncDegree_RDCost(inputPaths, epoch, isudIter_);
            if (minReducedCost_ > 0){
                RPTime_->stop();
                break;
            }
        }
        // solve RP with MIP solver
        CompPro_->fractionalZ_.clear();
        solveRP_MIP(pInst, 0, inputPaths);
        std::cout << "Objective value after RP improve: " << objValue_ << std::endl;
        RPTime_->stop();
        // if objective improves, the CP is build
        if (previousObj != objValue_){
            CPTime_->start();
//            pInst->saveISUDRoutes(inputPaths.getOutputEpochIsud(), epoch, isudIter_);
//            (*pLogIterSolutionStream_) << pInst->saveISUDRoutes(epoch, isudIter_);
  //          save_ISUDResults(epoch, inputPaths, "RP", MIPReducedPro_->compRoutes_.size());
            (*pLogIsudResultsStream_) << save_ISUDResults(epoch, "RP", (int)MIPReducedPro_->compRoutes_.size());
            isudIter_++;
            previousObj = objValue_;
 //           std::cout << "Objective Value after the RP improve: " << objValue_ << std::endl;

            CompPro_->routesToAdd_.clear();
            updateIncDegrees(pInst);
            if (pInst->parameters_->useMultiStage_)
                updateRoutesToAdd(cpIncDegree_, pInst);
            else
                updateRoutesToAdd(maxIncDegree_, pInst);
    //        std::cout << "CP problem size: " << CompPro_->routesToAdd_.size() << std::endl;
            CPBuildTime_->start();
            CompPro_->buildModel(pInst, zSolution_, routeSolution_);
            CPBuildTime_->stop();
            isCPBuilt = true;
            CPTime_->stop();
        }

        /************************************************************************************************/
        //                                     COMPLEMENTARY PROBLEM
        /************************************************************************************************/
        CPTime_->start();
        if (pInst->parameters_->initialStart_ == GREEDY_START){
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
        if (!isCPBuilt){
            CompPro_->routesToAdd_.clear();
            updateIncDegrees(pInst);
            if (pInst->parameters_->useMultiStage_)
                updateRoutesToAdd(cpIncDegree_, pInst);
            else
                updateRoutesToAdd(maxIncDegree_, pInst);
//            std::cout << "CP problem size: " << CompPro_->routesToAdd_.size() << std::endl;
            CPBuildTime_->start();
            CompPro_->buildModel(pInst, zSolution_, routeSolution_);
            CPBuildTime_->stop();
        }
        while (isCPImproved){
            isCPImproved = false;
            if (!CompPro_->routesToAdd_.empty()) {
   //             std::cout << "# IMPROVE THE SOLUTION BY SOLVING THE COMPLEMENTARY PROBLEM" << std::endl;
     //           CompPro_->solveModelIndex(pInst, zSolution_, routeSolution_, generatedRoutes_);
                CompPro_->solveModelIndex(pInst, zSolution_, routeSolution_);
                CPEpochSolveTime_ += CompPro_->solveTime_->dSinceStart().count();
                setObjValue();
                std::cout << "Objective value after CP improve: " << objValue_ << std::endl;

                if (CompPro_->status_ == FRACTIONAL) {
    //                std::cout << "# The Algorithm needs modification to find integer direction" << std::endl;
                    if (pInst->parameters_->useZoom_){
                        isudMIPTime_->start();
                        solveRP_MIP(pInst, pInst->parameters_->MIP_maxIncDegree_, inputPaths);
                        if (previousObj > objValue_) {
                            previousObj = objValue_;
 //                           (*pLogIterSolutionStream_) << pInst->saveISUDRoutes(epoch, isudIter_);
                            (*pLogIsudResultsStream_) << save_ISUDResults(epoch, "ZOOM", (int)MIPReducedPro_->compRoutes_.size());
                            isudIter_++;
                            //                     std::cout << "restarting CP after MIP improve" << std::endl;
                            isCPImproved = true;
                            CompPro_->routesToAdd_.clear();
                            updateIncDegrees(pInst);
                            updateRoutesToAdd(maxIncDegree_, pInst);
                            std::cout << "CP problem size: " << CompPro_->routesToAdd_.size() << std::endl;
                            CPBuildTime_->start();
                            CompPro_->buildModel(pInst, zSolution_, routeSolution_);
                            CPBuildTime_->stop();
                            isudMIPTime_->stop();
                        }
                        else {
                            isudMIPTime_->stop();
                            if (CPCounter == 0) {
                                restartAlgorithm = false;
                                break;
                            }
                        }
                    }
                    if (pInst->parameters_->useMultiStage_){
                        if (cpIncDegree_ < maxIncDegree_)
                            cpIncDegree_++;
                        else{
                            restartAlgorithm = false;
                            break;
                        }
                    }
                    else {
                        if (CPCounter == 0) {
                            restartAlgorithm = false;
                            break;
                        }
                    }
                }
                else if (CompPro_->status_ == POSITIVE_VALUE) {
      //              std::cout << "# The Algorithm can not find further direction of descent and terminated" << std::endl;
                    if (pInst->parameters_->useMultiStage_)
                        if (cpIncDegree_ < maxIncDegree_)
                            cpIncDegree_++;
                        else{
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
    //                std::cout << "# The Algorithm failed to optimized and terminated" << std::endl;
                    if (pInst->parameters_->useMultiStage_)
                        if (cpIncDegree_ < maxIncDegree_)
                            cpIncDegree_++;
                        else{
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
    //                std::cout << "# The Complementary Problems solved and find integer direction. " << std::endl;
                    setObjValue();
    //                std::cout << "Objective Value after the CP improve: " << objValue_ << std::endl;
 //                   pInst->saveISUDRoutes(inputPaths.getOutputEpochIsud(), epoch, isudIter_);
 //                   save_ISUDResults(epoch, inputPaths, "CP", CompPro_->IncRoute_.size() + routeSolution_.size());
 //                   (*pLogIterSolutionStream_) << pInst->saveISUDRoutes(epoch, isudIter_);
                    (*pLogIsudResultsStream_) << save_ISUDResults(epoch, "CP", (int)(CompPro_->IncRoute_.size() + routeSolution_.size()));
                    previousObj = objValue_;
                    isudIter_++;
                    isCPImproved = true;
                    CPCounter++;
                    if (pInst->parameters_->solutionMode_ == ANYTIME && isudTime_->dSinceStart().count() > availableTime_) {
                        restartAlgorithm = false;
                        break;
                    }
                }
            }
            else
                restartAlgorithm = false;
        }
 //       CPTime_->stop();
        /*ReducedPro_->requestDuals_ = CompPro_->requestDuals_;
        ReducedPro_->vehicleDuals_ = CompPro_->vehicleDuals_;*/
        MIPReducedPro_->requestDuals_ = CompPro_->requestDuals_;
        MIPReducedPro_->vehicleDuals_ = CompPro_->vehicleDuals_;
        minReducedCost_ = INFINITY;
        maxReducedCost_ = INFINITY;
//        updateDegreeTime_->start();
        for(auto & requestObj: pInst->requests_)
            requestObj->dual_ = requestObj->CPDual_;
        updateReducedCosts(pInst);
//        updateDegreeTime_->stop();
        if ((minReducedCost_ > 0)||(pInst->parameters_->solutionMode_ == ANYTIME && isudTime_->dSinceStart().count() > availableTime_))
            restartAlgorithm = false;

        CompPro_.reset();
        CompPro_ = std::make_shared<ComplementPro>();
        MIPReducedPro_.reset();
        MIPReducedPro_ = std::make_shared<ZoomReducedProblem>();
        CPTime_->stop();
    }
    std::cout << "# number of unserved requests: " << zSolution_.size() << std::endl;
    std::cout << "# Time spent on ISUD iteration  = " << isudTime_->dSinceStart().count() << " (seconds)" << std::endl;
    for (auto & requestObj : zSolution_)
        std::cout << "request " << requestObj->getRequestId() << " : " << requestObj->penalty_ << std::endl;
    isudTime_->stop();
}
void ISUDAlgorithm::solveISUD_Dual(PInstance &pInst, int epoch, InputPaths &inputPaths) {
    isudTime_->start();

    if (pInst->parameters_->initialStart_ == GREEDY_START){
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

    while (restartAlgorithm){
        isCPImproved = true;
        restartAlgorithm = true;
        /************************************************************************************************/
        //                                     REDUCED PROBLEM
        /************************************************************************************************/
        RPTime_->start();
        // solve RP with MIP solver
        while (true){
            CompPro_->fractionalZ_.clear();
            updateReducedCosts(pInst);
 //           std::cout << "min reduced: " << minReducedCost_ << std::endl;
            if (minReducedCost_ >= 0){
                break;
            }
            solveRP_MIP_Dual(pInst, 0, inputPaths);
            std::cout << "Objective value after RP improve: " << objValue_ << std::endl;
            (*pLogIsudResultsStream_) << save_ISUDResults(epoch, "RP", (int)MIPReducedPro_->compRoutes_.size());
            isudIter_++;
            if (previousObj < objValue_){
                previousObj = objValue_;
                MIPReducedPro_.reset();
                MIPReducedPro_ = std::make_shared<ZoomReducedProblem>();
            }
            else
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
        if (minReducedCost_ < 0){

            previousObj = objValue_;
            updateIncDegrees(pInst);
            CompPro_->routesToAdd_.clear();
            if (pInst->parameters_->useMultiStage_)
                updateRoutesToAdd(cpIncDegree_, pInst);
            else {
                //        std::cout << "Max degree:  " << maxIncDegree_ << std::endl;
                updateRoutesToAdd(10, pInst);
            }
            CPBuildTime_->start();
            CompPro_->buildModel(pInst, zSolution_, routeSolution_);
            CPBuildTime_->stop();
        }
        else{
            restartAlgorithm = false;
            isCPImproved = false;
        }

        while (isCPImproved){
            isCPImproved = false;
            if (!CompPro_->routesToAdd_.empty()) {
                CompPro_->solveModelIndex(pInst, zSolution_, routeSolution_);
                CPEpochSolveTime_ += CompPro_->solveTime_->dSinceStart().count();
                setObjValue();
                std::cout << "Objective value after CP improve: " << objValue_ << std::endl;

                if (CompPro_->status_ == FRACTIONAL) {
                    //  std::cout << "# The Algorithm needs modification to find integer direction" << std::endl;
                    if (pInst->parameters_->useZoom_){
                        isudMIPTime_->start();
                        solveRP_MIP_Dual(pInst, pInst->parameters_->MIP_maxIncDegree_, inputPaths);
                        if (previousObj > objValue_) {
                            previousObj = objValue_;
                            (*pLogIsudResultsStream_) << save_ISUDResults(epoch, "ZOOM", (int)MIPReducedPro_->compRoutes_.size());
                            isudIter_++;
                            isCPImproved = true;
                            CompPro_->routesToAdd_.clear();
                            updateIncDegrees(pInst);
                            updateRoutesToAdd(maxIncDegree_, pInst);
                            std::cout << "CP problem size: " << CompPro_->routesToAdd_.size() << std::endl;
                            CPBuildTime_->start();
                            CompPro_->buildModel(pInst, zSolution_, routeSolution_);
                            CPBuildTime_->stop();
                            isudMIPTime_->stop();
                        }
                        else {
                            isudMIPTime_->stop();
                            if (CPCounter == 0) {
                                restartAlgorithm = false;
                                break;
                            }
                        }
                    }
                    if (pInst->parameters_->useMultiStage_){
                        if (cpIncDegree_ < maxIncDegree_)
                            cpIncDegree_++;
                        else{
                            restartAlgorithm = false;
                            break;
                        }
                    }
                    else {
                        if (CPCounter == 0) {
                            restartAlgorithm = false;
                            break;
                        }
                    }
                }
                else if (CompPro_->status_ == POSITIVE_VALUE) {
                    //              std::cout << "# The Algorithm can not find further direction of descent and terminated" << std::endl;
                    if (pInst->parameters_->useMultiStage_)
                        if (cpIncDegree_ < maxIncDegree_)
                            cpIncDegree_++;
                        else{
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
                    //                std::cout << "# The Algorithm failed to optimized and terminated" << std::endl;
                    if (pInst->parameters_->useMultiStage_)
                        if (cpIncDegree_ < maxIncDegree_)
                            cpIncDegree_++;
                        else{
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
                    //                std::cout << "# The Complementary Problems solved and find integer direction. " << std::endl;
                    setObjValue();
                    (*pLogIsudResultsStream_) << save_ISUDResults(epoch, "CP", (int)(CompPro_->IncRoute_.size()));
                    previousObj = objValue_;
                    isudIter_++;
                    isCPImproved = true;
                    CPCounter++;
                    if (pInst->parameters_->solutionMode_ == ANYTIME && isudTime_->dSinceStart().count() > availableTime_) {
                        restartAlgorithm = false;
                        break;
                    }
                }
            }
            else
                restartAlgorithm = false;
        }

        if ((minReducedCost_ > 0)||(pInst->parameters_->solutionMode_ == ANYTIME && isudTime_->dSinceStart().count() > availableTime_))
            restartAlgorithm = false;

        CompPro_.reset();
        CompPro_ = std::make_shared<ComplementPro>();
        MIPReducedPro_.reset();
        MIPReducedPro_ = std::make_shared<ZoomReducedProblem>();
        CPTime_->stop();
    }
    std::cout << "# number of unserved requests: " << zSolution_.size() << std::endl;
    std::cout << "# Time spent on ISUD iteration  = " << isudTime_->dSinceStart().count() << " (seconds)" << std::endl;
    for (auto & requestObj : zSolution_)
        std::cout << "request " << requestObj->getRequestId() << " : " << requestObj->penalty_ << std::endl;
    isudTime_->stop();
}
void ISUDAlgorithm::solveISUD_Original(PInstance &pInst, int epoch, InputPaths &inputPaths) {
    isudTime_->start();
    if (pInst->parameters_->initialStart_ == GREEDY_START){
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

    while (restartAlgorithm){
        isCPImproved = true;
        restartAlgorithm = true;
        /************************************************************************************************/
        //                                     REDUCED PROBLEM
        /************************************************************************************************/
        RPTime_->start();
        // solve RP with MIP solver
        while (true){
            CompPro_->fractionalZ_.clear();
            updateReducedCosts(pInst);
            if (minReducedCost_ >= 0){
                break;
            }
            solveRP_MIP_Dual(pInst, 0, inputPaths);
            std::cout << "Objective value after RP improve: " << objValue_ << std::endl;
            (*pLogIsudResultsStream_) << save_ISUDResults(epoch, "RP", (int)MIPReducedPro_->compRoutes_.size());
            isudIter_++;
            if (previousObj < objValue_){
                previousObj = objValue_;
                MIPReducedPro_.reset();
                MIPReducedPro_ = std::make_shared<ZoomReducedProblem>();
            }
            else
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
        if (minReducedCost_ < 0){

            previousObj = objValue_;
            updateIncDegrees(pInst);
            CompPro_->routesToAdd_.clear();
            if (pInst->parameters_->useMultiStage_)
                updateRoutesToAdd(cpIncDegree_, pInst);
            else {
                //        std::cout << "Max degree:  " << maxIncDegree_ << std::endl;
                updateRoutesToAdd(10, pInst);
            }
            CPBuildTime_->start();
            CompPro_->buildModel(pInst, zSolution_, routeSolution_);
            CPBuildTime_->stop();
        }
        else{
            restartAlgorithm = false;
            isCPImproved = false;
        }

        while (isCPImproved){
            isCPImproved = false;
            if (!CompPro_->routesToAdd_.empty()) {
                CompPro_->solveModelIndex(pInst, zSolution_, routeSolution_);
                CPEpochSolveTime_ += CompPro_->solveTime_->dSinceStart().count();
                setObjValue();
                std::cout << "Objective value after CP improve: " << objValue_ << std::endl;

                if (CompPro_->status_ == FRACTIONAL) {
                    //  std::cout << "# The Algorithm needs modification to find integer direction" << std::endl;
                    if (pInst->parameters_->useZoom_){
                        isudMIPTime_->start();
                        solveRP_MIP(pInst, pInst->parameters_->MIP_maxIncDegree_, inputPaths);
                        if (previousObj > objValue_) {
                            previousObj = objValue_;
                            (*pLogIsudResultsStream_) << save_ISUDResults(epoch, "ZOOM", (int)MIPReducedPro_->compRoutes_.size());
                            isudIter_++;
                            isCPImproved = true;
                            CompPro_->routesToAdd_.clear();
                            updateIncDegrees(pInst);
                            updateRoutesToAdd(maxIncDegree_, pInst);
                            std::cout << "CP problem size: " << CompPro_->routesToAdd_.size() << std::endl;
                            CPBuildTime_->start();
                            CompPro_->buildModel(pInst, zSolution_, routeSolution_);
                            CPBuildTime_->stop();
                            isudMIPTime_->stop();
                        }
                        else {
                            isudMIPTime_->stop();
                            if (CPCounter == 0) {
                                restartAlgorithm = false;
                                break;
                            }
                        }
                    }
                    if (pInst->parameters_->useMultiStage_){
                        if (cpIncDegree_ < maxIncDegree_)
                            cpIncDegree_++;
                        else{
                            restartAlgorithm = false;
                            break;
                        }
                    }
                    else {
                        if (CPCounter == 0) {
                            restartAlgorithm = false;
                            break;
                        }
                    }
                }
                else if (CompPro_->status_ == POSITIVE_VALUE) {
                    //              std::cout << "# The Algorithm can not find further direction of descent and terminated" << std::endl;
                    if (pInst->parameters_->useMultiStage_)
                        if (cpIncDegree_ < maxIncDegree_)
                            cpIncDegree_++;
                        else{
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
                    //                std::cout << "# The Algorithm failed to optimized and terminated" << std::endl;
                    if (pInst->parameters_->useMultiStage_)
                        if (cpIncDegree_ < maxIncDegree_)
                            cpIncDegree_++;
                        else{
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
                    //                std::cout << "# The Complementary Problems solved and find integer direction. " << std::endl;
                    setObjValue();
                    (*pLogIsudResultsStream_) << save_ISUDResults(epoch, "CP", (int)(CompPro_->IncRoute_.size()));
                    previousObj = objValue_;
                    isudIter_++;
 //                   isCPImproved = true;
                    CPCounter++;
                }
            }
            else
                restartAlgorithm = false;
        }

        if (isudTime_->dSinceStart().count() > 8)
            restartAlgorithm = false;

        CompPro_.reset();
        CompPro_ = std::make_shared<ComplementPro>();
        MIPReducedPro_.reset();
        MIPReducedPro_ = std::make_shared<ZoomReducedProblem>();
        CPTime_->stop();
    }
    std::cout << "# number of unserved requests: " << zSolution_.size() << std::endl;
    std::cout << "# Time spent on ISUD iteration  = " << isudTime_->dSinceStart().count() << " (seconds)" << std::endl;
    for (auto & requestObj : zSolution_)
        std::cout << "request " << requestObj->getRequestId() << " : " << requestObj->penalty_ << std::endl;
    isudTime_->stop();
}
void ISUDAlgorithm::solveISUDMIP(PInstance &pInst, InputPaths &inputPaths) {
    isudMIPTime_->start();
    double previousObj = objValue_;
    // improve by solving the Reduced problem

    MIPReducedPro_->routesToAdd_.clear();
    RPBuildTime_->start();
    MIPReducedPro_->buildModel(pInst, zSolution_, routeSolution_);
    RPBuildTime_->stop();

    for (auto & vehicleObj : pInst->vehicles_) {
        for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]) {
            if (routeObj->incompatibilityDegree_ <= pInst->parameters_->MIP_maxIncDegree_)
                MIPReducedPro_->routesToAdd_.push_back(routeObj);
        }
    }
    RPBuildTime_->start();
    MIPReducedPro_->updateModel(pInst, CompPro_->fractionalZ_);
    RPBuildTime_->stop();
    MIPReducedPro_->solveModelDual(pInst, zSolution_, routeSolution_);
    RPEpochSolveTime_ += MIPReducedPro_->solveTime_->dSinceStart().count();
    setObjValue();

    /*std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cout << "+       Solution Result after MIP solve of ISUD:   +" << std::endl;
    std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    for (auto & routeObj : routeSolution_) {
        std::cout << routeObj->toString();
    }*/
    if (previousObj != objValue_){
        isudIter_++;
        std::cout << "Objective Value after the RP improve: " << objValue_ << std::endl;
        /*CPTime_->start();
        CompPro_->routesToAdd_.clear();
        updateDegreeTime_->start();
        updateIncDegrees(pInst);
        updateDegreeTime_->stop();
        updateRoutesToAdd(maxIncDegree_, pInst);
        CompPro_->buildModel(pInst, zSolution_, routeSolution_);
        CompPro_->solveModel(pInst, zSolution_, routeSolution_);
        setObjValue();
        CPTime_->stop();
        // UPDATE DUAL VALUES AFTER SOLVING CP
        MIPReducedPro_->requestDuals_ = CompPro_->requestDuals_;
        MIPReducedPro_->vehicleDuals_ = CompPro_->vehicleDuals_;*/

    }
    isudMIPTime_->stop();
}
void ISUDAlgorithm::solveISUD_Partial(PInstance &pInst, int epoch, InputPaths &inputPaths) {
    isudTime_->start();

    if (pInst->parameters_->initialStart_ == GREEDY_START){
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

    while (restartAlgorithm){
        isCPImproved = true;
        restartAlgorithm = true;
        /************************************************************************************************/
        //                                     REDUCED PROBLEM
        /************************************************************************************************/
        RPTime_->start();
        // solve RP with MIP solver
        while (true){
            CompPro_->fractionalZ_.clear();
            updateReducedCosts(pInst);

            if (minReducedCost_ >= 0){
                break;
            }
            solveRP_MIP_Partial(pInst, 0, inputPaths);
            std::cout << "Objective value after RP improve: " << objValue_ << std::endl;
            (*pLogIsudResultsStream_) << save_ISUDResults(epoch, "RP", (int)MIPReducedPro_->compRoutes_.size());
            isudIter_++;
            if (previousObj < objValue_){
                previousObj = objValue_;
                MIPReducedPro_.reset();
                MIPReducedPro_ = std::make_shared<ZoomReducedProblem>();
            }
            else
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
        if (minReducedCost_ < 0){
 //           std::cout << "min reduced: " << minReducedCost_ << std::endl;
            previousObj = objValue_;
            updateIncDegrees(pInst);
            CompPro_->routesToAdd_.clear();
            if (pInst->parameters_->useMultiStage_)
                updateRoutesToAdd(cpIncDegree_, pInst);
            else {
                //        std::cout << "Max degree:  " << maxIncDegree_ << std::endl;
                updateRoutesToAdd(10, pInst);
            }
            CPBuildTime_->start();
            CompPro_->buildModelPartial(pInst, zSolution_, routeSolution_, nbVehicles_);
            CPBuildTime_->stop();
        }
        else{
            restartAlgorithm = false;
            isCPImproved = false;
        }

        while (isCPImproved){
            isCPImproved = false;
            if (!CompPro_->routesToAdd_.empty()) {
                CompPro_->solveModelPartial(pInst, zSolution_, routeSolution_);
                CPEpochSolveTime_ += CompPro_->solveTime_->dSinceStart().count();
                setObjValue();
                std::cout << "Objective value after CP improve: " << objValue_ << std::endl;

                if (CompPro_->status_ == FRACTIONAL) {
                    //  std::cout << "# The Algorithm needs modification to find integer direction" << std::endl;
                    if (pInst->parameters_->useZoom_){
                        isudMIPTime_->start();
                        solveRP_MIP_Partial(pInst, pInst->parameters_->MIP_maxIncDegree_, inputPaths);
                        if (previousObj > objValue_) {
                            previousObj = objValue_;
                            (*pLogIsudResultsStream_) << save_ISUDResults(epoch, "ZOOM", (int)MIPReducedPro_->compRoutes_.size());
                            isudIter_++;
                            isCPImproved = true;
                            CompPro_->routesToAdd_.clear();
                            updateIncDegrees(pInst);
                            updateRoutesToAdd(maxIncDegree_, pInst);
                            std::cout << "CP problem size: " << CompPro_->routesToAdd_.size() << std::endl;
                            CPBuildTime_->start();
                            CompPro_->buildModelPartial(pInst, zSolution_, routeSolution_, nbVehicles_);
                            CPBuildTime_->stop();
                            isudMIPTime_->stop();
                        }
                        else {
                            isudMIPTime_->stop();
                            if (CPCounter == 0) {
                                restartAlgorithm = false;
                                break;
                            }
                        }
                    }
                    if (pInst->parameters_->useMultiStage_){
                        if (cpIncDegree_ < maxIncDegree_)
                            cpIncDegree_++;
                        else{
                            restartAlgorithm = false;
                            break;
                        }
                    }
                    else {
                        if (CPCounter == 0) {
                            restartAlgorithm = false;
                            break;
                        }
                    }
                }
                else if (CompPro_->status_ == POSITIVE_VALUE) {
                    //              std::cout << "# The Algorithm can not find further direction of descent and terminated" << std::endl;
                    if (pInst->parameters_->useMultiStage_)
                        if (cpIncDegree_ < maxIncDegree_)
                            cpIncDegree_++;
                        else{
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
                    //                std::cout << "# The Algorithm failed to optimized and terminated" << std::endl;
                    if (pInst->parameters_->useMultiStage_)
                        if (cpIncDegree_ < maxIncDegree_)
                            cpIncDegree_++;
                        else{
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
                    //                std::cout << "# The Complementary Problems solved and find integer direction. " << std::endl;
                    setObjValue();
                    (*pLogIsudResultsStream_) << save_ISUDResults(epoch, "CP", (int)(CompPro_->IncRoute_.size()));
                    previousObj = objValue_;
                    isudIter_++;
                    isCPImproved = true;
                    CPCounter++;
                }
            }
            else
                restartAlgorithm = false;
        }

        if (isudTime_->dSinceStart().count() > 8)
            restartAlgorithm = false;

        CompPro_.reset();
        CompPro_ = std::make_shared<ComplementPro>();
        MIPReducedPro_.reset();
        MIPReducedPro_ = std::make_shared<ZoomReducedProblem>();
        CPTime_->stop();
    }
    std::cout << "# number of unserved requests: " << zSolution_.size() << std::endl;
    std::cout << "# Time spent on ISUD iteration  = " << isudTime_->dSinceStart().count() << " (seconds)" << std::endl;
    for (auto & requestObj : zSolution_)
        std::cout << "request " << requestObj->getRequestId() << " : " << requestObj->penalty_ << std::endl;
    isudTime_->stop();
}
void ISUDAlgorithm::solveRP_MIP(PInstance &pInst, int compDegree, InputPaths &inputPaths) {
//    isudMIPTime_->start();
    // improve by solving the Reduced problem
    MIPReducedPro_->routesToAdd_.clear();
    RPBuildTime_->start();
    MIPReducedPro_->buildModel(pInst, zSolution_, routeSolution_);
    RPBuildTime_->stop();
    if (compDegree == 0) {
        if ((isudIter_ ==1)&&(pInst->parameters_->initialStart_ != GREEDY_START)){
            updateIncDegreesBit(pInst);
            updateRoutesToAdd(true, pInst);
        }
        else{
            updateIncDegrees(pInst);
            updateRoutesToAdd(compDegree, pInst);
        }
    }
    else
    {
        for (auto & vehicleObj : pInst->vehicles_) {
            for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]) {
                if (routeObj->incompatibilityDegree_ <= pInst->parameters_->MIP_maxIncDegree_)
                    MIPReducedPro_->routesToAdd_.push_back(routeObj);
            }
        }
    }

    if (!MIPReducedPro_->routesToAdd_.empty()){
        for (int v = 0; v < pInst->nbVehicles_; ++v) {
            if (!pInst->vehicles_[v]->currentRoute_->routeRequests_.empty())
                MIPReducedPro_->routesToAdd_.push_back(pInst->vehicles_[v]->emptyRoute_);
        }
        RPBuildTime_->start();
        MIPReducedPro_->updateModel(pInst, CompPro_->fractionalZ_);
        RPBuildTime_->stop();
        MIPReducedPro_->solveModel(pInst, zSolution_, routeSolution_);
        RPEpochSolveTime_ += MIPReducedPro_->solveTime_->dSinceStart().count();
        setObjValue();
    }
//    isudMIPTime_->stop();
}
void ISUDAlgorithm::solveRP_MIP_Dual(PInstance &pInst, int compDegree, InputPaths &inputPaths) {
//    isudMIPTime_->start();
    // improve by solving the Reduced problem
    MIPReducedPro_->routesToAdd_.clear();
    RPBuildTime_->start();
    MIPReducedPro_->buildModel(pInst, zSolution_, routeSolution_);
    RPBuildTime_->stop();
    if (compDegree == 0) {
        if ((isudIter_ ==1)&&(pInst->parameters_->initialStart_ != GREEDY_START)){
            updateIncDegreesBit(pInst);
            updateRoutesToAdd(true, pInst);
        }
        else{
            updateIncDegrees(pInst);
            updateRoutesToAdd(compDegree, pInst);
        }
    }
    else
    {
        for (auto & vehicleObj : pInst->vehicles_) {
            for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]) {
                if (routeObj->incompatibilityDegree_ <= pInst->parameters_->MIP_maxIncDegree_)
                    MIPReducedPro_->routesToAdd_.push_back(routeObj);
            }
        }
    }

    if (!MIPReducedPro_->routesToAdd_.empty()){
        for (int v = 0; v < pInst->nbVehicles_; ++v) {
            if (!pInst->vehicles_[v]->currentRoute_->routeRequests_.empty())
                MIPReducedPro_->routesToAdd_.push_back(pInst->vehicles_[v]->emptyRoute_);
        }
        RPBuildTime_->start();
        MIPReducedPro_->updateModel(pInst, CompPro_->fractionalZ_);
        RPBuildTime_->stop();
        MIPReducedPro_->solveModelDual(pInst, zSolution_, routeSolution_);
        RPEpochSolveTime_ += MIPReducedPro_->solveTime_->dSinceStart().count();
        setObjValue();
    }
//    isudMIPTime_->stop();
}
void ISUDAlgorithm::solveRP_MIP_Partial(PInstance &pInst, int compDegree, InputPaths &inputPaths) {
//    isudMIPTime_->start();
    // improve by solving the Reduced problem
    MIPReducedPro_->routesToAdd_.clear();
    RPBuildTime_->start();
    MIPReducedPro_->buildModelPartial(pInst, zSolution_, routeSolution_, nbVehicles_);
    RPBuildTime_->stop();
    if (compDegree == 0) {
        if ((isudIter_ ==1)&&(pInst->parameters_->initialStart_ != GREEDY_START)){
            updateIncDegreesBit(pInst);
            updateRoutesToAdd(true, pInst);
        }
        else{
            updateIncDegrees(pInst);
            updateRoutesToAdd(compDegree, pInst);
        }
    }
    else
    {
        for (auto & vehicleObj : pInst->vehicles_) {
            for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]) {
                if (routeObj->incompatibilityDegree_ <= pInst->parameters_->MIP_maxIncDegree_)
                    MIPReducedPro_->routesToAdd_.push_back(routeObj);
            }
        }
    }

    if (!MIPReducedPro_->routesToAdd_.empty()){
        for (int v = 0; v < pInst->nbVehicles_; ++v) {
            if (pInst->vehicles_[v]->vehicleIndex_ > -1)
                MIPReducedPro_->routesToAdd_.push_back(pInst->vehicles_[v]->emptyRoute_);
        }
        RPBuildTime_->start();
        MIPReducedPro_->updateModelPartial(pInst, CompPro_->fractionalZ_);
        RPBuildTime_->stop();
        MIPReducedPro_->solveModelPartial(pInst, zSolution_, routeSolution_);
        RPEpochSolveTime_ += MIPReducedPro_->solveTime_->dSinceStart().count();
        setObjValue();
    }
//    isudMIPTime_->stop();
}
// Display function
std::string ISUDAlgorithm::toString() const {

    std::stringstream repStr;
    repStr << "#" << std::endl;
    repStr << std::left << std::fixed << std::setprecision(2);
    repStr << "# TOTAL OBJECTIVE (WAITING TIMES + PENALTIES) = " << objValue_ << std::endl;
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

void ISUDAlgorithm::updateRoutesToAdd(int compDegree, PInstance &pInst) {
//    pInst->sortVehicles(BEST_REDUCE_COST);
    for (auto & vehicleObj : pInst->vehicles_) {
        int count = 0;
        if (compDegree == 0) {
            for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]) {
                if (routeObj->incompatibilityDegree_ > 0)
                    break;
                if ((routeObj->incompatibilityDegree_ == 0) && (routeObj->reducedCost_ < -0.001)) {
                    if (!routeObj->equal(*vehicleObj->currentRoute_)&&(!routeObj->routeRequests_.empty()))
                        MIPReducedPro_->routesToAdd_.push_back(routeObj);
//                        ReducedPro_->routesToAdd_.push_back(routeObj);
 //                   break;
                    /*if (ReducedPro_->routesToAdd_.empty()) {
                        ReducedPro_->routesToAdd_.push_back(routeObj);
                    }
                    else if((ReducedPro_->isColumnDisjoint(ReducedPro_->routesToAdd_, routeObj, ReducedPro_->requestToOrder_)) ||
                    (ReducedPro_->isColumnRepeat(ReducedPro_->routesToAdd_, routeObj, ReducedPro_->requestToOrder_))) {
                        ReducedPro_->routesToAdd_.push_back(routeObj);
                    }*/
                }
            }
            /*if (!ReducedPro_->routesToAdd_.empty())
                break;*/
        }
        else {
            for (int r = (int)availableRoutes_[vehicleObj->vehicleID_].size() - 1; r >= 0; --r) {
                if ((availableRoutes_[vehicleObj->vehicleID_][r]->incompatibilityDegree_ <= compDegree)&&
                    (availableRoutes_[vehicleObj->vehicleID_][r]->reducedCost_ < maxReducedCost_)) {
                    if (availableRoutes_[vehicleObj->vehicleID_][r]->incompatibilityDegree_ == 0)
                        break;
                    else {
                        if (!availableRoutes_[vehicleObj->vehicleID_][r]->routeRequests_.empty()) {
                            CompPro_->routesToAdd_.push_back(availableRoutes_[vehicleObj->vehicleID_][r]);
//                            std::cout << "reduced: " << availableRoutes_[vehicleObj->vehicleID_][r]->reducedCost_ << std::endl;
                            count ++;
                        }
                    }
                }
            }
            /*if (!availableRoutes_[vehicleObj->vehicleID_].empty())
                std::cout << vehicleObj->vehicleID_ << " : " << count << std::endl;*/
        }
    }
//    pInst->restVehicleOrder();
}
void ISUDAlgorithm::updateRoutesToAdd(bool compatible, PInstance &pInst) {
//    pInst->sortVehicles(BEST_REDUCE_COST);
    for (auto & vehicleObj : pInst->vehicles_) {
        if (compatible) {
            for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]) {
                if ((routeObj->isCompatible_) && (routeObj->reducedCost_ < -0.001)) {
                    if (!routeObj->equal(*vehicleObj->currentRoute_)&& !routeObj->routeRequests_.empty())
                        MIPReducedPro_->routesToAdd_.push_back(routeObj);
                }
            }
        }
        else {
            for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]) {
                if (!routeObj->isCompatible_ && !routeObj->routeRequests_.empty()) {
                    CompPro_->routesToAdd_.push_back(routeObj);
                }
            }
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

bool ISUDAlgorithm::isCompatible(PRoute &solutionRoute, PRoute &comingRoute, std::map<unsigned int, int> &requestToOrder) {
    Eigen::MatrixXd solutionPattern = Eigen::MatrixXd::Zero((int) requestToOrder.size(), 1);
    Eigen::MatrixXd comingPattern = Eigen::MatrixXd::Zero((int) requestToOrder.size(),1);

    for (auto & requestObj : solutionRoute->routeRequests_) {
        solutionPattern(requestToOrder[requestObj->getRequestId()], 0) = 1;
    }
    for (auto & requestObj : comingRoute->routeRequests_) {
        comingPattern(requestToOrder[requestObj->getRequestId()], 0) = 1;
    }

    Eigen::MatrixXd multiplication = solutionPattern.transpose() * comingPattern;
    if ((int) multiplication(0,0) == (int) solutionRoute->routeRequests_.size())
        return true;
    else
        return false;
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


std::string ISUDAlgorithm::save_ISUDResults(int epoch, const std::string& model, int nbColumns) const {
    std::stringstream repStr;

    repStr << epoch << ",";
    repStr << isudIter_ << ",";
    repStr << nbRoutes_ << ",";
    repStr << nbColumns << ",";
    repStr << model << ",";
    repStr << objValue_ << "\n";
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





















