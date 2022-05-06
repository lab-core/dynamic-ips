//
// Created by Ella on 2021-10-09.
//

#include "ISUDAlgorithm.h"

//---------------------------------------------------------------------------------------------
//  Reduced Problem class
//  Build and solve the Reduced problem of the ISUD
//---------------------------------------------------------------------------------------------

ISUDAlgorithm::ISUDAlgorithm() {
    ReducedPro_ = std::make_shared<ReducedProblem>();
    CompPro_ = std::make_shared<ComplementPro>();
    ZoomPro_ = std::make_shared<ZoomReducedProblem>();
    objValue_ = 0;
    isudTime_ = new Tools::Timer(); isudTime_->init();
    improveIter_ = 0;
    isudIter_ = 0;
}

ISUDAlgorithm::~ISUDAlgorithm() {
    delete isudTime_;
}


void ISUDAlgorithm::setObjValue() {
    objValue_ = 0.0;
    for (int r = 0; r < routeSolution_.size(); ++r) {
        objValue_ += routeSolution_[r]->totalDelay_;
    }
    for (int i = 0; i < zSolution_.size(); ++i) {
        objValue_+= zSolution_[i]->penalty_;
    }
}

// this function create initial routes serving only one request and fill zSolution_ with available requests
// Reduced problem is also solved to initialized dual costs
void ISUDAlgorithm::initialization(PInstance &pInst, bool emptyStart) {
    ReducedPro_->routesToAdd_.clear();
    isudIter_ = 0;
//    generatedRoutes_.clear();
    for (auto & vehicleObj : pInst->vehicles_) {
        if (vehicleObj->departID_ != vehicleObj->emptyRoute_->routeNodes_[0]->nodeID_)
            vehicleObj->setEmptyRoute(pInst);
        ReducedPro_->routesToAdd_.push_back(vehicleObj->emptyRoute_);
        generatedRoutes_.insert(std::pair <std::string , PRoute> ((vehicleObj->currentRoute_)->name_ ,
                                                                      (vehicleObj->currentRoute_)));
        generatedRoutes_.insert(std::pair <std::string , PRoute> ((vehicleObj->emptyRoute_)->name_ ,
                                                                      (vehicleObj->emptyRoute_)));

    }

    // adding new arrival requests to zSolutions
    for (int i = pInst->nbRequests_- pInst->nbNewRequests_; i < pInst->nbRequests_; ++i) {
        zSolution_.push_back(pInst->requests_[i]);
 //       if (routeSolution_.size() == 0) {
        if (pInst->nbOnboards_ == 0) {
            for (int v = 0; v < pInst->nbVehicles_; ++v) {
                // creating and empty route
                PRoute newRoute = std::make_shared<Route>(pInst->vehicles_[v]->vehicleID_);

                newRoute->addSource(pInst->instGraph_->nodes_[pInst->vehicles_[v]->departID_],
                                  pInst->vehicles_[v]->departTime_, pInst->vehicles_[v]->numPassengers_);
                static const NodeType nodeTypesInOrder[] = { PICKUP, DROPOFF};
                for ( const auto t : nodeTypesInOrder)
                {
                    std::string ID = Tools::createNodeID(pInst->requests_[i]->getRequestId(), t);
                    newRoute->addNode(pInst->instGraph_->nodes_[ID]);
                }
                generatedRoutes_.insert(std::pair <std::string , PRoute> (newRoute->name_ , newRoute));
                availableRoutes_[pInst->vehicles_[v]->vehicleID_].push_back(newRoute);
                ReducedPro_->routesToAdd_.push_back(newRoute);
            }
        }
    }
    std::cout << "# -----SOLVING THE REDUCED PROBLEM AT THE START OF EPOCH-------" << std::endl;
    isudTime_->start();
    ReducedPro_->buildModel(pInst, zSolution_, routeSolution_, emptyStart);
    ReducedPro_->solveModel(pInst, zSolution_, routeSolution_, generatedRoutes_);
    setObjValue();


    isudTime_->stop();
    std::cout << std::left;
    std::cout << std::setw(sentenceSize) << "# TIME SPENT ON ISUD INITIALIZATION " << "=" << isudTime_->dSinceInit().count() << " (seconds)" << std::endl;
    std::cout << std::setw(sentenceSize) << "# NUMBER OF RECEIVED REQUESTS " << "=" << pInst->nbNewRequests_ << std::endl;
}

// function to create M2 matrix for each column in the current solution
Eigen::MatrixXd ISUDAlgorithm::calcM2Matrix(PRoute solColumn) {
    int nbRows = solColumn->routeRequests.size()-1;
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
    incRequestToOrder_.clear();
    int orderCount = 0;
    sort(routeSolution_.begin(),routeSolution_.end(),[](const PRoute &lhs, const PRoute &rhs){
        return lhs->routeRequests.size() < rhs->routeRequests.size();});
    if (routeSolution_.back()->routeRequests.size() <= 1) {
        incMatrix_ = Eigen::MatrixXd::Zero(0,0);
    }
    else {
        incMatrix_ = Eigen::MatrixXd::Zero(0,0);
        for (int r = 0; r < routeSolution_.size(); ++r) {
            if (routeSolution_[r]->routeRequests.size() > 1) {
                for (int i = 0; i < routeSolution_[r]->routeRequests.size(); ++i) {
                    incRequestToOrder_[routeSolution_[r]->routeRequests[i]] = orderCount;
                    orderCount++;
                }
                int nbRows = routeSolution_[r]->routeRequests.size() - 1;
                int rowSize = incMatrix_.rows();
                int colSize = incMatrix_.cols();
                if (nbRows == 0) {
                    incMatrix_.conservativeResize(rowSize, colSize + 1);
                    incMatrix_.bottomRightCorner(rowSize, 1) = Eigen::MatrixXd::Zero(rowSize, 1);
                } else {

                    incMatrix_.conservativeResize(rowSize + nbRows, colSize + nbRows + 1);
                    incMatrix_.bottomRightCorner(nbRows, nbRows + 1) = calcM2Matrix(routeSolution_[r]);
                    incMatrix_.bottomLeftCorner(nbRows, colSize) = Eigen::MatrixXd::Zero(nbRows, colSize);
                    incMatrix_.topRightCorner(rowSize, nbRows + 1) = Eigen::MatrixXd::Zero(rowSize, nbRows + 1);
                }
            }
        }
    }
}

void ISUDAlgorithm::calcIncMatrixFull() {
    incRequestToOrder_.clear();
    incVehicleToOrder_.clear();
    int orderCount = 0;
    sort(routeSolution_.begin(),routeSolution_.end(),[](const PRoute &lhs, const PRoute &rhs){
        return lhs->routeRequests.size() < rhs->routeRequests.size();});
    if (routeSolution_.back()->routeRequests.size() == 0) {
        incMatrix_ = Eigen::MatrixXd::Zero(0,0);
    }
    else {
        incMatrix_ = Eigen::MatrixXd::Zero(0,0);
        for (int r = 0; r < routeSolution_.size(); ++r) {
            if (routeSolution_[r]->routeRequests.size() >= 1) {
                for (int i = 0; i < routeSolution_[r]->routeRequests.size(); ++i) {
                    incRequestToOrder_[routeSolution_[r]->routeRequests[i]] = orderCount;
                    orderCount++;
                }
                incVehicleToOrder_[routeSolution_[r]->vehicleID_] = orderCount;
                orderCount++;
                int nbRows = routeSolution_[r]->routeRequests.size();
                int rowSize = incMatrix_.rows();
                int colSize = incMatrix_.cols();

                incMatrix_.conservativeResize(rowSize + nbRows, colSize + nbRows + 1);
                incMatrix_.bottomRightCorner(nbRows, nbRows + 1) = calcM2Matrix(routeSolution_[r]->routeRequests.size());
                incMatrix_.bottomLeftCorner(nbRows, colSize) = Eigen::MatrixXd::Zero(nbRows, colSize);
                incMatrix_.topRightCorner(rowSize, nbRows + 1) = Eigen::MatrixXd::Zero(rowSize, nbRows + 1);
            }
        }
    }
}
// function to calculate incompatibility degree of a route
void ISUDAlgorithm::calcIncompatibility(PRoute &route) {
    if (incMatrix_.rows() > 0) {
        Eigen::MatrixXd pattern = Eigen::MatrixXd::Zero(incRequestToOrder_.size(),1);

        for (int i = 0; i < route->routeRequests.size(); ++i) {
            if (incRequestToOrder_.count(route->routeRequests[i])>0)
                pattern(incRequestToOrder_[route->routeRequests[i]],0) = 1;
        }
        Eigen::MatrixXd multiplication = incMatrix_ * pattern;
        route->incompatibilityDegree = 0;
        for (int i = 0; i < multiplication.rows(); ++i) {
            if (multiplication(i,0) != 0)
                route->incompatibilityDegree ++;
        }
    }
    else
        route->incompatibilityDegree = 0;
}

void ISUDAlgorithm::calcIncompatibilityFull(PRoute &route) {
    if (incMatrix_.rows() > 0) {
        Eigen::MatrixXd pattern = Eigen::MatrixXd::Zero(incRequestToOrder_.size() + incVehicleToOrder_.size(),1);

        for (int i = 0; i < route->routeRequests.size(); ++i) {
            if (incRequestToOrder_.count(route->routeRequests[i])>0)
                pattern(incRequestToOrder_[route->routeRequests[i]],0) = 1;
        }
        if (incVehicleToOrder_.count(route->vehicleID_)>0)
            pattern(incVehicleToOrder_[route->vehicleID_],0) = 1;
        Eigen::MatrixXd multiplication = incMatrix_ * pattern;
        route->incompatibilityDegree = 0;
        for (int i = 0; i < multiplication.rows(); ++i) {
            if (multiplication(i,0) != 0)
                route->incompatibilityDegree ++;
        }
    }
    else
        route->incompatibilityDegree = 0;
}

// this function update the incompatibility degree of availableRoutes and
// order them based on the incompatibility degree and reduced cost
void ISUDAlgorithm::updateRoutesIncDegree(int &vehicleID) {

    for (int r = 0; r < availableRoutes_[vehicleID].size(); ++r) {
//        calcIncompatibility(availableRoutes_[vehicleID][r]);
        calcIncompatibilityFull(availableRoutes_[vehicleID][r]);
    }

    // sort the routes based on their incompatibility degree
    sort(availableRoutes_[vehicleID].begin(),availableRoutes_[vehicleID].end(),[](const PRoute &lhs, const PRoute &rhs){
        return std::tie(lhs->incompatibilityDegree,lhs->reducedCost_) < std::tie(rhs->incompatibilityDegree,rhs->reducedCost_);
    });
}

// this function updates the reduced cost for the routes in the pool
void ISUDAlgorithm::updateReducedCosts(int &vehicleID) {


    for (int r = availableRoutes_[vehicleID].size()-1; r >= 0; --r) {
        availableRoutes_[vehicleID][r]->updateReducedCost(ReducedPro_->requestDuals_, ReducedPro_->vehicleDuals_,
                                               ReducedPro_->requestToOrder_);
        /*if (availableRoutes_[vehicleID][r]->reducedCost_ >= -0.001)
            availableRoutes_[vehicleID].erase(availableRoutes_[vehicleID].begin() + r);*/

        /*else {
            for (auto & routeObj : routeSolution_) {
                if(routeObj == availableRoutes_[vehicleID][r]) {
                    availableRoutes_[vehicleID].erase(availableRoutes_[vehicleID].begin() + r);
                    break;
                }
            }
        }*/
    }
}


void ISUDAlgorithm::solveISUD(PInstance &pInst, int epoch, string isudSolutionDir) {
    double previousObj = objValue_;
    bool restartAlgorithm = true;
    bool reducedImproved = true;
    bool iterationStart = true;
    int maxZoom = 5;
    isudTime_->start();
    // improve by solving the Reduced problem

    while (restartAlgorithm) {
        // when the CP find integer the whole loop is repeated
        restartAlgorithm = false;
        reducedImproved = true;
        while (reducedImproved) {
            ReducedPro_->routesToAdd_.clear();
            // if RP improve the solution another iteration is done and reducedImproved stay true
            reducedImproved = false;
            updateRoutesToAdd(0, pInst);

            if (ReducedPro_->routesToAdd_.size() > 0) {
                iterationStart = false;
                std::cout << "# IMPROVE THE SOLUTION BY SOLVING THE REDUCED PROBLEM" << std::endl;
                improveStatus_ = 1;
                for (int v = 0; v < pInst->nbVehicles_; ++v)
                    ReducedPro_->routesToAdd_.push_back(pInst->vehicles_[v]->emptyRoute_);

                // SOLVE BY MIP
                ZoomPro_->routesToAdd_.clear();
                ZoomPro_->routesToAdd_ = ReducedPro_->routesToAdd_;
                ZoomPro_->buildModel(pInst, zSolution_, routeSolution_,false);

                /*CompPro_->fractionalZ_.clear();
                ZoomPro_->updateModel(pInst, CompPro_->fractionalZ_);*/
                ZoomPro_->solveModel(pInst, zSolution_, routeSolution_, generatedRoutes_);
                /*ReducedPro_->buildModel(pInst, zSolution_, routeSolution_,false);
                ReducedPro_->updateModel(pInst, routeSolution_);
                ReducedPro_->solveModel(pInst, zSolution_, routeSolution_, generatedRoutes_);*/
                setObjValue();
                if (previousObj != objValue_) {
                    pInst->saveISUDRoutes(isudSolutionDir, epoch, isudIter_);
                    isudIter_ ++;
                    reducedImproved = true;
                    previousObj = objValue_;
                    std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
                    std::cout << "+         Solution Result after RP improve:        +" << std::endl;
                    std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
                    for (int r = 0; r < routeSolution_.size(); ++r) {
                        std::cout << routeSolution_[r]->toString();
                    }
                }
            }
        }
        // improve the solution by solving complementary Problem
        CP: CompPro_->routesToAdd_.clear();

        updateRoutesToAdd(2*pInst->nbRequests_, pInst);
        if (CompPro_->routesToAdd_.size() > 0) {
            std::cout << "# IMPROVE THE SOLUTION BY SOLVING THE COMPLEMENTARY PROBLEM" << std::endl;
            iterationStart = false;
            /*for (int v = 0; v < pInst->nbVehicles_; ++v)
                CompPro_->routesToAdd_.push_back(pInst->vehicles_[v]->emptyRoute_);*/
            CompPro_->buildModel(pInst, zSolution_, routeSolution_);
            CompPro_->solveModel(pInst, zSolution_, routeSolution_, generatedRoutes_);
            setObjValue();
            // UPDATE DUAL VALUES AFTER SOLVING CP
            ReducedPro_->requestDuals_ = CompPro_->requestDuals_;
            ReducedPro_->vehicleDuals_ = CompPro_->vehicleDuals_;
            for (auto & vehicleObj : pInst->vehicles_) {
                updateReducedCosts(vehicleObj->vehicleID_);
                sort(availableRoutes_[vehicleObj->vehicleID_].begin(),availableRoutes_[vehicleObj->vehicleID_].end(),[](const PRoute &lhs, const PRoute &rhs){
                    return lhs->reducedCost_ < rhs->reducedCost_;});
                vehicleObj->bestReducedCost_ = availableRoutes_[vehicleObj->vehicleID_][0]->reducedCost_;
            }
            if (CompPro_->status_ == FRACTIONAL) {
                std::cout << "# The Algorithm needs modification to find integer direction" << std::endl;
                ZoomPro_->routesToAdd_.clear();
                ZoomPro_->buildModel(pInst, zSolution_, routeSolution_,false);
                for (auto & routeObj : pInst->vehicles_) {
                    for (auto & routeObj : availableRoutes_[routeObj->vehicleID_]) {
          //              if (routeObj->reducedCost_ < -0.001)
                            ZoomPro_->routesToAdd_.push_back(routeObj);
                    }
                }
                ZoomPro_->updateModel(pInst, CompPro_->fractionalZ_);
                ZoomPro_->solveModel(pInst, zSolution_, routeSolution_, generatedRoutes_);
                setObjValue();

                std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
                std::cout << "+        Solution Result after Zoom Improve:       +" << std::endl;
                std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
                for (int r = 0; r < routeSolution_.size(); ++r) {
                    std::cout << routeSolution_[r]->toString();
                }
                if (previousObj > objValue_) {
                    previousObj = objValue_;
                    std::cout << "restarting CP after zoom improve" << std::endl;
                    goto CP;
                }
                else {
                    CompPro_->routesToAdd_.clear();
                    updateRoutesToAdd(2*pInst->nbRequests_, pInst);
                    CompPro_->buildModel(pInst, zSolution_, routeSolution_);
                    CompPro_->solveModel(pInst, zSolution_, routeSolution_, generatedRoutes_);
                    setObjValue();
                    // UPDATE DUAL VALUES AFTER SOLVING CP
                    ReducedPro_->requestDuals_ = CompPro_->requestDuals_;
                    ReducedPro_->vehicleDuals_ = CompPro_->vehicleDuals_;
                    /*ReducedPro_->buildModel(pInst, zSolution_, routeSolution_,false);
                    ReducedPro_->solveModel(pInst, zSolution_, routeSolution_, generatedRoutes_);*/
                    restartAlgorithm = false;
                }
            }
            else if (CompPro_->status_ == POSITIVE_VALUE) {
                std::cout << "# The Algorithm can not find further direction of descent and terminated" << std::endl;
                restartAlgorithm = false;
            }
            else if (CompPro_->status_ == INFEASIBLE) {
                std::cout << "# The Algorithm failed to optimized and terminated" << std::endl;
                restartAlgorithm = false;
            }
            else {
                std::cout << "# The Complementary Problems solved and find integer direction. " << std::endl;
                pInst->saveISUDRoutes(isudSolutionDir, epoch, isudIter_);
                previousObj = objValue_;
                isudIter_++;
                improveIter_++;
                restartAlgorithm = true;
                ReducedPro_->requestDuals_ = CompPro_->requestDuals_;
                ReducedPro_->vehicleDuals_ = CompPro_->vehicleDuals_;
                ReducedPro_->buildModel(pInst, zSolution_, routeSolution_,false);

                std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
                std::cout << "+         Solution Result after CP improve:        +" << std::endl;
                std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
                for (int r = 0; r < routeSolution_.size(); ++r) {
                    std::cout << routeSolution_[r]->toString();
                }
            }
        }
    }

    std::cout << "# Time spent on ISUD iteration  = " << isudTime_->dSinceStart().count() << " (seconds)" << std::endl;
    isudTime_->stop();
}

// Display function
std::string ISUDAlgorithm::toString() const {

    std::stringstream repStr;
    repStr << "#" << std::endl;
    repStr << "# Total waiting time plus the penalty    = " << objValue_ << std::endl;
    repStr << "# Number of requests that are not served = " << zSolution_.size() << std::endl;
    repStr << "# Time spent on ISUD improvement         = " << isudTime_->dSinceInit().count() << " (seconds)" << std::endl;
    for (int r = 0; r < routeSolution_.size(); ++r) {
        repStr << routeSolution_[r]->toString();
    }
    return repStr.str();
}

void ISUDAlgorithm::updateRoutesToAdd(int compDegree, PInstance &pInst) {
    calcIncMatrixFull();
//    calcIncMatrix();

    for (auto & vehicleObj : pInst->vehicles_) {
        if (availableRoutes_[vehicleObj->vehicleID_].size()>0) {
            updateRoutesIncDegree(vehicleObj->vehicleID_);
//            vehicleObj->bestReducedCost_ = availableRoutes_[vehicleObj->vehicleID_][0]->reducedCost_;
        }
    }
    pInst->sortVehicles(BEST_REDUCE_COST);
    for (auto & vehicleObj : pInst->vehicles_) {
        if (compDegree == 0) {
            for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]) {
                if (routeObj->incompatibilityDegree > 0)
                    break;
                if ((routeObj->incompatibilityDegree == 0) && (routeObj != vehicleObj->currentRoute_)&& (routeObj->reducedCost_ < -0.001)) {
                    ReducedPro_->routesToAdd_.push_back(routeObj);
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
            for (int r = availableRoutes_[vehicleObj->vehicleID_].size()-1; r >= 0 ; --r) {
                if (availableRoutes_[vehicleObj->vehicleID_][r]->incompatibilityDegree <= compDegree) {
                    if (availableRoutes_[vehicleObj->vehicleID_][r]->incompatibilityDegree == 0)
                        break;
                    else {
//                        if (availableRoutes_[vehicleObj->vehicleID_][r]->reducedCost_ < -0.001)
//                        if (availableRoutes_[vehicleObj->vehicleID_][r]->reducedCost_ <= 0)
                            CompPro_->routesToAdd_.push_back(availableRoutes_[vehicleObj->vehicleID_][r]);
                    }
                }
            }

        }
    }
    pInst->restVehicleOrder();
}

void ISUDAlgorithm::updateRoutesToAddZoom(PInstance &pInst) {
    // add compatible columns to the current solution
    for (auto & vehicleObj : pInst->vehicles_) {
        for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]) {
            ReducedPro_->routesToAdd_.push_back(routeObj);
            /*bool routeIsAdded = false;
            for (auto & fracRouteObj: CompPro_->fractionalRoutes_) {
                if (fracRouteObj->getRouteId() == routeObj->getRouteId()) {
                    routeIsAdded = true;
                    break;
                }
            }
            if (!routeIsAdded) {
                if ((routeObj != vehicleObj->currentRoute_)&& (routeObj->reducedCost_ < -0.001)) {
                    if (routeObj->incompatibilityDegree == 0) {
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
                                for (auto &requestID: routeObj->routeRequests) {
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

bool ISUDAlgorithm::isCompatible(PRoute &solutionRoute, PRoute &comingRoute, std::unordered_map<int, int> &requestToOrder) {
    Eigen::MatrixXd solutionPattern = Eigen::MatrixXd::Zero(requestToOrder.size(), 1);
    Eigen::MatrixXd comingPattern = Eigen::MatrixXd::Zero(requestToOrder.size(),1);

    for (int i = 0; i < solutionRoute->routeRequests.size(); ++i) {
        solutionPattern(requestToOrder[solutionRoute->routeRequests[i]], 0) = 1;
    }
    for (int i = 0; i < comingRoute->routeRequests.size(); ++i) {
        comingPattern(requestToOrder[comingRoute->routeRequests[i]], 0) = 1;
    }

    Eigen::MatrixXd multiplication = solutionPattern.transpose() * comingPattern;
    if (multiplication(0,0) == solutionRoute->routeRequests.size())
        return true;
    else
        return false;
}

void ISUDAlgorithm::solveISUDMIP(PInstance &pInst, int epoch, string isudSolutionDir) {
    double previousObj = objValue_;

    isudTime_->start();
    // improve by solving the Reduced problem

    ZoomPro_->routesToAdd_.clear();
    ZoomPro_->buildModel(pInst, zSolution_, routeSolution_,false);
    for (auto & routeObj : pInst->vehicles_) {
        for (auto & routeObj : availableRoutes_[routeObj->vehicleID_]) {
            ZoomPro_->routesToAdd_.push_back(routeObj);
            /*if (routeObj->reducedCost_ < -0.001)
                ZoomPro_->routesToAdd_.push_back(routeObj);*/
        }
    }
    ZoomPro_->updateModel(pInst, CompPro_->fractionalZ_);
    ZoomPro_->solveModel(pInst, zSolution_, routeSolution_, generatedRoutes_);

    previousObj = objValue_;
    std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cout << "+        Solution Result after Zoom Improve:       +" << std::endl;
    std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    for (int r = 0; r < routeSolution_.size(); ++r) {
        std::cout << routeSolution_[r]->toString();
    }

    CompPro_->routesToAdd_.clear();
    updateRoutesToAdd(2*pInst->nbRequests_, pInst);
    CompPro_->buildModel(pInst, zSolution_, routeSolution_);
    CompPro_->solveModel(pInst, zSolution_, routeSolution_, generatedRoutes_);
    setObjValue();
    // UPDATE DUAL VALUES AFTER SOLVING CP
    ReducedPro_->requestDuals_ = CompPro_->requestDuals_;
    ReducedPro_->vehicleDuals_ = CompPro_->vehicleDuals_;

    ReducedPro_->buildModel(pInst, zSolution_, routeSolution_,false);
    ReducedPro_->solveModel(pInst, zSolution_, routeSolution_, generatedRoutes_);
    setObjValue();
    isudTime_->stop();
}











