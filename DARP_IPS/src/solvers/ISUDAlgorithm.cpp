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
    objValue_ = 0;
    isudTime_ = new Tools::Timer(); isudTime_->init();
    improveIter_ = 0;
}

ISUDAlgorithm::~ISUDAlgorithm() {
    delete isudTime_;
}


// this function create initial routes serving only one request and fill zSolution_ with available requests
// Reduced problem is also solved to initialized dual costs
void ISUDAlgorithm::initialization(PInstance &pInst) {
//    createEmptyRoute(pInst);
    ReducedPro_->routesToAdd_.clear();
    generatedRoutes_.clear();
    for (int v = 0; v < pInst->nbVehicles_; ++v) {
        if (pInst->vehicles_[v]->departID_ != pInst->vehicles_[v]->emptyRoute_->routeNodes_[0]->nodeID_)
            pInst->vehicles_[v]->setEmptyRoute(pInst);
        generatedRoutes_.insert(std::pair <std::string , PRoute> ((pInst->vehicles_[v]->emptyRoute_)->name_ ,
                                                                  (pInst->vehicles_[v]->emptyRoute_)));
        ReducedPro_->routesToAdd_.push_back(pInst->vehicles_[v]->emptyRoute_);
        generatedRoutes_.insert(std::pair <std::string , PRoute> ((pInst->vehicles_[v]->currentRoute_)->name_ ,
                                                                  (pInst->vehicles_[v]->currentRoute_)));
    }

    for (int i = pInst->nbRequests_- pInst->nbNewRequests_; i < pInst->nbRequests_; ++i) {
        zSolution_.push_back(pInst->requests_[i]);
        for (int v = 0; v < pInst->nbVehicles_; ++v) {
            // creating and empty route
            PRoute newRoute = std::make_shared<Route>(pInst->vehicles_[v]->vehicleID_);
//            generatedRoutes_.emplace_back(std::make_shared<Route>(pInst->vehicles_[v]->vehicleID_));

            newRoute->addNode(pInst->instGraph_->nodes_[pInst->vehicles_[v]->departID_],
                              pInst->vehicles_[v]->departTime_, pInst->vehicles_[v]->numPassengers_);
            static const NodeType nodeTypesInOrder[] = { PICKUP, DROPOFF};
            for ( const auto t : nodeTypesInOrder)
            {
                std::string ID = Tools::createNodeID(pInst->requests_[i]->getRequestId(), t);
                newRoute->addNode(pInst->instGraph_->nodes_[ID], pInst->vehicles_[v]->departTime_,
                                  pInst->vehicles_[v]->numPassengers_);
            }
            newRoute->addNode(pInst->instGraph_->nodes_[pInst->vehicles_[v]->sinkID_],
                              pInst->vehicles_[v]->departTime_, pInst->vehicles_[v]->numPassengers_);

            generatedRoutes_.insert(std::pair <std::string , PRoute> (newRoute->name_ , newRoute));
            availableRoutes_[pInst->vehicles_[v]->vehicleID_].push_back(newRoute);
            ReducedPro_->routesToAdd_.push_back(newRoute);
        }
    }
    std::cout << "# -----SOLVING THE REDUCED PROBLEM AT THE START OF EPOCH-------" << std::endl;
    isudTime_->start();
    ReducedPro_->buildModel(pInst, zSolution_, routeSolution_);
    ReducedPro_->solveModel(pInst, zSolution_, routeSolution_, generatedRoutes_);
    isudTime_->stop();
    std::cout << "# Time spent on ISUD initialization    = " << isudTime_->dSinceInit().count() << " (seconds)" << std::endl;
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

// function to calculate incompatibility matrix
void ISUDAlgorithm::calcIncMatrix() {
    incRequestToOrder_.clear();
    int orderCount = 0;
    sort(routeSolution_.begin(),routeSolution_.end(),[](const PRoute &lhs, const PRoute &rhs){
        return lhs->routeRequests.size() < rhs->routeRequests.size();});
    if (routeSolution_.back()->routeRequests.size() <= 1) {

        for (int i = 0; i < routeSolution_.size(); ++i) {
            incRequestToOrder_[routeSolution_[i]->routeRequests[0]] = orderCount;
            orderCount ++;
        }
        for (int i = 0; i < zSolution_.size(); ++i) {
            incRequestToOrder_[zSolution_[i]->getRequestId()] = orderCount;
            orderCount++;
        }
        incMatrix_ = Eigen::MatrixXd::Zero(1,orderCount);
    }
    else {
        incMatrix_ = Eigen::MatrixXd::Zero(0,0);
        for (int r = 0; r < routeSolution_.size(); ++r) {
            if (routeSolution_[r]->routeRequests.size() >= 1) {
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
        incMatrix_.conservativeResize(incMatrix_.rows() , incMatrix_.cols()+zSolution_.size());
        incMatrix_.topRightCorner(incMatrix_.rows(), zSolution_.size()) = Eigen::MatrixXd::Zero(incMatrix_.rows(), zSolution_.size());
        for (int i = 0; i < zSolution_.size(); ++i) {
            incRequestToOrder_[zSolution_[i]->getRequestId()] = orderCount;
            orderCount ++;
        }
    }

    /*std::cout << "Incompatibility Matrix:" << std::endl;
    std::cout << incMatrix_ << std::endl;*/
}

// function to calculate incompatibility degree of a route
void ISUDAlgorithm::calcIncompatibility(PRoute &route) {
    Eigen::MatrixXd pattern = Eigen::MatrixXd::Zero(incRequestToOrder_.size(),1);
    for (int i = 0; i < route->routeRequests.size(); ++i) {
        pattern(incRequestToOrder_[route->routeRequests[i]],0) = 1;
    }
    /*std::cout << "Route Pattern: " << std::endl;
    std::cout << pattern << std::endl;*/
    Eigen::MatrixXd multiplication = incMatrix_ * pattern;

    /*std::cout << "multiplication: " << std::endl;
    std::cout << multiplication << std::endl;*/

    route->incompatibilityDegree = 0;
    for (int i = 0; i < multiplication.rows(); ++i) {
        if (multiplication(i,0) != 0)
            route->incompatibilityDegree ++;
    }
    /*std::cout << "incompatibility degree: " << std::endl;
    std::cout << route->incompatibilityDegree << std::endl;*/
}

// this function update the incompatibility degree of availableRoutes and
// order them based on the incompatibility degree and reduced cost
void ISUDAlgorithm::updateRoutesIncDegree(int &vehicleID) {

    for (int r = 0; r < availableRoutes_[vehicleID].size(); ++r) {
        calcIncompatibility(availableRoutes_[vehicleID][r]);
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
        if (availableRoutes_[vehicleID][r]->reducedCost_ >= -0.00001)
            availableRoutes_[vehicleID].erase(availableRoutes_[vehicleID].begin()+r);
    }
}

// this function create empty routes that only goes form source to sink
void ISUDAlgorithm::createEmptyRoute(PInstance &pInst) {
    for (int v = 0; v < pInst->nbVehicles_; ++v) {
        // creating and empty route
        /*PRoute newRoute = std::make_shared<Route>(pInst->vehicles_[v]->vehicleID_);
//            generatedRoutes_.emplace_back(std::make_shared<Route>(pInst->vehicles_[v]->vehicleID_));


        static const NodeType nodeTypesInOrder[] = { SOURCE, SINK};
        for ( const auto t : nodeTypesInOrder)
        {
            std::string ID;
            ID = Tools::createNodeID(0, SOURCE);

            newRoute->addNode(pInst->instGraph_->nodes_[ID], pInst->vehicles_[v]->departTime_,
                              pInst->vehicles_[v]->numPassengers_);
        }*/
//        generatedRoutes_.insert(std::pair <std::string , PRoute> (newRoute->name_ , newRoute));

    }
}

// improve the solution through solving Reduce problem
void ISUDAlgorithm::reduceProImprove(PInstance &pInst) {
    for (int v = 0; v < pInst->nbVehicles_; ++v) {
        updateReducedCosts(pInst->vehicles_[v]->vehicleID_);
        if (availableRoutes_[pInst->vehicles_[v]->vehicleID_].size()>0) {
            calcIncMatrix();
            updateRoutesIncDegree(pInst->vehicles_[v]->vehicleID_);
            if (availableRoutes_[pInst->vehicles_[v]->vehicleID_][0]->incompatibilityDegree == 0) {
                ReducedPro_->routesToAdd_.clear();
                ReducedPro_->routesToAdd_.push_back(pInst->vehicles_[v]->emptyRoute_);   // needs modification for all routes
                ReducedPro_->routesToAdd_.push_back(availableRoutes_[pInst->vehicles_[v]->vehicleID_][0]);
                ReducedPro_->updateModel(pInst, routeSolution_);
                ReducedPro_->solveModel(pInst, zSolution_, routeSolution_, generatedRoutes_);
                std::cout << "Solution Result after RP improve:" << std::endl;
                for (int r = 0; r < routeSolution_.size(); ++r) {
                    std::cout << routeSolution_[r]->toString();
                }

            }
        }
    }

}

// improve the solution through solving Complementary problem
void ISUDAlgorithm::CPImprove(PInstance &pInst) {
    calcIncMatrix();
    CompPro_->routesToAdd_.clear();
    for (int v = 0; v < pInst->nbVehicles_; ++v) {
        CompPro_->routesToAdd_.push_back(pInst->vehicles_[v]->emptyRoute_);
        updateReducedCosts(pInst->vehicles_[v]->vehicleID_);
        if (availableRoutes_[pInst->vehicles_[v]->vehicleID_].size()>0) {
            updateRoutesIncDegree(pInst->vehicles_[v]->vehicleID_);
            for (int r = availableRoutes_[pInst->vehicles_[v]->vehicleID_].size()-1; r >= 0 ; --r) {
                if (availableRoutes_[pInst->vehicles_[v]->vehicleID_][r]->incompatibilityDegree > 0) {
                    CompPro_->routesToAdd_.push_back(availableRoutes_[pInst->vehicles_[v]->vehicleID_][r]);
                }
                if (availableRoutes_[pInst->vehicles_[v]->vehicleID_][r]->incompatibilityDegree == 0)
                    break;
            }
        }
    }
    if (CompPro_->routesToAdd_.size() > 2) {
        CompPro_->buildModel(pInst, zSolution_, routeSolution_);
        CompPro_->solveModel(pInst, zSolution_, routeSolution_, generatedRoutes_);
        std::cout << "Solution Result after CP improve:" << std::endl;
        for (int r = 0; r < routeSolution_.size(); ++r) {
            std::cout << routeSolution_[r]->toString();
        }
    }
}

void ISUDAlgorithm::solveISUD(PInstance &pInst, int epoch) {
    isudTime_->start();
    // improve by solving the Reduced problem


    int improveFlag = 1;
    while (improveFlag == 1) {
        ReducedPro_->routesToAdd_.clear();
        improveFlag = 0;
        /*if ((epoch == 0) && (improveIter_ == 0)) {
            for (int v = 0; v < pInst->nbVehicles_; ++v) {
                for (int r = 0; r < availableRoutes_[pInst->vehicles_[v]->vehicleID_].size(); ++r) {
                    ReducedPro_->routesToAdd_.push_back(availableRoutes_[pInst->vehicles_[v]->vehicleID_][r]);
                }
            }
            improveIter_ = 1;
        }
        else {
            updateRoutesToAdd(0, pInst);
        }*/
        updateRoutesToAdd(0, pInst);

        if (ReducedPro_->routesToAdd_.size() > 0) {
            std::cout << "# IMPROVE THE SOLUTION BY SOLVING THE REDUCED PROBLEM" << std::endl;
            improveStatus_ = 1;
            for (int v = 0; v < pInst->nbVehicles_; ++v)
                ReducedPro_->routesToAdd_.push_back(pInst->vehicles_[v]->emptyRoute_);

            ReducedPro_->updateModel(pInst, routeSolution_);
            ReducedPro_->solveModel(pInst, zSolution_, routeSolution_, generatedRoutes_);
            /*std::cout << "Solution Result after RP improve:" << std::endl;
            for (int r = 0; r < routeSolution_.size(); ++r) {
                std::cout << routeSolution_[r]->toString();
            }*/
            improveFlag = 1;
        }
    }
    // improve the solution by solving complementary Problem

    improveFlag = 1;
    while (improveFlag == 1) {
        improveFlag = 0;
        CompPro_->routesToAdd_.clear();

        updateRoutesToAdd(pInst->nbRequests_, pInst);
        if (CompPro_->routesToAdd_.size() > 0) {
            std::cout << "# IMPROVE THE SOLUTION BY SOLVING THE COMPLEMENTARY PROBLEM" << std::endl;
            for (int v = 0; v < pInst->nbVehicles_; ++v)
                CompPro_->routesToAdd_.push_back(pInst->vehicles_[v]->emptyRoute_);

            CompPro_->buildModel(pInst, zSolution_, routeSolution_);
            CompPro_->solveModel(pInst, zSolution_, routeSolution_, generatedRoutes_);
            if (CompPro_->status_ == FRACTIONAL) {
                std::cout << "The Algorithm needs modification fo find integer direction" << std::endl;
                break;
            }
            else if (CompPro_->status_ == POSITIVE_VALUE) {
                std::cout << "The Algorithm can not find further direction of descent and terminated" << std::endl;
                break;
            }
            else {
                std::cout << "The Complementary Problems solved and find integer direction. " << std::endl;
                improveFlag = 1;
                improveIter_++;
                // test the reduced cost
                ReducedPro_->routesToAdd_.clear();
                ReducedPro_->buildModel(pInst, zSolution_, routeSolution_);
                ReducedPro_->solveModel(pInst, zSolution_, routeSolution_, generatedRoutes_);

            }
        }
    }



    std::cout << "# Time spent on ISUD iteration  = " << isudTime_->dSinceStart().count() << " (seconds)" << std::endl;
    objValue_ = 0.0;
    for (int r = 0; r < routeSolution_.size(); ++r) {
        objValue_ += routeSolution_[r]->totalDelay_;
    }
    for (int i = 0; i < zSolution_.size(); ++i) {
        objValue_+= zSolution_[i]->penalty_;
    }
    isudTime_->stop();
}

// Display function
std::string ISUDAlgorithm::toString() const {

    std::stringstream repStr;
    repStr << "#" << std::endl;
    repStr << "# Total waiting time plus the penalty    = " << objValue_ << std::endl;
    repStr << "# NUmber of requests that are not served = " << zSolution_.size() << std::endl;
    repStr << "# Time spent on ISUD improvement         = " << isudTime_->dSinceInit().count() << " (seconds)" << std::endl;
    for (int r = 0; r < routeSolution_.size(); ++r) {
        repStr << routeSolution_[r]->toString();
    }
    return repStr.str();
}

void ISUDAlgorithm::updateRoutesToAdd(int compDegree, PInstance &pInst) {
    calcIncMatrix();
    for (int v = 0; v < pInst->nbVehicles_; ++v) {
        updateReducedCosts(pInst->vehicles_[v]->vehicleID_);
        if (availableRoutes_[pInst->vehicles_[v]->vehicleID_].size()>0) {
            updateRoutesIncDegree(pInst->vehicles_[v]->vehicleID_);
            if (compDegree == 0) {
                if ((availableRoutes_[pInst->vehicles_[v]->vehicleID_][0]->incompatibilityDegree == 0) &&
                    (availableRoutes_[pInst->vehicles_[v]->vehicleID_][0] != pInst->vehicles_[v]->currentRoute_)) {
                    ReducedPro_->routesToAdd_.push_back(availableRoutes_[pInst->vehicles_[v]->vehicleID_][0]);
                }
            }
            else {
                for (int r = availableRoutes_[pInst->vehicles_[v]->vehicleID_].size()-1; r >= 0 ; --r) {
                    if (availableRoutes_[pInst->vehicles_[v]->vehicleID_][r]->incompatibilityDegree <= compDegree) {
                        if (availableRoutes_[pInst->vehicles_[v]->vehicleID_][r]->incompatibilityDegree == 0)
                            break;
                        else
                            CompPro_->routesToAdd_.push_back(availableRoutes_[pInst->vehicles_[v]->vehicleID_][r]);;
                    }
                }

            }
        }
    }
}



