//
// Created by Ella on 2021-10-09.
//

#include "ISUDAlgorithm.h"

//---------------------------------------------------------------------------------------------
//  Reduced Problem class
//  Build and solve the Reduced problem of the ISUD
//---------------------------------------------------------------------------------------------

ISUDAlgorithm::ISUDAlgorithm(PInstance &pInst) {
    ReducedPro_ = std::make_shared<ReducedProblem>(pInst);
}

// this function create initial routes serving only one request and fill zSolution_ with available requests
void ISUDAlgorithm::initialization(PInstance &pInst) {
    for (int i = 0; i < pInst->nbRequests_; ++i) {
        zSolution_.push_back(pInst->requests_[i]);
        createEmptyRoute(pInst);
        for (int v = 0; v < pInst->nbVehicles_; ++v) {
            // creating and empty route
            PRoute newRoute = std::make_shared<Route>(pInst->vehicles_[v]->vehicleID_);
//            generatedRoutes_.emplace_back(std::make_shared<Route>(pInst->vehicles_[v]->vehicleID_));


            static const NodeType nodeTypesInOrder[] = { SOURCE, PICKUP, DROPOFF, SINK};
            for ( const auto t : nodeTypesInOrder)
            {
                std::string ID;
                if (t==SOURCE || t == SINK)
                    ID = Tools::createNodeID(0, t);
                else
                    ID = Tools::createNodeID(pInst->requests_[i]->getRequestId(), t);
                /*generatedRoutes_.back()->addNode(pInst->mainGraph_->nodes_[ID], pInst->vehicles_[v]->departTime_,
                                           pInst->vehicles_[v]->numPassengers_);*/
                newRoute->addNode(pInst->mainGraph_->nodes_[ID], pInst->vehicles_[v]->departTime_,
                                  pInst->vehicles_[v]->numPassengers_);
            }
            generatedRoutes_.insert(std::pair <std::string , PRoute> (newRoute->name_ , newRoute));
            availableRoutes_[pInst->vehicles_[v]->vehicleID_].push_back(newRoute);
            ReducedPro_->routesToAdd_.push_back(newRoute);
        }
    }
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
    if (routeSolution_.back()->routeRequests.size() == 1) {
        incMatrix_ = Eigen::MatrixXd::Zero(1,routeSolution_.size()+zSolution_.size());
        for (int i = 0; i < routeSolution_.size(); ++i) {
            incRequestToOrder_[routeSolution_[i]->routeRequests[0]] = orderCount;
            orderCount ++;
        }
        for (int i = 0; i < zSolution_.size(); ++i) {
            incRequestToOrder_[zSolution_[i]->getRequestId()] = orderCount;
            orderCount++;
        }
    }
    else {
        incMatrix_ = Eigen::MatrixXd::Zero(0,0);
        for (int r = 0; r < routeSolution_.size(); ++r) {
            for (int i = 0; i < routeSolution_[r]->routeRequests.size(); ++i) {
                incRequestToOrder_[routeSolution_[r]->routeRequests[i]] = orderCount;
                orderCount++;
            }
            int nbRows = routeSolution_[r]->routeRequests.size() - 1;
            int rowSize = incMatrix_.rows();
            int colSize = incMatrix_.cols();
            if (nbRows == 0) {
                incMatrix_.conservativeResize(rowSize , colSize+1);
                incMatrix_.bottomRightCorner(rowSize, 1) = Eigen::MatrixXd::Zero(rowSize, 1);
            }
            else {

                incMatrix_.conservativeResize(rowSize+nbRows , colSize+nbRows+1);
                incMatrix_.bottomRightCorner(nbRows, nbRows+1) = calcM2Matrix(routeSolution_[r]);
                incMatrix_.bottomLeftCorner(nbRows, colSize) = Eigen::MatrixXd::Zero(nbRows, colSize);
                incMatrix_.topRightCorner(rowSize, nbRows+1) = Eigen::MatrixXd::Zero(rowSize, nbRows+1);
            }
        }
        incMatrix_.conservativeResize(incMatrix_.rows() , incMatrix_.cols()+zSolution_.size());
        for (int i = 0; i < zSolution_.size(); ++i) {
            incRequestToOrder_[zSolution_[i]->getRequestId()] = orderCount;
            orderCount ++;
        }
    }


}

// function to calculate incompatibility degree of a route
void ISUDAlgorithm::calcIncompatibility(PRoute &route) {
    Eigen::MatrixXd pattern = Eigen::MatrixXd::Zero(incRequestToOrder_.size(),1);
    for (int i = 0; i < route->routeRequests.size(); ++i) {
        pattern(incRequestToOrder_[route->routeRequests[i]],0) = 1;
    }
    Eigen::MatrixXd multiplication = incMatrix_ * pattern;
    route->incompatibilityDegree = 0;
    for (int i = 0; i < multiplication.rows(); ++i) {
        if (multiplication(i,0) > 0)
            route->incompatibilityDegree ++;
    }
}

// this function update the incompatibility degree of availableRoutes and
// order them based on the incompatibility degree and reduced cost
void ISUDAlgorithm::updateRoutesIncDegree(int &vehicleID) {
    /*for (std::map<int, std::vector<PRoute>>::iterator it = availableRoutes_.begin(); it != availableRoutes_.end(); it++)
    {
        for (int r = 0; r < it->second.size(); ++r) {
            calcIncompatibility(it->second[r]);
        }
        // sort the routes based on their incompatibility degree
        sort(it->second.begin(),it->second.end(),[](const PRoute &lhs, const PRoute &rhs){
            return std::tie(lhs->incompatibilityDegree,lhs->reducedCost_) < std::tie(rhs->incompatibilityDegree,rhs->reducedCost_);
        });
    }*/

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
    /*for (std::map<int, std::vector<PRoute>>::iterator it = availableRoutes_.begin(); it != availableRoutes_.end(); it++) {
        for (int r = 0; r < availableRoutes_.size(); ++r) {
            it->second[r]->updateReducedCost(ReducedPro_->requestDuals_, ReducedPro_->vehicleDuals_,
                                             ReducedPro_->requestToOrder_);
            if (it->second[r]->reducedCost_ > 0)
                it->second.erase(it->second.begin() + r);
        }
    }*/
    for (int r = 0; r < availableRoutes_.size(); ++r) {
        availableRoutes_[vehicleID][r]->updateReducedCost(ReducedPro_->requestDuals_, ReducedPro_->vehicleDuals_,
                                               ReducedPro_->requestToOrder_);
        if (availableRoutes_[vehicleID][r]->reducedCost_ > 0)
            availableRoutes_[vehicleID].erase(availableRoutes_[vehicleID].begin()+r);
    }
}

// improve the solution through solving Reduce problem
void ISUDAlgorithm::reduceProImprove(PInstance &pInst) {
    for (int v = 0; v < pInst->nbVehicles_; ++v) {
        if (availableRoutes_[pInst->vehicles_[v]->vehicleID_].size()>0) {
            if (availableRoutes_[pInst->vehicles_[v]->vehicleID_][0]->incompatibilityDegree == 0) {
                calcIncMatrix();
                updateRoutesIncDegree(pInst->vehicles_[v]->vehicleID_);
                ReducedPro_->routesToAdd_.clear();
                ReducedPro_->routesToAdd_.push_back(availableRoutes_[pInst->vehicles_[v]->vehicleID_][0]);
                ReducedPro_->updateModel(pInst, routeSolution_);
                ReducedPro_->solveModel(pInst, zSolution_, routeSolution_, generatedRoutes_);
                std::cout << "Solution Result after RP improve:" << std::endl;
                for (int r = 0; r < routeSolution_.size(); ++r) {
                    std::cout << routeSolution_[r]->toString();
                }
                updateReducedCosts(pInst->vehicles_[v]->vehicleID_);
            }
        }
    }

    /*if (availableRoutes_[0]->incompatibilityDegree == 0) {
        ReducedPro_->routesToAdd_.push_back(availableRoutes_[0]);
        ReducedPro_->updateModel(pInst, routeSolution_);
        ReducedPro_->solveModel(pInst, zSolution_, routeSolution_, generatedRoutes_);
        updateReducedCosts();
        reduceProImprove(pInst);
    }*/
}

void ISUDAlgorithm::createEmptyRoute(PInstance &pInst) {
    for (int v = 0; v < pInst->nbVehicles_; ++v) {
        // creating and empty route
        PRoute newRoute = std::make_shared<Route>(pInst->vehicles_[v]->vehicleID_);
//            generatedRoutes_.emplace_back(std::make_shared<Route>(pInst->vehicles_[v]->vehicleID_));


        static const NodeType nodeTypesInOrder[] = { SOURCE, SINK};
        for ( const auto t : nodeTypesInOrder)
        {
            std::string ID;
            ID = Tools::createNodeID(0, SOURCE);

            newRoute->addNode(pInst->mainGraph_->nodes_[ID], pInst->vehicles_[v]->departTime_,
                              pInst->vehicles_[v]->numPassengers_);
        }
        generatedRoutes_.insert(std::pair <std::string , PRoute> (newRoute->name_ , newRoute));
        availableRoutes_[pInst->vehicles_[v]->vehicleID_].push_back(newRoute);
        ReducedPro_->routesToAdd_.push_back(newRoute);
    }
}


// it should be deleted
const std::vector<PRoute> createInitialRoutes(PInstance &pInst) {
    std::vector<PRoute> initRoutes;
    for (int v = 0; v < pInst->nbVehicles_; ++v) {
        for (int i = 0; i < pInst->nbRequests_; ++i) {
            // creating and empty route
            initRoutes.emplace_back(std::make_shared<Route>(pInst->vehicles_[v]->vehicleID_));
            static const NodeType nodeTypesInOrder[] = { SOURCE, PICKUP, DROPOFF, SINK};
            for ( const auto t : nodeTypesInOrder)
            {
                std::string ID;
                if (t==SOURCE || t == SINK)
                    ID = Tools::createNodeID(0, t);
                else
                    ID = Tools::createNodeID(pInst->requests_[i]->getRequestId(), t);
                initRoutes.back()->addNode(pInst->mainGraph_->nodes_[ID], pInst->vehicles_[v]->departTime_,
                                           pInst->vehicles_[v]->numPassengers_);
            }
        }
    }
    return initRoutes;
}
