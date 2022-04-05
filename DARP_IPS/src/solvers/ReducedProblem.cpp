//
// Created by Ella on 2021-11-10.
//

#include "ReducedProblem.h"

//---------------------------------------------------------------------------------------------
//  Reduced Problem class
//  Build and solve the Reduced problem of the ISUD
//---------------------------------------------------------------------------------------------

// Constructor and Destructor
ReducedProblem::ReducedProblem() : MasterModeler() {

    // defining variable
    zVar_ = IloNumVarArray(env_, 0.0, 0.0, ILOFLOAT);
    routeVar_ = IloNumVarArray(env_, 0.0, 0.0, ILOFLOAT);
}


// this function initialized the model and define empty set of constraints
void ReducedProblem::ResetRPModel() {

    try {
        int modelExist = 0;
        for (int r = routeVar_.getSize()-1; r >= 0; --r) {
            routeVar_[r].end();
            routeVar_.remove(r,1);
            modelExist = 1;
        }
        for (int i = zVar_.getSize()-1; i >= 0; --i) {
            zVar_[i].end();
            zVar_.remove(i,1);
            modelExist = 1;
        }
        if (modelExist == 1) {
            Model_.remove(requestConst_);
            Model_.remove(vehicleConst_);
        }

    }
    catch (IloException& e) {
        std::cout << e << std::endl;
    }
}

// this function adds routeVar to the model
void ReducedProblem::addRouteVar(PRoute &newRoute) {
        MasterModeler::addRouteVar(routeVar_, newRoute, POSITIVE);
}

// this function adds zVar to the model used for the routes that served only one request
void ReducedProblem:: addZVar(PRequest &request) {
    MasterModeler::addZVar(zVar_, request, POSITIVE);
}

// this function add one route at each iteration of the algorithm during one epoch
void ReducedProblem::updateModel(PInstance &pInst, std::vector<PRoute> &routeSolution) {
    if (routesToAdd_.size() == 0) {
        std::cout << "There is no route to be added" << std::endl;
        throw Tools::myException("The input route is empty, No new column is passed to be added", __LINE__);
    }

    // add the new compatible column to the model
    for (int i = 0; i < routesToAdd_.size(); ++i) {
        addRouteVar(routesToAdd_[i]);
    }

    // add compatible z variables
    // just z variables related to requests that are served in routes with one request are compatible
    for (int r = 0; r < routeSolution.size(); ++r) {
        if (routeSolution[r]->routeRequests.size() == 1) {
            addZVar(*routeSolution[r]->routeNodes_[1]->related_Request_);
        }
    }
}


// this function build the model at the start of each epoch
void ReducedProblem::buildModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                                bool emptyStart) {

    // model initialization (defining empty set of constraints and adding objective)
    ResetRPModel();
    int rhs = 1;
    MasterModeler::initializeModel(pInst, rhs);


 //   MasterModeler::initializeModel(pInst, rhs);

        // adding request columns (z variables)
    for (auto & zSol : zSolution)
        addZVar(zSol);

    // adding route solution columns
    for (auto & routeSol : routeSolution) {
        if (emptyStart) {
            for (auto nodeObj : routeSol->routeNodes_) {
                if (nodeObj->type_ == PICKUP)
                    addZVar(*nodeObj->related_Request_);
            }
        }
        else {
            addRouteVar(routeSol);
            // add related compatible z variables
            if (routeSol->routeRequests.size() == 1) {
                addZVar(*routeSol->routeNodes_[1]->related_Request_);
            }
        }
    }

    //adding new route variables
    for (auto & routeObj : routesToAdd_)
        addRouteVar(routeObj);
}

// this function solve the model and remove all columns except than the current base
void ReducedProblem::solveModel(PInstance &pInst, std::vector<PRequest> &zSolution,
                                std::vector<PRoute> &routeSolution, std::unordered_map<std::string ,
                                PRoute> &generatedRoutes) {
    try {

        Cplex_ = IloCplex(Model_);
 //       env_.out() << Model_;
        Cplex_.solve();

        // getting dual values
        requestDuals_.clear();
        vehicleDuals_.clear();

        // define dual container size
        requestDuals_ = IloNumArray(env_, pInst->nbRequests_);
        vehicleDuals_ = IloNumArray(env_, pInst->nbVehicles_);

        std::cout << "REDUCED DUALS:" << std::endl;
        for (auto &requestObj: pInst->requests_) {
            if (requestObj->requestStatus_ == NO_ACTION) {
                int rowIndex = requestToOrder_[requestObj->getRequestId()];
                requestDuals_[rowIndex] = Cplex_.getDual(requestConst_[rowIndex]);
                requestObj->dual_ = requestDuals_[rowIndex];
                std::cout << "requestDuals[" << requestObj->getRequestId() << "]: " << requestObj->dual_
                          << std::endl;
            }
        }

        std::cout << "VEHICLE DUALS:" << std::endl;
        for (auto &vehicleObj: pInst->vehicles_) {
            vehicleDuals_[vehicleObj->vehicleID_] = Cplex_.getDual(vehicleConst_[vehicleObj->vehicleID_]);
            vehicleObj->dual_ = vehicleDuals_[vehicleObj->vehicleID_];
            std::cout << "vehicleDuals[" << vehicleObj->vehicleID_ << "]: " << vehicleObj->dual_ << std::endl;
        }

        // printing solution status
        std::cout << MasterModeler::toString();

        // saving the result and remove out of base variables
        zSolution.clear();
        routeSolution.clear();

        IloNumArray zVal(env_);
        IloNumArray routeVal(env_);


        Cplex_.getValues(zVal, zVar_);
        Cplex_.getValues(routeVal, routeVar_);
//        env_.out() << routeVal << std::endl;
//        env_.out() << zVal << std::endl;

        for (int r = routeVal.getSize()-1; r >= 0; --r) {
            /*if (routeVal[r] > 0)
                std::cout << routeVal[r] << std::endl;*/

            if (routeVal[r] > 0.1) {
                routeSolution.push_back(generatedRoutes[routeVar_[r].getName()]);
                pInst->vehicles_[generatedRoutes[routeVar_[r].getName()]->vehicleID_]->setCurrentRoute(generatedRoutes[routeVar_[r].getName()]);
            }
            else {
                routeVar_[r].end();
                routeVar_.remove(r,1);
            }
        }
//        std::cout << "------------" << std::endl;
        for (int i = zVal.getSize()-1; i >= 0; --i) {
            /*if (zVal[i] > 0)
                std::cout << zVal[i] << std::endl;*/
            if (zVal[i] > 0.9)
                zSolution.push_back(pInst->nameToRequest_[zVar_[i].getName()]);
            else {
                zVar_[i].end();
                zVar_.remove(i,1);
            }
        }
        int nbRequests = 0;
        for (auto & requestObj: pInst->requests_) {
            if (requestObj->requestStatus_ == NO_ACTION)
                nbRequests++;
        }
        std::cout << "# from " << nbRequests << " request, " << nbRequests - zSolution.size()
        << " are selected to served." << std::endl;
    }
    catch (IloException& e) {
        std::cout << e << std::endl;
    }

}


// function to check whether two routes are column disjoint or not
bool ReducedProblem::isColumnDisjoint(vector<PRoute> &routeSet, PRoute &newRoute, std::unordered_map<int, int> &requestToOrder){
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(requestToOrder.size(), routeSet.size()+1);
    vector<PRoute> selectedRoutes = routeSet;
    selectedRoutes.push_back(newRoute);
    for (int r = 0; r < selectedRoutes.size(); ++r) {
        for (int i = 0; i < selectedRoutes[r]->routeRequests.size(); ++i)
            A(requestToOrder[selectedRoutes[r]->routeRequests[i]],r) = 1;
    }
    Eigen::MatrixXd ATA = A.transpose()*A;
    // setting diagonal elements to zero
    for (int i = 0; i < ATA.rows(); ++i) {
        ATA(i,i) = 0;
    }
    if (ATA == Eigen::MatrixXd::Zero(ATA.rows(), ATA.cols()))
        return true;
    else
        return false;
}

//************************************************************************
// Display function
//************************************************************************
std::string ReducedProblem::toString() const {
    std::stringstream repStr;
    repStr << std::endl;
    repStr << "# =======================  REDUCED PROBLEM SOLVED  ======================= " << std::endl;
    repStr << MasterModeler::toString();
    return repStr.str();
}

// function to check whether the route is repeated before
bool ReducedProblem::isColumnRepeat(vector<PRoute> &routeSet, PRoute &newRoute,
                                    std::unordered_map<int, int> &requestToOrder) {
    Eigen::MatrixXd newCol = Eigen::MatrixXd::Zero(requestToOrder.size(), 1);

    for (int i = 0; i < newRoute->routeRequests.size(); ++i)
        newCol(requestToOrder[newRoute->routeRequests[i]],0) = 1;

    for (int r = 0; r < routeSet.size(); ++r) {
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(requestToOrder.size(), 1);
        for (int i = 0; i < routeSet[r]->routeRequests.size(); ++i) {
            A(requestToOrder[routeSet[r]->routeRequests[i]], 0) = 1;
        }
        if (newCol == A)
            return true;
    }
    return false;
}







