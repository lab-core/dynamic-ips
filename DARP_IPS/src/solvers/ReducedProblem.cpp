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
    zVar_ = IloNumVarArray(env_, 0.0, 0.0, IloInfinity,ILOINT);
    routeVar_ = IloNumVarArray(env_, 0.0, 0.0, IloInfinity,ILOINT);
    compRoutes_.clear();
}


// this function initialized the model and define empty set of constraints
void ReducedProblem::ResetRPModel() {

    try {
        /*int modelExist = 0;
        for (int r = (int)routeVar_.getSize()-1; r >= 0; --r) {
            routeVar_[r].end();
            routeVar_.remove(r,1);
            modelExist = 1;
        }
        for (int i = (int)zVar_.getSize()-1; i >= 0; --i) {
            zVar_[i].end();
            zVar_.remove(i,1);
            modelExist = 1;
        }
        if (modelExist == 1) {
            Model_.remove(requestConst_);
            Model_.remove(vehicleConst_);
        }*/

        bool isModelExist = false;
        if (routeVar_.getSize() > 0)
            isModelExist = true;
        if (isModelExist){
            routeVar_.endElements();
            routeVar_.clear();
            compRoutes_.clear();
            zVar_.endElements();
            zVar_.clear();
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
    MasterModeler::addRouteVarInt(routeVar_, newRoute, POSITIVE);
    compRoutes_.push_back(newRoute);
}

void ReducedProblem::addRouteVarPartial(PRoute &newRoute, PInstance &pInst) {
    MasterModeler::addRouteVarIntPartial(routeVar_, newRoute, POSITIVE, pInst);
    compRoutes_.push_back(newRoute);
}

void ReducedProblem::addRouteVars(std::vector<PRoute> &newRoutes) {
    MasterModeler::addRouteVars(routeVar_, newRoutes, POSITIVE);
    for (auto & newRoute : newRoutes)
        compRoutes_.push_back(newRoute);
}

// this function adds zVar to the model used for the routes that served only one request
void ReducedProblem:: addZVars(std::vector<PRequest> &requests) {
    MasterModeler::addZVars(zVar_, requests, POSITIVE);
}

void ReducedProblem:: addZVar(PRequest &request) {
    MasterModeler::addZVarInt(zVar_, request, POSITIVE);
}

// this function add one route at each iteration of the algorithm during one epoch
void ReducedProblem::updateModel(PInstance &pInst, std::vector<PRoute> &routeSolution) {
    if (routesToAdd_.empty()) {
        std::cout << "There is no route to be added" << std::endl;
        throw myTools::myException("The input route is empty, No new column is passed to be added", __LINE__);
    }

    // add the new compatible column to the model
    for (auto & routeObj : routesToAdd_) {
        addRouteVar(routeObj);
    }

    // add compatible z variables
    // just z variables related to requests that are served in routes with one request are compatible
    /*for (int r = 0; r < routeSolution.size(); ++r) {
        if (routeSolution[r]->routeRequests_.size() == 1) {
            addZVarInt(routeSolution[r]->routeNodes_[1]->related_Request_);
        }
    }*/
}


// this function build the model at the start of each epoch
void ReducedProblem::buildModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution) {

    // model initialization (defining empty set of constraints and adding objective)
//    ResetRPModel();
    int rhs = 1;
    MasterModeler::initializeModel(pInst, rhs);

    // adding request columns (z variables)
//    addZVars(zSolution);
    for (auto & zSol : zSolution)
        addZVar(zSol);

    // adding route solution columns
 //   addRouteVars(routeSolution);
    for (auto & routeSol : routeSolution){
        addRouteVar(routeSol);
    }



    //adding new route variables
    for (auto & routeObj : routesToAdd_) {
        addRouteVar(routeObj);
    }
//    env_.out() << Model_;
}
void ReducedProblem::buildModelPartial(PInstance &pInst, std::vector<PRequest> &zSolution,
                                       std::vector<PRoute> &routeSolution, int nbVehicles) {

    // model initialization (defining empty set of constraints and adding objective)
//    ResetRPModel();
    int rhs = 1;
    MasterModeler::initializeModelPartial(pInst, rhs, nbVehicles);

    // adding request columns (z variables)
//    addZVars(zSolution);
    for (auto & zSol : zSolution)
        addZVar(zSol);

    // adding route solution columns
    //   addRouteVars(routeSolution);
    for (auto & routeSol : routeSolution){
        if (pInst->vehicles_[routeSol->vehicleID_]->vehicleIndex_ > -1)
            addRouteVarPartial(routeSol, pInst);
    }



    //adding new route variables
    for (auto & routeObj : routesToAdd_) {
        if (pInst->vehicles_[routeObj->vehicleID_]->vehicleIndex_ > -1)
            addRouteVarPartial(routeObj, pInst);
    }
//    env_.out() << Model_;
}
// this function solve the model and remove all columns except than the current base
void ReducedProblem::solveModel(PInstance &pInst, std::vector<PRequest> &zSolution,
                                std::vector<PRoute> &routeSolution, std::map<std::string ,PRoute> &generatedRoutes) {
    try {

        Cplex_ = IloCplex(Model_);
        Cplex_.setParam(IloCplex::Param::Threads, pInst->parameters_->nbThreads_);
        Cplex_.setOut(env_.getNullStream());
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
//                int rowIndex = requestToOrder_[requestObj->getRequestId()];
//                int rowIndex = pInst->requestToOrder_[requestObj->getRequestId()];
                int rowIndex = requestObj->taskIndex_;
                requestDuals_[rowIndex] = Cplex_.getDual(requestConst_[rowIndex]);
                requestObj->dual_ = requestDuals_[rowIndex];
                requestObj->CPDual_ = requestDuals_[rowIndex];
                std::cout << "requestDuals[" << requestObj->getRequestId() << "]: " << requestObj->dual_
                          << std::endl;
            }
        }

        std::cout << "VEHICLE DUALS:" << std::endl;
        for (auto &vehicleObj: pInst->vehicles_) {
            vehicleDuals_[vehicleObj->vehicleID_] = Cplex_.getDual(vehicleConst_[vehicleObj->vehicleID_]);
            vehicleObj->dual_ = vehicleDuals_[vehicleObj->vehicleID_];
            vehicleObj->CPDual_ = vehicleDuals_[vehicleObj->vehicleID_];
            std::cout << "vehicleDuals[" << vehicleObj->vehicleID_ << "]: " << vehicleObj->dual_ << std::endl;
        }

        // printing solution status
  //      std::cout << toString();

        // saving the result and remove out of base variables
        zSolution.clear();
        routeSolution.clear();

        IloNumArray zVal(env_);
        IloNumArray routeVal(env_);


        Cplex_.getValues(zVal, zVar_);
        Cplex_.getValues(routeVal, routeVar_);
//        env_.out() << routeVal << std::endl;
//        env_.out() << zVal << std::endl;

        for (int r = (int)routeVal.getSize()-1; r >= 0; --r) {
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
        for (int i = (int)zVal.getSize()-1; i >= 0; --i) {
            if (zVal[i] > 0.9) {
//                std::cout << zVar_[i].getName() << std::endl;
                zSolution.push_back(pInst->nameToRequest_[zVar_[i].getName()]);
            }
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
        Cplex_.clearModel();
    }
    catch (IloException& e) {
        std::cout << e << std::endl;
    }

}


// function to check whether two routes are column disjoint or not
bool ReducedProblem::isColumnDisjoint(vector<PRoute> &routeSet, PRoute &newRoute, std::map<unsigned int, int> &requestToOrder){
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero((signed) requestToOrder.size(), (signed) routeSet.size()+1);
    vector<PRoute> selectedRoutes = routeSet;
    selectedRoutes.push_back(newRoute);
    for (int r = 0; r < selectedRoutes.size(); ++r) {
        for (int i = 0; i < selectedRoutes[r]->routeRequests_.size(); ++i)
            A(selectedRoutes[r]->routeRequests_[i]->taskIndex_, r) = 1;
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
                                    std::map<unsigned int, int> &requestToOrder) {
    Eigen::MatrixXd newCol = Eigen::MatrixXd::Zero((signed) requestToOrder.size(), 1);

    for (auto & requestObj : newRoute->routeRequests_)
        newCol(requestObj->taskIndex_, 0) = 1;

    for (auto & routeObj : routeSet) {
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero((signed) requestToOrder.size(), 1);
        for (auto & requestObj : routeObj->routeRequests_) {
            A(requestObj->taskIndex_, 0) = 1;
        }
        if (newCol == A)
            return true;
    }
    return false;
}

void ReducedProblem::restartRP() {
    zVar_ = IloNumVarArray(env_, 0.0, 0.0, IloInfinity,ILOINT);
    routeVar_ = IloNumVarArray(env_, 0.0, 0.0, IloInfinity,ILOINT);
}







