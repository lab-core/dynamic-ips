//
// Created by Ella on 2021-11-10.
//

#include "ReducedProblem.h"

//---------------------------------------------------------------------------------------------
//  Reduced Problem class
//  Build and solve the Reduced problem of the ISUD
//---------------------------------------------------------------------------------------------

// Constructor and Destructor
ReducedProblem::ReducedProblem(PInstance &pInst) {
    RPModel_ = IloModel(env_);

    requestRHS_ = IloNumArray(env_);
    vehicleRHS_ = IloNumArray(env_);

    requestDuals_ = IloNumArray(env_);
    vehicleDuals_ = IloNumArray(env_);

    // defining variable
    zVar_ = IloNumVarArray(env_, 0.0, IloInfinity, ILOFLOAT);
    routeVar_ = IloNumVarArray(env_, 0.0, IloInfinity, ILOFLOAT);
}

ReducedProblem::~ReducedProblem() {
    env_.end();
}

// this function reset the model based the current set of routes and changed the set of constraints (size)
void ReducedProblem::updateRequestOrder(PInstance &pInst) {
    orderToRequest_.clear();
    requestToOrder_.clear();
    for (int i = 0; i < pInst->nbRequests_; ++i) {
        orderToRequest_.push_back(pInst->requests_[i]->getRequestId());
        requestToOrder_[pInst->requests_[i]->getRequestId()] = i;
    }
}

// this function clear all objects from the model at the start of each epoch
void ReducedProblem::clearModel(PInstance &pInst) {
    RPModel_.end();
}

// this function initialized the model and define empty set of constraints
void ReducedProblem::initializeModel(PInstance &pInst) {

    // update order of requests
    updateRequestOrder(pInst);

    // define and add objective
    reducedObj_ = IloMinimize(env_);
    RPModel_.add(reducedObj_);


    // defining constraints
    createIloNumArray (requestRHS_, pInst->nbRequests_, 1);
    createIloNumArray (vehicleRHS_, pInst->nbVehicles_, 1);

    requestConst_ = IloRangeArray(env_, requestRHS_, requestRHS_);
    vehicleConst_ = IloRangeArray(env_, vehicleRHS_, vehicleRHS_);

    RPModel_.add(requestConst_);
    RPModel_.add(vehicleConst_);

}

// this function adds routeVar to the model
void ReducedProblem::addRouteVar(PRoute &newRoute) {
    IloNumArray columnVar(env_, orderToRequest_.size());
    createPattern(columnVar, newRoute, requestToOrder_);
    IloNumVar numVar = IloNumVar(reducedObj_(newRoute->totalDelay_) + requestConst_(columnVar)
                                 + vehicleConst_[newRoute->vehicleID_](1));
    numVar.setName(newRoute->name_);
    routeVar_.add(numVar);
}

// this function adds zVar to the model used for the routes that served only one request
void ReducedProblem:: addZVar(PRequest &request) {
    IloNumVar numVar = IloNumVar(reducedObj_(request->penalty_) +
                                 requestConst_[requestToOrder_[request->getRequestId()]](1));
    numVar.setName(request->name_);
    zVar_.add(numVar);
}

// this function add one route at each iteration of the algorithm during one epoch
void ReducedProblem::updateModel(PInstance &pInst, std::vector<PRoute> &routeSolution) {
    if (routesToAdd_.size() == 0) {
        std::cout << "There is no route to be added" << std::endl;
        throw Tools::myException("The input route is empty, No new column is passed to be added", __LINE__);
    }
    std::cout << "number of route columns before update: " << routeVar_.getSize() << std::endl;
    std::cout << "number of z columns before update: " << zVar_.getSize() << std::endl;

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
    std::cout << "number of route columns after update: " << routeVar_.getSize() << std::endl;
    std::cout << "number of z columns after update: " << zVar_.getSize() << std::endl;
}

// this function build the model at the start of each epoch
void ReducedProblem::buildModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution) {

    // model initialization (defining empty set of constraints and adding objective)
    initializeModel(pInst);

        // adding request columns (z variables)
    for (int i = 0; i < zSolution.size(); ++i) {
        addZVar(zSolution[i]);
    }

    // adding solution columns
    for (int r = 0; r < routeSolution.size(); ++r) {
        addRouteVar(routeSolution[r]);

        // add related compatible z variables
        if (routeSolution[r]->routeRequests.size() == 1) {
            addZVar(*routeSolution[r]->routeNodes_[1]->related_Request_);
        }
    }

    //adding new route variables
    for (int r = 0; r < routesToAdd_.size(); ++r) {
        addRouteVar(routesToAdd_[r]);
    }
}

// this function solve the model and remove all columns except than the current base
void ReducedProblem::solveModel(PInstance &pInst, std::vector<PRequest> &zSolution,
                                std::vector<PRoute> &routeSolution, std::map<std::string , PRoute> &generatedRoutes) {
    try {
        RPCplex_ = IloCplex(RPModel_);
        RPCplex_.solve();

        // getting dual values
        requestDuals_.clear();
        vehicleDuals_.clear();

        // define dual container size
        requestDuals_ = IloNumArray(env_, pInst->nbRequests_);
        vehicleDuals_ = IloNumArray(env_, pInst->nbVehicles_);

        for (int r = 0; r < pInst->nbRequests_; ++r) {
            requestDuals_[r] = RPCplex_.getDual(requestConst_[r]);
            std::cout << "requestDuals[" << r <<"]: " << requestDuals_[r] << std::endl;
        }
        for (int v = 0; v < pInst->nbVehicles_; ++v) {
            vehicleDuals_[v] = RPCplex_.getDual(vehicleConst_[v]);
            std::cout << "vehicleDuals[" << v <<"]: " << vehicleDuals_[v] << std::endl;
        }

        // printing solution status
        std::cout << toString();

        // saving the result and remove out of base variables
        zSolution.clear();
        routeSolution.clear();

        IloNumArray zVal(env_);
        IloNumArray routeVal(env_);

        RPCplex_.getValues(zVal, zVar_);
        RPCplex_.getValues(routeVal, routeVar_);

        for (int r = routeVal.getSize()-1; r >= 0; --r) {
            if (routeVal[r] > 0.9) {
                routeSolution.push_back(generatedRoutes[routeVar_[r].getName()]);
            }
            else {
                routeVar_[r].end();
                routeVar_.remove(r,1);
            }
        }

        for (int i = zVal.getSize()-1; i >= 0; --i) {
            if (zVal[i] > 0.9)
                zSolution.push_back(pInst->nameToRequest_[zVar_[i].getName()]);
            else {
                zVar_[i].end();
                zVar_.remove(i,1);
            }
        }
        std::cout << "# from " << pInst->nbRequests_ << " request, " << pInst->nbRequests_ - zSolution.size()
        << " are selected to served." << std::endl;
    }
    catch (IloException& e) {
        std::cout << e << std::endl;
    }

}

//************************************************************************
// Display function
//************************************************************************
std::string ReducedProblem::toString() const {
    std::stringstream repStr;
    repStr << std::endl;
    repStr << "# REDUCED PROBLEM SOLVED: " << std::endl;
    repStr << "#" << std::endl;
    repStr << "# Solution status = " << RPCplex_.getStatus() << std::endl;
    repStr << "# Incumbent objective value = " << RPCplex_.getObjValue() << std::endl;

    return repStr.str();
}




