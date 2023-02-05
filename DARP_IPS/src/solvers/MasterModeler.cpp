//
// Created by Ella on 2021-11-17.
//

#include "MasterModeler.h"

// Constructor and Destructor
MasterModeler::MasterModeler() {
    Model_ = IloModel(env_);
    objFunction_ = IloMinimize(env_);
//    Model_.add(objFunction_);

    requestDuals_ = IloNumArray(env_);
    vehicleDuals_ = IloNumArray(env_);

    requestRHS_ = IloNumArray(env_);
    vehicleRHS_ = IloNumArray(env_);
}

MasterModeler::~MasterModeler() {
    env_.end();
}

// this function reset the model based the current set of routes and changed the set of constraints (size)
/*void MasterModeler::updateRequestOrder(PInstance &pInst) {
    orderToRequest_.clear();
    requestToOrder_.clear();

    int orderCounter = 0;
    for (auto & requestObj : pInst->requests_){
        requestToOrder_[requestObj->getRequestId()] = orderCounter;
        orderToRequest_.push_back(requestObj->getRequestId());
        orderCounter++;
    }
}*/

// this function clear all objects from the model at the start of each epoch
void MasterModeler::clearModel() {
    Model_.end();
    std::cout << "The Model is destroyed" << std::endl;
    Model_ = IloModel(env_);
    objFunction_ = IloMinimize(env_);
    Model_.add(objFunction_);
}

// Display function
std::string MasterModeler::toString() const {
    std::stringstream repStr;
    repStr << "#" << std::endl;
    repStr << "# Solution status = " << Cplex_.getStatus() << std::endl;
    repStr << "# Incumbent objective value = " << Cplex_.getObjValue() << std::endl;

    return repStr.str();
}

// function to create pattern from routes
void MasterModeler::createPattern(IloNumArray &pattern, PRoute &route, VarSign sign) {
    if (sign == POSITIVE) {
        for (auto & requestObj : route->routeRequests_) {
            pattern[requestObj->taskIndex_] = 1;
        }
    }
    else if (sign == NEGATIVE) {
        for (auto & requestObj : route->routeRequests_) {
            pattern[requestObj->taskIndex_] = -1;
        }
    }
}

// this function adds zVar to the model
void MasterModeler::addZVar(IloNumVarArray &zVar, PRequest &request, VarSign sign) {

    try {
        IloNumVar numVar;
        if (sign == POSITIVE)
            numVar = IloNumVar(objFunction_(request->penalty_) +
                               requestConst_[request->taskIndex_](1));
        else
            numVar = IloNumVar(objFunction_(-request->penalty_) +
                               requestConst_[request->taskIndex_](-1));
        numVar.setName(request->name_);
        zVar.add(numVar);
    }
    catch (IloException& e) {
        std::cout << e << std::endl;
    }
}

void MasterModeler::addZVars(IloNumVarArray &zVar, std::vector<PRequest> &requests, VarSign sign) {
    try {
        for (auto & request : requests) {
            IloNumVar numVar;
            if (sign == POSITIVE)
                numVar = IloNumVar(objFunction_(request->penalty_) +
                                   requestConst_[request->taskIndex_](1));
            else
                numVar = IloNumVar(objFunction_(-request->penalty_) +
                                   requestConst_[request->taskIndex_](-1));
            numVar.setName(request->name_);
            zVar.add(numVar);
        }
    }
    catch (IloException& e) {
        std::cout << e << std::endl;
    }
}

// this function adds routeVar to the model
void MasterModeler::addRouteVar(IloNumVarArray &routeVar, PRoute &newRoute, VarSign sign) {
    IloNumArray columnVar(env_, (signed) orderToRequest_.size());
    createPattern(columnVar, newRoute, sign);
//    IloNumVar numVar;
    if (sign == POSITIVE)
        routeVar.add(IloNumVar(objFunction_(newRoute->totalDelay_) + requestConst_(columnVar)
                                 + vehicleConst_[newRoute->vehicleID_](1)));
    else
        routeVar.add(IloNumVar(objFunction_(-newRoute->totalDelay_) + requestConst_(columnVar)
                           + vehicleConst_[newRoute->vehicleID_](-1)));
 //   numVar.setName(newRoute->name_);
 //   routeVar.add(numVar);
}

// this function initialized the model
void MasterModeler::initializeModel(PInstance &pInst, int rhs) {
// update order of requests
//    updateRequestOrder(pInst);
    orderToRequest_ = pInst->orderToRequest_;

    // define and add objective

    createIloNumArray (requestRHS_, orderToRequest_.size(), rhs);
    createIloNumArray (vehicleRHS_, pInst->nbVehicles_, rhs);

    requestConst_ = IloRangeArray(env_, requestRHS_, requestRHS_);
    vehicleConst_ = IloRangeArray(env_, vehicleRHS_, vehicleRHS_);

//    Model_.add(requestConst_);
//    Model_.add(vehicleConst_);
}

void MasterModeler::addRouteVars(IloNumVarArray &routeVar, vector<PRoute> &newRoutes, VarSign sign) {
//    IloNumColumnArray cols(env_, newRoutes.size());
    for (int r = 0; r < newRoutes.size(); r++){
        IloNumArray columnVar(env_, (signed) orderToRequest_.size());
        createPattern(columnVar, newRoutes[r], sign);
        if (sign == POSITIVE)
            routeVar.add(IloNumVar(objFunction_(newRoutes[r]->totalDelay_) + requestConst_(columnVar)
                                   + vehicleConst_[newRoutes[r]->vehicleID_](1)));
            /*cols[r] = objFunction_(newRoutes[r]->totalDelay_) + requestConst_(columnVar)
                     + vehicleConst_[newRoutes[r]->vehicleID_](1);*/
        else
            routeVar.add(IloNumVar(objFunction_(newRoutes[r]->totalDelay_) + requestConst_(columnVar)
                                   + vehicleConst_[newRoutes[r]->vehicleID_](-1)));
            /*cols[r] = objFunction_(newRoutes[r]->totalDelay_) + requestConst_(columnVar)
                      + vehicleConst_[newRoutes[r]->vehicleID_](-1);*/
        columnVar.end();
    }
    /*try {
        routeVar.add(IloNumVarArray(env_, cols));
    }
    catch (IloException& e) {
        std::cout << e << std::endl;
    }*/
//    cols.end();
}

