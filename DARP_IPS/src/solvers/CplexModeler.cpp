//
// Created by Ella on 2021-11-17.
//

#include "CplexModeler.h"

// Constructor and Destructor
CplexModeler::CplexModeler() {
    Model_ = IloModel(env_);
    objFunction_ = IloMinimize(env_);
//    Model_.add(objFunction_);

    requestDuals_ = IloNumArray(env_);
    vehicleDuals_ = IloNumArray(env_);

    requestRHS_ = IloNumArray(env_);
    vehicleRHS_ = IloNumArray(env_);
    solveTime_ = new myTools::Timer(); solveTime_->init();
}

CplexModeler::~CplexModeler() {
    delete solveTime_;
    env_.end();
}

// this function reset the model based the current set of routes and changed the set of constraints (size)
/*void CplexModeler::updateRequestOrder(PInstance &pInst) {
    nbTasks_.clear();
    requestToOrder_.clear();

    int orderCounter = 0;
    for (auto & requestObj : pInst->requests_){
        requestToOrder_[requestObj->getRequestId()] = orderCounter;
        nbTasks_.push_back(requestObj->getRequestId());
        orderCounter++;
    }
}*/

// this function clear all objects from the model at the start of each epoch
void CplexModeler::clearModel() {
    Model_.end();
    std::cout << "The Model is destroyed" << std::endl;
    Model_ = IloModel(env_);
    objFunction_ = IloMinimize(env_);
    Model_.add(objFunction_);
}

// Display function
std::string CplexModeler::toString() const {
    std::stringstream repStr;
    repStr << "#" << std::endl;
    repStr << "# Solution status = " << Cplex_.getStatus() << std::endl;
    repStr << "# Incumbent objective value = " << Cplex_.getObjValue() << std::endl;

    return repStr.str();
}

// function to create pattern from routes
void CplexModeler::createPattern(IloNumArray &pattern, PRoute &route, VarSign sign) {
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
void CplexModeler::addZVarInt(IloNumVarArray &zVar, PRequest &request, VarSign sign) {

    try {
        IloNumColumn numVar;
        if (sign == POSITIVE)
            numVar = objFunction_(request->penalty_) + requestConst_[request->taskIndex_](1);
        else
            numVar = objFunction_(-request->penalty_) + requestConst_[request->taskIndex_](-1);
        zVar.add(IloNumVar(numVar,0,IloInfinity,ILOINT));
        zVar[zVar.getSize()-1].setName(request->name_);
    }
    catch (IloException& e) {
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}

void CplexModeler::addZVarFloat(IloNumVarArray &zVar, PRequest &request, VarSign sign) {

    try {
        IloNumColumn numVar;
        if (sign == POSITIVE)
            numVar = objFunction_(request->penalty_) + requestConst_[request->taskIndex_](1);
        else
            numVar = objFunction_(-request->penalty_) + requestConst_[request->taskIndex_](-1);
        zVar.add(IloNumVar(numVar));
        zVar[zVar.getSize()-1].setName(request->name_);
    }
    catch (IloException& e) {
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}


// this function adds routeVar to the model
void CplexModeler::addRouteVarInt(IloNumVarArray &routeVar, PRoute &newRoute, VarSign sign, PInstance &pInst) {
    IloNumArray columnVar(env_, nbRequestTask_);
    createPattern(columnVar, newRoute, sign);
    IloNumColumn numVar;
    if (sign == POSITIVE) {
        numVar = objFunction_(newRoute->totalDelay_) + requestConst_(columnVar)
                 + vehicleConst_[pInst->vehicles_[newRoute->vehicleID_]->vehicleIndex_](1);
    }
    else {
        numVar = objFunction_(-newRoute->totalDelay_) + requestConst_(columnVar)
                 + vehicleConst_[pInst->vehicles_[newRoute->vehicleID_]->vehicleIndex_](-1);
    }
    routeVar.add(IloNumVar(numVar,0,IloInfinity,ILOINT));
 //   numVar.setName(newRoute->name_);
    routeVar[routeVar.getSize()-1].setName(newRoute->name_);
}

// this function adds routeVar to the model
void CplexModeler::addRouteVarFloat(IloNumVarArray &routeVar, PRoute &newRoute, VarSign sign, PInstance &pInst) {
    IloNumArray columnVar(env_, nbRequestTask_);
    createPattern(columnVar, newRoute, sign);
    IloNumColumn numVar;
    if (sign == POSITIVE) {
        numVar = objFunction_(newRoute->totalDelay_) + requestConst_(columnVar)
                 + vehicleConst_[pInst->vehicles_[newRoute->vehicleID_]->vehicleIndex_](1);
    }
    else {
        numVar = objFunction_(-newRoute->totalDelay_) + requestConst_(columnVar)
                 + vehicleConst_[pInst->vehicles_[newRoute->vehicleID_]->vehicleIndex_](-1);

    }
    routeVar.add(IloNumVar(numVar));
    routeVar[routeVar.getSize()-1].setName(newRoute->name_);
}

// this function initialized the model
void CplexModeler::initializeModel(PInstance &pInst, int rhs, int nbVehicles) {
// update order of requests
    nbRequestTask_ = pInst->nbTasks_;

    // define and add objective

    createIloNumArray (requestRHS_, nbRequestTask_, rhs);
    createIloNumArray (vehicleRHS_, nbVehicles, rhs);

    requestConst_ = IloRangeArray(env_, requestRHS_, requestRHS_);
    vehicleConst_ = IloRangeArray(env_, vehicleRHS_, vehicleRHS_);

}


