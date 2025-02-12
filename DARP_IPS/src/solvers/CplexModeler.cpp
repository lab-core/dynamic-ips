//
// Created by Ella on 2021-11-17.
//

#include "CplexModeler.h"

// Constructor and Destructor
CplexModeler::CplexModeler() {
    Model_ = IloModel(env_);
    Cplex_ = IloCplex(Model_);
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
    int signMultiplier = (sign == POSITIVE) ? 1 : -1;
    for (auto & requestObj : route->routeRequests_) {
        pattern[requestObj->taskIndex_] = signMultiplier;
    }
}

// this function adds zVar to the model
void CplexModeler::addZVarInt(IloNumVarArray &zVar, PRequest &request, VarSign sign) {

    try {
        int signMultiplier = (sign == POSITIVE) ? 1 : -1;
        IloNumColumn numVar = objFunction_(signMultiplier * request->penalty_) +
                              requestConst_[request->taskIndex_](signMultiplier);

        IloNumVar newVar(numVar, 0, IloInfinity, ILOINT);
        newVar.setName(request->name_);
        zVar.add(newVar);
    }
    catch (const IloException& e) {
        std::cerr << "Error in CplexModeler::addZVarInt at line " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}

void CplexModeler::addZVarFloat(IloNumVarArray &zVar, PRequest &request, VarSign sign) {

    try {
        int signMultiplier = (sign == POSITIVE) ? 1 : -1;
        IloNumColumn numVar = objFunction_(signMultiplier * request->penalty_) +
                              requestConst_[request->taskIndex_](signMultiplier);
        IloNumVar newVar(numVar, 0, IloInfinity, ILOFLOAT);
        newVar.setName(request->name_);
        zVar.add(newVar);
    }
    catch (const IloException& e) {
        std::cerr << "Error in CplexModeler::addZVarFloat at line " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}


// this function adds routeVar to the model
void CplexModeler::addRouteVarInt(IloNumVarArray &routeVar, PRoute &newRoute, VarSign sign, PInstance &pInst) {
    try {
        IloNumArray columnVar(env_, nbRequestTask_);
        createPattern(columnVar, newRoute, sign);
        int signMultiplier = (sign == POSITIVE) ? 1 : -1;
        IloNumColumn numVar = objFunction_(signMultiplier * newRoute->totalDelay_) + requestConst_(columnVar)
                     + vehicleConst_[pInst->vehicles_[newRoute->vehicleID_]->vehicleIndex_](signMultiplier);

        routeVar.add(IloNumVar(numVar,0,IloInfinity,ILOINT));
        routeVar[routeVar.getSize()-1].setName(newRoute->name_);
    }
    catch (const IloException& e) {
        std::cerr << "Error in CplexModeler::addRouteVarInt at line " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}

// this function adds routeVar to the model
void CplexModeler::addRouteVarFloat(IloNumVarArray &routeVar, PRoute &newRoute, VarSign sign, PInstance &pInst) {
    try {
        IloNumArray columnVar(env_, nbRequestTask_);
        createPattern(columnVar, newRoute, sign);
        int signMultiplier = (sign == POSITIVE) ? 1 : -1;
        IloNumColumn numVar = objFunction_(signMultiplier * newRoute->totalDelay_) + requestConst_(columnVar)
                     + vehicleConst_[pInst->vehicles_[newRoute->vehicleID_]->vehicleIndex_](signMultiplier);

        routeVar.add(IloNumVar(numVar, 0,IloInfinity,ILOFLOAT));
        routeVar[routeVar.getSize()-1].setName(newRoute->name_);
    }
    catch (const IloException& e) {
        std::cerr << "Error in CplexModeler::addRouteVarFloat at line " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}

void CplexModeler::setParameters(PInstance &pInst, float availableTime) {
    Cplex_.setParam(IloCplex::Param::Threads, pInst->parameters_->nbThreads_);
    Cplex_.setParam(IloCplex::Param::Preprocessing::Presolve, 0);
    Cplex_.setParam(IloCplex::Param::RootAlgorithm, 2);
    if (pInst->parameters_->MIPGap_ > 0.0001)
        Cplex_.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, pInst->parameters_->MIPGap_);
    Cplex_.setParam(IloCplex::Param::TimeLimit, availableTime);

    /*Cplex_.setParam(IloCplex::Param::MIP::Strategy::NodeSelect, 1);  // Best-bound search
    Cplex_.setParam(IloCplex::Param::MIP::Cuts::Cliques, 0);         // Aggressive cliques
    Cplex_.setParam(IloCplex::Param::MIP::Cuts::Covers, 0);          // Aggressive covers
    Cplex_.setParam(IloCplex::Param::MIP::Cuts::FlowCovers, 0);      // Aggressive flow covers
    Cplex_.setParam(IloCplex::Param::MIP::Cuts::Gomory, 2);
    Cplex_.setParam(IloCplex::Param::MIP::Cuts::ZeroHalfCut, 2);*/
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


