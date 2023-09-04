//
// Created by Elahe Amiri on 2023-04-22.
//

#include "MasterPro.h"

MasterPro::MasterPro() {
// defining variable
    zVar_ = IloNumVarArray(env_, 0.0, 0.0, IloInfinity,ILOFLOAT);
    routeVar_ = IloNumVarArray(env_, 0.0, 0.0, IloInfinity,ILOFLOAT);
    compRoutes_.clear();
}

void MasterPro::buildModelMP(PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution, int nbVehicles) {
    // model initialization (defining empty set of constraints and adding objective)
    int rhs = 1;
    MasterModeler::initializeModel(pInst, rhs, nbVehicles);
    zLb_.clear();
    rLb_.clear();

    // adding request columns (z variables)
    for (auto & zSol : pInst->requests_)
        MasterModeler::addZVarFloat(zVar_, zSol, POSITIVE);

    // adding route solution columns
    for (auto & routeSol : routeSolution){
        if (pInst->vehicles_[routeSol->vehicleID_]->vehicleIndex_ > -1) {
            MasterModeler::addRouteVarFloat(routeVar_, routeSol, POSITIVE, pInst);
            compRoutes_.push_back(routeSol);
            routeSol->mpAdded_ = true;
        }
    }

    //adding new route variables
    for (auto & routeObj : routesToAdd_) {
        MasterModeler::addRouteVarFloat(routeVar_, routeObj, POSITIVE, pInst);
        compRoutes_.push_back(routeObj);
        routeObj->mpAdded_ = true;
    }
    Model_.add(requestConst_);
    Model_.add(vehicleConst_);
    Model_.add(objFunction_);
}

void MasterPro::updateModel(PInstance &pInst) {
    if (routesToAdd_.empty()) {
        std::cout << "There is no route to be added" << std::endl;
//        throw myTools::myException("The input route is empty, No new column is passed to be added", __LINE__);
    }

    // add the new compatible column to the model
    for (auto routeObj : routesToAdd_) {
        MasterModeler::addRouteVarFloat(routeVar_, routeObj, POSITIVE, pInst);
        compRoutes_.push_back(routeObj);
        routeObj->mpAdded_ = true;
    }
}

