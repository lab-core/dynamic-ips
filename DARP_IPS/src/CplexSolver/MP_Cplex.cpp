//
// Created by Elahe Amiri on 2023-04-22.
//

#include "MP_Cplex.h"

MP_Cplex::MP_Cplex() {
// defining variable
    compRoutes_.clear();
}

void MP_Cplex::buildModelMP(PInstance &pInst, vector<PRoute> &routeSolution, int nbVehicles) {
    // model initialization (defining an empty set of constraints and adding the objective function)
    int rhs = 1;
    initializeModel(pInst, rhs, nbVehicles);

    // adding request columns (z variables)
    for (auto & zSol : pInst->requests_) {
        if (zSol->committedPickTime_ == LARGE_CONSTANT)
            addZVarFloat(zVar_, zSol, POSITIVE);
    }

    // adding route solution columns
    for (auto & routeSol : routeSolution){
        if (pInst->vehicles_[routeSol->vehicleID_]->vehicleIndex_ > -1) {
            addRouteVarFloat(routeSol, pInst);
        }
    }

    //adding new route variables
    for (auto & routeObj : routesToAdd_) {
        addRouteVarFloat(routeObj, pInst);
    }
}

void MP_Cplex::updateModel(PInstance &pInst) {
    // add the new compatible column to the model
    for (auto routeObj : routesToAdd_) {
        addRouteVarFloat(routeObj, pInst);
    }
}

