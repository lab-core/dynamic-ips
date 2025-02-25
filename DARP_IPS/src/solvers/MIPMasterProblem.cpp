//
// Created by Elahe Amiri on 2023-04-22.
//

#include "MIPMasterProblem.h"

MIPMasterProblem::MIPMasterProblem() {
// defining variable
    compRoutes_.clear();
}

void MIPMasterProblem::buildModelMP(PInstance &pInst, vector<PRoute> &routeSolution, int nbVehicles) {
    // model initialization (defining empty set of constraints and adding objective)
    int rhs = 1;
    initializeModel(pInst, rhs, nbVehicles);

    // adding request columns (z variables)
    for (auto & zSol : pInst->requests_) {
        addZVarFloat(zVar_, zSol, POSITIVE);
        if (pInst->parameters_->initialDual_ == AUX_box)
            addUVarFloat(uVar_, zSol);
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
    Model_.add(requestConst_);
    Model_.add(vehicleConst_);
    Model_.add(objFunction_);
}

void MIPMasterProblem::updateModel(PInstance &pInst) {
    // add the new compatible column to the model
    for (auto routeObj : routesToAdd_) {
        addRouteVarFloat(routeObj, pInst);
    }
}

