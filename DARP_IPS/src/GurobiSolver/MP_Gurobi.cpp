//
// Created by Elahe Amiri on 2025-06-25.
//

#include "MP_Gurobi.h"

#include <iostream>

// Constructor
MP_Gurobi::MP_Gurobi(std::string outputLog) : RP_Gurobi(outputLog) {
    compRoutes_.clear();
}

// Build the master problem model
void MP_Gurobi::buildModelMP(PInstance& pInst, std::vector<PRoute>& routeSolution, int nbVehicles) {
    try {
        // Model initialization (defining an empty set of constraints and adding the objective function)
        int rhs = 1;
        initializeModel(pInst, rhs, nbVehicles);

        // Use batch mode for efficiency when adding many variables
        beginBatchUpdate();

        // Adding request columns (z variables)
        for (auto& zSol : pInst->requests_) {
            if (zSol->committedPickTime_ == LARGE_CONSTANT) {
                addZVarFloat(zSol, POSITIVE);
                zVar_.push_back(getZVar().back());
            }
        }

        // Adding route solution columns
        for (auto& routeSol : routeSolution) {
            if (pInst->vehicles_[routeSol->vehicleID_]->vehicleIndex_ > -1) {
                addRouteVarFloat(routeSol, pInst);
            }
        }

        // Adding new route variables from routesToAdd_
        for (auto& routeObj : routesToAdd_) {
            addRouteVarFloat(routeObj, pInst);
        }

        // End batch update
        endBatchUpdate();

    } catch (GRBException& e) {
        std::cerr << "Error in buildModelMP: " << e.getMessage() << std::endl;
        throw;
    }
}

// Update model with new routes
void MP_Gurobi::updateModel(PInstance& pInst) {
    try {
        // Use batch mode for efficiency
        beginBatchUpdate();

        // Add the new compatible columns to the model
        for (auto routeObj : routesToAdd_) {
            addRouteVarFloat(routeObj, pInst);
        }

        endBatchUpdate();

    } catch (GRBException& e) {
        std::cerr << "Error in updateModel: " << e.getMessage() << std::endl;
        throw;
    }
}
