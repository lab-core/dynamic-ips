//
// Created by Elahe Amiri on 2025-06-25.
//

#include "MP_Gurobi.h"

#include <iostream>

// Constructor
MP_Gurobi::MP_Gurobi(std::string outputLog) : RP_Gurobi(outputLog) {
    compRoutes_.clear();
    routesToAdd_.clear();
}

// Build the master problem model
void MP_Gurobi::buildModelMP(PInstance& pInst, std::vector<PRoute>& routeSolution, int nbVehicles) {
    try {
        // Model initialization (defining an empty set of constraints and adding the objective function)
        int rhs = 1;
        initializeModel(pInst, rhs, nbVehicles);

        // Use batch mode for efficiency when adding many variables

        // Adding request columns (z variables)
        for (auto& zSol : pInst->requests_) {
            if (zSol->committedPickTime_ == LARGE_CONSTANT) {
                addZVarFloat(zSol, POSITIVE);
            }
        }

        // Adding route solution columns
        for (auto& routeSol : routeSolution) {
            if (pInst->vehicles_[routeSol->vehicleID_]->vehicleIndex_ > -1) {
                addRouteVarFloat_RP(routeSol, pInst);
            }
        }

        // Adding new route variables from routesToAdd_
        for (auto& routeObj : routesToAdd_) {
            addRouteVarFloat_RP(routeObj, pInst);
        }

        // End batch update
        model_->update();

    } catch (GRBException& e) {
        std::cerr << "Error in buildModelMP: " << e.getMessage() << std::endl;
        throw;
    }
}

// Update model with new routes
void MP_Gurobi::updateModel_columns(PInstance& pInst) {
    try {
        // Use batch mode for efficiency
        // Add the new compatible columns to the model
        for (auto routeObj : routesToAdd_) {
            addRouteVarFloat_RP(routeObj, pInst);
        }

        model_->update();

    } catch (GRBException& e) {
        std::cerr << "Error in updateModel: " << e.getMessage() << std::endl;
        throw;
    }
}


// Batch update model with new (continuous) routes
void MP_Gurobi::updateModel(PInstance& pInst) {
    try {
        if (routesToAdd_.empty()) return;

        const size_t numRoutes = routesToAdd_.size();

        // Bounds / objective / types / columns for bulk add
        std::vector<double> lb(numRoutes, 0.0);
        std::vector<double> ub(numRoutes, GRB_INFINITY);
        std::vector<double> obj(numRoutes);
        std::vector<char>   vtype(numRoutes, GRB_CONTINUOUS);
        std::vector<GRBColumn> columns(numRoutes);

        // Avoid reallocations
        compRoutes_.reserve(compRoutes_.size() + numRoutes);
        routeVar_.reserve(routeVar_.size() + numRoutes);

        size_t k = 0;
        for (const auto& routeObj : routesToAdd_) {
            // Fill arrays with range-based loop
            obj[k]     = routeObj->objCoef_;
            columns[k] = createColumn(routeObj, POSITIVE, pInst);
            compRoutes_.emplace_back(routeObj);
            routeObj->mpAdded_ = true;
            ++k;
        }

        // Bulk-add the variables
        std::unique_ptr<GRBVar[]> newVars(model_->addVars(
            lb.data(),
            ub.data(),
            obj.data(),
            vtype.data(),
            nullptr,          // no names
            columns.data(),
            static_cast<int>(numRoutes)
        ));

        // Store created vars
        routeVar_.insert(routeVar_.end(), newVars.get(), newVars.get() + numRoutes);

        // Single update call
        model_->update();

    } catch (const GRBException& e) {
        std::cerr << "Error in updateModel_batch: " << e.getMessage() << std::endl;
        throw;
    } catch (const std::exception& e) {
        std::cerr << "Error in updateModel_batch: " << e.what() << std::endl;
        throw;
    }
}

