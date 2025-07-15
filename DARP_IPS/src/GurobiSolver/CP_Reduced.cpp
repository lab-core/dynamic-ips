//
// Created by Elahe Amiri on 2025-07-13.
//

#include "CP_Reduced.h"

void CP_Reduced::initializeCPModel(const PInstance &pInst, int nbVehicles, int nbPenalties) {
    try {
        int rhs = 0;
        nbRequestTask_ = pInst->nbTasks_ - nbPenalties;

        // Initialize RHS arrays
        requestRHS_.resize(nbRequestTask_, rhs);

        // Reserve space for constraints
        requestConstr_.reserve(nbRequestTask_);

        // Clear existing constraints (in case of reinitialization)
        requestConstr_.clear();

        // Add request constraints (= rhs)
        for (int i = 0; i < nbRequestTask_; ++i) {
            GRBLinExpr expr = 0;
            requestConstr_.push_back(model_->addConstr(expr == rhs));
        }

        // Set basic parameters
        model_->set(GRB_IntParam_Threads, pInst->parameters_->nbThreads_);


        // Add normalization constraint (sum = 1)
        GRBLinExpr normalExpr = 0;
        normalConst_ = model_->addConstr(normalExpr == 1.0, "normalConst");

        IncRoute_.clear();
        // Update model after adding normalization constraint
        model_->update();
    }
    catch (GRBException& e) {
        std::cerr << "Error in initializeCPModel: " << e.getMessage() << std::endl;
        throw;
    }
}

void CP_Reduced::buildModel(const PInstance &pInst) {
}
