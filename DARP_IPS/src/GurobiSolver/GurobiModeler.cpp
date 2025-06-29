//
// Created by Elahe Amiri on 2025-06-25.
//

#include "GurobiModeler.h"
#include <iostream>
#include <iomanip>

// Constructor
GurobiModeler::GurobiModeler(std::string outputLog) : env_(true), outputLog_(outputLog) {
    try {
        // Set environment parameters before creating model
        env_.start();

        model_ = new GRBModel(env_);
        nbRequestTask_ = 0;

        solveTime_ = new myTools::Timer(); solveTime_->init();

        // Set model to minimize
        model_->set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

        // Apply tuned parameters
        model_->set(GRB_IntParam_Method, GRB_METHOD_DUAL);      // Dual simplex is typically best for CG
        model_->set(GRB_DoubleParam_Heuristics, 0.001);        // Minimal heuristics
        model_->set(GRB_IntParam_Presolve, 0);                  // Disable presolve
        model_->set(GRB_IntParam_UpdateMode, 0);                // Immediate updates
        model_->set(GRB_IntParam_LogToConsole, 0);
        model_->set(GRB_StringParam_LogFile, outputLog);
        //       model_->set(GRB_IntParam_OutputFlag, 1);

    } catch (GRBException& e) {
        std::cerr << "Error in GurobiModeler constructor: " << e.getMessage() << std::endl;
        throw;
    }
}

// Destructor
GurobiModeler::~GurobiModeler() {
    delete solveTime_;
    if (model_) {
        delete model_;
    }
}

// Display function
std::string GurobiModeler::toString() const {
    std::stringstream repStr;
    repStr << "#" << std::endl;
    repStr << "# Solution status = " << getStatus() << std::endl;
    try {
        repStr << "# Incumbent objective value = " << std::fixed << std::setprecision(4)
               << getObjValue() << std::endl;
    } catch (...) {
        repStr << "# No solution available" << std::endl;
    }
    return repStr.str();
}

// Initialize the model
void GurobiModeler::initializeModel(const PInstance& pInst, int rhs, int nbVehicles) {
    try {
        nbRequestTask_ = pInst->nbTasks_;

        // Initialize RHS arrays
        requestRHS_.resize(nbRequestTask_, rhs);
        vehicleRHS_.resize(nbVehicles, rhs);

        // Initialize dual arrays
        requestDuals_.resize(nbRequestTask_, 0.0);
        vehicleDuals_.resize(nbVehicles, 0.0);

        // Reserve space for constraints
        requestConstr_.reserve(nbRequestTask_);
        vehicleConstr_.reserve(nbVehicles);

        // Clear existing constraints (in case of reinitialization)
        requestConstr_.clear();
        vehicleConstr_.clear();

        // Add request constraints (= rhs)
        for (int i = 0; i < nbRequestTask_; ++i) {
            std::string constrName = "request_" + std::to_string(i);
            GRBLinExpr expr = 0;
            requestConstr_.push_back(model_->addConstr(expr == rhs, constrName));
        }

        // Add vehicle constraints (= rhs)
        for (int i = 0; i < nbVehicles; ++i) {
            std::string constrName = "vehicle_" + std::to_string(i);
            GRBLinExpr expr = 0;
            vehicleConstr_.push_back(model_->addConstr(expr == rhs, constrName));
        }

        // Set basic parameters
        model_->set(GRB_IntParam_Threads, pInst->parameters_->nbThreads_);
        model_->set(GRB_DoubleParam_MIPGap, pInst->parameters_->MIPGap_);

        // Update model
        model_->update();

    } catch (GRBException& e) {
        std::cerr << "Error in initializeModel: " << e.getMessage() << std::endl;
        throw;
    }
}

// Helper function to create column
GRBColumn GurobiModeler::createColumn(const PRoute& route, VarSign sign, const PInstance& pInst) {
    GRBColumn col;
    int signMultiplier = (sign == POSITIVE) ? 1 : -1;

    // Add coefficients for request constraints
    for (auto& requestObj : route->routeRequests_) {
        col.addTerm(signMultiplier, requestConstr_[requestObj->taskIndex_]);
    }

    // Add coefficient for vehicle constraint
    int vehIndex = pInst->vehicles_[route->vehicleID_]->vehicleIndex_;
    col.addTerm(signMultiplier, vehicleConstr_[vehIndex]);

    return col;
}

// Add integer z variable
void GurobiModeler::addZVarInt(const PRequest& request, VarSign sign) {
    try {
        int signMultiplier = (sign == POSITIVE) ? 1 : -1;

        GRBColumn col;
        col.addTerm(signMultiplier, requestConstr_[request->taskIndex_]);

        double objCoeff = signMultiplier * request->penalty_;
        GRBVar var = model_->addVar(0.0, GRB_INFINITY, objCoeff, GRB_INTEGER, col, request->name_);
        zVar_.push_back(var);

    } catch (GRBException& e) {
        std::cerr << "Error in addZVarInt: " << e.getMessage() << std::endl;
        throw;
    }
}

// Add continuous z variable
void GurobiModeler::addZVarFloat(const PRequest& request, VarSign sign) {
    try {
        int signMultiplier = (sign == POSITIVE) ? 1 : -1;

        GRBColumn col;
        col.addTerm(signMultiplier, requestConstr_[request->taskIndex_]);

        double objCoeff = signMultiplier * request->penalty_;
        GRBVar var = model_->addVar(0.0, GRB_INFINITY, objCoeff, GRB_CONTINUOUS, col, request->name_);
        zVar_.push_back(var);

    } catch (GRBException& e) {
        std::cerr << "Error in addZVarFloat: " << e.getMessage() << std::endl;
        throw;
    }
}


// Add integer route variable
void GurobiModeler::addRouteVarInt(const PRoute& newRoute, VarSign sign, const PInstance& pInst) {
    try {
        int signMultiplier = (sign == POSITIVE) ? 1 : -1;
        GRBColumn col = createColumn(newRoute, sign, pInst);

        double objCoeff = signMultiplier * newRoute->totalDelay_;
        GRBVar var = model_->addVar(0.0, GRB_INFINITY, objCoeff, GRB_INTEGER, col, newRoute->name_);
        routeVar_.push_back(var);

    } catch (GRBException& e) {
        std::cerr << "Error in addRouteVarInt: " << e.getMessage() << std::endl;
        throw;
    }
}

// Add continuous route variable
void GurobiModeler::addRouteVarFloat(const PRoute& newRoute, VarSign sign, const PInstance& pInst) {
    try {
        int signMultiplier = (sign == POSITIVE) ? 1 : -1;
        GRBColumn col = createColumn(newRoute, sign, pInst);

        double objCoeff = signMultiplier * newRoute->totalDelay_;
        GRBVar var = model_->addVar(0.0, GRB_INFINITY, objCoeff, GRB_CONTINUOUS, col, newRoute->name_);
        routeVar_.push_back(var);

    } catch (GRBException& e) {
        std::cerr << "Error in addRouteVarFloat: " << e.getMessage() << std::endl;
        throw;
    }
}

// Begin batch update
void GurobiModeler::beginBatchUpdate() {
    try {
        model_->set(GRB_IntParam_UpdateMode, 1); // Enable batch mode
    } catch (GRBException& e) {
        std::cerr << "Error in beginBatchUpdate: " << e.getMessage() << std::endl;
        throw;
    }
}

// End batch update
void GurobiModeler::endBatchUpdate() {
    try {
        model_->update();
        model_->set(GRB_IntParam_UpdateMode, 0); // Disable batch mode
    } catch (GRBException& e) {
        std::cerr << "Error in endBatchUpdate: " << e.getMessage() << std::endl;
        throw;
    }
}

// Set parameters
void GurobiModeler::setParameters(const PInstance& pInst, float availableTime) {
    try {
        // Thread control
        model_->set(GRB_IntParam_Threads, pInst->parameters_->nbThreads_);
        // MIP gap
        model_->set(GRB_IntParam_Method, GRB_METHOD_DUAL);      // Dual simplex is typically best for CG
        model_->set(GRB_IntParam_ScaleFlag, 1);                 // Enable scaling
        model_->set(GRB_IntParam_NormAdjust, 2);                // Aggressive normalization
        model_->set(GRB_IntParam_DegenMoves, 0);                // Disable degeneracy moves
        model_->set(GRB_DoubleParam_Heuristics, 0.001);        // Minimal heuristics
        model_->set(GRB_IntParam_Presolve, 0);                  // Disable presolve
        model_->set(GRB_IntParam_UpdateMode, 0);                // Immediate updates

        model_->set(GRB_DoubleParam_TimeLimit, availableTime);      // time limit

        // Output control
//        model_->set(GRB_IntParam_OutputFlag, 0); // Silent mode


        // For MIP solving (if needed)
        /*model_->set(GRB_IntParam_PrePasses, 0);
        model_->set(GRB_IntParam_Aggregate, 0);
        model_->set(GRB_IntParam_ScaleFlag, 1);                   // Enable scaling
        model_->set(GRB_IntParam_NormAdjust, 2);                // Aggressive normalization
        model_->set(GRB_IntParam_DegenMoves, 0);                // Disable degeneracy moves
        model_->set(GRB_IntParam_NodeMethod, 1); // Dual simplex for nodes
        model_->set(GRB_IntParam_Cuts, 0); // Disable all cuts
        model_->set(GRB_IntParam_MIPFocus, 1); // Focus on finding feasible solutions

        model_->set(GRB_IntParam_DualReductions, 0); // Disable dual reductions
        model_->set(GRB_DoubleParam_MarkowitzTol, 0.999);*/ // Numerical stability

    } catch (GRBException& e) {
        std::cerr << "Error in setParameters: " << e.getMessage() << std::endl;
        throw;
    }
}

// Solve the model
int GurobiModeler::solve() {
    try {
        model_->optimize();
        return model_->get(GRB_IntAttr_Status);

    } catch (GRBException& e) {
        std::cerr << "Error in solve: " << e.getMessage() << std::endl;
        throw;
    }
}

// Get solution status
int GurobiModeler::getStatus() const {
    try {
        return model_->get(GRB_IntAttr_Status);
    } catch (GRBException& e) {
        return -1;
    }
}

// Get objective value
double GurobiModeler::getObjValue() const {
    try {
        return model_->get(GRB_DoubleAttr_ObjVal);
    } catch (GRBException& e) {
        std::cerr << "Error getting objective value: " << e.getMessage() << std::endl;
        throw;
    }
}

// Get dual values
void GurobiModeler::getDuals() {
    try {
        // Get request constraint duals
        for (size_t i = 0; i < requestConstr_.size(); ++i) {
            requestDuals_[i] = requestConstr_[i].get(GRB_DoubleAttr_Pi);
        }

        // Get vehicle constraint duals
        for (size_t i = 0; i < vehicleConstr_.size(); ++i) {
            vehicleDuals_[i] = vehicleConstr_[i].get(GRB_DoubleAttr_Pi);
        }

    } catch (GRBException& e) {
        std::cerr << "Error in getDuals: " << e.getMessage() << std::endl;
        throw;
    }
}



// Get variable value
double GurobiModeler::getVarValue(const GRBVar& var) const {
    try {
        return var.get(GRB_DoubleAttr_X);
    } catch (GRBException& e) {
        std::cerr << "Error getting variable value: " << e.getMessage() << std::endl;
        throw;
    }
}

void GurobiModeler::getDualsFromRelaxed(GRBModel& relaxedModel) {
    try {
        // Get all constraints from the relaxed model
        GRBConstr* allConstrs = relaxedModel.getConstrs();
        int numConstrs = relaxedModel.get(GRB_IntAttr_NumConstrs);

        int constrIndex = 0;

        // Get request constraint duals (assuming they were added first)
        for (size_t i = 0; i < requestDuals_.size(); ++i) {
            requestDuals_[i] = allConstrs[constrIndex].get(GRB_DoubleAttr_Pi);
            constrIndex++;
        }

        // Get vehicle constraint duals (assuming they were added second)
        for (size_t i = 0; i < vehicleDuals_.size(); ++i) {
            vehicleDuals_[i] = allConstrs[constrIndex].get(GRB_DoubleAttr_Pi);
            constrIndex++;
        }

    } catch (GRBException& e) {
        std::cerr << "Error in getDualsFromRelaxed: " << e.getMessage() << std::endl;
        throw;
    }
}