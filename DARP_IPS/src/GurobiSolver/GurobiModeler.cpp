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
        env_.set(GRB_IntParam_LogToConsole, 0);
        env_.set(GRB_IntParam_UpdateMode, 1);
        env_.start();

        model_ = new GRBModel(env_);
        nbRequestTask_ = 0;

        solveTime_ = new myTools::Timer(); solveTime_->init();

        // Set model to minimize
        model_->set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

        // Apply tuned parameters
        model_->set(GRB_IntParam_OutputFlag, 0);
        model_->set(GRB_IntParam_Method, GRB_METHOD_DUAL);      // Dual simplex is typically best for CG
        model_->set(GRB_IntParam_Crossover, 1);
        model_->set(GRB_DoubleParam_Heuristics, 0.001);        // Minimal heuristics
        model_->set(GRB_IntParam_Presolve, 0);                  // Disable presolve
        model_->set(GRB_StringParam_LogFile, outputLog);

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

        // Reserve space for constraints
        requestConstr_.reserve(nbRequestTask_);
        vehicleConstr_.reserve(nbVehicles);

        // Clear existing constraints (in case of reinitialization)
        requestConstr_.clear();
        vehicleConstr_.clear();

        // Add request constraints (= rhs)
        for (int i = 0; i < nbRequestTask_; ++i) {
            requestConstr_.push_back(model_->addConstr(GRBLinExpr() == rhs, "R" + std::to_string(pInst->requests_[i]->getRequestId())));
        }



        // Add vehicle constraints (= rhs)
        for (int i = 0; i < nbVehicles; ++i) {
            vehicleConstr_.push_back(model_->addConstr(GRBLinExpr() == rhs, "V_" + std::to_string(i)));
        }

        // Set basic parameters
        model_->set(GRB_IntParam_Threads, pInst->parameters_->nbThreads_);
        model_->set(GRB_IntParam_Method, GRB_METHOD_DUAL);
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

        double objCoeff = signMultiplier * request->Req_W3_ * request->penalty_;
        GRBVar var = model_->addVar(0.0, GRB_INFINITY, objCoeff, GRB_BINARY, col, request->name_);
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

        double objCoeff = signMultiplier * request->Req_W3_ * request->penalty_;
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

        double objCoeff = signMultiplier * newRoute->objCoef_;
        GRBVar var = model_->addVar(0.0, GRB_INFINITY, objCoeff, GRB_BINARY, col, nullptr);
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

        double objCoeff = signMultiplier * newRoute->objCoef_;
        GRBVar var = model_->addVar(0.0, GRB_INFINITY, objCoeff, GRB_CONTINUOUS, col, nullptr);
        routeVar_.push_back(var);

    } catch (GRBException& e) {
        std::cerr << "Error in addRouteVarFloat: " << e.getMessage() << std::endl;
        throw;
    }
}

void GurobiModeler::convertToInt() {
    for (auto& var : zVar_)
        var.set(GRB_CharAttr_VType, GRB_BINARY);

    for (auto& var : routeVar_)
        var.set(GRB_CharAttr_VType, GRB_BINARY);
    model_->update();
}

void GurobiModeler::convertToFloat() {
    for (auto& var : zVar_)
        var.set(GRB_CharAttr_VType, GRB_CONTINUOUS);

    for (auto& var : routeVar_)
        var.set(GRB_CharAttr_VType, GRB_CONTINUOUS);
    model_->update();
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

// Get variable value
double GurobiModeler::getVarValue(const GRBVar& var) const {
    try {
        return var.get(GRB_DoubleAttr_X);
    } catch (GRBException& e) {
        std::cerr << "Error getting variable value: " << e.getMessage() << std::endl;
        throw;
    }
}

// Get dual values
void GurobiModeler::getDuals(const PInstance& pInst) {
    try {

        // Get request constraint duals
        for (size_t i = 0; i < requestConstr_.size(); ++i) {
            pInst->requests_[i]->dual_ = requestConstr_[i].get(GRB_DoubleAttr_Pi);
            if (std::fabs(pInst->requests_[i]->dual_) <= EPS)
                std::cout << "Zero Dual for Request: " << pInst->requests_[i]->getRequestId() << std::endl;
            /*double slack = requestConstr_[i].get(GRB_DoubleAttr_Slack);

            if (pInst->requests_[i]->dual_ == slack)
                std::cout << pInst->requests_[i]->getRequestId()
                      << " dual=" << pInst->requests_[i]->dual_
                      << " slack=" << slack << std::endl;*/
        }

        // Get vehicle constraint duals
        for (size_t i = 0; i < vehicleConstr_.size(); ++i) {
            if (pInst->vehicles_[i]->vehicleIndex_ > -1) {
                pInst->vehicles_[i]->dual_ = vehicleConstr_[i].get(GRB_DoubleAttr_Pi);
 //               pInst->vehicles_[i]->dual_ = 0;
            }
            else {
                pInst->vehicles_[i]->dual_ = 0;
            }
        }

    } catch (GRBException& e) {
        std::cerr << "Error in getDuals: " << e.getMessage() << std::endl;
        throw;
    }
}

void GurobiModeler::getBarrierDuals(const PInstance& pInst) {
    try {

        // Get request constraint duals
        for (size_t i = 0; i < requestConstr_.size(); ++i) {

            /*std::cout << pInst->requests_[i]->getRequestId()
                  << " old dual=" << pInst->requests_[i]->dual_
                  << " new dual=" << requestConstr_[i].get(GRB_DoubleAttr_BarPi) << std::endl;*/
            pInst->requests_[i]->dual_ = requestConstr_[i].get(GRB_DoubleAttr_BarPi);

        }

        // Get vehicle constraint duals
        for (size_t i = 0; i < vehicleConstr_.size(); ++i) {
            if (pInst->vehicles_[i]->vehicleIndex_ > -1) {
                pInst->vehicles_[i]->dual_ = vehicleConstr_[i].get(GRB_DoubleAttr_BarPi);
            }
            else {
                pInst->vehicles_[i]->dual_ = 0;
            }
        }

    } catch (GRBException& e) {
        std::cerr << "Error in getDuals: " << e.getMessage() << std::endl;
        throw;
    }
}

void GurobiModeler::getDualsFromRelaxed(GRBModel& relaxedModel, const PInstance& pInst) {
    try {
        // Get all constraints from the relaxed model
        GRBConstr* allConstrs = relaxedModel.getConstrs();
        int constrIndex = 0;

        // Update request duals (same constraint ordering)
        for (size_t i = 0; i < requestConstr_.size(); ++i) {
            pInst->requests_[i]->dual_ = allConstrs[constrIndex].get(GRB_DoubleAttr_Pi);
            constrIndex++;
        }

        // Get vehicle constraint duals (assuming they were added second)
        for (size_t i = 0; i < vehicleConstr_.size(); ++i) {
            if (pInst->vehicles_[i]->vehicleIndex_ > -1) {
                pInst->vehicles_[i]->dual_ = allConstrs[constrIndex].get(GRB_DoubleAttr_Pi);
            }
            else {
                pInst->vehicles_[i]->dual_ = 0;
            }
            constrIndex++;
        }
    } catch (GRBException& e) {
        std::cerr << "Error in getDualsFromRelaxed: " << e.getMessage() << std::endl;
        throw;
    }
}