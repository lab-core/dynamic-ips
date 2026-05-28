//
// Created by Elahe Amiri on 2025-09-16.
//

#include "CPModeler.h"
#include "solvers/SolverEnv.h"

// Constructor. env_ is now a reference to the process-wide Gurobi env so the
// per-instance env setup has moved into solverEnv::gurobi() (runs once).
CPModeler::CPModeler(std::string outputLog): env_(solverEnv::gurobi()), outputLog_(outputLog) {
    try {
        model_ = new GRBModel(env_);
        nbRequestTask_ = 0;

        solveTime_ = new myTools::Timer(); solveTime_->init();

        // Set model to minimize
        model_->set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

        // Apply tuned parameters
        model_->set(GRB_IntParam_OutputFlag, 0);
        model_->set(GRB_IntParam_Method, GRB_METHOD_DUAL);      // Dual simplex is typically best for CG
 //       model_->set(GRB_IntParam_Crossover, 1);
  //      model_->set(GRB_DoubleParam_Heuristics, 0.001);        // Minimal heuristics
 //       model_->set(GRB_IntParam_Presolve, 0);                  // Disable presolve
        model_->set(GRB_StringParam_LogFile, outputLog);



    } catch (GRBException& e) {
        std::cerr << "Error in GurobiModeler constructor: " << e.getMessage() << std::endl;
        throw;
    }
}

CPModeler::~CPModeler() {
    delete solveTime_;
    delete model_;
}

void CPModeler::initializeCP(const PInstance &pInst, bool reduced) {
    try {
        int rhs = 0;
        nbRequestTask_ = pInst->nbTasks_;

        // Initialize RHS arrays
        requestRHS_.clear();
        requestRHS_.resize(nbRequestTask_, rhs);

        // Clear existing constraints & Reserve space for constraints)
        for (auto& c : requestConstr_) model_->remove(c);
        requestConstr_.clear();
        requestConstr_.reserve(nbRequestTask_);

        // Add request constraints (= rhs)
        for (int i = 0; i < nbRequestTask_; ++i) {
            requestConstr_.push_back(model_->addConstr(GRBLinExpr() == rhs, "R" + std::to_string(pInst->requests_[i]->getRequestId())));
        }

        if (!reduced) {
            int nbVehicles = pInst->nbVehicles_;
            for (auto& c : vehicleConstr_) model_->remove(c);
            vehicleRHS_.clear();
            vehicleRHS_.resize(nbVehicles, rhs);
            vehicleConstr_.clear();
            vehicleConstr_.reserve(nbVehicles);

            // Add vehicle constraints (= rhs)
            for (int i = 0; i < nbVehicles; ++i) {
                GRBLinExpr expr = 0;
                vehicleConstr_.push_back(model_->addConstr(expr == rhs, "V" + std::to_string(pInst->vehicles_[i]->vehicleID_)));
            }
        }

        // Set basic parameters
        model_->set(GRB_IntParam_Threads, pInst->parameters_->nbThreads_);
        model_->set(GRB_IntParam_Method, GRB_METHOD_DUAL);

        // Add normalization constraint (sum = 1)
        GRBLinExpr normalExpr = 0;
        normalConst_ = model_->addConstr(normalExpr == 1.0, "normalConst");

        IncRoute_.clear();
        // Update model after adding normalization constraint
        model_->update();
    }
    catch (GRBException& e) {
        std::cerr << "Error in initializeCP: " << e.getMessage() << std::endl;
        throw;
    }
}

GRBColumn CPModeler::createRouteColumn(const PRoute &newRoute, const PRoute &currentVehicleRoute) const {
    GRBColumn col;

    newRoute->IncScore_ = 0;
    // Add coefficients for request constraints
    for (auto& requestObj : currentVehicleRoute->routeRequests_) {
        if (!newRoute->column_.test(requestObj->taskIndex_) ) {
            col.addTerm(-1, requestConstr_[requestObj->taskIndex_]);
            newRoute->IncScore_++;
        }
    }
    for (auto& requestObj : newRoute->routeRequests_) {
        if (!currentVehicleRoute->column_.test(requestObj->taskIndex_) ) {
            col.addTerm(1, requestConstr_[requestObj->taskIndex_]);
            newRoute->IncScore_++;
        }
    }

    // Add to normalization constraint
    col.addTerm(newRoute->IncScore_, normalConst_);
    return col;
}

GRBColumn CPModeler::createRouteColumn(const PRoute &newRoute, VarSign sign) const {
    GRBColumn col;
    int signMultiplier = (sign == POSITIVE) ? 1 : -1;

    // Add coefficients for request constraints
    for (auto& requestObj : newRoute->routeRequests_) {
        col.addTerm(signMultiplier, requestConstr_[requestObj->taskIndex_]);
    }

    // Add coefficient for vehicle constraint
    int vehIndex = newRoute->vehicleID_;
    col.addTerm(signMultiplier, vehicleConstr_[vehIndex]);

    return col;
}

void CPModeler::addRouteVar(const PRoute &newRoute, const PRoute &currentVehicleRoute) {
    try {
        GRBColumn col = createRouteColumn(newRoute, currentVehicleRoute);
 //       if (newRoute->IncScore_ > 0) {
            double objCoeff = newRoute->objCoef_ - currentVehicleRoute->objCoef_;

            // Create variable
            GRBVar var = model_->addVar(0.0, GRB_INFINITY, objCoeff, GRB_CONTINUOUS, col, nullptr);
            routeIncVar_.push_back(var);
            IncRoute_.push_back(newRoute);
            newRoute->cpAdded_ = true;
 //       }
    }
    catch (GRBException& e) {
        std::cerr << "Error in addRouteVar: " << e.getMessage() << std::endl;
        throw;
    }
}

void CPModeler::addRouteVar(const PRoute &newRoute, VarSign sign) {
    try {
        int signMultiplier = (sign == POSITIVE) ? 1 : -1;
        GRBColumn col = createRouteColumn(newRoute, sign);

        double objCoeff = signMultiplier * newRoute->objCoef_;

        if (sign == NEGATIVE) {
            // Call parent class method for negative sign
            // Create variable
            GRBVar var = model_->addVar(0.0, GRB_INFINITY, objCoeff, GRB_CONTINUOUS, col, nullptr);
            routeSolVar_.push_back(var);
        }
        else {

            // Add to normalization constraint
            col.addTerm(1.0, normalConst_);

            // Create variable
            GRBVar var = model_->addVar(0.0, GRB_INFINITY, objCoeff, GRB_CONTINUOUS, col, nullptr);
            routeIncVar_.push_back(var);
            IncRoute_.push_back(newRoute);
        }
    }
    catch (GRBException& e) {
        std::cerr << "Error in addRouteVar: " << e.getMessage() << std::endl;
        throw;
    }
}

void CPModeler::addZVar(const PRequest &request) {
    try {
        GRBColumn col;
        col.addTerm(1.0, requestConstr_[request->taskIndex_]);
        col.addTerm(1.0, normalConst_);

        // Create variable
        GRBVar var = model_->addVar(0.0, GRB_INFINITY, request->Req_W3_ * request->penalty_, GRB_CONTINUOUS, col, request->name_);
        zIncVar_.push_back(var);
    }
    catch (GRBException& e) {
        std::cerr << "Error in addRouteVar: " << e.getMessage() << std::endl;
        throw;
    }
}

void CPModeler::addZVar(const PRequest &request, VarSign sign) {
    try {
        if (sign == NEGATIVE) {
            // Build column coefficient expression
            GRBColumn col;

            // Add to request constraint
            col.addTerm(-1, requestConstr_[request->taskIndex_]);

            // Create variable
            GRBVar var = model_->addVar(0.0, GRB_INFINITY, (-1) * request->Req_W3_ * request->penalty_, GRB_CONTINUOUS,
                                        col, request->name_);
            zSolVar_.push_back(var);
        }
        else if (sign == POSITIVE)
            addZVar(request);
    }
    catch (GRBException& e) {
        std::cerr << "Error in addZVar: " << e.getMessage() << std::endl;
        throw;
    }
}

void CPModeler::addRouteVarBatch(const PInstance &pInst) {
    const size_t numRoutes = routesToAdd_.size();

    std::vector lb(numRoutes, 0.0);
    std::vector ub(numRoutes, GRB_INFINITY);
    std::vector<double> obj(numRoutes);
    std::vector<char> vtype(numRoutes, GRB_CONTINUOUS);
    std::vector<GRBColumn> columns(numRoutes);

    IncRoute_.reserve(IncRoute_.size() + numRoutes);

    // Fill arrays with range-based loop
    size_t k = 0;
    for (const auto& routeObj : routesToAdd_) {
        obj[k] = routeObj->objCoef_ - pInst->vehicles_[routeObj->vehicleID_]->currentRoute_->objCoef_;
        columns[k] = createRouteColumn(routeObj, pInst->vehicles_[routeObj->vehicleID_]->currentRoute_);
        IncRoute_.emplace_back(routeObj);
        routeObj->cpAdded_ = true;
        ++k;
    }

    // Batch addition using vector data
    std::unique_ptr<GRBVar[]> newVars(model_->addVars(
        lb.data(),
        ub.data(),
        obj.data(),
        vtype.data(),
        nullptr,
        columns.data(),
        numRoutes));

    // Add variables to container
    routeIncVar_.insert(routeIncVar_.end(), newVars.get(), newVars.get() + numRoutes);
}

void CPModeler::addRouteSolVarBatch(const std::vector<PRoute> &routeSolution) {
    const int numRoutes = static_cast<int>(routeSolution.size());

    std::vector lb(numRoutes, 0.0);
    std::vector ub(numRoutes, GRB_INFINITY);
    std::vector<double> obj(numRoutes);
    std::vector<char> vtype(numRoutes, GRB_CONTINUOUS);
    std::vector<GRBColumn> columns(numRoutes);

    // Fill arrays with range-based loop
    size_t k = 0;
    for (const auto& routeObj : routeSolution) {
        obj[k] = (-1) * routeObj->objCoef_ ;
        columns[k] = createRouteColumn(routeObj, NEGATIVE);
        routeObj->cpAdded_ = true;
        ++k;
    }

    // Batch addition using vector data
    std::unique_ptr<GRBVar[]> newVars(model_->addVars(
        lb.data(), ub.data(), obj.data(), vtype.data(), nullptr, columns.data(), numRoutes));

    // Add variables to container
    routeSolVar_.insert(routeSolVar_.end(), newVars.get(), newVars.get() + numRoutes);
}

void CPModeler::addRouteIncVarBatch() {
    const size_t numRoutes = routesToAdd_.size();

    std::vector lb(numRoutes, 0.0);
    std::vector ub(numRoutes, GRB_INFINITY);
    std::vector<double> obj(numRoutes);
    std::vector<char> vtype(numRoutes, GRB_CONTINUOUS);
    std::vector<GRBColumn> columns(numRoutes);

    IncRoute_.reserve(IncRoute_.size() + numRoutes);

    // Fill arrays with range-based loop
    size_t k = 0;
    for (const auto& routeObj : routesToAdd_) {
        obj[k] = routeObj->objCoef_;
        columns[k] = createRouteColumn(routeObj, POSITIVE);
        columns[k].addTerm(1, normalConst_);
        IncRoute_.emplace_back(routeObj);
        routeObj->cpAdded_ = true;
        ++k;
    }

    // Batch addition using vector data
    std::unique_ptr<GRBVar[]> newVars(model_->addVars(
        lb.data(), ub.data(), obj.data(), vtype.data(), nullptr, columns.data(), numRoutes));

    // Add variables to container
    routeIncVar_.insert(routeIncVar_.end(), newVars.get(), newVars.get() + numRoutes);
}

void CPModeler::addZVarBatch(const PInstance &pInst) {
    if (pInst->nbRequests_ == 0) return;

    std::vector<double> lb, ub, obj;
    std::vector<char> vtype;
    std::vector<GRBColumn> cols;
    std::vector<std::string> names;

    for (int i = 0; i < pInst->nbRequests_; ++i) {
        const auto& req = pInst->requests_[i];
        if (req->committedPickTime_ == LARGE_CONSTANT && req->solVehicleID_ < LARGE_CONSTANT) {
    //    if (req->solVehicleID_ < LARGE_CONSTANT) {
            lb.push_back(0.0);
            ub.push_back(GRB_INFINITY);
            obj.push_back(req->Req_W3_ * req->penalty_);
            vtype.push_back(GRB_CONTINUOUS);

            GRBColumn col;
            col.addTerm(1.0, requestConstr_[req->taskIndex_]);
            col.addTerm(1.0, normalConst_);
            cols.push_back(col);
            names.push_back(req->name_);
        }
    }
    if (lb.empty()) return;

    std::unique_ptr<GRBVar[]> zvars(model_->addVars(
        lb.data(), ub.data(), obj.data(), vtype.data(),
        names.data(), cols.data(), (int)lb.size()));

    zIncVar_.insert(zIncVar_.end(), zvars.get(), zvars.get() + lb.size());
}

void CPModeler::buildModel(const PInstance &pInst) {
    try {
        // Adding incompatible route columns
        for (auto& routeAdd : routesToAdd_)
            addRouteVar(routeAdd, pInst->vehicles_[routeAdd->vehicleID_]->currentRoute_);

        // Adding z columns out of the basis
        for (int i = 0; i < pInst->nbRequests_; ++i)
            addZVar(pInst->requests_[i]);

        model_->update();
    }
    catch (GRBException& e) {
        std::cerr << "Error in buildModel: " << e.getMessage() << std::endl;
        throw;
    }
}

void CPModeler::buildModel(const PInstance &pInst, const std::vector<PRoute> &routeSolution) {
    try {
        // Adding solution route columns
        for (auto& routeObj : routeSolution) {
            addRouteVar(routeObj, NEGATIVE);
            routeObj->cpAdded_ = true;
        }

        for (auto& requestObj : pInst->requests_) {
            // Adding solution z columns
            if (requestObj->solVehicleID_ == LARGE_CONSTANT)
                addZVar(requestObj, NEGATIVE);
            // Adding z columns out of the basis
            else if (requestObj->committedPickTime_ == LARGE_CONSTANT)
                addZVar(requestObj, POSITIVE);
        }
        model_->update();
    }
    catch (GRBException& e) {
        std::cerr << "Error in buildModel: " << e.getMessage() << std::endl;
        throw;
    }
}

void CPModeler::buildModel_batch(const PInstance &pInst) {
    try {
        if (routesToAdd_.empty()) return;

        // Validate model before proceeding
        if (!model_) {
            throw std::runtime_error("Model is not initialized");
        }

        // Adding incompatible route columns
        addRouteVarBatch(pInst);

        // Adding z columns out of the basis
        addZVarBatch(pInst);
        model_->update();

    } catch (const GRBException& e) {
        std::cerr << "Error in buildModel_batch: " << e.getMessage() << std::endl;
        throw;
    } catch (const std::exception& e) {
        std::cerr << "Error in buildModel_batch: " << e.what() << std::endl;
        throw;
    }
}

void CPModeler::buildModel_batch(const PInstance &pInst, const std::vector<PRoute> &routeSolution) {
    try {
        // Adding solution route columns
        addRouteSolVarBatch(routeSolution);

        for (auto& requestObj : pInst->requests_) {
            // Adding solution z columns
            if (requestObj->solVehicleID_ == LARGE_CONSTANT)
                addZVar(requestObj, NEGATIVE);
        }
        // Adding incompatible z columns
        addZVarBatch(pInst);

        model_->update();
    }
    catch (GRBException& e) {
        std::cerr << "Error in buildModel: " << e.getMessage() << std::endl;
        throw;
    }
}

void CPModeler::repairModel(const PInstance &pInst, const std::vector<PRoute> &routeSolution) {
    try {
        // Remove existing solution variables

        for (auto& var : routeSolVar_) {
            model_->remove(var);
        }
        routeSolVar_.clear();

        for (auto& var : zSolVar_) {
            model_->remove(var);
        }
        zSolVar_.clear();

        addRouteSolVarBatch(routeSolution);

        for (auto& requestObj : pInst->requests_) {
            // Adding solution z columns
            if (requestObj->solVehicleID_ == LARGE_CONSTANT)
                addZVar(requestObj, NEGATIVE);
        }
        addZVarBatch(pInst);

        model_->update();
    }
    catch (GRBException& e) {
        std::cerr << "Error in repairModel: " << e.getMessage() << std::endl;
        throw;
    }
}

void CPModeler::updateModel() {
    try {
        // Adding incompatible route columns
        addRouteIncVarBatch();
        model_->update();
    }
    catch (GRBException& e) {
        std::cerr << "Error in updateModel: " << e.getMessage() << std::endl;
        throw;
    }
}

void CPModeler::updateModel(const PInstance& pInst) {
    try {
        // Adding incompatible route columns
        addRouteIncVarBatch();
        model_->update();
    }
    catch (GRBException& e) {
        std::cerr << "Error in updateModel: " << e.getMessage() << std::endl;
        throw;
    }
}

void CPModeler::updateNormalCoefficients() {
    // Validate model and constraint before proceeding
    if (!model_) {
        throw std::runtime_error("Model is not initialized for coefficient update");
    }

    // Update coefficients of existing variables in normalConst_ based on current incompatibilityDegree_
    for (size_t i = 0; i < IncRoute_.size(); ++i) {
        // Validate route object and corresponding variable exist
        if (!IncRoute_[i]) {
            std::cerr << "Warning: Null route object at index " << i << std::endl;
            continue;
        }

        if (i >= routeIncVar_.size()) {
            std::cerr << "Warning: Route index " << i << " exceeds variable array size" << std::endl;
            break;
        }

        // Get current coefficient and update if changed
        double currentCoeff = model_->getCoeff(normalConst_, routeIncVar_[i]);
        double newCoeff = IncRoute_[i]->incompatibilityDegree_;

        // Only update if coefficient has changed to avoid unnecessary operations
        if (std::abs(currentCoeff - newCoeff) > 1e-9) {
            model_->chgCoeff(normalConst_, routeIncVar_[i], newCoeff);
        }
    }
}

void CPModeler::resetForNextIteration() {
    try {
        nbRequestTask_ = 0;

        // Reset route flags
        for (auto &routeObj : IncRoute_)
            routeObj->cpAdded_ = false;

        IncRoute_.clear();
        fractionalRoutes_.clear();
        fractionalZ_.clear();
        vehicleRHS_.clear();
        requestRHS_.clear();
        routesToAdd_.clear();

        // --- Remove constraints ---
        for (auto& constr : requestConstr_)
            model_->remove(constr);
        requestConstr_.clear();

        for (auto& constr : vehicleConstr_)
            model_->remove(constr);
        vehicleConstr_.clear();

        model_->remove(normalConst_);

        // --- Remove variables ---
        for (auto& var : routeIncVar_) model_->remove(var);
        for (auto& var : zIncVar_)     model_->remove(var);
        for (auto& var : routeSolVar_) model_->remove(var);
        for (auto& var : zSolVar_)     model_->remove(var);

        routeIncVar_.clear();
        zIncVar_.clear();
        routeSolVar_.clear();
        zSolVar_.clear();
        model_->reset(1);

        // --- Apply all changes ---
        model_->update();

    } catch (const GRBException& e) {
        std::cerr << "Gurobi error in resetForNextIteration: "
                  << e.getMessage() << std::endl;
        throw;
    }
}

void CPModeler::getDuals(const PInstance &pInst) {
    try {

        // Get request constraint duals
        for (size_t i = 0; i < requestConstr_.size(); ++i) {
            /*std::cout << "Request: " << std::left << std::setw(5) << pInst->requests_[i]->getRequestId()
                      << " LP dual=" << std::setw(6) << pInst->requests_[i]->dual_
                      << " CP dual=" << std::setw(6) << requestConstr_[i].get(GRB_DoubleAttr_Pi)
                      << "  CBasis=" << std::setw(6) << requestConstr_[i].get(GRB_IntAttr_CBasis) << std::endl;*/

 //           if (requestConstr_[i].get(GRB_DoubleAttr_Pi) > 0)
                pInst->requests_[i]->dual_ = requestConstr_[i].get(GRB_DoubleAttr_Pi);
            /*if (pInst->requests_[i]->dual_ == 0)
                std::cout << "Request: " << std::left << std::setw(5) << pInst->requests_[i]->getRequestId()
                      << " LP dual=" << std::setw(6) << pInst->requests_[i]->dual_
                      << " CP dual=" << std::setw(6) << requestConstr_[i].get(GRB_DoubleAttr_Pi)
                      << "  CBasis=" << std::setw(6) << requestConstr_[i].get(GRB_IntAttr_CBasis) << std::endl;*/
        }

        // Get vehicle constraint duals
        for (size_t i = 0; i < vehicleConstr_.size(); ++i) {
            /*std::cout << "Vehicle: " << std::left << std::setw(5) << i
                      << " LP dual=" << std::setw(6) << pInst->vehicles_[i]->dual_
                      << " CP dual=" << std::setw(6) << vehicleConstr_[i].get(GRB_DoubleAttr_Pi)
                      << "  CBasis=" << std::setw(6) << vehicleConstr_[i].get(GRB_IntAttr_CBasis) << std::endl;*/
            pInst->vehicles_[i]->dual_ = vehicleConstr_[i].get(GRB_DoubleAttr_Pi);
        }

    } catch (GRBException& e) {
        std::cerr << "Error in getDuals: " << e.getMessage() << std::endl;
        throw;
    }
}

int CPModeler::getStatus() const {
    try {
        return model_->get(GRB_IntAttr_Status);
    } catch (GRBException& e) {
        return -1;
    }
}

double CPModeler::getObjValue() const {
    try {
        return model_->get(GRB_DoubleAttr_ObjVal);
    } catch (GRBException& e) {
        std::cerr << "Error getting objective value: " << e.getMessage() << std::endl;
        throw;
    }
}

double CPModeler::getVarValue(const GRBVar &var) const {
    try {
        return var.get(GRB_DoubleAttr_X);
    } catch (GRBException& e) {
        std::cerr << "Error getting variable value: " << e.getMessage() << std::endl;
        throw;
    }
}
// Add these methods to CPModeler

void CPModeler::saveModelBeforeSolve(const std::string& prefix) {
    model_->update();
    model_->write(prefix + ".lp");
    model_->write(prefix + ".mps");
}

void CPModeler::printBasisVariables(const std::string& outputFile) {
    try {
        std::ofstream out(outputFile);
        if (!out.is_open()) {
            std::cerr << "Failed to open " << outputFile << std::endl;
            return;
        }

        out << "=== BASIS VARIABLES (non-zero values) ===" << std::endl;
        out << std::fixed << std::setprecision(6);

        // Route increment variables
        out << "\n--- Route Inc Variables ---" << std::endl;
        for (size_t i = 0; i < routeIncVar_.size(); ++i) {
            double val = routeIncVar_[i].get(GRB_DoubleAttr_X);
            if (val > 1e-9) {
                out << "RouteInc[" << i << "]: " << val
                    << " (obj=" << routeIncVar_[i].get(GRB_DoubleAttr_Obj)
                    << ", RC=" << routeIncVar_[i].get(GRB_DoubleAttr_RC) << ")"
                    << " VehID=" << IncRoute_[i]->vehicleID_
                    << std::endl;
            }
        }

        // Route solution variables
        out << "\n--- Route Sol Variables ---" << std::endl;
        for (size_t i = 0; i < routeSolVar_.size(); ++i) {
            double val = routeSolVar_[i].get(GRB_DoubleAttr_X);
            if (val > 1e-9) {
                out << "RouteSol[" << i << "]: " << val
                    << " (obj=" << routeSolVar_[i].get(GRB_DoubleAttr_Obj)
                    << ", RC=" << routeSolVar_[i].get(GRB_DoubleAttr_RC) << ")"
                    << std::endl;
            }
        }

        // Z increment variables
        out << "\n--- Z Inc Variables ---" << std::endl;
        for (size_t i = 0; i < zIncVar_.size(); ++i) {
            double val = zIncVar_[i].get(GRB_DoubleAttr_X);
            if (val > 1e-9) {
                out << "ZInc[" << zIncVar_[i].get(GRB_StringAttr_VarName) << "]: " << val
                    << " (obj=" << zIncVar_[i].get(GRB_DoubleAttr_Obj)
                    << ", RC=" << zIncVar_[i].get(GRB_DoubleAttr_RC) << ")"
                    << std::endl;
            }
        }

        // Z solution variables
        out << "\n--- Z Sol Variables ---" << std::endl;
        for (size_t i = 0; i < zSolVar_.size(); ++i) {
            double val = zSolVar_[i].get(GRB_DoubleAttr_X);
            if (val > 1e-9) {
                out << "ZSol[" << zSolVar_[i].get(GRB_StringAttr_VarName) << "]: " << val
                    << " (obj=" << zSolVar_[i].get(GRB_DoubleAttr_Obj)
                    << ", RC=" << zSolVar_[i].get(GRB_DoubleAttr_RC) << ")"
                    << std::endl;
            }
        }

        // Dual values
        out << "\n--- Constraint Duals ---" << std::endl;
        for (size_t i = 0; i < requestConstr_.size(); ++i) {
            double pi = requestConstr_[i].get(GRB_DoubleAttr_Pi);
            if (std::abs(pi) > 1e-9) {
                out << requestConstr_[i].get(GRB_StringAttr_ConstrName)
                    << ": Pi=" << pi << std::endl;
            }
        }

        out << "\nNormal constraint dual: " << normalConst_.get(GRB_DoubleAttr_Pi) << std::endl;
        out << "Objective value: " << model_->get(GRB_DoubleAttr_ObjVal) << std::endl;

        out.close();
        std::cout << "Basis variables written to " << outputFile << std::endl;

    } catch (GRBException& e) {
        std::cerr << "Error in printBasisVariables: " << e.getMessage() << std::endl;
    }
}

void CPModeler::saveBasisModel(const std::string& prefix) {
    try {
        // Create a new model with only basis variables
        GRBModel basisModel(env_);
        basisModel.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

        // Collect non-zero variables and recreate constraints
        std::vector<std::pair<GRBVar*, double>> basisVars;

        auto collectNonZero = [&](std::vector<GRBVar>& vars) {
            for (auto& var : vars) {
                double val = var.get(GRB_DoubleAttr_X);
                if (val > 1e-9) {
                    basisVars.push_back({&var, val});
                }
            }
        };

        collectNonZero(routeIncVar_);
        collectNonZero(routeSolVar_);
        collectNonZero(zIncVar_);
        collectNonZero(zSolVar_);

        // Write solution file (shows only non-zero variables)
        model_->write(prefix + "_solution.sol");

        // Write the full model for reference
        model_->write(prefix + "_full.lp");

        std::cout << "Basis model saved with prefix: " << prefix << std::endl;
        std::cout << "Non-zero variables in basis: " << basisVars.size() << std::endl;

    } catch (GRBException& e) {
        std::cerr << "Error in saveBasisModel: " << e.getMessage() << std::endl;
    }
}

// Verification method to check solution consistency
bool CPModeler::verifyBasisSolution(const PInstance& pInst) {
    try {
        std::cout << "\n=== SOLUTION VERIFICATION ===" << std::endl;

        // Check constraint satisfaction
        double normalSum = 0.0;
        std::vector<double> requestCoverage(nbRequestTask_, 0.0);

        // Check route inc variables contribution
        for (size_t i = 0; i < routeIncVar_.size(); ++i) {
            double val = routeIncVar_[i].get(GRB_DoubleAttr_X);
            if (val > 1e-9) {
                normalSum += val * IncRoute_[i]->incompatibilityDegree_;
                for (auto& req : IncRoute_[i]->routeRequests_) {
                    requestCoverage[req->taskIndex_] += val;
                }
            }
        }

        // Check z inc variables contribution
        for (size_t i = 0; i < zIncVar_.size(); ++i) {
            double val = zIncVar_[i].get(GRB_DoubleAttr_X);
            if (val > 1e-9) {
                normalSum += val;
                auto req = pInst->nameToRequest_[zIncVar_[i].get(GRB_StringAttr_VarName)];
                requestCoverage[req->taskIndex_] += val;
            }
        }

        std::cout << "Normal constraint sum: " << normalSum << " (should be 1.0)" << std::endl;

        bool valid = std::abs(normalSum - 1.0) < 1e-6;

        // Check for over-coverage
        for (int i = 0; i < nbRequestTask_; ++i) {
            if (requestCoverage[i] > 1.0 + 1e-6) {
                std::cout << "WARNING: Request " << i << " over-covered: " << requestCoverage[i] << std::endl;
                valid = false;
            }
        }

        std::cout << "Solution valid: " << (valid ? "YES" : "NO") << std::endl;
        return valid;

    } catch (GRBException& e) {
        std::cerr << "Error in verifyBasisSolution: " << e.getMessage() << std::endl;
        return false;
    }
}

void CPModeler::solveCPModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                             InputPaths &inputPaths) {
    try {
 //       saveModelBeforeSolve("debug_pre_solve");
        // Set up logging
        solveTime_->start();
  //      dump_gurobi();
        model_->optimize();
        solveTime_->stop();

        if (getStatus() != GRB_OPTIMAL) {
            status_ = INFEASIBLE;
            std::cout << "Failed to optimize the problem" << std::endl;
        }
        else {
            // Get dual values
            getDuals(pInst);

            // Check objective value
            double objVal = getObjValue();

            if (objVal < -0.1) {
                status_ = NEGATIVE_VALUE;

                // Collect incoming and outgoing variables
                std::vector<PRoute> routeResult;
                std::vector<PRequest> zResult;

                std::vector<int> InRouteVar;
                std::vector<int> InRequestVar;

                // Determine incoming variables
                for (size_t r = 0; r < routeIncVar_.size(); ++r) {
                    if (getVarValue(routeIncVar_[r]) > 0) {
                        routeResult.push_back(IncRoute_[r]);
                        InRouteVar.push_back(static_cast<int>(r));
                    }
                }

                for (size_t i = 0; i < zIncVar_.size(); ++i) {
                    if (getVarValue(zIncVar_[i]) > 0) {
                        zResult.push_back(pInst->nameToRequest_[zIncVar_[i].get(GRB_StringAttr_VarName)]);
                        InRequestVar.push_back(static_cast<int>(i));
                    }
                }

                // Check if solution is column disjoint
                if (isColumnDisjoint(routeResult, pInst->nbRequests_, pInst->nbVehicles_)) {

                    // Process incoming variables
                    for (int idx = InRouteVar.size() - 1; idx >= 0; --idx) {
                        int r = InRouteVar[idx];
                        routeSolution.push_back(IncRoute_[r]);
                    }

                    for (int idx = InRequestVar.size() - 1; idx >= 0; --idx) {
                        int i = InRequestVar[idx];
                        zSolution.push_back(pInst->nameToRequest_[zIncVar_[i].get(GRB_StringAttr_VarName)]);
                    }


                    for (auto& routeObj : routeResult)
                        pInst->vehicles_[routeObj->vehicleID_]->currentRoute_ = routeObj;
                    routeSolution.clear();
                    for (auto & vehicleObj: pInst->vehicles_)
                        routeSolution.push_back(vehicleObj->currentRoute_);
                    pInst->resetAssignedVehicles();
                    for (auto &routeObj : routeSolution) {
                        pInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
                    }
                    zSolution.clear();
                    for (auto &requestObj: pInst->requests_) {
                        if (requestObj->solVehicleID_ == LARGE_CONSTANT)
                            zSolution.push_back(requestObj);
                    }

                } else {
                    status_ = FRACTIONAL;
                    if (pInst->parameters_->useZoom_) {
                        fractionalRoutes_.clear();
                        fractionalZ_.clear();

                        for (auto& r : InRouteVar) {
                            fractionalRoutes_.push_back(IncRoute_[r]);
                        }
                    }
                }
            } else {
                status_ = POSITIVE_VALUE;
            }
        }
  //      model_->update();
    }
    catch (GRBException& e) {
        std::cerr << "Error in solveCPModel: " << e.getMessage() << std::endl;
        throw;
    }
}

void CPModeler::solveCPModel(PInstance &pInst, std::vector<PRequest> &zSolution,
    std::vector<PRoute> &routeSolution, InputPaths &inputPaths, bool update) {
    try {
 //       saveModelBeforeSolve("debug_pre_solve");
        // Set up logging
        solveTime_->start();
        model_->update();
 //       dump_gurobi();
        model_->optimize();
        solveTime_->stop();

        if (getStatus() == GRB_INFEASIBLE) {
            model_->computeIIS();
            model_->write("infeasible.ilp");  // IIS in LP-like format

            // Optional: print members of the IIS in code
            int nC = model_->get(GRB_IntAttr_NumConstrs);
            for (int i = 0; i < nC; ++i) {
                GRBConstr c = model_->getConstr(i);
                if (c.get(GRB_IntAttr_IISConstr)) {
                    std::cout << "IIS constr: " << c.get(GRB_StringAttr_ConstrName) << "\n";
                }
            }
            int nV = model_->get(GRB_IntAttr_NumVars);
            for (int j = 0; j < nV; ++j) {
                GRBVar v = model_->getVar(j);
                if (v.get(GRB_IntAttr_IISLB) || v.get(GRB_IntAttr_IISUB)) {
                    std::cout << "IIS var bound: " << v.get(GRB_StringAttr_VarName)
                              << " (LB in IIS=" << v.get(GRB_IntAttr_IISLB)
                              << ", UB in IIS=" << v.get(GRB_IntAttr_IISUB) << ")\n";
                }
            }
        }

        if (getStatus() != GRB_OPTIMAL) {
            status_ = INFEASIBLE;
            std::cout << "Failed to optimize the problem" << std::endl;
            myTools::myException::throwError("CP solution is not valid!!!");
        }
        else {
            // Get dual values
            getDuals(pInst);

            // Check objective value
            double objVal = getObjValue();

            if (objVal < -0.1) {
                status_ = NEGATIVE_VALUE;

 //               printBasisVariables("debug_after_solve");

                // Collect incoming and outgoing variables
                std::vector<PRoute> routeResult;
                std::vector<PRequest> zResult;

                std::vector<int> InRouteVar;
                std::vector<int> InRequestVar;

                std::vector<int> OutRouteVar;
                std::vector<int> OutRequestVar;

                // Determine incoming variables
                for (size_t r = 0; r < routeIncVar_.size(); ++r) {
                    if (getVarValue(routeIncVar_[r]) > 0) {
                        routeResult.push_back(IncRoute_[r]);
                        InRouteVar.push_back(static_cast<int>(r));
                    }
                }

                for (size_t i = 0; i < zIncVar_.size(); ++i) {
                    if (getVarValue(zIncVar_[i]) > 0) {
                        zResult.push_back(pInst->nameToRequest_[zIncVar_[i].get(GRB_StringAttr_VarName)]);
                        InRequestVar.push_back(static_cast<int>(i));
                    }
                }

                // Determine outgoing variables
                for (size_t r = 0; r < routeSolVar_.size(); ++r) {
                    if (getVarValue(routeSolVar_[r]) > 0) {
                        OutRouteVar.push_back(static_cast<int>(r));
                    }
                }

                for (size_t i = 0; i < zSolVar_.size(); ++i) {
                    if (getVarValue(zSolVar_[i]) > 0) {
                        OutRequestVar.push_back(static_cast<int>(i));
                    }
                }

                // Check if solution is column disjoint
                if (isColumnDisjoint(routeResult, pInst->nbRequests_, pInst->nbVehicles_)) {

                    // Process outgoing variables (reverse index order so erasures don't invalidate indices)
                    for (int idx = OutRouteVar.size() - 1; idx >= 0; --idx) {
                        int r = OutRouteVar[idx];
                        PRoute outRoute = routeSolution[r];
                        if (update) {
                            addRouteVar(outRoute, POSITIVE);
                            model_->remove(routeSolVar_[r]);
                            routeSolVar_.erase(routeSolVar_.begin() + r);
                        }
                        routeSolution.erase(routeSolution.begin() + r);
                    }

                    for (int idx = OutRequestVar.size() - 1; idx >= 0; --idx) {
                        int i = OutRequestVar[idx];
                        PRequest outRequest = zSolution[i];
                        if (update) {
                            addZVar(outRequest, POSITIVE);
                            model_->remove(zSolVar_[i]);
                            zSolVar_.erase(zSolVar_.begin() + i);
                        }
                        zSolution.erase(zSolution.begin() + i);
                    }

                    // Process incoming variables (in reverse order)
                    for (int idx = InRouteVar.size() - 1; idx >= 0; --idx) {
                        int r = InRouteVar[idx];
                        routeSolution.push_back(IncRoute_[r]);
                        if (update) {
                            addRouteVar(routeSolution.back(), NEGATIVE);
                            model_->remove(routeIncVar_[r]);
                            routeIncVar_.erase(routeIncVar_.begin() + r);
                            IncRoute_.erase(IncRoute_.begin() + r);
                        }
                    }

                    for (int idx = InRequestVar.size() - 1; idx >= 0; --idx) {
                        int i = InRequestVar[idx];
                        zSolution.push_back(pInst->nameToRequest_[zIncVar_[i].get(GRB_StringAttr_VarName)]);
                        if (update) {
                            addZVar(zSolution.back(), NEGATIVE);
                            model_->remove(zIncVar_[i]);
                            zIncVar_.erase(zIncVar_.begin() + i);
                        }
                    }
                } else {
                    status_ = FRACTIONAL;
                    if (pInst->parameters_->useZoom_) {
                        fractionalRoutes_.clear();
                        fractionalZ_.clear();

                        for (auto& r : OutRouteVar) {
                            fractionalRoutes_.push_back(routeSolution[r]);
                        }

                        for (auto& r : InRouteVar) {
                            fractionalRoutes_.push_back(IncRoute_[r]);
                        }
                    }
                }
            } else {
                status_ = POSITIVE_VALUE;
            }
        }
 //       if (!isColumnDisjoint(routeSolution, pInst->nbRequests_, pInst->nbVehicles_))
 //           myTools::myException::throwError("CP solution is not valid!!!");
  //      model_->update();
    }
    catch (GRBException& e) {
        std::cerr << "Error in solveCPModel: " << e.getMessage() << std::endl;
        throw;
    }
}

void CPModeler::solveCPDual(PInstance &pInst, InputPaths &inputPaths) {
    try {
        // Set up logging
        solveTime_->start();
        model_->optimize();
        dump_gurobi();
        solveTime_->stop();

        if (getStatus() != GRB_OPTIMAL) {
            status_ = INFEASIBLE;
            std::cout << "Failed to optimize the problem" << std::endl;
        }
        else {
            // Get dual values
            getDuals(pInst);
        }
    }
    catch (GRBException& e) {
        std::cerr << "Error in solveCPModel: " << e.getMessage() << std::endl;
        throw;
    }
}

bool CPModeler::isColumnDisjoint(const std::vector<PRoute> &routeResults, int nbRequests, int nbVehicles) {
    boost::dynamic_bitset<> coveredRequests(nbRequests);
    boost::dynamic_bitset<> vehicles(nbVehicles);

    // for each route: test for any overlap, then mark bits and vehicle
    for (auto& route : routeResults) {
        if ((coveredRequests & route->column_).any())
            return false;
        if (vehicles.test(route->vehicleID_))
            return false;

        coveredRequests |= route->column_;
        vehicles.set(route->vehicleID_, true);
    }
    return true;
}

bool CPModeler::isColumnDisjointFast(const std::vector<PRequest>& zResults,
                                         const std::vector<PRoute>& routeResults,
                                         const PInstance& pInst) {
    boost::dynamic_bitset<> coveredRequests(pInst->nbRequests_);
    boost::dynamic_bitset<> usedVehicles(pInst->nbVehicles_);

    // mark each request index; exit on duplicate
    for (auto& requestObj : zResults) {
        if (coveredRequests.test(requestObj->taskIndex_))
            return false;
        coveredRequests.set(requestObj->taskIndex_);
    }

    // for each route: test for any overlap, then mark bits and vehicle
    for (auto& route : routeResults) {
        if ((coveredRequests & route->column_).any())
            return false;
        coveredRequests |= route->column_;

        if (usedVehicles.test(route->vehicleID_))
            return false;
        usedVehicles.set(route->vehicleID_);
    }

    return true;
}

bool CPModeler::isColumnDisjointBit(const vector<PRequest> &zResults, const vector<PRoute> &routeResults, const PInstance &pInst) {
    std::vector<boost::dynamic_bitset<>> Columns;
    boost::dynamic_bitset<> unions(pInst->nbRequests_);
    boost::dynamic_bitset<> vehicles(pInst->nbVehicles_);
    int counts = 0;
    int veh = 0;
    for (auto & requestObj : zResults) {
        unions.set(requestObj->taskIndex_, true);
        counts ++;
    }
    for (auto & routeObj : routeResults) {
        unions |= routeObj->column_;
        counts += static_cast<int>(routeObj->column_.count());
        vehicles.set(routeObj->vehicleID_, true);
        veh ++;
    }

    if (unions.count() != counts || vehicles.count() != veh)
        return false;
    return true;  // Sets are disjoint
}

bool CPModeler::isColumnDisjoint(const std::vector<PRequest> &zResults, const std::vector<PRoute> &routeResults,
    const PInstance &pInst) {
    boost::dynamic_bitset<> requestUnion(pInst->nbRequests_);

    // Process individual requests
    for (const auto& request : zResults) {
        const size_t taskIndex = request->taskIndex_;
        if (requestUnion.test(taskIndex)) {
            return false; // Early exit on conflict
        }
        requestUnion.set(taskIndex);
    }

    // Process routes - BLAZING FAST with precomputed columns
    for (const auto& route : routeResults) {
        if ((requestUnion & route->column_).any()) {
            return false; // Early exit on conflict
        }
        requestUnion |= route->column_;
    }

    return true; // No conflicts found
}

void CPModeler::dump_gurobi(){
    // Make sure all pending changes are applied
    model_->update();

    // Human-readable (easy to inspect), and precise/stable MPS
    model_->write("gurobi_orig.lp");
    model_->write("gurobi_orig.mps");

    // Presolved snapshot (without optimizing)
    GRBModel pre = model_->presolve();         // returns a model object
    pre.write("gurobi_presolved.lp");

    // Save the *changed* parameter settings in PRM format
    model_->getEnv().writeParams("gurobi_.prm");
}