//
// Created by Elahe Amiri on 2025-07-13.
//

#include "CP_Reduced.h"

void CP_Reduced::addRouteVar(const PRoute &newRoute, const PRoute &currentVehicleRoute) {
    try {
        GRBColumn col = createCPColumnM1(newRoute, currentVehicleRoute);
        if (newRoute->IncScore_ > 0) {
            double objCoeff = newRoute->totalDelay_ - currentVehicleRoute->totalDelay_;

            // Create variable
            GRBVar var = model_->addVar(0.0, GRB_INFINITY, objCoeff, GRB_CONTINUOUS, col, nullptr);
            routeIncVar_.push_back(var);
            IncRoute_.push_back(newRoute);
            newRoute->cpAdded_ = true;
        }
    }
    catch (GRBException& e) {
        std::cerr << "Error in addRouteVar: " << e.getMessage() << std::endl;
        throw;
    }
}

void CP_Reduced::addRouteVarBatch(const PInstance &pInst, bool greedyBase) {
    const size_t numRoutes = routesToAdd_.size();

    std::vector<double> lb(numRoutes, 0.0);
    std::vector<double> ub(numRoutes, GRB_INFINITY);
    std::vector<double> obj(numRoutes);
    std::vector<char> vtype(numRoutes, GRB_CONTINUOUS);
    std::vector<GRBColumn> columns(numRoutes);

    IncRoute_.reserve(IncRoute_.size() + numRoutes);

    // Fill arrays with range-based loop
    size_t k = 0;
    for (const auto& routeObj : routesToAdd_) {
        if (greedyBase) {
            obj[k] = routeObj->totalDelay_ - pInst->vehicles_[routeObj->vehicleID_]->greedyRoute_->totalDelay_;
            columns[k] = createCPColumnM1(routeObj, pInst->vehicles_[routeObj->vehicleID_]->greedyRoute_);
        }
        else {
            obj[k] = routeObj->totalDelay_ - pInst->vehicles_[routeObj->vehicleID_]->currentRoute_->totalDelay_;
            columns[k] = createCPColumnM1(routeObj, pInst->vehicles_[routeObj->vehicleID_]->currentRoute_);
        }
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

void CP_Reduced::addZVar(const PRequest &request) {
    try {
        GRBColumn col;
        double objCoeff;
        objCoeff = request->penalty_;
        col.addTerm(1.0, requestConstr_[request->taskIndex_]);
        col.addTerm(1.0, normalConst_);

        // Create variable
        GRBVar var = model_->addVar(0.0, GRB_INFINITY, objCoeff, GRB_CONTINUOUS, col, request->name_);
        zIncVar_.push_back(var);
    }
    catch (GRBException& e) {
        std::cerr << "Error in addRouteVar: " << e.getMessage() << std::endl;
        throw;
    }
}

void CP_Reduced::addZVarBatch(const PInstance& pInst) {
    const int n = pInst->nbRequests_;
    if (n == 0) return;

    std::vector<double> lb(n, 0.0);
    std::vector<double> ub(n, GRB_INFINITY);
    std::vector<double> obj(n);
    std::vector<char>    vtype(n, GRB_CONTINUOUS);
    std::vector<GRBColumn> cols(n);
 //   std::vector<std::string> names(n);

    for (int i = 0; i < n; ++i) {
        if (pInst->requests_[i]->committedPickTime_ == LARGE_CONSTANT) {
            const auto& req = pInst->requests_[i];
            obj[i]   = req->penalty_;
            cols[i].addTerm(1.0, requestConstr_[req->taskIndex_]);
            cols[i].addTerm(1.0, normalConst_);
            //       names[i] = req->name_;
        }
    }

    // addVars has an overload that accepts names; if your Gurobi version doesn’t,
    // call without names and set them afterward.
    std::unique_ptr<GRBVar[]> zvars(model_->addVars(
        lb.data(), ub.data(), obj.data(), vtype.data(),
        nullptr,
        cols.data(), n));

    zIncVar_.insert(zIncVar_.end(), zvars.get(), zvars.get() + n);
}

GRBColumn CP_Reduced::createCPColumnM1(const PRoute &newRoute, const PRoute &currentVehicleRoute) {
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

void CP_Reduced::resetForNextIteration() {
    try {

        for (auto &routeObj: IncRoute_)
            routeObj->cpAdded_ = false;
        IncRoute_.clear();
        fractionalRoutes_.clear();
        fractionalZ_.clear();

        // Get all variables and remove them
        // Remove existing solution variables
        beginBatchUpdate();

        for (auto& var : routeIncVar_) {
            model_->remove(var);
        }
        routeIncVar_.clear();

        for (auto& var : zIncVar_) {
            model_->remove(var);
        }
        zIncVar_.clear();


        // Remove old normalization constraint
        model_->remove(normalConst_);

        // Add new empty normalization constraint
        GRBLinExpr normalExpr = 0;
        normalConst_ = model_->addConstr(normalExpr == 1.0, "normalConst");

        // Update model once after all removals
        endBatchUpdate();

    } catch (const GRBException& e) {
        std::cerr << "Gurobi error in resetForNextIteration: " << e.getMessage() << std::endl;
        throw;
    }
}

void CP_Reduced::initializeCPModel(const PInstance &pInst) {
    try {
        int rhs = 0;
        nbRequestTask_ = pInst->nbTasks_;

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
        model_->set(GRB_IntParam_Method, 2);
        model_->set(GRB_IntParam_Crossover, 0);

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

void CP_Reduced::buildModel(const PInstance &pInst, bool greedyBase) {
    try {
        beginBatchUpdate();

        // Adding incompatible route columns
        for (auto& routeAdd : routesToAdd_) {
            if (greedyBase)
                addRouteVar(routeAdd, pInst->vehicles_[routeAdd->vehicleID_]->greedyRoute_);
            else
                addRouteVar(routeAdd, pInst->vehicles_[routeAdd->vehicleID_]->currentRoute_);
        }

        // Adding z columns out of the basis
        for (int i = 0; i < pInst->nbRequests_; ++i) {
            addZVar(pInst->requests_[i]);
        }

        endBatchUpdate();
    }
    catch (GRBException& e) {
        std::cerr << "Error in updateModel: " << e.getMessage() << std::endl;
        throw;
    }
}

void CP_Reduced::buildModel_batch(const PInstance &pInst, bool greedyBase) {
    try {
        if (routesToAdd_.empty()) return;

        // Validate model before proceeding
        if (!model_) {
            throw std::runtime_error("Model is not initialized");
        }

        beginBatchUpdate();
        // Adding incompatible route columns
        addRouteVarBatch(pInst, greedyBase);

        // Adding z columns out of the basis
        addZVarBatch(pInst);

        // Single update call
        endBatchUpdate();

    } catch (const GRBException& e) {
        std::cerr << "Error in updateModel: " << e.getMessage() << std::endl;
        throw;
    } catch (const std::exception& e) {
        std::cerr << "Error in updateModel: " << e.what() << std::endl;
        throw;
    }
}

void CP_Reduced::updateNormalCoefficients() {
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

void CP_Reduced::solveCPModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
    InputPaths &inputPaths) {
    try {
        // Set up logging
        solveTime_->start();
        int status = solve();
        solveTime_->stop();

        if (status != GRB_OPTIMAL) {
            status_ = INFEASIBLE;
            std::cout << "Failed to optimize the problem" << std::endl;
        }
        else {
            // Get dual values
            for (size_t i = 0; i < requestConstr_.size(); ++i) {
                pInst->requests_[i]->dual_ = requestConstr_[i].get(GRB_DoubleAttr_Pi);
            }
            for (size_t i = 0; i < pInst->vehicles_.size(); ++i) {
                pInst->vehicles_[i]->dual_ = 0;
            }

            // Check objective value
            double objVal = getObjValue();

            if (objVal < -0.1) {
                status_ = NEGATIVE_VALUE;

                // Collect incoming and outgoing variables
                std::vector<PRoute> routeResult;
                std::vector<int> InRouteVar;

                // Determine incoming variables
                for (size_t r = 0; r < routeIncVar_.size(); ++r) {
                    if (getVarValue(routeIncVar_[r]) > 0) {
                        routeResult.push_back(IncRoute_[r]);
                        InRouteVar.push_back(static_cast<int>(r));
                    }
                }

                // Check if solution is column disjoint
                if (isColumnDisjoint(routeResult, pInst->nbRequests_, pInst->nbVehicles_)) {
                    status_ = NEGATIVE_VALUE;

                    for (auto& routeObj : routeResult) {
                        pInst->vehicles_[routeObj->vehicleID_]->currentRoute_ = routeObj;
                    }

                    endBatchUpdate();
                    routeSolution.clear();
                    for (auto & vehicleObj: pInst->vehicles_) {
                        routeSolution.push_back(vehicleObj->currentRoute_);
                    }

                } else {
                    status_ = FRACTIONAL;
                }
            } else {
                status_ = POSITIVE_VALUE;
            }
        }
    }
    catch (GRBException& e) {
        std::cerr << "Error in solveCPModel: " << e.getMessage() << std::endl;
        throw;
    }
}

void CP_Reduced::solveCPDual(PInstance &pInst, InputPaths& inputPaths) {
    try {
        // Set up logging
        solveTime_->start();
        int status = solve();
        solveTime_->stop();

        if (status != GRB_OPTIMAL) {
            status_ = INFEASIBLE;
            std::cout << "Failed to optimize the problem" << std::endl;
        }
        else {
            // Get dual values
            for (size_t i = 0; i < requestConstr_.size(); ++i) {
                pInst->requests_[i]->dual_ = requestConstr_[i].get(GRB_DoubleAttr_Pi);
            }
            for (size_t i = 0; i < pInst->vehicles_.size(); ++i) {
                pInst->vehicles_[i]->dual_ = 0;
            }
        }
    }
    catch (GRBException& e) {
        std::cerr << "Error in solveCPModel: " << e.getMessage() << std::endl;
        throw;
    }
}

bool CP_Reduced::isColumnDisjoint(const std::vector<PRoute> &routeResults, int nbRequests, int nbVehicles) {
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
