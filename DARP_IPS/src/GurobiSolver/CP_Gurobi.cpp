//
// Created by Elahe Amiri on 2025-06-26.
//

#include "CP_Gurobi.h"
#include <unordered_set>

// Constructor
CP_Gurobi::CP_Gurobi(std::string outputLog): GurobiModeler(outputLog) {
}

void CP_Gurobi::resetForNextIteration() {
    try {
        for (auto &routeObj: IncRoute_)
            routeObj->cpAdded_ = false;
        IncRoute_.clear();
        // Get all variables and remove them
        // Remove existing solution variables
        beginBatchUpdate();

        for (auto& var : routeSolVar_) {
            model_->remove(var);
        }
        routeSolVar_.clear();

        for (auto& var : zSolVar_) {
            model_->remove(var);
        }
        zSolVar_.clear();

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

// Initialize the model and define an empty set of constraints
void CP_Gurobi::initializeCPModel(const PInstance& pInst, int nbVehicles) {
    try {
        int rhs = 0;
        initializeModel(pInst, rhs, nbVehicles);

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

// Add z variable to the model
void CP_Gurobi::addZVar(const PRequest& request, VarSign sign) {
    try {
        int signMultiplier = (sign == POSITIVE) ? 1 : -1;
        double objCoeff = signMultiplier * request->penalty_;

        // Build column coefficient expression
        GRBColumn col;

        // Add to request constraint
        col.addTerm(signMultiplier, requestConstr_[request->taskIndex_]);


        if (sign == NEGATIVE) {
            // Create variable
            GRBVar var = model_->addVar(0.0, GRB_INFINITY, objCoeff, GRB_CONTINUOUS,
                                        col, request->name_);
            zSolVar_.push_back(var);
        }
        else if (sign == POSITIVE) {

            // Add to normalization constraint
            col.addTerm(1.0, normalConst_);

            // Create variable
            GRBVar var = model_->addVar(0.0, GRB_INFINITY, objCoeff, GRB_CONTINUOUS,
                                        col, request->name_);
            zIncVar_.push_back(var);
        }
    }
    catch (GRBException& e) {
        std::cerr << "Error in addZVar: " << e.getMessage() << std::endl;
        throw;
    }
}

// Add route variable to the model
void CP_Gurobi::addRouteVar(const PRoute& newRoute, VarSign sign, const PInstance& pInst) {
    try {
        int signMultiplier = (sign == POSITIVE) ? 1 : -1;
        GRBColumn col = createColumn(newRoute, sign, pInst);

        double objCoeff = signMultiplier * newRoute->totalDelay_;

        if (sign == NEGATIVE) {
            // Call parent class method for negative sign
            // Create variable
            GRBVar var = model_->addVar(0.0, GRB_INFINITY, objCoeff, GRB_CONTINUOUS, col, nullptr);
            routeSolVar_.push_back(var);
        }
        else {

            // Add to normalization constraint
            col.addTerm(newRoute->incompatibilityDegree_, normalConst_);

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

// Build the model at each iteration
void CP_Gurobi::buildModel(const PInstance& pInst, const std::vector<PRequest>& zSolution,
                               const std::vector<PRoute>& routeSolution, int nbVehicles) {
    try {
        // Model initialization
//        initializeCPModel(pInst, nbVehicles);

        beginBatchUpdate();

        // Adding solution route columns
        for (auto& routeObj : routeSolution) {
            if (pInst->vehicles_[routeObj->vehicleID_]->vehicleIndex_ > -1) {
                addRouteVar(routeObj, NEGATIVE, pInst);
                routeObj->cpAdded_ = true;
            }
        }

        // Adding solution z columns
        for (auto& zSol : zSolution) {
            addZVar(zSol, NEGATIVE);
        }

        // Adding z columns out of the basis
        for (int i = 0; i < pInst->nbRequests_; ++i) {
            if (pInst->requests_[i]->solVehicleID_ < LARGE_CONSTANT &&
                pInst->requests_[i]->committedPickTime_ == LARGE_CONSTANT) {
                addZVar(pInst->requests_[i], POSITIVE);
            }
        }

        // Adding empty routes for vehicles
        for (int v = 0; v < pInst->nbVehicles_; ++v) {
            if (pInst->vehicles_[v]->vehicleIndex_ > -1 &&
                !pInst->vehicles_[v]->emptyRoute_->cpAdded_ &&
                !pInst->vehicles_[v]->currentRoute_->routeRequests_.empty()) {
                addRouteVar(pInst->vehicles_[v]->emptyRoute_, POSITIVE, pInst);
                pInst->vehicles_[v]->emptyRoute_->cpAdded_ = true;
            }
        }

        endBatchUpdate();
    }
    catch (GRBException& e) {
        std::cerr << "Error in buildModel: " << e.getMessage() << std::endl;
        throw;
    }
}

// Repair model
void CP_Gurobi::repairModel(const PInstance& pInst, const std::vector<PRequest>& zSolution,
                                const std::vector<PRoute>& routeSolution) {
    try {
        // Remove existing solution variables
        beginBatchUpdate();

        for (auto& var : routeSolVar_) {
            model_->remove(var);
        }
        routeSolVar_.clear();

        for (auto& var : zSolVar_) {
            model_->remove(var);
        }
        zSolVar_.clear();

        endBatchUpdate();

        // Add new solution variables
        beginBatchUpdate();

        // Adding solution route columns
        for (auto& routeObj : routeSolution) {
            addRouteVar(routeObj, NEGATIVE, pInst);
            routeObj->cpAdded_ = true;
        }

        // Adding solution z columns
        for (auto& zSol : zSolution) {
            addZVar(zSol, NEGATIVE);
        }

        endBatchUpdate();
    }
    catch (GRBException& e) {
        std::cerr << "Error in repairModel: " << e.getMessage() << std::endl;
        throw;
    }
}

// Update the model
void CP_Gurobi::updateModel(const PInstance& pInst) {
    try {
        beginBatchUpdate();

        // Adding incompatible route columns
        for (auto& routeAdd : routesToAdd_) {
            addRouteVar(routeAdd, POSITIVE, pInst);
            routeAdd->cpAdded_ = true;
        }

        endBatchUpdate();
    }
    catch (GRBException& e) {
        std::cerr << "Error in updateModel: " << e.getMessage() << std::endl;
        throw;
    }
}

void CP_Gurobi::updateNormalCoefficients() {
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

void CP_Gurobi::updateModel_Batch(const PInstance& pInst) {
    try {
        if (routesToAdd_.empty()) return;

        // Validate model before proceeding
        if (!model_) {
            throw std::runtime_error("Model is not initialized");
        }

        beginBatchUpdate();
  //      updateNormalCoefficients();

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
            obj[k] = routeObj->totalDelay_;
            columns[k] = createColumn(routeObj, POSITIVE, pInst);
            columns[k].addTerm(routeObj->incompatibilityDegree_, normalConst_);
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


// Solve the model
void CP_Gurobi::solveCPModel(PInstance& pInst, std::vector<PRequest>& zSolution,
                                 std::vector<PRoute>& routeSolution, InputPaths& inputPaths) {
    try {
        // Set up logging
        myTools::CoutRedirector redirector(inputPaths.getOutputSolverLog(), "CP");

        solveTime_->start();
        int status = solve();
        solveTime_->stop();

 //       env_.set(GRB_IntParam_OutputFlag, 0);

        if (status != GRB_OPTIMAL) {
            status_ = INFEASIBLE;
            std::cout << "Failed to optimize the problem" << std::endl;
        }
        else {
            // Get dual values
            getDuals(pInst);

            // Check objective value
            double objVal = getObjValue();

            if (objVal < -0.01) {
                status_ = NEGATIVE_VALUE;

                // Collect incoming and outgoing variables
                std::vector<PRequest> zResult;
                std::vector<PRoute> routeResult;
                std::vector<int> InRouteVar;
                std::vector<int> OutRouteVar;
                std::vector<int> InRequestVar;
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
                if (isColumnDisjointFast(zResult, routeResult, pInst)) {
                    beginBatchUpdate();
                    // Process outgoing variables (in reverse order to avoid index issues)
                    for (int idx = OutRouteVar.size() - 1; idx >= 0; --idx) {
                        int r = OutRouteVar[idx];
                        addRouteVar(routeSolution[r], POSITIVE, pInst);
                        model_->remove(routeSolVar_[r]);
                        routeSolVar_.erase(routeSolVar_.begin() + r);
                        routeSolution.erase(routeSolution.begin() + r);
                    }

                    for (int idx = OutRequestVar.size() - 1; idx >= 0; --idx) {
                        int i = OutRequestVar[idx];
                        addZVar(zSolution[i], POSITIVE);
                        model_->remove(zSolVar_[i]);
                        zSolVar_.erase(zSolVar_.begin() + i);
                        zSolution.erase(zSolution.begin() + i);
                    }

                    // Process incoming variables (in reverse order)
                    for (int idx = InRouteVar.size() - 1; idx >= 0; --idx) {
                        int r = InRouteVar[idx];
                        routeSolution.push_back(IncRoute_[r]);
                        addRouteVar(routeSolution.back(), NEGATIVE, pInst);
                        model_->remove(routeIncVar_[r]);
                        routeIncVar_.erase(routeIncVar_.begin() + r);
                        IncRoute_.erase(IncRoute_.begin() + r);
                    }

                    for (int idx = InRequestVar.size() - 1; idx >= 0; --idx) {
                        int i = InRequestVar[idx];
                        zSolution.push_back(pInst->nameToRequest_[zIncVar_[i].get(GRB_StringAttr_VarName)]);
                        addZVar(zSolution.back(), NEGATIVE);
                        model_->remove(zIncVar_[i]);
                        zIncVar_.erase(zIncVar_.begin() + i);
                    }
                    endBatchUpdate();

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
    }
    catch (GRBException& e) {
        std::cerr << "Error in solveCPModel: " << e.getMessage() << std::endl;
        throw;
    }
}

void CP_Gurobi::solveCPModelFresh(PInstance& pInst, std::vector<PRequest>& zSolution,
                                 std::vector<PRoute>& routeSolution, InputPaths& inputPaths) {
    try {
        // Set up logging
        myTools::CoutRedirector redirector(inputPaths.getOutputSolverLog(), "CP");

 //       env_.set(GRB_IntParam_OutputFlag, 1);

        solveTime_->start();
        int status = solve();
        solveTime_->stop();

 //       env_.set(GRB_IntParam_OutputFlag, 0);

        if (status != GRB_OPTIMAL) {
            status_ = INFEASIBLE;
            std::cout << "Failed to optimize the problem" << std::endl;
        }
        else {
            // Get dual values
            getDuals(pInst);

            // Check objective value
            double objVal = getObjValue();

            if (objVal < -0.01) {
                status_ = NEGATIVE_VALUE;

                // Collect incoming and outgoing variables
                std::vector<PRequest> zResult;
                std::vector<PRoute> routeResult;
                std::vector<int> InRouteVar;
                std::vector<int> OutRouteVar;
                std::vector<int> InRequestVar;
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
                        std::string varName = zIncVar_[i].get(GRB_StringAttr_VarName);
                        auto reqIt = pInst->nameToRequest_.find(varName);
                        if (reqIt != pInst->nameToRequest_.end()) {
                            zResult.push_back(reqIt->second);
                            InRequestVar.push_back(static_cast<int>(i));
                        }
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

                // Check if the solution is column disjoint
                if (isColumnDisjointFast(zResult, routeResult, pInst)) {
                    // Process outgoing variables (in reverse order to avoid index issues)
                    for (int idx = OutRouteVar.size() - 1; idx >= 0; --idx) {
                        int r = OutRouteVar[idx];
                        routeSolution.erase(routeSolution.begin() + r);
                    }

                    for (int idx = OutRequestVar.size() - 1; idx >= 0; --idx) {
                        int i = OutRequestVar[idx];
                        zSolution.erase(zSolution.begin() + i);
                    }

                    // Process incoming variables (in reverse order)
                    for (int idx = InRouteVar.size() - 1; idx >= 0; --idx) {
                        int r = InRouteVar[idx];
                        routeSolution.push_back(IncRoute_[r]);
                    }

                    for (int idx = InRequestVar.size() - 1; idx >= 0; --idx) {
                        int i = InRequestVar[idx];
                        std::string varName = zIncVar_[i].get(GRB_StringAttr_VarName);
                        auto reqIt = pInst->nameToRequest_.find(varName);
                        if (reqIt != pInst->nameToRequest_.end()) {
                            zSolution.push_back(reqIt->second);
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
    }
    catch (GRBException& e) {
        std::cerr << "Error in solveCPModel: " << e.getMessage() << std::endl;
        throw;
    }
}


bool CP_Gurobi::isColumnDisjointFast(const std::vector<PRequest>& zResults,
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

bool CP_Gurobi::isColumnDisjointBit(const vector<PRequest> &zResults, const vector<PRoute> &routeResults, const PInstance &pInst) {
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

bool CP_Gurobi::isColumnDisjoint(const std::vector<PRequest> &zResults, const std::vector<PRoute> &routeResults,
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

// Display function
std::string CP_Gurobi::toString() const {
    std::stringstream repStr;
    repStr << std::endl;
    repStr << "# ====================  COMPLEMENTARY PROBLEM SOLVED  ==================== " << std::endl;
    repStr << "# COMPLEMENTARY PROBLEM SOLVED: " << std::endl;
    repStr << GurobiModeler::toString();
    return repStr.str();
}
