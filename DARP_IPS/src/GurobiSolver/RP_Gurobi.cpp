//
// Created by Elahe Amiri on 2025-06-25.
//

#include "RP_Gurobi.h"

RP_Gurobi::RP_Gurobi(std::string outputLog) : GurobiModeler(outputLog) {
    compRoutes_.clear();
    auxObjValue_ = 0;
    objValue_ = 0;
}


// Add route variable (integer version)
void RP_Gurobi::addRouteVar(PRoute &newRoute, PInstance &pInst) {
    try {
        addRouteVarInt(newRoute, POSITIVE, pInst);
        compRoutes_.push_back(newRoute);
        newRoute->mpAdded_ = true;
    }
    catch (GRBException& e) {
        std::cerr << "Error in addRouteVar: " << e.getMessage() << std::endl;
        throw;
    }
}

// Add route variable (float version)
void RP_Gurobi::addRouteVarFloat(PRoute &newRoute, PInstance &pInst) {
    try {
        GurobiModeler::addRouteVarFloat(newRoute, POSITIVE, pInst);
        compRoutes_.push_back(newRoute);
        newRoute->mpAdded_ = true;
    }
    catch (GRBException& e) {
        std::cerr << "Error in addRouteVarFloat: " << e.getMessage() << std::endl;
        throw;
    }
}

void RP_Gurobi::addZVar(PRequest &request) {
    try {
        addZVarInt(request, POSITIVE);
        // Track the index for later retrieval
        requestNameToIndex_[request->name_] = zVar_.size() - 1;
    }
    catch (GRBException& e) {
        std::cerr << "Error in addZVar: " << e.getMessage() << std::endl;
        throw;
    }
}

// Update model with new routes
void RP_Gurobi::updateRPModel(PInstance &pInst) {
    try {
        // Use batch mode for efficiency
        beginBatchUpdate();

        // Add the new compatible columns to the model
        for (auto& routeObj : routesToAdd_) {
            if (pInst->vehicles_[routeObj->vehicleID_]->vehicleIndex_ > -1) {
                addRouteVar(routeObj, pInst);
            }
        }

        endBatchUpdate();
    }
    catch (GRBException& e) {
        std::cerr << "Error in updateRPModel: " << e.getMessage() << std::endl;
        throw;
    }
}

void RP_Gurobi::buildModelRP(PInstance &pInst, std::vector<PRoute> &routeSolution, int nbVehicles) {
    try {
        // Model initialization
        int rhs = 1;
        initializeModel(pInst, rhs, nbVehicles);

        // Use batch mode for efficiency
        beginBatchUpdate();

        // Adding request columns (z variables)
        for (auto& zSol : pInst->requests_) {
            if (zSol->committedPickTime_ == LARGE_CONSTANT) {
                addZVar(zSol);
                requestNameToIndex_[zSol->name_] = zVar_.size() - 1;
            }
        }

        // Adding route solution columns
        for (auto& routeSol : routeSolution) {
            if (pInst->vehicles_[routeSol->vehicleID_]->vehicleIndex_ > -1) {
                addRouteVar(routeSol, pInst);
            }
        }

        // Adding new route variables
        for (auto& routeObj : routesToAdd_) {
            if (pInst->vehicles_[routeObj->vehicleID_]->vehicleIndex_ > -1) {
                addRouteVar(routeObj, pInst);
            }
        }

        endBatchUpdate();
    }
    catch (GRBException& e) {
        std::cerr << "Error in buildModel: " << e.getMessage() << std::endl;
        throw;
    }
}

// Solve as LP
void RP_Gurobi::solveModelLP(const PInstance &pInst, const InputPaths &inputPaths) {
    try {
        // Redirect output to log file
        // Configure Gurobi logging
        myTools::CoutRedirector redirector(inputPaths.getOutputSolverLog(), "MP");

        model_->update();

        solveTime_->start();
        int status = solve();
        solveTime_->stop();

        if (status != GRB_OPTIMAL && status != GRB_SUBOPTIMAL) {
            std::cerr << "Failed to optimize the LMP. Status: " << status << std::endl;
            throw std::runtime_error("Failed to optimize the LMP");
        }
        else {
            objValue_ = static_cast<float>(getObjValue());

            // Getting dual values
            getDuals();
            const auto& requestDuals = getRequestDuals();
            const auto& vehicleDuals = getVehicleDuals();

            // Update request duals
            for (auto& requestObj : pInst->requests_) {
                requestObj->dual_ = static_cast<float>(requestDuals[requestObj->taskIndex_]);
                requestObj->InitialDual_ = requestObj->dual_;
            }

            // Update vehicle duals
            for (auto& vehicleObj : pInst->vehicles_) {
                int index = vehicleObj->vehicleIndex_;
                if (index > -1) {
                    vehicleObj->dual_ = static_cast<float>(vehicleDuals[index]);
                    vehicleObj->InitialDual_ = vehicleObj->dual_;
                }
                else {
                    vehicleObj->dual_ = 0;
                    vehicleObj->InitialDual_ = 0;
                }
            }
        }
    }
    catch (GRBException& e) {
        std::cerr << "Error occurred in solveModelLP at line: " << __LINE__ << std::endl;
        std::cerr << "GRB Error: " << e.getMessage() << std::endl;
        throw;
    }
}

void RP_Gurobi::solveModelInt(const PInstance &pInst, std::vector<PRequest> &zSolution,
    std::vector<PRoute> &routeSolution, const InputPaths &inputPaths, float availableTime, float preObj) {
    try {
        // Convert variables to integer
        beginBatchUpdate();

        // Convert z variables to integer
        for (auto& var : zVar_) {
            var.set(GRB_CharAttr_VType, GRB_INTEGER);
        }

        // Convert route variables to integer
        for (auto& var : routeVar_) {
            var.set(GRB_CharAttr_VType, GRB_INTEGER);
        }

        // Redirect output to log file
        myTools::CoutRedirector redirector(inputPaths.getOutputSolverLog(), "LMP");

        // Set time limit
        model_->set(GRB_DoubleParam_TimeLimit, availableTime);

        endBatchUpdate();

        solveTime_->start();
        int status = solve();
        solveTime_->stop();

        if (status != GRB_OPTIMAL && status != GRB_SUBOPTIMAL && status != GRB_TIME_LIMIT) {
            throw std::runtime_error("Failed to optimize the MP");
        }
        else {
            double currentObj = getObjValue();

            if (currentObj >= preObj) {
                // Solution is not better, convert back to continuous
                beginBatchUpdate();

                for (auto& var : zVar_) {
                    var.set(GRB_CharAttr_VType, GRB_CONTINUOUS);
                }

                for (auto& var : routeVar_) {
                    var.set(GRB_CharAttr_VType, GRB_CONTINUOUS);
                }

                endBatchUpdate();
            }
            else {
                objValue_ = static_cast<float>(currentObj);

                // Saving the result
                zSolution.clear();
                routeSolution.clear();
                routeSolutionIndex_.clear();

                // Get route variable values
                for (size_t r = 0; r < routeVar_.size(); ++r) {
                    double val = getVarValue(routeVar_[r]);
                    if (val > 0.5) {
                        routeSolution.push_back(compRoutes_[r]);
                        routeSolutionIndex_.push_back(static_cast<int>(r));
                    }
                }

                // Get z variable values
                for (size_t i = 0; i < zVar_.size(); ++i) {
                    double val = getVarValue(zVar_[i]);
                    if (val > 0.5) {
                        // Find the request by name
                        std::string varName = zVar_[i].get(GRB_StringAttr_VarName);
                        auto reqIt = pInst->nameToRequest_.find(varName);
                        if (reqIt != pInst->nameToRequest_.end()) {
                            zSolution.push_back(reqIt->second);
                        }
                    }
                }

                // Convert back to continuous for next iteration
                beginBatchUpdate();

                for (auto& var : zVar_) {
                    var.set(GRB_CharAttr_VType, GRB_CONTINUOUS);
                }

                for (auto& var : routeVar_) {
                    var.set(GRB_CharAttr_VType, GRB_CONTINUOUS);
                }

                endBatchUpdate();
            }
        }
    }
    catch (GRBException& e) {
        std::cerr << "Error occurred in solveModelInt at line: " << __LINE__ << std::endl;
        std::cerr << "GRB Error: " << e.getMessage() << std::endl;
        throw;
    }
}

// Solve LP then convert to MIP
void RP_Gurobi::solveModelLPInt(const PInstance& pInst, std::vector<PRequest>& zSolution,
                                     std::vector<PRoute>& routeSolution, const InputPaths& inputPaths,
                                     float availableTime, float preObj) {
    try {
        // Configure Gurobi logging
        myTools::CoutRedirector redirector(inputPaths.getOutputSolverLog(), "MP");

        solveTime_->start();
        int status = solve();
        solveTime_->stop();

        if (status != GRB_OPTIMAL && status != GRB_SUBOPTIMAL) {
            std::cerr << "Failed to optimize the LMP" << std::endl;
            throw std::runtime_error("Failed to optimize the LMP");
        }
        else {
            objValue_ = static_cast<float>(getObjValue());

            // Getting dual values
            getDuals();
            const auto& requestDuals = getRequestDuals();
            const auto& vehicleDuals = getVehicleDuals();

            // Update request duals
            for (auto& requestObj : pInst->requests_) {
                requestObj->dual_ = static_cast<float>(requestDuals[requestObj->taskIndex_]);
                requestObj->InitialDual_ = requestObj->dual_;
            }

            // Update vehicle duals
            for (auto& vehicleObj : pInst->vehicles_) {
                int index = vehicleObj->vehicleIndex_;
                if (index > -1) {
                    vehicleObj->dual_ = static_cast<float>(vehicleDuals[index]);
                    vehicleObj->InitialDual_ = vehicleObj->dual_;
                }
                else {
                    vehicleObj->dual_ = 0;
                    vehicleObj->InitialDual_ = 0;
                }
            }

            // Convert to integer
            std::cout << "----------------------- MP ------------------------" << std::endl;

            beginBatchUpdate();

            // Convert z variables to integer
            for (auto& var : zVar_) {
                var.set(GRB_CharAttr_VType, GRB_INTEGER);
            }

            // Convert route variables to integer
            for (auto& var : routeVar_) {
                var.set(GRB_CharAttr_VType, GRB_INTEGER);
            }

            endBatchUpdate();

            // Set time limit
            model_->set(GRB_DoubleParam_TimeLimit, availableTime);

            solveTime_->start();
            status = solve();
            solveTime_->stop();

            if (status != GRB_OPTIMAL && status != GRB_SUBOPTIMAL && status != GRB_TIME_LIMIT) {
                throw std::runtime_error("Failed to optimize the MP");
            }
            else {
                double currentObj = getObjValue();

                if (currentObj >= preObj) {
                    // Solution is not better
                    beginBatchUpdate();

                    // Convert back to continuous
                    for (auto& var : zVar_) {
                        var.set(GRB_CharAttr_VType, GRB_CONTINUOUS);
                    }

                    for (auto& var : routeVar_) {
                        var.set(GRB_CharAttr_VType, GRB_CONTINUOUS);
                    }

                    endBatchUpdate();
                }
                else {
                    objValue_ = static_cast<float>(currentObj);

                    // Saving the result
                    zSolution.clear();
                    routeSolution.clear();
                    routeSolutionIndex_.clear();

                    // Get route variable values
                    for (size_t r = 0; r < routeVar_.size(); ++r) {
                        double val = getVarValue(routeVar_[r]);
                        if (val > 0.9) {
                            routeSolution.push_back(compRoutes_[r]);
                            routeSolutionIndex_.push_back(static_cast<int>(r));
                        }
                    }

                    // Get z variable values
                    for (size_t i = 0; i < zVar_.size(); ++i) {
                        double val = getVarValue(zVar_[i]);
                        if (val > 0.9) {
                            std::string varName = zVar_[i].get(GRB_StringAttr_VarName);
                            auto reqIt = pInst->nameToRequest_.find(varName);
                            if (reqIt != pInst->nameToRequest_.end()) {
                                zSolution.push_back(reqIt->second);
                            }
                        }
                    }

                    // Convert back to continuous for next iteration
                    beginBatchUpdate();

                    for (auto& var : zVar_) {
                        var.set(GRB_CharAttr_VType, GRB_CONTINUOUS);
                    }

                    for (auto& var : routeVar_) {
                        var.set(GRB_CharAttr_VType, GRB_CONTINUOUS);
                    }

                    endBatchUpdate();
                }
            }
        }
    }
    catch (GRBException& e) {
        std::cerr << "Error occurred in solveModelLPInt at line: " << __LINE__ << std::endl;
        std::cerr << "GRB Error: " << e.getMessage() << std::endl;
        throw;
    }
}

void RP_Gurobi::solveModelRelaxInt(const PInstance &pInst, std::vector<PRequest> &zSolution,
    std::vector<PRoute> &routeSolution, const InputPaths &inputPaths, float availableTime, float preObj) {
    try {
        // Configure Gurobi logging
        myTools::CoutRedirector redirector(inputPaths.getOutputSolverLog(), "MP");

        // Set time limit
        model_->set(GRB_DoubleParam_TimeLimit, availableTime);

        std::cout << "----------------------- MP ------------------------" << std::endl;

        // Solve MIP first
        solveTime_->start();
        int status = solve();
        solveTime_->stop();

        if (status != GRB_OPTIMAL && status != GRB_SUBOPTIMAL && status != GRB_TIME_LIMIT) {
            throw std::runtime_error("Failed to optimize the MP");
        }

        double currentObj = getObjValue();

        if (currentObj >= preObj) {
            return;
        }

        // Solution is better, store integer solution
        objValue_ = static_cast<float>(currentObj);

        // Save the integer solution
        zSolution.clear();
        routeSolution.clear();
        routeSolutionIndex_.clear();

        // Get route variable values
        for (size_t r = 0; r < routeVar_.size(); ++r) {
            double val = getVarValue(routeVar_[r]);
            if (val > 0.9) {
                routeSolution.push_back(compRoutes_[r]);
                routeSolutionIndex_.push_back(static_cast<int>(r));
            }
        }

        // Get z variable values
        for (size_t i = 0; i < zVar_.size(); ++i) {
            double val = getVarValue(zVar_[i]);
            if (val > 0.9) {
                std::string varName = zVar_[i].get(GRB_StringAttr_VarName);
                auto reqIt = pInst->nameToRequest_.find(varName);
                if (reqIt != pInst->nameToRequest_.end()) {
                    zSolution.push_back(reqIt->second);
                }
            }
        }

        // Create relaxed model from the solved MIP to get duals from LP relaxation
        GRBModel relaxedModel = model_->relax();

        // Solve the relaxed model (should be very fast due to warm start)
        relaxedModel.optimize();

        if (relaxedModel.get(GRB_IntAttr_Status) != GRB_OPTIMAL) {
            std::cerr << "Failed to solve LP relaxation for dual values" << std::endl;
            throw std::runtime_error("Failed to solve LP relaxation for dual values");
        }

        getDualsFromRelaxed(relaxedModel);

        // Update request duals (assuming same constraint ordering)
        for (auto& requestObj : pInst->requests_) {
            requestObj->dual_ = static_cast<float>(requestDuals_[requestObj->taskIndex_]);
            requestObj->InitialDual_ = requestObj->dual_;
        }

        // Update vehicle duals (assuming same constraint ordering)
        for (auto& vehicleObj : pInst->vehicles_) {
            int index = vehicleObj->vehicleIndex_;
            if (index > -1) {
                vehicleObj->dual_ = static_cast<float>(vehicleDuals_[index]);
                vehicleObj->InitialDual_ = vehicleObj->dual_;
            }
            else {
                vehicleObj->dual_ = 0;
                vehicleObj->InitialDual_ = 0;
            }
        }

    }
    catch (GRBException& e) {
        std::cerr << "Error occurred in solveModelLPInt at line: " << __LINE__ << std::endl;
        std::cerr << "GRB Error: " << e.getMessage() << std::endl;
        throw;
    }
}

// Solve with auxiliary problem
void RP_Gurobi::solveModelIntAux_P(const PInstance& pInst, std::vector<PRequest>& zSolution,
                                        std::vector<PRoute>& routeSolution, const InputPaths& inputPaths,
                                        float availableTime, float preObj, float lpObj) {
    try {
        // Convert to integer
        beginBatchUpdate();

        for (auto& var : zVar_) {
            var.set(GRB_CharAttr_VType, GRB_INTEGER);
        }

        for (auto& var : routeVar_) {
            var.set(GRB_CharAttr_VType, GRB_INTEGER);
        }

        endBatchUpdate();

        // Configure Gurobi logging
        myTools::CoutRedirector redirector(inputPaths.getOutputSolverLog(), "MP");

        // Set time limit
        model_->set(GRB_DoubleParam_TimeLimit, availableTime);

        solveTime_->start();
        int status = solve();
        solveTime_->stop();

        if (status != GRB_OPTIMAL && status != GRB_SUBOPTIMAL && status != GRB_TIME_LIMIT) {
            throw std::runtime_error("Failed to optimize the MP");
        }
        else {
            double currentObj = getObjValue();

            if (currentObj >= preObj) {
                // Solution is not better
                beginBatchUpdate();

                // Convert back to continuous
                for (auto& var : zVar_) {
                    var.set(GRB_CharAttr_VType, GRB_CONTINUOUS);
                }

                for (auto& var : routeVar_) {
                    var.set(GRB_CharAttr_VType, GRB_CONTINUOUS);
                }

                endBatchUpdate();
            }
            else {
                objValue_ = static_cast<float>(currentObj);

                // Store variable values
                std::vector<double> zVals(zVar_.size());
                std::vector<double> routeVals(routeVar_.size());

                for (size_t i = 0; i < zVar_.size(); ++i) {
                    zVals[i] = getVarValue(zVar_[i]);
                }

                for (size_t r = 0; r < routeVar_.size(); ++r) {
                    routeVals[r] = getVarValue(routeVar_[r]);
                }

                // Saving the result
                zSolution.clear();
                routeSolution.clear();

                for (size_t r = 0; r < routeVals.size(); ++r) {
                    if (routeVals[r] > 0.5) {
                        routeSolution.push_back(compRoutes_[r]);
                    }
                }

                for (size_t i = 0; i < zVals.size(); ++i) {
                    if (zVals[i] > 0.5) {
                        std::string varName = zVar_[i].get(GRB_StringAttr_VarName);
                        auto reqIt = pInst->nameToRequest_.find(varName);
                        if (reqIt != pInst->nameToRequest_.end()) {
                            zSolution.push_back(reqIt->second);
                        }
                    }
                }

                auxObjValue_ = 0;

                // If LP objective is better than current integer solution
                if (lpObj < objValue_) {
                    // Fix variables at their current values
                    beginBatchUpdate();

                    for (size_t r = 0; r < routeVar_.size(); ++r) {
                        if (routeVals[r] > 0.5) {
                            routeVar_[r].set(GRB_DoubleAttr_UB, 1.0);
                        } else {
                            routeVar_[r].set(GRB_DoubleAttr_UB, 0.0);
                        }
                    }

                    for (size_t i = 0; i < zVar_.size(); ++i) {
                        if (zVals[i] > 0.5) {
                            zVar_[i].set(GRB_DoubleAttr_UB, 1.0);
                        } else {
                            zVar_[i].set(GRB_DoubleAttr_UB, 0.0);
                        }
                    }

                    // Change to maximization
                    model_->set(GRB_IntAttr_ModelSense, GRB_MAXIMIZE);

                    // Change RHS of constraints to 0
                    auto& requestConstr = const_cast<std::vector<GRBConstr>&>(requestConstr_);
                    auto& vehicleConstr = const_cast<std::vector<GRBConstr>&>(vehicleConstr_);

                    for (auto& constr : requestConstr) {
                        constr.set(GRB_DoubleAttr_RHS, 0.0);
                    }

                    for (auto& constr : vehicleConstr) {
                        constr.set(GRB_DoubleAttr_RHS, 0.0);
                    }

                    endBatchUpdate();

                    // Solve auxiliary problem
                    solveTime_->start();
                    status = solve();
                    solveTime_->stop();

                    if (status != GRB_OPTIMAL && status != GRB_SUBOPTIMAL) {
                        throw std::runtime_error("Failed to optimize the Aux MP");
                    }
                    else {
                        objValue_ = static_cast<float>(getObjValue());

                        // Get new dual values
                        getDuals();
                        const auto& requestDuals = getRequestDuals();
                        const auto& vehicleDuals = getVehicleDuals();

                        for (auto& requestObj : pInst->requests_) {
                            requestObj->dual_ = static_cast<float>(requestDuals[requestObj->taskIndex_]);
                        }

                        for (auto& vehicleObj : pInst->vehicles_) {
                            if (vehicleObj->vehicleIndex_ > -1) {
                                int index = vehicleObj->vehicleIndex_;
                                vehicleObj->dual_ = static_cast<float>(vehicleDuals[index]);
                            } else {
                                vehicleObj->dual_ = 0;
                            }
                        }

                        // Reset changes
                        beginBatchUpdate();

                        // Change back to minimization
                        model_->set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

                        // Reset variable bounds
                        for (auto& var : zVar_) {
                            var.set(GRB_DoubleAttr_LB, 0.0);
                            var.set(GRB_DoubleAttr_UB, GRB_INFINITY);
                            var.set(GRB_CharAttr_VType, GRB_CONTINUOUS);
                        }

                        for (auto& var : routeVar_) {
                            var.set(GRB_DoubleAttr_LB, 0.0);
                            var.set(GRB_DoubleAttr_UB, GRB_INFINITY);
                            var.set(GRB_CharAttr_VType, GRB_CONTINUOUS);
                        }

                        // Reset RHS values
                        for (size_t i = 0; i < requestConstr.size(); ++i) {
                            requestConstr[i].set(GRB_DoubleAttr_RHS, requestRHS_[i]);
                        }

                        for (size_t i = 0; i < vehicleConstr.size(); ++i) {
                            vehicleConstr[i].set(GRB_DoubleAttr_RHS, vehicleRHS_[i]);
                        }

                        endBatchUpdate();
                    }
                }
            }
        }

 //       model_->set(GRB_IntParam_OutputFlag, 0);
    }
    catch (GRBException& e) {
        std::cerr << "Error occurred in solveModelIntAux_P at line: " << __LINE__ << std::endl;
        std::cerr << "GRB Error: " << e.getMessage() << std::endl;
        throw;
    }
}


void RP_Gurobi::tuneModel(const InputPaths &inputPaths, float tuneTimeLimit,
                          float individualSolveTimeLimit, int numTuneResults) {
    try {
        // Save current variable types
        std::vector<char> originalZVTypes, originalRouteVTypes;
        for (const auto& var : zVar_) {
            originalZVTypes.push_back(var.get(GRB_CharAttr_VType));
        }
        for (const auto& var : routeVar_) {
            originalRouteVTypes.push_back(var.get(GRB_CharAttr_VType));
        }

        // Convert variables to integer for tuning (since you solve as integer)
        beginBatchUpdate();
        for (auto& var : zVar_) {
            var.set(GRB_CharAttr_VType, GRB_INTEGER);
        }
        for (auto& var : routeVar_) {
            var.set(GRB_CharAttr_VType, GRB_INTEGER);
        }
        endBatchUpdate();

        // Set tuning parameters
        model_->set(GRB_DoubleParam_TimeLimit, individualSolveTimeLimit);
        model_->set(GRB_DoubleParam_TuneTimeLimit, tuneTimeLimit);
        model_->set(GRB_IntParam_TuneResults, numTuneResults);

        // Set up logging for tuning
        std::string tuneLogPath = inputPaths.getOutputSolverLog() + "_tune.log";
        model_->set(GRB_StringParam_LogFile, tuneLogPath);


        std::cout << "Starting parameter tuning..." << std::endl;
        std::cout << "Tune time limit: " << tuneTimeLimit << " seconds" << std::endl;
        std::cout << "Individual solve time limit: " << individualSolveTimeLimit << " seconds" << std::endl;

        // Perform tuning
        solveTime_->start();
        model_->tune();
        solveTime_->stop();

        // Get number of tuning results
        int resultCount = model_->get(GRB_IntAttr_TuneResultCount);
        std::cout << "Tuning completed. Found " << resultCount << " parameter settings." << std::endl;

        if (resultCount > 0) {
            // Load the best parameter setting (index resultCount-1 is typically the best)
            int bestIndex = std::min(resultCount - 1, numTuneResults - 1);
            model_->getTuneResult(bestIndex);

            // Save the best parameters
            std::string paramFile = inputPaths.getOutputSolverLog() + "_best_params.prm";
            model_->write(paramFile);
            std::cout << "Best parameters saved to: " << paramFile << std::endl;

            // Test the tuned parameters with a solve
            std::cout << "Testing tuned parameters..." << std::endl;
            model_->optimize();

            int status = model_->get(GRB_IntAttr_Status);
            if (status == GRB_OPTIMAL || status == GRB_SUBOPTIMAL || status == GRB_TIME_LIMIT) {
                double tunedObj = model_->get(GRB_DoubleAttr_ObjVal);
                double tunedTime = model_->get(GRB_DoubleAttr_Runtime);
                std::cout << "Tuned solve - Objective: " << tunedObj
                         << ", Time: " << tunedTime << " seconds" << std::endl;
            }

            // Optional: Compare different tuning results
            if (resultCount > 1) {
                std::cout << "\nComparing tuning results:" << std::endl;
                for (int i = 0; i < std::min(resultCount, numTuneResults); ++i) {
                    model_->getTuneResult(i);
                    model_->optimize();

                    int testStatus = model_->get(GRB_IntAttr_Status);
                    if (testStatus == GRB_OPTIMAL || testStatus == GRB_SUBOPTIMAL || testStatus == GRB_TIME_LIMIT) {
                        double obj = model_->get(GRB_DoubleAttr_ObjVal);
                        double time = model_->get(GRB_DoubleAttr_Runtime);
                        std::cout << "Result " << i << " - Objective: " << obj
                                 << ", Time: " << time << " seconds" << std::endl;
                    }
                }

                // Load back the best result
                model_->getTuneResult(bestIndex);
            }
        } else {
            std::cout << "No tuning results found. Using default parameters." << std::endl;
        }

        // Restore original variable types
        beginBatchUpdate();
        for (size_t i = 0; i < zVar_.size(); ++i) {
            zVar_[i].set(GRB_CharAttr_VType, originalZVTypes[i]);
        }
        for (size_t i = 0; i < routeVar_.size(); ++i) {
            routeVar_[i].set(GRB_CharAttr_VType, originalRouteVTypes[i]);
        }
        endBatchUpdate();

        // Reset output settings
  //      model_->set(GRB_IntParam_OutputFlag, 0);

    } catch (GRBException& e) {
        std::cerr << "Error occurred in tuneModel: " << e.getMessage() << std::endl;
    }
}

// Helper function to load previously saved parameters
void RP_Gurobi::loadTunedParameters(const InputPaths &inputPaths) {
    try {
        std::string paramFile = inputPaths.getOutputSolverLog() + "_best_params.prm";
        model_->read(paramFile);
        std::cout << "Loaded tuned parameters from: " << paramFile << std::endl;
    } catch (GRBException& e) {
        std::cout << "Could not load tuned parameters: " << e.getMessage() << std::endl;
        std::cout << "Using default parameters." << std::endl;
    }
}