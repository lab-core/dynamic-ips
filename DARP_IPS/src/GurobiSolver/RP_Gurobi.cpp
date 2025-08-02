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
void RP_Gurobi::addRouteVarFloat_RP(PRoute &newRoute, PInstance &pInst) {
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


void RP_Gurobi::updateModel_batch(PInstance& pInst) {
    try {
        if (routesToAdd_.empty()) return;

        beginBatchUpdate();

        const size_t numRoutes = routesToAdd_.size();

        std::vector<double> lb(numRoutes, 0.0);
        std::vector<double> ub(numRoutes, GRB_INFINITY);
        std::vector<double> obj(numRoutes);
        std::vector<char> vtype(numRoutes, GRB_INTEGER);
        std::vector<GRBColumn> columns(numRoutes);

        compRoutes_.reserve(compRoutes_.size() + numRoutes);   // avoid reallocations

        size_t k = 0;
        // Fill arrays with range-based loop
        for (const auto& routeObj : routesToAdd_) {
            obj[k] = routeObj->totalDelay_;
            columns[k] = createColumn(routeObj, POSITIVE, pInst);
            compRoutes_.emplace_back(routeObj);
            routeObj->mpAdded_ = true;
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
        routeVar_.insert(routeVar_.end(), newVars.get(), newVars.get() + numRoutes);

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

void RP_Gurobi::extractSolution(const PInstance &pInst, std::vector<PRequest> &zSolution,
    std::vector<PRoute> &routeSolution) {
    // Saving the result
    zSolution.clear();
    routeSolution.clear();
    routeSolutionIndex_.clear();

    // Get route variable values
    for (size_t r = 0; r < routeVar_.size(); ++r) {
        if (getVarValue(routeVar_[r]) > 0.5) {
            routeSolution.push_back(compRoutes_[r]);
            routeSolutionIndex_.push_back(static_cast<int>(r));
        }
    }

    // Get z variable values
    for (size_t i = 0; i < zVar_.size(); ++i) {
        double val = getVarValue(zVar_[i]);
        if (val > 0.5) {
            zSolution.push_back(pInst->nameToRequest_[zVar_[i].get(GRB_StringAttr_VarName)]);
        }
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
            getDuals(pInst);

        }
    }
    catch (GRBException& e) {
        std::cerr << "Error occurred in solveModelLP at line: " << __LINE__ << std::endl;
        std::cerr << "GRB Error: " << e.getMessage() << std::endl;
        throw;
    }
}

void RP_Gurobi::solveInteriorLP(const PInstance &pInst, const InputPaths &inputPaths) {
    try {
        // Redirect output to log file
        // Configure Gurobi logging
        myTools::CoutRedirector redirector(inputPaths.getOutputSolverLog(), "MP");
        model_->set(GRB_IntParam_Method, 2);
        model_->set(GRB_IntParam_Crossover, 0);
        model_->update();

        solveTime_->start();
        int status = solve();
        solveTime_->stop();

        if (status != GRB_OPTIMAL && status != GRB_SUBOPTIMAL) {
            std::cerr << "Failed to optimize the LMP. Status: " << status << std::endl;
            throw std::runtime_error("Failed to optimize the LMP");
        }
        // Getting dual values
        getDuals(pInst);
        model_->set(GRB_IntParam_Method, GRB_METHOD_DUAL);
        model_->set(GRB_IntParam_Crossover, 1);
        model_->update();
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
                convertToFloat();
            }
            else {
                objValue_ = static_cast<float>(currentObj);

                // Saving the result
                extractSolution(pInst, zSolution, routeSolution);

                // Convert back to continuous for next iteration
                convertToFloat();
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
            getDuals(pInst);
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

            // Set time limit
            model_->set(GRB_DoubleParam_TimeLimit, availableTime);

            endBatchUpdate();

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
                    convertToFloat();
                }
                else {
                    objValue_ = static_cast<float>(currentObj);

                    // Saving the result
                    extractSolution(pInst, zSolution, routeSolution);

                    // Convert back to continuous for next iteration
                    convertToFloat();
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
        extractSolution(pInst, zSolution, routeSolution);

        // Create relaxed model from the solved MIP to get duals from LP relaxation
        GRBModel relaxedModel = model_->relax();

        // Solve the relaxed model (should be very fast due to warm start)
        relaxedModel.optimize();

        if (relaxedModel.get(GRB_IntAttr_Status) != GRB_OPTIMAL) {
            std::cerr << "Failed to solve LP relaxation for dual values" << std::endl;
            throw std::runtime_error("Failed to solve LP relaxation for dual values");
        }

        getDualsFromRelaxed(relaxedModel, pInst);

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
        // Convert variables to integer
        convertToInt();

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
        double currentObj = getObjValue();


        if (currentObj >= preObj) {
            // Convert variables back to continuous
            convertToFloat();
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
            extractSolution(pInst, zSolution, routeSolution);

            auxObjValue_ = 0;

            // If LP objective is better than current integer solution
            if (lpObj < objValue_) {
                // Fix variables at their current values
                beginBatchUpdate();
                // Convert variables back to continuous
                for (auto& var : zVar_)
                    var.set(GRB_CharAttr_VType, GRB_CONTINUOUS);

                for (auto& var : routeVar_)
                    var.set(GRB_CharAttr_VType, GRB_CONTINUOUS);

                for (size_t r = 0; r < routeVar_.size(); ++r) {
                    if (routeVals[r] > 0.5) {
                        routeVar_[r].set(GRB_DoubleAttr_UB, 1.0);
                        routeVar_[r].set(GRB_DoubleAttr_LB, -GRB_INFINITY);
                    } else {
                        routeVar_[r].set(GRB_DoubleAttr_UB, 0.0);
                        routeVar_[r].set(GRB_DoubleAttr_LB, -GRB_INFINITY);
                    }
                }

                for (size_t i = 0; i < zVar_.size(); ++i) {
                    if (zVals[i] > 0.5) {
                        zVar_[i].set(GRB_DoubleAttr_UB, 1.0);
                        zVar_[i].set(GRB_DoubleAttr_LB, -GRB_INFINITY);
                    } else {
                        zVar_[i].set(GRB_DoubleAttr_UB, 0.0);
                        zVar_[i].set(GRB_DoubleAttr_LB, -GRB_INFINITY);
                    }
                }

                // Change to maximization
                model_->set(GRB_IntAttr_ModelSense, GRB_MAXIMIZE);

                // Change RHS of constraints to 0

                for (auto& constr : requestConstr_) {
                    constr.set(GRB_DoubleAttr_RHS, 0.0);
                }

                for (auto& constr : vehicleConstr_) {
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
                    auxObjValue_ = static_cast<float>(getObjValue());

                    // Get new dual values
                    getDuals(pInst);

                    // Reset changes
                    beginBatchUpdate();

                    // Change back to minimization
                    model_->set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

                    // Reset variable bounds
                    for (auto& var : zVar_) {
                        var.set(GRB_DoubleAttr_LB, 0.0);
                        var.set(GRB_DoubleAttr_UB, GRB_INFINITY);
                    }

                    for (auto& var : routeVar_) {
                        var.set(GRB_DoubleAttr_LB, 0.0);
                        var.set(GRB_DoubleAttr_UB, GRB_INFINITY);
                    }

                    // Reset RHS values
                    for (size_t i = 0; i < requestConstr_.size(); ++i) {
                        requestConstr_[i].set(GRB_DoubleAttr_RHS, requestRHS_[i]);
                    }

                    for (size_t i = 0; i < vehicleConstr_.size(); ++i) {
                        vehicleConstr_[i].set(GRB_DoubleAttr_RHS, vehicleRHS_[i]);
                    }

                    endBatchUpdate();
                }
            }
        }

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
        convertToInt();

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

void RP_Gurobi::recoverModelForDuals(PInstance &pInst, boost::dynamic_bitset<> &removedRequests) {
    try {

        // 1. Remove request constraints (no need to update internal vectors)
        for (int i = static_cast<int>(removedRequests.size())-1; i >= 0; --i) {
            if (removedRequests[i]) {
                model_->remove(requestConstr_[i]);
                requestConstr_.erase(requestConstr_.begin() + i);
                pInst->requests_.erase(pInst->requests_.begin() + i);
            }
        }

        // 2. Remove non-basis route variables that cover removed requests
        for (int i = static_cast<int>(compRoutes_.size())-1; i >= 0; --i) {

            if (pInst->vehicles_[compRoutes_[i]->vehicleID_]->currentRoute_->getRouteId() != compRoutes_[i]->getRouteId()) {
                if ((compRoutes_[i]->column_ & removedRequests).any()) {
                    model_->remove(routeVar_[i]);
                    routeVar_.erase(routeVar_.begin() + i);
                    compRoutes_.erase(compRoutes_.begin() + i);
                }
            }
            else {
                // Update objective coefficient
                routeVar_[i].set(GRB_DoubleAttr_Obj, compRoutes_[i]->totalDelay_);
            }
        }

        // 4. Single model update
        model_->update();

        // 1. Solve the modified LP
        model_->optimize();

        // Get request constraint duals
        for (size_t i = 0; i < requestConstr_.size(); ++i) {
            pInst->requests_[i]->dual_ = requestConstr_[i].get(GRB_DoubleAttr_Pi);
            std::cout << pInst->requests_[i]->InitialDual_ << " - " << pInst->requests_[i]->dual_ << std::endl;
        }

        for (size_t j = 0; j < vehicleConstr_.size(); ++j)
            pInst->vehicles_[j]->dual_ = vehicleConstr_[j].get(GRB_DoubleAttr_Pi);

    } catch (const GRBException& e) {
        std::cerr << "Gurobi error in recoverModelForDuals: " << e.getMessage() << std::endl;
        throw;
    }
}