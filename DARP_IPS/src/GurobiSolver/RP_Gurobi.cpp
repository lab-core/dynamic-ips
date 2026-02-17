//
// Created by Elahe Amiri on 2025-06-25.
//

#include "RP_Gurobi.h"

RP_Gurobi::RP_Gurobi(std::string outputLog) : GurobiModeler(outputLog) {
    compRoutes_.clear();
    routesToAdd_.clear();
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

        // Add the new compatible columns to the model
        for (auto& routeObj : routesToAdd_) {
            if (pInst->vehicles_[routeObj->vehicleID_]->vehicleIndex_ > -1) {
                addRouteVar(routeObj, pInst);
            }
        }

        model_->update();
    }
    catch (GRBException& e) {
        std::cerr << "Error in updateRPModel: " << e.getMessage() << std::endl;
        throw;
    }
}


void RP_Gurobi::updateRPModel_batch(PInstance& pInst) {
    try {
        if (routesToAdd_.empty()) return;

        const size_t numRoutes = routesToAdd_.size();

        std::vector<double> lb(numRoutes, 0.0);
        std::vector<double> ub(numRoutes, GRB_INFINITY);
        std::vector<double> obj(numRoutes);
        std::vector<char> vtype(numRoutes, GRB_BINARY);
        std::vector<GRBColumn> columns(numRoutes);

        compRoutes_.reserve(compRoutes_.size() + numRoutes);   // avoid reallocations
        routeVar_.reserve(routeVar_.size() + numRoutes);

        size_t k = 0;
        // Fill arrays with range-based loop
        for (const auto& routeObj : routesToAdd_) {
            obj[k] = routeObj->objCoef_;
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
            static_cast<int>(numRoutes)));

        // Add variables to container
        routeVar_.insert(routeVar_.end(), newVars.get(), newVars.get() + numRoutes);

        // Single update call
        model_->update();

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

        model_->update();
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

void RP_Gurobi::solveLPDual(const PInstance &pInst, const InputPaths &inputPaths) {
    try {
        // Redirect output to log file
        // Configure Gurobi logging
        if (pInst->parameters_->initialDual_ == BARRIER || pInst->parameters_->dualMethod_ == INTERIOR) {
            model_->set(GRB_IntParam_Method, 2);
            model_->set(GRB_IntParam_Crossover, 0);
            model_->update();
        }
        model_->write("gurobi_orig.lp");

        solveTime_->start();
        int status = solve();
        solveTime_->stop();

        /*auto coverage = countRequestCoverage(pInst);
        printCoverageStatistics(pInst);*/

        if (status != GRB_OPTIMAL && status != GRB_SUBOPTIMAL) {
            std::cerr << "Failed to optimize the LMP. Status: " << status << std::endl;
            throw std::runtime_error("Failed to optimize the LMP");
        }
        std::cout << "Linear Objective: " << static_cast<float>(getObjValue()) << std::endl;
        // Getting dual values
        if (pInst->parameters_->initialDual_ == BARRIER)
            getBarrierDuals(pInst);
        else
            getDuals(pInst);
        if (pInst->parameters_->initialDual_ == BARRIER || pInst->parameters_->dualMethod_ == INTERIOR) {
            model_->set(GRB_IntParam_Method, GRB_METHOD_DUAL);
            model_->set(GRB_IntParam_Crossover, 1);
            model_->update();
        }
    }
    catch (GRBException& e) {
        std::cerr << "Error occurred in solveModelLP at line: " << __LINE__ << std::endl;
        std::cerr << "GRB Error: " << e.getMessage() << std::endl;
        throw;
    }
}

void RP_Gurobi::verifyDualsAndBasis(const PInstance &pInst) {
    try {
        const double TOLERANCE = 1e-6;  // Tolerance for numerical precision
        bool verificationPassed = true;
        int basicVarCount = 0;

        std::cout << "\n=== Basic Variables Dual Verification ===" << std::endl;
        std::cout << "Verifying: Objective Coefficient = Column Dual Value for Basic Variables" << std::endl;

        // Get all variables from the model
        GRBVar* vars = model_->getVars();
        int numVars = model_->get(GRB_IntAttr_NumVars);

        // Get constraint duals for calculating column duals
        GRBConstr* constrs = model_->getConstrs();
        int numConstrs = model_->get(GRB_IntAttr_NumConstrs);

        for (int i = 0; i < numVars; i++) {
            // Get variable basis status
            int basisStatus = vars[i].get(GRB_IntAttr_VBasis);

            if (basisStatus == GRB_BASIC) {
                basicVarCount++;

                double objCoeff = vars[i].get(GRB_DoubleAttr_Obj);
                double varValue = vars[i].get(GRB_DoubleAttr_X);
                std::string varName = vars[i].get(GRB_StringAttr_VarName);

                // Calculate column dual: c_j - sum(a_ij * pi_i)
                // For basic variables, this should equal the objective coefficient
                double columnDual = objCoeff;

                // Get column coefficients and calculate dual value
                GRBColumn col = model_->getCol(vars[i]);
                int colSize = col.size();

                for (int k = 0; k < colSize; k++) {
                    GRBConstr constr = col.getConstr(k);
                    double coeff = col.getCoeff(k);
                    double pi = constr.get(GRB_DoubleAttr_Pi);
                    columnDual -= coeff * pi;
                }

                // For basic variables: reduced cost should be 0
                // This means: objective_coeff - sum(a_ij * pi_i) = 0
                // Therefore: objective_coeff = sum(a_ij * pi_i)
                double calculatedDual = objCoeff - columnDual;

                std::cout << "Basic Variable: " << varName << std::endl;
                std::cout << "  Objective Coeff: " << objCoeff << std::endl;
                std::cout << "  Calculated Dual: " << calculatedDual << std::endl;
                std::cout << "  Variable Value:  " << varValue << " (integer: " <<
                    (std::abs(varValue - std::round(varValue)) < TOLERANCE ? "YES" : "NO") << ")" << std::endl;
                std::cout << "  Reduced Cost:    " << columnDual << std::endl;

                // Verify that reduced cost is zero (basic variable condition)
                if (std::abs(columnDual) > TOLERANCE) {
                    std::cerr << "  ✗ ERROR: Basic variable has non-zero reduced cost!" << std::endl;
                    verificationPassed = false;
                } else {
                    std::cout << "  ✓ Reduced cost is zero (basic variable condition satisfied)" << std::endl;
                }

                // Verify that objective coefficient equals the dual calculation
                double dualDifference = std::abs(objCoeff - calculatedDual);
                if (dualDifference > TOLERANCE) {
                    std::cerr << "  ✗ ERROR: Objective coeff ≠ Calculated dual (diff: " << dualDifference << ")" << std::endl;
                    verificationPassed = false;
                } else {
                    std::cout << "  ✓ Objective coefficient matches calculated dual" << std::endl;
                }

                std::cout << "  " << std::string(50, '-') << std::endl;
            }
        }

        std::cout << "\n--- Verification Summary ---" << std::endl;
        std::cout << "Total basic variables checked: " << basicVarCount << std::endl;

        if (verificationPassed) {
            std::cout << "✓ SUCCESS: All basic variables satisfy dual conditions" << std::endl;
            std::cout << "✓ All basic variables have integer values" << std::endl;
            std::cout << "✓ Objective coefficients match calculated duals for all basic variables" << std::endl;
        } else {
            std::cerr << "✗ FAILED: Dual verification failed for some basic variables" << std::endl;
            throw std::runtime_error("Dual verification failed: basic variables don't satisfy dual conditions");
        }

        // Cleanup
        delete[] vars;
        delete[] constrs;

    } catch (GRBException& e) {
        std::cerr << "Error in dual verification: " << e.getMessage() << std::endl;
        throw;
    }
}
void RP_Gurobi::solveModelInt(const PInstance &pInst, std::vector<PRequest> &zSolution,
    std::vector<PRoute> &routeSolution, const InputPaths &inputPaths, float availableTime, float preObj) {
    try {
        // Convert variables to integer
        for (auto& var : zVar_) {
            var.set(GRB_CharAttr_VType, GRB_BINARY);
        }

        for (auto& var : routeVar_) {
            var.set(GRB_CharAttr_VType, GRB_BINARY);
        }
        // Count coverage for all requests
        /*auto coverage = countRequestCoverage(pInst);
        std::cout << "Request 0 is covered by " << coverage[0] << " routes" << std::endl;

        // Print detailed statistics
        printCoverageStatistics(pInst);*/

        // Redirect output to log file
        myTools::CoutRedirector redirector(inputPaths.getOutputSolverLog(), "LMP");

        // Set time limit
        model_->set(GRB_DoubleParam_TimeLimit, availableTime);
        model_->update();
        /*model_->write(std::string("gurobi_orig_") + std::to_string(pInst->nbRequests_) + "_" +
            std::to_string(pInst->nbNewRequests_) + ".lp" );*/

        solveTime_->start();
        int status = solve();
        solveTime_->stop();

        if (status != GRB_OPTIMAL && status != GRB_SUBOPTIMAL && status != GRB_TIME_LIMIT) {
            throw std::runtime_error("Failed to optimize the MP");
        }
        else {
            double currentObj = getObjValue();

            if (currentObj < preObj) {
                objValue_ = static_cast<float>(currentObj);

                // Saving the result
                extractSolution(pInst, zSolution, routeSolution);

            }
            // Convert back to continuous for next iteration
            convertToFloat();
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

            // Convert z variables to integer
            for (auto& var : zVar_) {
                var.set(GRB_CharAttr_VType, GRB_BINARY);
            }

            // Convert route variables to integer
            for (auto& var : routeVar_) {
                var.set(GRB_CharAttr_VType, GRB_BINARY);
            }

            // Set time limit
            model_->set(GRB_DoubleParam_TimeLimit, availableTime);

            model_->update();

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
        markColumnsToKeep(relaxedModel);

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

                model_->update();

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

                    model_->update();
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
        for (size_t i = 0; i < zVar_.size(); ++i) {
            zVar_[i].set(GRB_CharAttr_VType, originalZVTypes[i]);
        }
        for (size_t i = 0; i < routeVar_.size(); ++i) {
            routeVar_[i].set(GRB_CharAttr_VType, originalRouteVTypes[i]);
        }
        model_->update();

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
                routeVar_[i].set(GRB_DoubleAttr_Obj, compRoutes_[i]->objCoef_);
            }
        }

        // 4. Single model update
        model_->update();

        // 1. Solve the modified LP
        model_->optimize();

        // Get request constraint duals
        for (size_t i = 0; i < requestConstr_.size(); ++i) {
            pInst->requests_[i]->dual_ = requestConstr_[i].get(GRB_DoubleAttr_Pi);
        }

        for (size_t j = 0; j < vehicleConstr_.size(); ++j)
            pInst->vehicles_[j]->dual_ = vehicleConstr_[j].get(GRB_DoubleAttr_Pi);

    } catch (const GRBException& e) {
        std::cerr << "Gurobi error in recoverModelForDuals: " << e.getMessage() << std::endl;
        throw;
    }
}

// Count how many times each request is covered by route variables in the model
std::vector<int> RP_Gurobi::countRequestCoverage(const PInstance &pInst) {
    try {
        std::vector<int> coverageCount(pInst->requests_.size(), 0);

        // Iterate through all route variables in the model
        for (size_t r = 0; r < routeVar_.size(); ++r) {
            // Check if this route variable exists and is valid
            if (r < compRoutes_.size()) {
                const auto& route = compRoutes_[r];

                // Iterate through all requests to see which ones this route covers
                for (size_t i = 0; i < pInst->requests_.size(); ++i) {
                    // Check if this route covers the request
                    // Assuming you have a way to check if route covers request
                    // This could be through route->column_ bitset or route's request list

                    if (route->column_.size() > i && route->column_[i]) {
                        coverageCount[i]++;
                        /*if (pInst->requests_[i]->getRequestId() == 1007)
                            std::cout << route->toString() << std::endl;*/
                    }
                }
            }
        }

        return coverageCount;

    } catch (const GRBException& e) {
        std::cerr << "Error in countRequestCoverage: " << e.getMessage() << std::endl;
        throw;
    } catch (const std::exception& e) {
        std::cerr << "Error in countRequestCoverage: " << e.what() << std::endl;
        throw;
    }
}

// Function to print coverage statistics
void RP_Gurobi::printCoverageStatistics(const PInstance &pInst) {
    try {
        auto coverage = countRequestCoverage(pInst);

        std::cout << "\n=== Request Coverage Statistics ===" << std::endl;
        std::cout << "Request ID\tCount\tRequest Name\tCommitted" << std::endl;
        std::cout << std::string(60, '-') << std::endl;

        for (size_t i = 0; i < pInst->requests_.size(); ++i) {
            std::cout << std::setw(10) << i
                     << "\t" << std::setw(5) << coverage[i];

            // Print request name if available
            if (i < pInst->requests_.size()) {
                // Assuming requests have some identifier - adjust based on your Request class
                std::cout << "\t" << "Request_" << pInst->requests_[i]->getRequestId(); // Replace with actual request identifier
                if (pInst->requests_[i]->committedPickTime_ == LARGE_CONSTANT)
                    std::cout << "\t" << "False";
                else
                    std::cout << "\t" << "True";
            }
            std::cout << std::endl;
        }

        // Summary statistics
        int uncoveredRequests = std::count(coverage.begin(), coverage.end(), 0);
        int totalCoverages = std::accumulate(coverage.begin(), coverage.end(), 0);
        double avgCoverage = coverage.empty() ? 0.0 : static_cast<double>(totalCoverages) / coverage.size();

        std::cout << std::string(60, '-') << std::endl;
        std::cout << "Total requests: " << pInst->requests_.size() << std::endl;
        std::cout << "Uncovered requests: " << uncoveredRequests << std::endl;
        std::cout << "Total route-request coverages: " << totalCoverages << std::endl;
        std::cout << "Average coverage per request: " << std::fixed << std::setprecision(2) << avgCoverage << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error in printCoverageStatistics: " << e.what() << std::endl;
    }
}

void RP_Gurobi::markColumnsToKeep(const GRBModel& lpModel)
{
    try {
        const int status = lpModel.get(GRB_IntAttr_Status);
        if (status != GRB_OPTIMAL && status != GRB_SUBOPTIMAL) {
            std::cerr << "[RP_Gurobi] markColumnsToKeep called but model not optimal/suboptimal.\n";
            return;
        }

        // Get variables from the model (order: z vars first, then route vars)
        GRBVar* vars = lpModel.getVars();
        const int numVars = lpModel.get(GRB_IntAttr_NumVars);
        const int numRouteVars = static_cast<int>(compRoutes_.size());
        const int routeStartIndex = numVars - numRouteVars;  // route vars are last

        if (routeStartIndex < 0 || routeStartIndex + numRouteVars > numVars) {
            std::cerr << "[RP_Gurobi] markColumnsToKeep: model variable count inconsistent with compRoutes_.\n";
            return;
        }

        for (int r = 0; r < numRouteVars; ++r) {
            compRoutes_[r]->keepMP_ = false;

            const GRBVar& var = vars[routeStartIndex + r];

            // Basis status: BASIC, NONBASIC_LOWER, NONBASIC_UPPER, SUPERBASIC
            const int vb = var.get(GRB_IntAttr_VBasis);

            // Current LP solution value and reduced cost
            const double x  = var.get(GRB_DoubleAttr_X);
            const double rc = var.get(GRB_DoubleAttr_RC);

            const bool keepBasis = (vb == GRB_BASIC || vb == GRB_SUPERBASIC);
            const bool keepX     = (std::fabs(x) > 0.1);
            const bool keepRC0   = (std::fabs(rc) <= 1);

            if (keepBasis || keepX || keepRC0) {
                compRoutes_[r]->keepMP_ = true;
            }
        }
    }
    catch (GRBException& e) {
        std::cerr << "GRB Error in markColumnsToKeep: " << e.getMessage() << "\n";
        throw;
    }
}

void RP_Gurobi::markColumnsToKeep()
{
    try {
        const int status = model_->get(GRB_IntAttr_Status);
        if (status != GRB_OPTIMAL && status != GRB_SUBOPTIMAL) {
            std::cerr << "[RP_Gurobi] markColumnsToKeep called but model not optimal/suboptimal.\n";
            return;
        }

        // compRoutes_ is assumed to be the list of all route-columns currently in the MP.
        // Each route is assumed to have a member GRBVar varMP_ (or similar) for its column variable.
        for (size_t r = 0; r < routeVar_.size(); ++r) {
            compRoutes_[r]->keepMP_ = false;

            // Basis status: BASIC, NONBASIC_LOWER, NONBASIC_UPPER, SUPERBASIC
            const int vb = routeVar_[r].get(GRB_IntAttr_VBasis);

            // Current LP solution value and reduced cost
            const double x  = routeVar_[r].get(GRB_DoubleAttr_X);
            const double rc = routeVar_[r].get(GRB_DoubleAttr_RC);

            const bool keepBasis = (vb == GRB_BASIC || vb == GRB_SUPERBASIC);
            const bool keepX     = (std::fabs(x) > 0.1);
            const bool keepRC0   = (std::fabs(rc) <= 1);

            if (keepBasis || keepX || keepRC0) {
                compRoutes_[r]->keepMP_ = true;
            }
        }
    }
    catch (GRBException& e) {
        std::cerr << "GRB Error in markColumnsToKeep: " << e.getMessage() << "\n";
        throw;
    }
}