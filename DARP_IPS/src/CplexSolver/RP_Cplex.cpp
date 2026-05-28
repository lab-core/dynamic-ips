//
// Created by Ella on 2021-11-10.
//

#include "RP_Cplex.h"
#include "solvers/SolverEnv.h"

//---------------------------------------------------------------------------------------------
//  Reduced Problem class
//  Build and solve the Reduced problem of the ISUD
//---------------------------------------------------------------------------------------------

// Constructor and Destructor
RP_Cplex::RP_Cplex() : CplexModeler() {

    // defining variable
    zVar_ = IloNumVarArray(env_, 0.0, 0.0, IloInfinity,ILOFLOAT);
    routeVar_ = IloNumVarArray(env_, 0.0, 0.0, IloInfinity,ILOFLOAT);
    compRoutes_.clear();
    auxObjValue_ = 0;
    objValue_ = 0;
}


// this function initializes the model and defines an empty set of constraints
void RP_Cplex::ResetRPModel() {

    try {
        bool isModelExist = false;
        if (routeVar_.getSize() > 0)
            isModelExist = true;
        if (isModelExist){
            routeVar_.endElements();
            routeVar_.clear();
            compRoutes_.clear();
            zVar_.endElements();
            zVar_.clear();
            Model_.remove(requestConst_);
            Model_.remove(vehicleConst_);
        }
    }
    catch (IloException& e) {
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}

void RP_Cplex::resetForNextIteration() {
    compRoutes_.clear();
    routesToAdd_.clear();
    objValue_    = 0;
    auxObjValue_ = 0;

    // Nuclear env reset (debug): end the entire IloEnv and start fresh.
    renewEnv();

    // Reinitialise RP-specific var arrays against the new env.
    routeVar_ = IloNumVarArray(env_, 0.0, 0.0, IloInfinity, ILOFLOAT);
    zVar_     = IloNumVarArray(env_, 0.0, 0.0, IloInfinity, ILOFLOAT);
}

// this function adds routeVar to the model
void RP_Cplex::addRouteVar(PRoute &newRoute, PInstance &pInst) {
    addRouteVarInt(routeVar_, newRoute, POSITIVE, pInst);
    compRoutes_.push_back(newRoute);
    newRoute->mpAdded_ = true;
}

void RP_Cplex::addRouteVarFloat(PRoute &newRoute, PInstance &pInst) {
    CplexModeler::addRouteVarFloat(routeVar_, newRoute, POSITIVE, pInst);
    compRoutes_.push_back(newRoute);
    newRoute->mpAdded_ = true;
}


void RP_Cplex:: addZVar(PRequest &request) {
    addZVarInt(zVar_, request, POSITIVE);
}

// this function adds one route at each iteration of the algorithm during one epoch
void RP_Cplex::updateRPModel(PInstance &pInst) {
    // add the new compatible column to the model
    for (auto & routeObj : routesToAdd_) {
        if (pInst->vehicles_[routeObj->vehicleID_]->vehicleIndex_ > -1)
            addRouteVarFloat(routeObj, pInst);
    }
}


// this function builds the model at the start of each epoch
void RP_Cplex::buildModelRP(PInstance &pInst, std::vector<PRoute> &routeSolution,
                                int nbVehicles) {
    // model initialization (defining an empty set of constraints and adding the objective function)
    int rhs = 1;
    initializeModel(pInst, rhs, nbVehicles);

    // adding request columns (z variables)
    for (auto & zSol : pInst->requests_) {
        if (zSol->committedPickTime_ == LARGE_CONSTANT)
            addZVarFloat(zVar_, zSol, POSITIVE);
    }

    // adding route solution columns
    for (auto & routeSol : routeSolution){
        if (pInst->vehicles_[routeSol->vehicleID_]->vehicleIndex_ > -1)
            addRouteVarFloat(routeSol, pInst);
    }


    //adding new route variables
    for (auto & routeObj : routesToAdd_) {
        if (pInst->vehicles_[routeObj->vehicleID_]->vehicleIndex_ > -1)
            addRouteVarFloat(routeObj, pInst);
    }
}

void RP_Cplex::solveModelLP(const PInstance &pInst, const InputPaths &inputPaths) {
    try {
        myTools::CoutRedirector redirector(inputPaths.getOutputSolverLog(), "LMP");

        Cplex_.extract(Model_);
        solveTime_->start();
        if (!Cplex_.solve()) {
            solveTime_->stop();
            env_.out() << Model_ << std::endl;

            std::cout << "Failed to optimize the LMP" << std::endl;
            Cplex_.clearModel();
            myTools::myException::throwError("Failed to optimize the LMP");
        }
        else {
            solveTime_->stop();

            objValue_ = static_cast<float>(Cplex_.getObjValue());
            // getting dual values
            getDuals(pInst);
        }
//        Cplex_.clearModel();
    }
    catch (IloException& e) {
        std::cout << "Error occurred in RP_Cplex::solveModelLP at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}

void RP_Cplex::solveModelInt(const PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
                                   const InputPaths &inputPaths, float availableTime, float preObj) {
    try {

        IloConversion convZ(env_, zVar_, ILOINT);
        IloConversion convR(env_, routeVar_, ILOINT);
        Model_.add(convZ);
        Model_.add(convR);

        Cplex_.extract(Model_);

        myTools::CoutRedirector redirector(inputPaths.getOutputSolverLog(), "MP");
        Cplex_.setParam(IloCplex::Param::TimeLimit, availableTime);

        solveTime_->start();
        if (!Cplex_.solve()) {
            solveTime_->stop();
            Cplex_.clearModel();
            myTools::myException::throwError("Failed to optimize the MP");
        }
        else {
            solveTime_->stop();
            if (Cplex_.getObjValue() >= preObj) {
                convR.end();
                convZ.end();
                Cplex_.clearModel();
            }
            else {
                objValue_ = static_cast<float>(Cplex_.getObjValue());

                // saving the result and remove out of base variables
                zSolution.clear();
                routeSolution.clear();
                IloNumArray zVal(env_);
                IloNumArray routeVal(env_);
                Cplex_.getValues(zVal, zVar_);
                Cplex_.getValues(routeVal, routeVar_);
                convR.end();
                convZ.end();

                for (IloInt r = routeVal.getSize() - 1; r >= 0; --r) {
                    if (routeVal[r] > 0.5) {
                        routeSolution.push_back(compRoutes_[r]);
                    }
                }

                for (IloInt i = zVal.getSize() - 1; i >= 0; --i) {
                    if (zVal[i] > 0.5) {
                        zSolution.push_back(pInst->nameToRequest_[zVar_[i].getName()]);
                    }
                }
                zVal.end();
                routeVal.end();
            }
        }
//        Cplex_.clearModel();
    }
    catch (IloException& e) {
        std::cout << "Error occurred in RP_Cplex::solveModelInt at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}

void RP_Cplex::solveModelRelaxInt(const PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
                                     const InputPaths &inputPaths, float availableTime, float preObj) {
    try {

        // Solve the Linear Relaxation
        myTools::CoutRedirector redirector(inputPaths.getOutputSolverLog(), "MP");

        Cplex_.extract(Model_);
        solveTime_->start();
        if (!Cplex_.solve()) {
            solveTime_->stop();
            std::cout << "Failed to optimize the LMP" << std::endl;
        }
        else {
            solveTime_->stop();

            objValue_ = static_cast<float>(Cplex_.getObjValue());
            getDuals(pInst);

            // Convert to integer
            IloConversion convZ(env_, zVar_, ILOINT);
            IloConversion convR(env_, routeVar_, ILOINT);

            Model_.add(convZ);
            Model_.add(convR);
//            Cplex_.extract(Model_);
            std::cout << "----------------------- MP ------------------------"<< std::endl;
//            setParameters(pInst, availableTime);
            Cplex_.setParam(IloCplex::Param::TimeLimit, availableTime);


            solveTime_->start();
            if (!Cplex_.solve()) {
                solveTime_->stop();
                Cplex_.clearModel();
                myTools::myException::throwError("Failed to optimize the MP");
            }
            else {
                solveTime_->stop();
                if (Cplex_.getObjValue() >= preObj) {
                    convR.end();
                    convZ.end();
                    Cplex_.clearModel();
                }
                else {
                    objValue_ = static_cast<float>(Cplex_.getObjValue());

                    // saving the result and remove out of base variables
                    zSolution.clear();
                    routeSolution.clear();
                    IloNumArray zVal(env_);
                    IloNumArray routeVal(env_);
                    Cplex_.getValues(zVal, zVar_);
                    Cplex_.getValues(routeVal, routeVar_);
                    convR.end();
                    convZ.end();
                    for (IloInt r = routeVal.getSize() - 1; r >= 0; --r) {
                        if (routeVal[r] > 0.9) {
                            routeSolution.push_back(compRoutes_[r]);
                        }
                    }

                    for (IloInt i = zVal.getSize() - 1; i >= 0; --i) {
                        if (zVal[i] > 0.9) {
                            zSolution.push_back(pInst->nameToRequest_[zVar_[i].getName()]);
                        }
                    }
                    zVal.end();
                    routeVal.end();
               }
            }
        }

        Cplex_.clearModel();
    }
    catch (IloException& e) {
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}

void RP_Cplex::solveModelIntAux_P(const PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
                                      const InputPaths &inputPaths, float availableTime, float preObj, float lpObj) {
    try {

        IloConversion convZ (env_, zVar_, ILOINT);
        IloConversion convR (env_, routeVar_, ILOINT);
        Model_.add(convZ);
        Model_.add(convR);

        Cplex_.extract(Model_);
        myTools::CoutRedirector redirector(inputPaths.getOutputSolverLog(), "MP");
//        setParameters(pInst, availableTime);
        Cplex_.setParam(IloCplex::Param::TimeLimit, availableTime);

        solveTime_->start();
        if (!Cplex_.solve()) {
            env_.out() << Model_ << std::endl;
            solveTime_->stop();
 //           std::cout << "Failed to optimize the MP" << std::endl;
            myTools::myException::throwError("Failed to optimize the MP");
        }
        else {
            solveTime_->stop();
            if (Cplex_.getObjValue() >= preObj) {
                convR.end();
                convZ.end();
                Cplex_.clearModel();
            }
            else {
                objValue_ = static_cast<float>(Cplex_.getObjValue());

                // saving the result and remove out of base variables
                zSolution.clear();
                routeSolution.clear();
                IloNumArray zVal(env_);
                IloNumArray routeVal(env_);
                Cplex_.getValues(zVal, zVar_);
                Cplex_.getValues(routeVal, routeVar_);
                convR.end();
                convZ.end();

                for (IloInt r = routeVal.getSize() - 1; r >= 0; --r) {
                    if (routeVal[r] > 0.5) {
                        routeSolution.push_back(compRoutes_[r]);
 //                       pInst->vehicles_[compRoutes_[r]->vehicleID_]->setCurrentRoute(compRoutes_[r]);
                    }
                }
                for (IloInt i = zVal.getSize() - 1; i >= 0; --i) {
                    if (zVal[i] > 0.5)
                        zSolution.push_back(pInst->nameToRequest_[zVar_[i].getName()]);
                }
                auxObjValue_ = 0;

                if (lpObj < objValue_) {
                    for (IloInt r = routeVal.getSize() - 1; r >= 0; --r) {
                        if (routeVal[r] > 0.5) {
                            routeVar_[r].setBounds(-IloInfinity, 1.0);
                        }
                        else
                            routeVar_[r].setBounds(-IloInfinity, 0.0);
                    }
                    for (IloInt i = zVal.getSize() - 1; i >= 0; --i) {
                        if (zVal[i] > 0.5) {
                            zVar_[i].setBounds(-IloInfinity, 1.0);
                        }
                        else
                            zVar_[i].setBounds(-IloInfinity, 0.0);
                    }

                    // change the objective function to maximize
                    objFunction_.setSense(IloObjective::Maximize);

                    IloNumArray requestRHS(env_);
                    IloNumArray vehicleRHS(env_);

                    createIloNumArray (requestRHS, nbRequestTask_, 0.0);
                    createIloNumArray (vehicleRHS, pInst->nbVehicles_, 0.0);
                    requestConst_.setBounds(requestRHS, requestRHS);
                    vehicleConst_.setBounds(vehicleRHS, vehicleRHS);
                    requestRHS.end();
                    vehicleRHS.end();

                    Cplex_.extract(Model_);

 //                   Cplex_.setParam(IloCplex::Param::RootAlgorithm, 2);
                    solveTime_->start();
                    if (!Cplex_.solve()) {
                        env_.out() << Model_ << std::endl;
                        solveTime_->stop();
                        myTools::myException::throwError("Failed to optimize the Aux MP");
                    }
                    else {
                        solveTime_->stop();

                        auxObjValue_ = static_cast<float>(Cplex_.getObjValue());
                        // getting dual values
                        getDuals(pInst);

                        // reset changes
                        objFunction_.setSense(IloObjective::Minimize);
                        for (IloInt i = zVal.getSize() - 1; i >= 0; --i)
                            zVar_[i].setBounds(0.0, IloInfinity);

                        for (IloInt r = routeVal.getSize() - 1; r >= 0; --r)
                            routeVar_[r].setBounds(0.0, IloInfinity);
                        requestConst_.setBounds(requestRHS_, requestRHS_);
                        vehicleConst_.setBounds(vehicleRHS_, vehicleRHS_);
                    }
                }
                zVal.end();
                routeVal.end();
            }
        }

        Cplex_.clearModel();
    }
    catch (IloException& e) {
        std::cout << "Error occurred in RP_Cplex::solveModelIntAux at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}

//************************************************************************
// Display function
//************************************************************************
std::string RP_Cplex::toString() const {
    std::stringstream repStr;
    repStr << std::endl;
    repStr << "# =======================  REDUCED PROBLEM SOLVED  ======================= " << std::endl;
    repStr << CplexModeler::toString();
    return repStr.str();
}

void RP_Cplex::solveLPDual(const PInstance &pInst, const InputPaths &inputPaths) {
    try {
        myTools::CoutRedirector redirector(inputPaths.getOutputSolverLog(), "LMP");
        // Configure CPLEX to use interior point method
        if (pInst->parameters_->initialDual_ == BARRIER || pInst->parameters_->dualMethod_ == INTERIOR) {
            Cplex_.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Barrier);
            // Disable crossover (equivalent to Gurobi's Crossover=0)
            Cplex_.setParam(IloCplex::Param::Barrier::Crossover, IloCplex::NoAlg);
        }

        Cplex_.extract(Model_);
        solveTime_->start();
        if (!Cplex_.solve()) {
            solveTime_->stop();
            env_.out() << Model_ << std::endl;

            std::cout << "Failed to optimize the LMP" << std::endl;
            Cplex_.clearModel();
            myTools::myException::throwError("Failed to optimize the LMP");
        }
        else {
            solveTime_->stop();

            // getting dual values
            getDuals(pInst);
        }
        if (pInst->parameters_->initialDual_ == BARRIER || pInst->parameters_->dualMethod_ == INTERIOR) {
            // Reset to dual simplex method (equivalent to Gurobi's METHOD_DUAL)
            Cplex_.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Dual);
            // Re-enable crossover
            Cplex_.setParam(IloCplex::Param::Barrier::Crossover, IloCplex::Auto);
        }
        //        Cplex_.clearModel();
    }
    catch (IloException& e) {
        std::cout << "Error occurred in RP_Cplex::solveModelLP at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}

void RP_Cplex::markColumnsToKeep() {
    try {
        if (Cplex_.getStatus() != IloAlgorithm::Optimal &&
            Cplex_.getStatus() != IloAlgorithm::Feasible) {
            std::cerr << "[RP_Cplex] markColumnsToKeep called but model not optimal/feasible.\n";
            return;
        }

        const int numRouteVars = routeVar_.getSize();
        if (numRouteVars != static_cast<int>(compRoutes_.size())) {
            std::cerr << "[RP_Cplex] markColumnsToKeep: variable count inconsistent with compRoutes_.\n";
            return;
        }

        IloNumArray values(env_);
        IloNumArray reducedCosts(env_);
        IloCplex::BasisStatusArray basisStatuses(env_);

        Cplex_.getValues(values, routeVar_);
        Cplex_.getReducedCosts(reducedCosts, routeVar_);
        Cplex_.getBasisStatuses(basisStatuses, routeVar_);

        for (int r = 0; r < numRouteVars; ++r) {
            compRoutes_[r]->keepMP_ = false;

            const bool keepBasis = (basisStatuses[r] == IloCplex::Basic ||
                                    basisStatuses[r] == IloCplex::FreeOrSuperbasic);
            const bool keepX     = (std::fabs(values[r]) > 0.1);
            const bool keepRC0   = (std::fabs(reducedCosts[r]) <= 1.0);

            if (keepBasis || keepX || keepRC0)
                compRoutes_[r]->keepMP_ = true;
        }
        values.end();
        reducedCosts.end();
        basisStatuses.end();
    }
    catch (const IloException& e) {
        std::cerr << "CPLEX Error in markColumnsToKeep: " << e << "\n";
        throw;
    }
}
