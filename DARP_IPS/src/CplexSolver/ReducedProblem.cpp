//
// Created by Ella on 2021-11-10.
//

#include "ReducedProblem.h"

//---------------------------------------------------------------------------------------------
//  Reduced Problem class
//  Build and solve the Reduced problem of the ISUD
//---------------------------------------------------------------------------------------------

// Constructor and Destructor
ReducedProblem::ReducedProblem() : CplexModeler(){

    // defining variable
    zVar_ = IloNumVarArray(env_, 0.0, 0.0, IloInfinity,ILOFLOAT);
    routeVar_ = IloNumVarArray(env_, 0.0, 0.0, IloInfinity,ILOFLOAT);
    uVar_ = IloNumVarArray(env_, 0.0, 0.0, 0.0,ILOFLOAT);
    vVar_ = IloNumVarArray(env_, 0.0, 0.0, 0.0,ILOFLOAT);
    compRoutes_.clear();
    auxObjValue_ = 0;
    objValue_ = 0;
}


// this function initializes the model and defines an empty set of constraints
void ReducedProblem::ResetRPModel() {

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

// this function adds routeVar to the model
void ReducedProblem::addRouteVar(PRoute &newRoute, PInstance &pInst) {
    addRouteVarInt(routeVar_, newRoute, POSITIVE, pInst);
    compRoutes_.push_back(newRoute);
    newRoute->mpAdded_ = true;
}

void ReducedProblem::addRouteVarFloat(PRoute &newRoute, PInstance &pInst) {
    CplexModeler::addRouteVarFloat(routeVar_, newRoute, POSITIVE, pInst);
    compRoutes_.push_back(newRoute);
    newRoute->mpAdded_ = true;
}


void ReducedProblem:: addZVar(PRequest &request) {
    addZVarInt(zVar_, request, POSITIVE);
}

// this function adds one route at each iteration of the algorithm during one epoch
void ReducedProblem::updateRPModel(PInstance &pInst) {
    // add the new compatible column to the model
    for (auto & routeObj : routesToAdd_) {
        if (pInst->vehicles_[routeObj->vehicleID_]->vehicleIndex_ > -1)
            addRouteVarFloat(routeObj, pInst);
    }
}


// this function builds the model at the start of each epoch
void ReducedProblem::buildModel(PInstance &pInst, std::vector<PRoute> &routeSolution,
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

void ReducedProblem::solveModelLP(const PInstance &pInst, const InputPaths &inputPaths) {
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
        std::cout << "Error occurred in ReducedProblem::solveModelLP at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}

void ReducedProblem::solveModelInt(const PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
                                   const InputPaths &inputPaths, float availableTime, float preObj) {
    try {

        IloConversion convZ(env_, zVar_, ILOINT);
        IloConversion convR(env_, routeVar_, ILOINT);
        Model_.add(convZ);
        Model_.add(convR);

        Cplex_.extract(Model_);

        myTools::CoutRedirector redirector(inputPaths.getOutputSolverLog(), "MP");
//        setParameters(pInst, availableTime);
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

                // std::cout << "Route,vehicle,index,RC,Committed,nbRequets,waitScore,duals,totalDealy" << std::endl;
                for (IloInt r = routeVal.getSize() - 1; r >= 0; --r) {
                    if (routeVal[r] > 0.5) {
                        /*float duals = pInst->vehicles_[compRoutes_[r]->vehicleID_]->dual_;
                        for (int i = 0; i < compRoutes_[r]->routeRequests_.size(); ++i) {
                            duals += compRoutes_[r]->routeRequests_[i]->dual_;
                        }
                        std::cout << compRoutes_[r]->getRouteId() << "," ;
                        std::cout << compRoutes_[r]->vehicleID_ << "," << compRoutes_[r]->createTime_ << "," ;
                        std::cout << compRoutes_[r]->reducedCost_ << "," << compRoutes_[r]->nbCommitted_ << ",";
                        std::cout << compRoutes_[r]->routeRequests_.size() << ","<< compRoutes_[r]->waitScore_ << ",";
                        std::cout << duals << ","<< compRoutes_[r]->totalDelay_ << std::endl;*/
                        routeSolution.push_back(compRoutes_[r]);
                        routeSolutionIndex_.push_back(static_cast<int>(r));
 //                       pInst->vehicles_[compRoutes_[r]->vehicleID_]->setCurrentRoute(compRoutes_[r]);
                    }
                }

                for (IloInt i = zVal.getSize() - 1; i >= 0; --i) {
                    if (zVal[i] > 0.5) {
                        zSolution.push_back(pInst->nameToRequest_[zVar_[i].getName()]);
                    }
                }

            }
        }
//        Cplex_.clearModel();
    }
    catch (IloException& e) {
        std::cout << "Error occurred in ReducedProblem::solveModelInt at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}

void ReducedProblem::solveModelLPInt(const PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
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
  //                  std::cout << "Route,vehicle,index,RC,Committed,nbRequsets,IncDegree,waitScore" << std::endl;
                    for (IloInt r = routeVal.getSize() - 1; r >= 0; --r) {
                        if (routeVal[r] > 0.9) {
                            routeSolution.push_back(compRoutes_[r]);
                            /*std::cout << compRoutes_[r]->getRouteId() << "," ;
                            std::cout << compRoutes_[r]->vehicleID_ << "," << compRoutes_[r]->createTime_ << "," ;
                            std::cout << compRoutes_[r]->reducedCost_ << "," << compRoutes_[r]->nbCommitted_ << ",";
                            std::cout << compRoutes_[r]->routeRequests_.size()<< "," << compRoutes_[r]->incompatibilityDegree_ << "," << compRoutes_[r]->waitScore_ << std::endl;*/
                            routeSolutionIndex_.push_back(static_cast<int>(r));
 //                           pInst->vehicles_[compRoutes_[r]->vehicleID_]->setCurrentRoute(compRoutes_[r]);
                        }
                    }

                    for (IloInt i = zVal.getSize() - 1; i >= 0; --i) {
                        if (zVal[i] > 0.9) {
                            zSolution.push_back(pInst->nameToRequest_[zVar_[i].getName()]);
                        }
                    }


                    /*if (routeSolution.size() != pInst->nbVehicles_)
                        myTools::throwError("Number of routes in the solution does not match with the vehicles!!!");*/
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

void ReducedProblem::solveModelIntAux_P(const PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
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
            }
        }

        Cplex_.clearModel();
    }
    catch (IloException& e) {
        std::cout << "Error occurred in ReducedProblem::solveModelIntAux at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}

//************************************************************************
// Display function
//************************************************************************
std::string ReducedProblem::toString() const {
    std::stringstream repStr;
    repStr << std::endl;
    repStr << "# =======================  REDUCED PROBLEM SOLVED  ======================= " << std::endl;
    repStr << CplexModeler::toString();
    return repStr.str();
}

void ReducedProblem::solveModelInt_box(const PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
                          const InputPaths &inputPaths, float availableTime, float preObj, float lpObj) {
    try {

        IloConversion convZ (env_, zVar_, ILOINT);
        IloConversion convR (env_, routeVar_, ILOINT);
        Model_.add(convZ);
        Model_.add(convR);

        Cplex_.extract(Model_);
        myTools::CoutRedirector redirector(inputPaths.getOutputSolverLog(), "MP");
 //       setParameters(pInst, availableTime);
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

                for (IloInt i = zVal.getSize() - 1; i >= 0; --i) {
                    uVar_[i].setBounds(0.0, IloInfinity);
                    vVar_[i].setBounds(0.0, IloInfinity);
                }
                Cplex_.extract(Model_);

//                Cplex_.setParam(IloCplex::Param::RootAlgorithm, 2);
                solveTime_->start();
                if (!Cplex_.solve()) {
                    env_.out() << Model_ << std::endl;
                    solveTime_->stop();
                    myTools::myException::throwError("Failed to optimize the Aux MP");
                }
                else {
                    solveTime_->stop();

                    objValue_ = static_cast<float>(Cplex_.getObjValue());
                    // getting dual values
                    getDuals(pInst);

                    IloNumArray uVal(env_);
                    IloNumArray vVal(env_);
                    Cplex_.getValues(uVal, uVar_);
                    Cplex_.getValues(vVal, vVar_);

                    for (IloInt i = zVal.getSize() - 1; i >= 0; --i) {
                        if (uVal[i] > 0)
                            std::cout << "U[" << zVar_[i].getName()  << "] : " << uVal[i] << std::endl;
                        if (vVal[i] > 0)
                            std::cout << "V[" << zVar_[i].getName()  << "] : " << vVal[i] << std::endl;
                    }

                    // rest the model
                    for (IloInt i = zVal.getSize() - 1; i >= 0; --i) {
                        uVar_[i].setBounds(0.0, 0.0);
                        vVar_[i].setBounds(0.0, 0.0);
                    }
                }
            }
        }

        Cplex_.clearModel();
    }
    catch (IloException& e) {
        std::cout << "Error occurred in ReducedProblem::solveModelIntAux at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}


void ReducedProblem::solveModelIntAux_D(PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
                                   InputPaths &inputPaths, float availableTime, float preObj, const PDualAuxSolver &DualAuxSolver_) {
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
            myTools::myException::throwError("Failed to optimize the MP");
            Cplex_.clearModel();
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
                        routeSolutionIndex_.push_back(static_cast<int>(r));
            //            pInst->vehicles_[compRoutes_[r]->vehicleID_]->setCurrentRoute(compRoutes_[r]);
            //            DualAuxSolver_->routeConst_.add(DualAuxSolver_->routeExpr_[r] - DualAuxSolver_->epsilonVar_[r] <= compRoutes_[r]->totalDelay_);
            //            DualAuxSolver_->routeConst_.add(DualAuxSolver_->routeExpr_[r] + DualAuxSolver_->epsilonVar_[r] >= compRoutes_[r]->totalDelay_);
                        DualAuxSolver_->routeConst_.add(DualAuxSolver_->routeExpr_[r] + DualAuxSolver_->epsilonVar_[r] == compRoutes_[r]->totalDelay_);
                    }
                    else {
                        DualAuxSolver_->routeConst_.add(DualAuxSolver_->routeExpr_[r] <= compRoutes_[r]->totalDelay_);
                        DualAuxSolver_->epsilonVar_[r].setUB(0.0);
                    }
                }

                for (IloInt i = zVal.getSize() - 1; i >= 0; --i) {
                    if (zVal[i] > 0.5) {
                        zSolution.push_back(pInst->nameToRequest_[zVar_[i].getName()]);
             //           DualAuxSolver_->zConst_.add(DualAuxSolver_->zExpr_[zSolution.back()->taskIndex_] - DualAuxSolver_->deltaVar_[i]<= zSolution.back()->penalty_);
             //           DualAuxSolver_->zConst_.add(DualAuxSolver_->zExpr_[zSolution.back()->taskIndex_] + DualAuxSolver_->deltaVar_[i]>= zSolution.back()->penalty_);
                        DualAuxSolver_->zConst_.add(DualAuxSolver_->zExpr_[zSolution.back()->taskIndex_] + DualAuxSolver_->deltaVar_[i] == zSolution.back()->penalty_);
                    }
                    else {
                        DualAuxSolver_->zConst_.add(DualAuxSolver_->zExpr_[pInst->nameToRequest_[
                            zVar_[i].getName()]->taskIndex_] <= pInst->nameToRequest_[zVar_[i].getName()]->penalty_);
                        DualAuxSolver_->deltaVar_[i].setUB(0.0);
                    }
                }
                DualAuxSolver_->solveModel(pInst, inputPaths);
            }
        }
        Cplex_.clearModel();
    }
    catch (IloException& e) {
        std::cout << "Error occurred in ReducedProblem::solveModelInt at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}

/*if (!routeSolutionIndex_.empty()) {
      IloNumVarArray startVar(env_);
      IloNumArray startVal(env_);
      for (int r = 0; r < routeSolutionIndex_.size(); ++r) {
          startVar.add(routeVar_[routeSolutionIndex_[r]]);
          startVal.add(1);
      }
      Cplex_.addMIPStart(startVar, startVal, IloCplex::MIPStartAuto, "m1");
      startVal.end();
      startVar.end();
  }*/


void ReducedProblem::solveLPDual(const PInstance &pInst, const InputPaths &inputPaths) {
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
        std::cout << "Error occurred in ReducedProblem::solveModelLP at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}
