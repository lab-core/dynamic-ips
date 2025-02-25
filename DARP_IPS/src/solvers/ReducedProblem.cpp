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
    compRoutes_.clear();
}


// this function initialized the model and define empty set of constraints
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

// this function add one route at each iteration of the algorithm during one epoch
void ReducedProblem::updateModel(PInstance &pInst) {
    // add the new compatible column to the model
    for (auto & routeObj : routesToAdd_) {
        if (pInst->vehicles_[routeObj->vehicleID_]->vehicleIndex_ > -1)
            addRouteVar(routeObj, pInst);
    }
}


// this function build the model at the start of each epoch
void ReducedProblem::buildModel(PInstance &pInst, std::vector<PRoute> &routeSolution,
                                int nbVehicles) {
    // model initialization (defining empty set of constraints and adding objective)
    int rhs = 1;
    initializeModel(pInst, rhs, nbVehicles);

    // adding request columns (z variables)
    for (auto & zSol : pInst->requests_)
        addZVarFloat(zVar_, zSol, POSITIVE);

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
    env_.out() << Model_ << std::endl;
    Model_.add(requestConst_);
    Model_.add(vehicleConst_);
    Model_.add(objFunction_);
    env_.out() << Model_ << std::endl;
}

void ReducedProblem::solveModelLP(PInstance &pInst, InputPaths &inputPaths) {
    try {
        myTools::CoutRedirector redirector(inputPaths.getOutputCplexLog(), "LMP");

        Cplex_.extract(Model_);
        Cplex_.setParam(IloCplex::Param::Threads, pInst->parameters_->nbThreads_);
        Cplex_.setParam(IloCplex::Param::Preprocessing::Presolve, 0);
        Cplex_.setParam(IloCplex::Param::RootAlgorithm, 2);
        solveTime_->start();
        if (!Cplex_.solve()) {
            solveTime_->stop();
            std::cout << "Failed to optimize the LMP" << std::endl;
            Cplex_.clearModel();
            myTools::myException::throwError("Failed to optimize the LMP");
        }
        else {
            solveTime_->stop();

            objValue_ = Cplex_.getObjValue();
            // getting dual values
            requestDuals_.clear();
            vehicleDuals_.clear();

            Cplex_.getDuals(requestDuals_, requestConst_);
            Cplex_.getDuals(vehicleDuals_, vehicleConst_);

            for (auto &requestObj: pInst->requests_) {
                requestObj->dual_ = requestDuals_[requestObj->taskIndex_];
                requestObj->InitialDual_ = requestObj->dual_;
            }

            for (auto &vehicleObj: pInst->vehicles_) {
                int index = vehicleObj->vehicleIndex_;
                if (index > -1) {
                    vehicleObj->dual_ = vehicleDuals_[index];
                    vehicleObj->InitialDual_ = vehicleObj->dual_;
                }
                else {
                    vehicleObj->dual_ = 0;
                    vehicleObj->InitialDual_ = 0;
                }
            }
        }
        Cplex_.clearModel();
    }
    catch (IloException& e) {
        std::cout << "Error occurred in ReducedProblem::solveModelLP at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}

void ReducedProblem::solveModelInt(PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
                                   InputPaths &inputPaths, float availableTime, float preObj) {
    try {

        IloConversion convZ(env_, zVar_, ILOINT);
        IloConversion convR(env_, routeVar_, ILOINT);
        Model_.add(convZ);
        Model_.add(convR);

        Cplex_.extract(Model_);

        myTools::CoutRedirector redirector(inputPaths.getOutputCplexLog(), "MP");
        setParameters(pInst, availableTime);

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
                objValue_ = Cplex_.getObjValue();

                // saving the result and remove out of base variables
                zSolution.clear();
                routeSolution.clear();
                IloNumArray zVal(env_);
                IloNumArray routeVal(env_);
                Cplex_.getValues(zVal, zVar_);
                Cplex_.getValues(routeVal, routeVar_);
                convR.end();
                convZ.end();


                for (int r = (int) routeVal.getSize() - 1; r >= 0; --r) {
                    if (routeVal[r] > 0.5) {
                        routeSolution.push_back(compRoutes_[r]);
                        routeSolutionIndex_.push_back(r);
                        pInst->vehicles_[compRoutes_[r]->vehicleID_]->setCurrentRoute(compRoutes_[r]);
                    }
                }

                for (int i = (int) zVal.getSize() - 1; i >= 0; --i) {
                    if (zVal[i] > 0.5) {
                        zSolution.push_back(pInst->nameToRequest_[zVar_[i].getName()]);
                    }
                }

            }
        }
        Cplex_.clearModel();
    }
    catch (IloException& e) {
        std::cout << "Error occurred in ReducedProblem::solveModelInt at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}

void ReducedProblem::solveModelLPInt(PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
                                     InputPaths &inputPaths, float availableTime, float preObj) {
    try {

        // Solve the Linear Relaxation
        myTools::CoutRedirector redirector(inputPaths.getOutputCplexLog(), "MP");

        Cplex_.extract(Model_);
        Cplex_.setParam(IloCplex::Param::Threads, pInst->parameters_->nbThreads_);
        Cplex_.setParam(IloCplex::Param::Preprocessing::Presolve, 0);
        Cplex_.setParam(IloCplex::Param::RootAlgorithm, 2);
        solveTime_->start();
        if (!Cplex_.solve()) {
            solveTime_->stop();
            std::cout << "Failed to optimize the LMP" << std::endl;
        }
        else {
            solveTime_->stop();

            objValue_ = Cplex_.getObjValue();
            // getting dual values
            requestDuals_.clear();
            vehicleDuals_.clear();

            Cplex_.getDuals(requestDuals_, requestConst_);
            Cplex_.getDuals(vehicleDuals_, vehicleConst_);

            for (auto &requestObj: pInst->requests_) {
                requestObj->dual_ = requestDuals_[requestObj->taskIndex_];
                requestObj->InitialDual_ = requestObj->dual_;
            }

            for (auto &vehicleObj: pInst->vehicles_) {
                int index = vehicleObj->vehicleIndex_;
                if (index > -1) {
                    vehicleObj->dual_ = vehicleDuals_[index];
                    vehicleObj->InitialDual_ = vehicleObj->dual_;
                }
                else {
                    vehicleObj->dual_ = 0;
                    vehicleObj->InitialDual_ = 0;
                }
            }

            // Convert to integer
            IloConversion convZ(env_, zVar_, ILOINT);
            IloConversion convR(env_, routeVar_, ILOINT);

            Model_.add(convZ);
            Model_.add(convR);
            Cplex_.extract(Model_);
            std::cout << "----------------------- MP ------------------------"<< std::endl;
            setParameters(pInst, availableTime);


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
                    objValue_ = Cplex_.getObjValue();

                    // saving the result and remove out of base variables
                    zSolution.clear();
                    routeSolution.clear();
                    IloNumArray zVal(env_);
                    IloNumArray routeVal(env_);
                    Cplex_.getValues(zVal, zVar_);
                    Cplex_.getValues(routeVal, routeVar_);
                    convR.end();
                    convZ.end();

                    for (int r = (int) routeVal.getSize() - 1; r >= 0; --r) {
                        if (routeVal[r] > 0.9) {
                            routeSolution.push_back(compRoutes_[r]);
                            routeSolutionIndex_.push_back(r);
                            pInst->vehicles_[compRoutes_[r]->vehicleID_]->setCurrentRoute(compRoutes_[r]);
                        }
                    }

                    for (int i = (int) zVal.getSize() - 1; i >= 0; --i) {
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

void ReducedProblem::solveModelIntAux_P(PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
                                      InputPaths &inputPaths, float availableTime, float preObj, float lpObj) {
    try {

        IloConversion convZ (env_, zVar_, ILOINT);
        IloConversion convR (env_, routeVar_, ILOINT);
        Model_.add(convZ);
        Model_.add(convR);

        Cplex_.extract(Model_);
        myTools::CoutRedirector redirector(inputPaths.getOutputCplexLog(), "MP");
        setParameters(pInst, availableTime);

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
                objValue_ = Cplex_.getObjValue();

                // saving the result and remove out of base variables
                zSolution.clear();
                routeSolution.clear();
                IloNumArray zVal(env_);
                IloNumArray routeVal(env_);
                Cplex_.getValues(zVal, zVar_);
                Cplex_.getValues(routeVal, routeVar_);
                convR.end();
                convZ.end();

                for (int r = (int) routeVal.getSize() - 1; r >= 0; --r) {
                    if (routeVal[r] > 0.5) {
                        routeSolution.push_back(compRoutes_[r]);
                        pInst->vehicles_[compRoutes_[r]->vehicleID_]->setCurrentRoute(compRoutes_[r]);
                    }
                }
                for (int i = (int) zVal.getSize() - 1; i >= 0; --i) {
                    if (zVal[i] > 0.5)
                        zSolution.push_back(pInst->nameToRequest_[zVar_[i].getName()]);
                }
                auxObjValue_ = 0;

                if (lpObj < objValue_) {
                    for (int r = (int) routeVal.getSize() - 1; r >= 0; --r) {
                        if (routeVal[r] > 0.5) {
                            routeVar_[r].setBounds(-IloInfinity, 0.0);
                        }
                        else
                            routeVar_[r].setBounds(0.0, 1.0);
                    }
                    for (int i = (int) zVal.getSize() - 1; i >= 0; --i) {
                        if (zVal[i] > 0.5) {
                            zVar_[i].setBounds(-IloInfinity, 0.0);
                        }
                        else
                            zVar_[i].setBounds(0.0, 1.0);
                    }

                    // change objective to maximize
                    objFunction_.setSense(IloObjective::Maximize);

                    IloNumArray requestRHS(env_);
                    IloNumArray vehicleRHS(env_);

                    createIloNumArray (requestRHS, nbRequestTask_, 0.0);
                    createIloNumArray (vehicleRHS, pInst->nbVehicles_, 0.0);
                    requestConst_.setBounds(requestRHS, requestRHS);
                    vehicleConst_.setBounds(vehicleRHS, vehicleRHS);

                    Cplex_.extract(Model_);

                    Cplex_.setParam(IloCplex::Param::RootAlgorithm, 2);
                    solveTime_->start();
                    if (!Cplex_.solve()) {
                        env_.out() << Model_ << std::endl;
                        solveTime_->stop();
                        myTools::myException::throwError("Failed to optimize the Aux MP");
                    }
                    else {
                        solveTime_->stop();

                        auxObjValue_ = Cplex_.getObjValue();
                        // getting dual values
                        requestDuals_.clear();
                        vehicleDuals_.clear();

                        Cplex_.getDuals(requestDuals_, requestConst_);
                        Cplex_.getDuals(vehicleDuals_, vehicleConst_);

                        for (auto &requestObj: pInst->requests_) {
                            requestObj->dual_ = requestDuals_[requestObj->taskIndex_];
                            /*if (requestObj->InitialDual_ != requestObj->dual_)
                                std::cout << requestObj->InitialDual_ << "  :  " << requestObj->dual_ << std::endl;*/
                        }


                        for (auto &vehicleObj: pInst->vehicles_) {
                            if (vehicleObj->vehicleIndex_ > -1) {
                                int index = pInst->vehicles_[vehicleObj->vehicleID_]->vehicleIndex_;
                                vehicleObj->dual_ = vehicleDuals_[index];
                                /*if (vehicleObj->InitialDual_ != vehicleObj->dual_)
                                     std::cout << vehicleObj->InitialDual_ << "  :  " << vehicleObj->dual_ << std::endl;*/
                            }
                            else {
                                vehicleObj->dual_ = 0;
                            }
                        }

                        // reset changes
                        objFunction_.setSense(IloObjective::Minimize);
                        for (int i = (int) zVal.getSize() - 1; i >= 0; --i)
                            zVar_[i].setBounds(0.0, IloInfinity);

                        for (int r = (int) routeVal.getSize() - 1; r >= 0; --r)
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

void ReducedProblem::solveModelInt_box(PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
                          InputPaths &inputPaths, float availableTime, float preObj, float lpObj) {
    try {

        IloConversion convZ (env_, zVar_, ILOINT);
        IloConversion convR (env_, routeVar_, ILOINT);
        Model_.add(convZ);
        Model_.add(convR);

        Cplex_.extract(Model_);
        myTools::CoutRedirector redirector(inputPaths.getOutputCplexLog(), "MP");
        setParameters(pInst, availableTime);

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
                objValue_ = Cplex_.getObjValue();

                // saving the result and remove out of base variables
                zSolution.clear();
                routeSolution.clear();
                IloNumArray zVal(env_);
                IloNumArray routeVal(env_);
                Cplex_.getValues(zVal, zVar_);
                Cplex_.getValues(routeVal, routeVar_);
                convR.end();
                convZ.end();

                for (int r = (int) routeVal.getSize() - 1; r >= 0; --r) {
                    if (routeVal[r] > 0.5) {
                        routeSolution.push_back(compRoutes_[r]);
                        pInst->vehicles_[compRoutes_[r]->vehicleID_]->setCurrentRoute(compRoutes_[r]);
                    }
                }
                for (int i = (int) zVal.getSize() - 1; i >= 0; --i) {
                    if (zVal[i] > 0.5)
                        zSolution.push_back(pInst->nameToRequest_[zVar_[i].getName()]);
                }
                auxObjValue_ = 0;

                for (int i = (int) zVal.getSize() - 1; i >= 0; --i) {
                    uVar_[i].setBounds(0.0, IloInfinity);
                }
                Cplex_.extract(Model_);

                Cplex_.setParam(IloCplex::Param::RootAlgorithm, 2);
                solveTime_->start();
                if (!Cplex_.solve()) {
                    env_.out() << Model_ << std::endl;
                    solveTime_->stop();
                    myTools::myException::throwError("Failed to optimize the Aux MP");
                }
                else {
                    solveTime_->stop();

                    auxObjValue_ = Cplex_.getObjValue();
                    // getting dual values
                    requestDuals_.clear();
                    vehicleDuals_.clear();

                    IloNumArray uVal(env_);
                    Cplex_.getValues(uVal, uVar_);

                    Cplex_.getDuals(requestDuals_, requestConst_);
                    Cplex_.getDuals(vehicleDuals_, vehicleConst_);

                    for (int i = (int) zVal.getSize() - 1; i >= 0; --i) {
                        if (uVal[i] > 0.5)
                            std::cout << "U[" << zVar_[i].getName()  << "] : " << uVal[i] << std::endl;
                    }

                    for (auto &requestObj: pInst->requests_) {
                        requestObj->dual_ = requestDuals_[requestObj->taskIndex_];
                        /*if (requestObj->InitialDual_ != requestObj->dual_)
                            std::cout << requestObj->InitialDual_ << "  :  " << requestObj->dual_ << std::endl;*/
                    }
                    std::cout << "======================================" << std::endl;


                    for (auto &vehicleObj: pInst->vehicles_) {
                        if (vehicleObj->vehicleIndex_ > -1) {
                            int index = pInst->vehicles_[vehicleObj->vehicleID_]->vehicleIndex_;
                            vehicleObj->dual_ = vehicleDuals_[index];
                            /*if (vehicleObj->InitialDual_ != vehicleObj->dual_)
                                 std::cout << vehicleObj->InitialDual_ << "  :  " << vehicleObj->dual_ << std::endl;*/
                        }
                        else {
                            vehicleObj->dual_ = 0;
                        }
                    }

                    for (int i = (int) zVal.getSize() - 1; i >= 0; --i)
                        uVar_[i].setBounds(0.0, 0.0);
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
                                   InputPaths &inputPaths, float availableTime, float preObj, PDualAuxSolver &DualAuxSolver_) {
    try {

        IloConversion convZ(env_, zVar_, ILOINT);
        IloConversion convR(env_, routeVar_, ILOINT);
        Model_.add(convZ);
        Model_.add(convR);

        Cplex_.extract(Model_);
        myTools::CoutRedirector redirector(inputPaths.getOutputCplexLog(), "MP");
        setParameters(pInst, availableTime);

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
                objValue_ = Cplex_.getObjValue();

                // saving the result and remove out of base variables
                zSolution.clear();
                routeSolution.clear();
                IloNumArray zVal(env_);
                IloNumArray routeVal(env_);
                Cplex_.getValues(zVal, zVar_);
                Cplex_.getValues(routeVal, routeVar_);
                convR.end();
                convZ.end();

                for (int r = (int) routeVal.getSize() - 1; r >= 0; --r) {
                    if (routeVal[r] > 0.5) {
                        routeSolution.push_back(compRoutes_[r]);
                        routeSolutionIndex_.push_back(r);
                        pInst->vehicles_[compRoutes_[r]->vehicleID_]->setCurrentRoute(compRoutes_[r]);
            //            DualAuxSolver_->routeConst_.add(DualAuxSolver_->routeExpr_[r] - DualAuxSolver_->epsilonVar_[r] <= compRoutes_[r]->totalDelay_);
            //            DualAuxSolver_->routeConst_.add(DualAuxSolver_->routeExpr_[r] + DualAuxSolver_->epsilonVar_[r] >= compRoutes_[r]->totalDelay_);
                        DualAuxSolver_->routeConst_.add(DualAuxSolver_->routeExpr_[r] + DualAuxSolver_->epsilonVar_[r] == compRoutes_[r]->totalDelay_);
                    }
                    else {
                        DualAuxSolver_->routeConst_.add(DualAuxSolver_->routeExpr_[r] <= compRoutes_[r]->totalDelay_);
                        DualAuxSolver_->epsilonVar_[r].setUB(0.0);
                    }
                }

                for (int i = (int) zVal.getSize() - 1; i >= 0; --i) {
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


