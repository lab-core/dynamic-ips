//
// Created by Ella on 2021-11-10.
//

#include "ReducedProblem.h"

//---------------------------------------------------------------------------------------------
//  Reduced Problem class
//  Build and solve the Reduced problem of the ISUD
//---------------------------------------------------------------------------------------------

// Constructor and Destructor
ReducedProblem::ReducedProblem() : MasterModeler() {

    // defining variable
    zVar_ = IloNumVarArray(env_, 0.0, 0.0, IloInfinity,ILOINT);
    routeVar_ = IloNumVarArray(env_, 0.0, 0.0, IloInfinity,ILOINT);
    compRoutes_.clear();
    zLb_ = IloNumArray(env_);
    rLb_ = IloNumArray(env_);
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
    MasterModeler::addRouteVarInt(routeVar_, newRoute, POSITIVE, pInst);
    rLb_.add(0.0);
    compRoutes_.push_back(newRoute);
    newRoute->mpAdded_ = true;
}

void ReducedProblem::addRouteVars(std::vector<PRoute> &newRoutes) {
    MasterModeler::addRouteVars(routeVar_, newRoutes, POSITIVE);
    for (auto & newRoute : newRoutes)
        compRoutes_.push_back(newRoute);
}

// this function adds zVar to the model used for the routes that served only one request
void ReducedProblem:: addZVars(std::vector<PRequest> &requests) {
    MasterModeler::addZVars(zVar_, requests, POSITIVE);
}

void ReducedProblem:: addZVar(PRequest &request) {
    MasterModeler::addZVarInt(zVar_, request, POSITIVE);
    zLb_.add(0.0);
}

// this function add one route at each iteration of the algorithm during one epoch
void ReducedProblem::updateModel(PInstance &pInst, std::vector<PRoute> &routeSolution) {
    if (routesToAdd_.empty()) {
        std::cout << "There is no route to be added" << std::endl;
        throw myTools::myException("The input route is empty, No new column is passed to be added", __LINE__);
    }

    // add the new compatible column to the model
    for (auto & routeObj : routesToAdd_) {
        addRouteVar(routeObj, pInst);
    }

    // add compatible z variables
    // just z variables related to requests that are served in routes with one request are compatible
    /*for (int r = 0; r < routeSolution.size(); ++r) {
        if (routeSolution[r]->routeRequests_.size() == 1) {
            addZVarInt(routeSolution[r]->routeNodes_[1]->related_Request_);
        }
    }*/
}


// this function build the model at the start of each epoch
void ReducedProblem::buildModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                                int nbVehicles) {

    // model initialization (defining empty set of constraints and adding objective)
//    ResetRPModel();
    int rhs = 1;
    MasterModeler::initializeModel(pInst, rhs, nbVehicles);

    // adding request columns (z variables)
//    addZVars(zSolution);
    for (auto & zSol : zSolution)
        addZVar(zSol);

    // adding route solution columns
 //   addRouteVars(routeSolution);
    for (auto & routeSol : routeSolution){
        if (pInst->vehicles_[routeSol->vehicleID_]->vehicleIndex_ > -1)
            addRouteVar(routeSol, pInst);
    }



    //adding new route variables
    for (auto & routeObj : routesToAdd_) {
        if (pInst->vehicles_[routeObj->vehicleID_]->vehicleIndex_ > -1)
            addRouteVar(routeObj, pInst);
    }
//    env_.out() << Model_;
}

void ReducedProblem::solveModelLP(PInstance &pInst, InputPaths &inputPaths) {
    try {
        Model_.add(requestConst_);
        Model_.add(vehicleConst_);
        Model_.add(objFunction_);

        IloConversion convZ = IloConversion(env_, zVar_, ILOFLOAT);
        IloConversion convR = IloConversion(env_, routeVar_, ILOFLOAT);

        Model_.add(convZ);
        Model_.add(convR);

        std::ofstream logFile(inputPaths.getOutputCplexLog(), std::ofstream::app);
        logFile << "----------------------- LMP ------------------------"<< std::endl;
        std::streambuf* coutBuffer = std::cout.rdbuf();
        std::cout.rdbuf(logFile.rdbuf());

        Cplex_ = IloCplex(Model_);
        Cplex_.setParam(IloCplex::Param::Threads, pInst->parameters_->nbThreads_);
        Cplex_.setParam(IloCplex::Param::Preprocessing::Presolve, 0);

        solveTime_->start();
        Cplex_.solve();
        solveTime_->stop();

        objValue_ = Cplex_.getObjValue();
        // getting dual values
        requestDuals_.clear();
        vehicleDuals_.clear();

        // define dual container size
        requestDuals_ = IloNumArray(env_, pInst->nbRequests_);
        vehicleDuals_ = IloNumArray(env_, pInst->nbVehicles_);

        for (auto &requestObj: pInst->requests_) {
            int rowIndex = requestObj->taskIndex_;
            requestDuals_[rowIndex] = Cplex_.getDual(requestConst_[rowIndex]);
            requestObj->dual_ = requestDuals_[rowIndex];
            /*if (requestObj->CPDual_ > 0 && requestObj->dual_!= requestObj->CPDual_)
                std::cout << "request " << requestObj->getRequestId() << " dual == " << requestObj->CPDual_ << " --> " <<  requestObj->dual_ << std::endl;*/
            requestObj->CPDual_ = requestDuals_[rowIndex];
        }

        for (auto &vehicleObj: pInst->vehicles_) {
            if (vehicleObj->vehicleIndex_ > -1) {
                int index = pInst->vehicles_[vehicleObj->vehicleID_]->vehicleIndex_;
                vehicleDuals_[index] = Cplex_.getDual(vehicleConst_[index]);
                vehicleObj->dual_ = vehicleDuals_[index];
                vehicleObj->CPDual_ = vehicleDuals_[index];
            }
            else {
                vehicleObj->dual_ = 0;
                vehicleObj->CPDual_ = 0;
            }
        }
        convR.end();
        convZ.end();
        std::cout.rdbuf(coutBuffer);
        logFile.close();
        Cplex_.clearModel();
    }
    catch (IloException& e) {
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}

void ReducedProblem::solveModelInt(PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
                                   InputPaths &inputPaths, float availableTime, double preObj) {
    try {
        Model_.add(requestConst_);
        Model_.add(vehicleConst_);
        Model_.add(objFunction_);

        IloConversion convZ = IloConversion(env_, zVar_, ILOINT);
        IloConversion convR = IloConversion(env_, routeVar_, ILOINT);

        Model_.add(convZ);
        Model_.add(convR);

        Cplex_ = IloCplex(Model_);

        std::ofstream logFile(inputPaths.getOutputCplexLog(), std::ofstream::app);
        logFile << "----------------------- MP ------------------------"<< std::endl;
        std::streambuf* coutBuffer = std::cout.rdbuf();
        std::cout.rdbuf(logFile.rdbuf());

        Cplex_.setParam(IloCplex::Param::Threads, pInst->parameters_->nbThreads_);
        Cplex_.setParam(IloCplex::Param::Preprocessing::Presolve, 0);
        if (pInst->parameters_->MIPGap_ > 0.0001)
            Cplex_.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, pInst->parameters_->MIPGap_);
        Cplex_.setParam(IloCplex::Param::TimeLimit, availableTime);

        solveTime_->start();
        if (!Cplex_.solve()) {
            solveTime_->stop();
            std::cout << "Failed to optimize the MP" << std::endl;
        }
        else {
            solveTime_->stop();
            if (Cplex_.getObjValue() <= preObj) {
                objValue_ = Cplex_.getObjValue();

                // saving the result and remove out of base variables
                zSolution.clear();
                routeSolution.clear();

                IloNumArray zVal(env_);
                IloNumArray routeVal(env_);

                Cplex_.getValues(zVal, zVar_);
                Cplex_.getValues(routeVal, routeVar_);


                for (int r = (int) routeVal.getSize() - 1; r >= 0; --r) {
                    if (routeVal[r] > 0.9) {
                        routeSolution.push_back(compRoutes_[r]);
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
        convR.end();
        convZ.end();
        std::cout.rdbuf(coutBuffer);
        logFile.close();
        Cplex_.clearModel();
    }
    catch (IloException& e) {
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}

void ReducedProblem::solveModelLPInt(PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
                                     InputPaths &inputPaths, float availableTime, double preObj) {
    try {

        // Solve the Linear Relaxation
        Model_.add(requestConst_);
        Model_.add(vehicleConst_);
        Model_.add(objFunction_);

        IloConversion convZ = IloConversion(env_, zVar_, ILOFLOAT);
        IloConversion convR = IloConversion(env_, routeVar_, ILOFLOAT);

        Model_.add(convZ);
        Model_.add(convR);

        std::ofstream logFile(inputPaths.getOutputCplexLog(), std::ofstream::app);
        logFile << "----------------------- LMP ------------------------"<< std::endl;
        std::streambuf* coutBuffer = std::cout.rdbuf();
        std::cout.rdbuf(logFile.rdbuf());

        Cplex_ = IloCplex(Model_);
        Cplex_.setParam(IloCplex::Param::Threads, pInst->parameters_->nbThreads_);
        Cplex_.setParam(IloCplex::Param::Preprocessing::Presolve, 0);

        solveTime_->start();
        Cplex_.solve();
//        std::cout.rdbuf(coutBuffer);
        solveTime_->stop();

        objValue_ = Cplex_.getObjValue();
        // getting dual values
        requestDuals_.clear();
        vehicleDuals_.clear();

        // define dual container size
        requestDuals_ = IloNumArray(env_, pInst->nbRequests_);
        vehicleDuals_ = IloNumArray(env_, pInst->nbVehicles_);

        for (auto &requestObj: pInst->requests_) {
            int rowIndex = requestObj->taskIndex_;
            requestDuals_[rowIndex] = Cplex_.getDual(requestConst_[rowIndex]);
            requestObj->dual_ = requestDuals_[rowIndex];
            requestObj->CPDual_ = requestDuals_[rowIndex];
        }

        for (auto &vehicleObj: pInst->vehicles_) {
            if (vehicleObj->vehicleIndex_ > -1) {
                int index = pInst->vehicles_[vehicleObj->vehicleID_]->vehicleIndex_;
                vehicleDuals_[index] = Cplex_.getDual(vehicleConst_[index]);
                vehicleObj->dual_ = vehicleDuals_[index];
                vehicleObj->CPDual_ = vehicleDuals_[index];
            }
            else {
                vehicleObj->dual_ = 0;
                vehicleObj->CPDual_ = 0;
            }
        }
        convR.end();
        convZ.end();

        // Convert to integer
        convZ = IloConversion(env_, zVar_, ILOINT);
        convR = IloConversion(env_, routeVar_, ILOINT);

        Model_.add(convZ);
        Model_.add(convR);
        logFile << "----------------------- MP ------------------------"<< std::endl;
//        std::cout.rdbuf(logFile.rdbuf());
        if (pInst->parameters_->MIPGap_ > 0.0001)
            Cplex_.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, pInst->parameters_->MIPGap_);
        Cplex_.setParam(IloCplex::Param::TimeLimit, availableTime);
        solveTime_->start();
        if (!Cplex_.solve()) {
            solveTime_->stop();
            std::cout << "Failed to optimize the MP" << std::endl;
        }
        else {
            solveTime_->stop();
            if (Cplex_.getObjValue() <= preObj) {
                objValue_ = Cplex_.getObjValue();

                // saving the result and remove out of base variables
                zSolution.clear();
                routeSolution.clear();

                IloNumArray zVal(env_);
                IloNumArray routeVal(env_);

                Cplex_.getValues(zVal, zVar_);
                Cplex_.getValues(routeVal, routeVar_);


                for (int r = (int) routeVal.getSize() - 1; r >= 0; --r) {
                    if (routeVal[r] > 0.9) {
                        routeSolution.push_back(compRoutes_[r]);
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

        convR.end();
        convZ.end();
        std::cout.rdbuf(coutBuffer);
        logFile.close();

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
    repStr << MasterModeler::toString();
    return repStr.str();
}

// function to check whether the route is repeated before
bool ReducedProblem::isColumnRepeat(vector<PRoute> &routeSet, PRoute &newRoute,
                                    std::map<unsigned int, int> &requestToOrder) {
    Eigen::MatrixXd newCol = Eigen::MatrixXd::Zero((signed) requestToOrder.size(), 1);

    for (auto & requestObj : newRoute->routeRequests_)
        newCol(requestObj->taskIndex_, 0) = 1;

    for (auto & routeObj : routeSet) {
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero((signed) requestToOrder.size(), 1);
        for (auto & requestObj : routeObj->routeRequests_) {
            A(requestObj->taskIndex_, 0) = 1;
        }
        if (newCol == A)
            return true;
    }
    return false;
}

void ReducedProblem::restartRP() {
    zVar_ = IloNumVarArray(env_, 0.0, 0.0, IloInfinity,ILOINT);
    routeVar_ = IloNumVarArray(env_, 0.0, 0.0, IloInfinity,ILOINT);
}

void ReducedProblem::solveModelIntAux(PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
                                      InputPaths &inputPaths, float availableTime, double preObj) {
    try {
        Model_.add(requestConst_);
        Model_.add(vehicleConst_);
        Model_.add(objFunction_);

        IloConversion convZ = IloConversion(env_, zVar_, ILOINT);
        IloConversion convR = IloConversion(env_, routeVar_, ILOINT);
        Model_.add(convZ);
        Model_.add(convR);

        Cplex_ = IloCplex(Model_);
        std::ofstream logFile(inputPaths.getOutputCplexLog(), std::ofstream::app);
        logFile << "----------------------- MP ------------------------"<< std::endl;
        std::streambuf* coutBuffer = std::cout.rdbuf();
        std::cout.rdbuf(logFile.rdbuf());

        Cplex_.setParam(IloCplex::Param::Threads, pInst->parameters_->nbThreads_);
        Cplex_.setParam(IloCplex::Param::Preprocessing::Presolve, 0);
        if (pInst->parameters_->MIPGap_ > 0.0001)
            Cplex_.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, pInst->parameters_->MIPGap_);
        Cplex_.setParam(IloCplex::Param::TimeLimit, availableTime);

        solveTime_->start();
        if (!Cplex_.solve()) {
            env_.out() << Model_ << std::endl;
            solveTime_->stop();
            std::cout << "Failed to optimize the MP" << std::endl;
        }
        else {
            solveTime_->stop();
            if (Cplex_.getObjValue() <= preObj) {
                objValue_ = Cplex_.getObjValue();

                // saving the result and remove out of base variables
                zSolution.clear();
                routeSolution.clear();

                IloNumArray zVal(env_);
                IloNumArray routeVal(env_);

                Cplex_.getValues(zVal, zVar_);
                Cplex_.getValues(routeVal, routeVar_);

                // change objective to maximize
                objFunction_.setSense(IloObjective::Maximize);

                convR.end();
                convZ.end();
                // Convert to integer
                convZ = IloConversion(env_, zVar_, ILOFLOAT);
                convR = IloConversion(env_, routeVar_, ILOFLOAT);
                Model_.add(convZ);
                Model_.add(convR);



                for (int r = (int) routeVal.getSize() - 1; r >= 0; --r) {
                    if (routeVal[r] > 0.9) {
                        routeSolution.push_back(compRoutes_[r]);
                        pInst->vehicles_[compRoutes_[r]->vehicleID_]->setCurrentRoute(compRoutes_[r]);
                        routeVar_[r].setBounds(-IloInfinity, 1.0);
                    }
                    else
                        routeVar_[r].setBounds(-IloInfinity, 0.0);
                }
                for (int i = (int) zVal.getSize() - 1; i >= 0; --i) {
                    if (zVal[i] > 0.9) {
                        zSolution.push_back(pInst->nameToRequest_[zVar_[i].getName()]);
                        zVar_[i].setBounds(-IloInfinity, IloInfinity);
                    }
                    else
                        zVar_[i].setBounds(-IloInfinity, 0.0);
                }

                IloNumArray requestRHS(env_);
                IloNumArray vehicleRHS(env_);

                createIloNumArray (requestRHS, orderToRequest_.size(), 0.0);
                createIloNumArray (vehicleRHS, pInst->nbVehicles_, 0.0);
                requestConst_.setBounds(requestRHS, requestRHS);
                vehicleConst_.setBounds(vehicleRHS, vehicleRHS);

                solveTime_->start();
                Cplex_.solve();
                solveTime_->stop();

                auxObjValue_ = Cplex_.getObjValue();
                // getting dual values
                requestDuals_.clear();
                vehicleDuals_.clear();

                // define dual container size
                requestDuals_ = IloNumArray(env_, pInst->nbRequests_);
                vehicleDuals_ = IloNumArray(env_, pInst->nbVehicles_);
//                std::cout << " ===================" << std::endl;
                for (auto &requestObj: pInst->requests_) {
                    int rowIndex = requestObj->taskIndex_;
                    requestDuals_[rowIndex] = Cplex_.getDual(requestConst_[rowIndex]);
                    requestObj->dual_ = requestDuals_[rowIndex];
                    /*if (requestObj->CPDual_ > 0 && requestObj->dual_!= requestObj->CPDual_)
                        std::cout << "request " << requestObj->getRequestId() << " dual == " << requestObj->CPDual_ << " --> " <<  requestObj->dual_ << std::endl;*/
                    requestObj->CPDual_ = requestDuals_[rowIndex];
                }

                //               std::cout << " ----------------" << std::endl;

                for (auto &vehicleObj: pInst->vehicles_) {
                    if (vehicleObj->vehicleIndex_ > -1) {
                        int index = pInst->vehicles_[vehicleObj->vehicleID_]->vehicleIndex_;
                        vehicleDuals_[index] = Cplex_.getDual(vehicleConst_[index]);
                        vehicleObj->dual_ = vehicleDuals_[index];
                        vehicleObj->CPDual_ = vehicleDuals_[index];
                    }
                    else {
                        vehicleObj->dual_ = 0;
                        vehicleObj->CPDual_ = 0;
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
        convR.end();
        convZ.end();

        std::cout.rdbuf(coutBuffer);
        logFile.close();

        Cplex_.clearModel();
    }
    catch (IloException& e) {
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }

}











