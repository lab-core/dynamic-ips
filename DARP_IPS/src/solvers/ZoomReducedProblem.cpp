//
// Created by Ella on 2/28/2022.
//

#include "ZoomReducedProblem.h"



void ZoomReducedProblem::updateModel(PInstance &pInst, vector<PRequest> &fractionalZ) {
//    env_.out() << Model_;
    if (routesToAdd_.empty()) {
        std::cout << "There is no route to be added" << std::endl;
//        throw myTools::myException("The input route is empty, No new column is passed to be added", __LINE__);
    }

    // add the new compatible column to the model
    for (auto routeObj : routesToAdd_) {
        if (pInst->vehicles_[routeObj->vehicleID_]->vehicleIndex_ > -1)
            addRouteVar(routeObj, pInst);
    }

 //   addZVars(fractionalZ);
    for (auto & zRequest : fractionalZ)
        addZVar(zRequest);
}

void ZoomReducedProblem::solveModel(PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
                                    InputPaths &inputPaths, int availableTime, double preObj) {
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
        logFile << "----------------------- RP ------------------------"<< std::endl;
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
            std::cout << "Failed to optimize the RP" << std::endl;
        }

        else {
            solveTime_->stop();

            if (Cplex_.getObjValue() <= preObj) {

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
                std::cout << "# from " << pInst->nbRequests_ << " request, " << pInst->nbRequests_ - zSolution.size()
                          << " are selected to served." << std::endl;

                /*if (routeSolution.size() != pInst->nbVehicles_)
                    myTools::throwError("Number of routes in the solution does not match with the vehicles!!!");*/
            }
        }
        Cplex_.clearModel();
        std::cout.rdbuf(coutBuffer);
        logFile.close();
    }
    catch (IloException& e) {
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}

/*void ZoomReducedProblem::solveModelDualLP(PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
                                        InputPaths &inputPaths, int availableTime, double preObj) {
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
        logFile << "----------------------- LRP ------------------------"<< std::endl;
        std::streambuf* coutBuffer = std::cout.rdbuf();
        std::cout.rdbuf(logFile.rdbuf());

        Cplex_ = IloCplex(Model_);
        Cplex_.setParam(IloCplex::Param::Threads, pInst->parameters_->nbThreads_);
        Cplex_.setParam(IloCplex::Param::Preprocessing::Presolve, 0);

        solveTime_->start();
        if (!Cplex_.solve()) {
            solveTime_->stop();
            std::cout << "Failed to optimize the RP" << std::endl;
        }
        else {
            solveTime_->stop();

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
                requestObj->InitialDual_ = requestDuals_[rowIndex];
            }

            for (auto &vehicleObj: pInst->vehicles_) {
                if (vehicleObj->vehicleIndex_ > -1) {
                    int index = pInst->vehicles_[vehicleObj->vehicleID_]->vehicleIndex_;
                    vehicleDuals_[index] = Cplex_.getDual(vehicleConst_[index]);
                    vehicleObj->dual_ = vehicleDuals_[index];
                    vehicleObj->InitialDual_ = vehicleDuals_[index];
                }
                else {
                    vehicleObj->dual_ = 0;
                    vehicleObj->InitialDual_ = 0;
                }
            }
            convR.end();
            convZ.end();

            // Convert to integer
            convZ = IloConversion(env_, zVar_, ILOINT);
            convR = IloConversion(env_, routeVar_, ILOINT);

            Model_.add(convZ);
            Model_.add(convR);
            logFile << "----------------------- RP ------------------------" << std::endl;
            if (pInst->parameters_->MIPGap_ > 0.0001)
                Cplex_.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, pInst->parameters_->MIPGap_);
            Cplex_.setParam(IloCplex::Param::TimeLimit, availableTime);
            solveTime_->start();
            if (!Cplex_.solve()) {
                solveTime_->stop();
                std::cout << "Failed to optimize the RP" << std::endl;
            }
            else{
                solveTime_->stop();
                if (Cplex_.getObjValue() <= preObj) {

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


                    *//*if (routeSolution.size() != pInst->nbVehicles_)
                        myTools::throwError("Number of routes in the solution does not match with the vehicles!!!");*//*
                }
            }
        }
        std::cout.rdbuf(coutBuffer);
        logFile.close();

        convR.end();
        convZ.end();
        Cplex_.clearModel();

    }
    catch (IloException& e) {
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}*/

std::string ZoomReducedProblem::toString() const {
    std::stringstream repStr;
    repStr << std::endl;
    repStr << "# ====================  REDUCED PROBLEM SOLVED INTEGER MODE ==================== " << std::endl;
    repStr << MasterModeler::toString();
    return repStr.str();
}

