//
// Created by Ella on 2/28/2022.
//

#include "ZoomReducedProblem.h"



void ZoomReducedProblem::updateModel(PInstance &pInst, vector<PRequest> &fractionalZ) {
//    env_.out() << Model_;
    if (routesToAdd_.empty()) {
        std::cout << "There is no route to be added" << std::endl;
        throw myTools::myException("The input route is empty, No new column is passed to be added", __LINE__);
    }

    // add the new compatible column to the model
    for (auto routeObj : routesToAdd_) {
        addRouteVar(routeObj);
    }
//    ReducedProblem::addRouteVars(routesToAdd_);


 //   addZVars(fractionalZ);
    for (auto & zRequest : fractionalZ)
        addZVar(zRequest);
}
void ZoomReducedProblem::updateModelPartial(PInstance &pInst, vector<PRequest> &fractionalZ) {
//    env_.out() << Model_;
    if (routesToAdd_.empty()) {
        std::cout << "There is no route to be added" << std::endl;
        throw myTools::myException("The input route is empty, No new column is passed to be added", __LINE__);
    }

    // add the new compatible column to the model
    for (auto routeObj : routesToAdd_) {
        if (pInst->vehicles_[routeObj->vehicleID_]->vehicleIndex_ > -1)
            addRouteVarPartial(routeObj, pInst);
    }
//    ReducedProblem::addRouteVars(routesToAdd_);


    //   addZVars(fractionalZ);
    for (auto & zRequest : fractionalZ)
        addZVar(zRequest);
}

void ZoomReducedProblem::solveModel(PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
                                    InputPaths &inputPaths) {
    try {
        Model_.add(requestConst_);
        Model_.add(vehicleConst_);
        Model_.add(objFunction_);
        /*Model_.add(IloConversion(env_, zVar_, ILOINT));
        Model_.add(IloConversion(env_, routeVar_, ILOINT));*/

  //      std::cout << routeVar_[0].getType() << std::endl;
        Cplex_ = IloCplex(Model_);
        Cplex_.setParam(IloCplex::Param::Threads, pInst->parameters_->nbThreads_);
//        Cplex_.setParam(IloCplex::Param::MIP::Pool::Intensity, 1);
//        Cplex_.setParam(IloCplex::Param::TimeLimit, 5);
        Cplex_.setParam(IloCplex::Param::Preprocessing::Presolve, 0);

//        Cplex_.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, 0.3);
        std::ofstream logFile(inputPaths.getOutputCplexLog(), std::ofstream::app);
        logFile << "----------------------- RP ------------------------"<< std::endl;
        std::streambuf* coutBuffer = std::cout.rdbuf();
        std::cout.rdbuf(logFile.rdbuf());
        solveTime_->start();
        Cplex_.solve();
        std::cout.rdbuf(coutBuffer);
        logFile.close();
        solveTime_->stop();

        // printing solution status
 //       std::cout << toString();

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
                /*routeSolution.push_back(generatedRoutes[routeVar_[r].getName()]);
                pInst->vehicles_[generatedRoutes[routeVar_[r].getName()]->vehicleID_]->setCurrentRoute(
                        generatedRoutes[routeVar_[r].getName()]);*/
            }
        }

        for (int i = (int) zVal.getSize() - 1; i >= 0; --i) {
            if (zVal[i] > 0.9) {
                zSolution.push_back(pInst->nameToRequest_[zVar_[i].getName()]);
            }
        }
        std::cout << "# from " << pInst->nbRequests_ << " request, " << pInst->nbRequests_ - zSolution.size()
                  << " are selected to served." << std::endl;
        Cplex_.clearModel();
        if (routeSolution.size() != pInst->nbVehicles_)
            myTools::throwError("Number of routes in the solution does not match with the vehicles!!!");
    }
    catch (IloException& e) {
        std::cout << e << std::endl;
    }
}

void ZoomReducedProblem::solveModelDual(PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
                                        InputPaths &inputPaths) {
    try {
        Model_.add(requestConst_);
        Model_.add(vehicleConst_);
        Model_.add(objFunction_);

        Cplex_ = IloCplex(Model_);
        Cplex_.setParam(IloCplex::Param::Threads, pInst->parameters_->nbThreads_);
        Cplex_.setParam(IloCplex::Param::Preprocessing::Presolve, 0);
        std::ofstream logFile(inputPaths.getOutputCplexLog(), std::ofstream::app);
        logFile << "----------------------- RP ------------------------"<< std::endl;
        std::streambuf* coutBuffer = std::cout.rdbuf();
        std::cout.rdbuf(logFile.rdbuf());
        solveTime_->start();
        Cplex_.solve();
        std::cout.rdbuf(coutBuffer);
        logFile.close();
        solveTime_->stop();

 //       std::cout << "RP Objective value: " << Cplex_.getObjValue() << std::endl;

        // saving the result and remove out of base variables
        zSolution.clear();
        routeSolution.clear();

        IloNumArray zVal(env_);
        IloNumArray routeVal(env_);

        Cplex_.getValues(zVal, zVar_);
        Cplex_.getValues(routeVal, routeVar_);

        for (int r = (int) routeVal.getSize() - 1; r >= 0; --r) {
            if (routeVal[r] > 0.9) {
//                std::cout << compRoutes_[r]->getRouteId() << " : " << compRoutes_[r]->vehicleID_ << std::endl;
                routeSolution.push_back(compRoutes_[r]);
                pInst->vehicles_[compRoutes_[r]->vehicleID_]->setCurrentRoute(compRoutes_[r]);
            }
        }

        for (int i = (int) zVal.getSize() - 1; i >= 0; --i) {
            if (zVal[i] > 0.9) {
                zSolution.push_back(pInst->nameToRequest_[zVar_[i].getName()]);
            }
        }

        /*std::cout << "# from " << pInst->nbRequests_ << " request, " << pInst->nbRequests_ - zSolution.size()
                  << " are selected to served." << std::endl;*/

        if (routeSolution.size() != pInst->nbVehicles_)
            myTools::throwError("Number of routes in the solution does not match with the vehicles!!!");

        IloInt incomID = Cplex_.getIncumbentNode();
        // fixed the values on integer solution

        /*Model_.add(IloConversion(env_, zVar_, ILOFLOAT));
        Model_.add(IloConversion(env_, routeVar_, ILOFLOAT));*/
        Cplex_.solveFixed(incomID);
//        std::cout << "Linear RP Objective value: " << Cplex_.getObjValue() << std::endl;

        // getting dual values
        requestDuals_.clear();
        vehicleDuals_.clear();

        // define dual container size
        requestDuals_ = IloNumArray(env_, pInst->nbRequests_);
        vehicleDuals_ = IloNumArray(env_, pInst->nbVehicles_);

        for (auto &requestObj: pInst->requests_) {
            int rowIndex = requestObj->taskIndex_;
            requestDuals_[rowIndex] = Cplex_.getDual(requestConst_[rowIndex]);
//            std::cout << "request " << requestObj->getRequestId() << " dual == " << requestObj->dual_;
            requestObj->dual_ = requestDuals_[rowIndex];
            requestObj->CPDual_ = requestDuals_[rowIndex];
//            std::cout << " --> " << requestObj->dual_ << std:: endl;
        }

        for (auto &vehicleObj: pInst->vehicles_) {
            vehicleDuals_[vehicleObj->vehicleID_] = Cplex_.getDual(vehicleConst_[vehicleObj->vehicleID_]);
//            std::cout << "vehicle " << vehicleObj->vehicleID_ << " dual == " << vehicleObj->dual_;
            vehicleObj->dual_ = vehicleDuals_[vehicleObj->vehicleID_];
            vehicleObj->CPDual_ = vehicleDuals_[vehicleObj->vehicleID_];
//            std::cout << " --> " << vehicleObj->dual_ << std:: endl;
        }
        Cplex_.clearModel();
    }
    catch (IloException& e) {
        std::cout << e << std::endl;
    }
}

void ZoomReducedProblem::solveModelPartial(PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
                                           InputPaths &inputPaths) {
    try {
        Model_.add(requestConst_);
        Model_.add(vehicleConst_);
        Model_.add(objFunction_);

        Cplex_ = IloCplex(Model_);
        Cplex_.setParam(IloCplex::Param::Threads, pInst->parameters_->nbThreads_);
        Cplex_.setParam(IloCplex::Param::Preprocessing::Presolve, 0);
        std::ofstream logFile(inputPaths.getOutputCplexLog(), std::ofstream::app);
        logFile << "----------------------- RP ------------------------"<< std::endl;
        std::streambuf* coutBuffer = std::cout.rdbuf();
        std::cout.rdbuf(logFile.rdbuf());
        solveTime_->start();
        Cplex_.solve();
        std::cout.rdbuf(coutBuffer);
        logFile.close();
        solveTime_->stop();

        //       std::cout << "RP Objective value: " << Cplex_.getObjValue() << std::endl;

        // saving the result and remove out of base variables
        zSolution.clear();
        std::vector<int> indexes;
        for (int r = routeSolution.size()-1; r >=0; r--){
            if (pInst->vehicles_[routeSolution[r]->vehicleID_]->vehicleIndex_ > -1)
                indexes.push_back(r);
        }
        std::sort(indexes.begin(), indexes.end());
        for (int r = indexes.size()-1; r >=0; r--){
            routeSolution.erase(routeSolution.begin()+indexes[r]);
        }


        IloNumArray zVal(env_);
        IloNumArray routeVal(env_);

        Cplex_.getValues(zVal, zVar_);
        Cplex_.getValues(routeVal, routeVar_);

        for (int r = (int) routeVal.getSize() - 1; r >= 0; --r) {
            if (routeVal[r] > 0.9) {
 //               std::cout << compRoutes_[r]->getRouteId() << " : " << compRoutes_[r]->vehicleID_ << std::endl;
                routeSolution.push_back(compRoutes_[r]);
                pInst->vehicles_[compRoutes_[r]->vehicleID_]->setCurrentRoute(compRoutes_[r]);
            }
        }

        for (int i = (int) zVal.getSize() - 1; i >= 0; --i) {
            if (zVal[i] > 0.9) {
                zSolution.push_back(pInst->nameToRequest_[zVar_[i].getName()]);
            }
        }

        /*std::cout << "# from " << pInst->nbRequests_ << " request, " << pInst->nbRequests_ - zSolution.size()
                  << " are selected to served." << std::endl;*/

        if (routeSolution.size() != pInst->nbVehicles_)
            myTools::throwError("Number of routes in the solution does not match with the vehicles!!!");

        IloInt incomID = Cplex_.getIncumbentNode();
        // fixed the values on integer solution

        /*Model_.add(IloConversion(env_, zVar_, ILOFLOAT));
        Model_.add(IloConversion(env_, routeVar_, ILOFLOAT));*/
        Cplex_.solveFixed(incomID);
//        std::cout << "Linear RP Objective value: " << Cplex_.getObjValue() << std::endl;

        // getting dual values
        requestDuals_.clear();
        vehicleDuals_.clear();

        // define dual container size
        requestDuals_ = IloNumArray(env_, pInst->nbRequests_);
        vehicleDuals_ = IloNumArray(env_, pInst->nbVehicles_);

        for (auto &requestObj: pInst->requests_) {
            int rowIndex = requestObj->taskIndex_;
            requestDuals_[rowIndex] = Cplex_.getDual(requestConst_[rowIndex]);
//            std::cout << "request " << requestObj->getRequestId() << " dual == " << requestObj->dual_;
            requestObj->dual_ = requestDuals_[rowIndex];
            requestObj->CPDual_ = requestDuals_[rowIndex];
//            std::cout << " --> " << requestObj->dual_ << std:: endl;
        }

        for (auto &vehicleObj: pInst->vehicles_) {
            if (vehicleObj->vehicleIndex_ > -1) {
                vehicleDuals_[pInst->vehicles_[vehicleObj->vehicleID_]->vehicleIndex_] = Cplex_.getDual(
                        vehicleConst_[pInst->vehicles_[vehicleObj->vehicleID_]->vehicleIndex_]);
//                std::cout << "vehicle " << vehicleObj->vehicleID_ << " dual == " << vehicleObj->dual_;
                vehicleObj->dual_ = vehicleDuals_[pInst->vehicles_[vehicleObj->vehicleID_]->vehicleIndex_];
                vehicleObj->CPDual_ = vehicleDuals_[pInst->vehicles_[vehicleObj->vehicleID_]->vehicleIndex_];
//                std::cout << " --> " << vehicleObj->dual_ << std:: endl;
            }
        }
        Cplex_.clearModel();
    }
    catch (IloException& e) {
        std::cout << e << std::endl;
    }
}

std::string ZoomReducedProblem::toString() const {
    std::stringstream repStr;
    repStr << std::endl;
    repStr << "# ====================  REDUCED PROBLEM SOLVED INTEGER MODE ==================== " << std::endl;
    repStr << MasterModeler::toString();
    return repStr.str();
}


