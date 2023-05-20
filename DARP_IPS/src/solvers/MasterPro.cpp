//
// Created by Elahe Amiri on 2023-04-22.
//

#include "MasterPro.h"

MasterPro::MasterPro() {
// defining variable
    zVar_ = IloNumVarArray(env_, 0.0, 0.0, IloInfinity,ILOFLOAT);
    routeVar_ = IloNumVarArray(env_, 0.0, 0.0, IloInfinity,ILOFLOAT);
    compRoutes_.clear();
}

void MasterPro::buildModelMP(PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution) {
    // model initialization (defining empty set of constraints and adding objective)
    int rhs = 1;
    MasterModeler::initializeModel(pInst, rhs);

    // adding request columns (z variables)
    for (auto & zSol : pInst->requests_)
        MasterModeler::addZVarFloat(zVar_, zSol, POSITIVE);

    // adding route solution columns
    for (auto & routeSol : routeSolution){
        MasterModeler::addRouteVarFloat(routeVar_, routeSol, POSITIVE);
        compRoutes_.push_back(routeSol);
        routeSol->isAdded_ = true;
    }

    //adding new route variables
    for (auto & routeObj : routesToAdd_) {
        MasterModeler::addRouteVarFloat(routeVar_, routeObj, POSITIVE);
        compRoutes_.push_back(routeObj);
        routeObj->isAdded_ = true;
    }
}

void MasterPro::updateModel() {
    if (routesToAdd_.empty()) {
        std::cout << "There is no route to be added" << std::endl;
        throw myTools::myException("The input route is empty, No new column is passed to be added", __LINE__);
    }

    // add the new compatible column to the model
    for (auto routeObj : routesToAdd_) {
        MasterModeler::addRouteVarFloat(routeVar_, routeObj, POSITIVE);
        compRoutes_.push_back(routeObj);
        routeObj->isAdded_ = true;
    }
}

void MasterPro::solveModelLP(PInstance &pInst) {
    try {
        Model_.add(requestConst_);
        Model_.add(vehicleConst_);
        Model_.add(objFunction_);

        Model_.add(IloConversion(env_, zVar_, ILOFLOAT));
        Model_.add(IloConversion(env_, routeVar_, ILOFLOAT));

        Cplex_ = IloCplex(Model_);
        Cplex_.setParam(IloCplex::Param::Threads, pInst->parameters_->nbThreads_);
        Cplex_.setParam(IloCplex::Param::Preprocessing::Presolve, 0);
        Cplex_.setOut(env_.getNullStream());
        solveTime_->start();
        Cplex_.solve();
        solveTime_->stop();

        /*IloNumArray zVal(env_);
        IloNumArray routeVal(env_);

        Cplex_.getValues(zVal, zVar_);
        Cplex_.getValues(routeVal, routeVar_);
        for (int r = (int)routeVal.getSize()-1; r >= 0; --r) {
            if (routeVal[r] > 0.1 && routeVal[r] < 1)
                std::cout << routeVal[r] << "  ";
        }
        std::cout << std::endl;

        for (int r = (int)zVal.getSize()-1; r >= 0; --r) {
            if (zVal[r] > 0.1 && zVal[r] < 1)
                std::cout << zVal[r] << "  ";
        }
        std::cout << std::endl;*/


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

void MasterPro::solveModelInt(PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution) {
    try {
        Model_.add(requestConst_);
        Model_.add(vehicleConst_);
        Model_.add(objFunction_);

        IloConversion convZ = IloConversion(env_, zVar_, ILOINT);
        IloConversion convR = IloConversion(env_, routeVar_, ILOINT);

        Model_.add(convZ);
        Model_.add(convR);

        Cplex_ = IloCplex(Model_);
        Cplex_.setParam(IloCplex::Param::Threads, pInst->parameters_->nbThreads_);
        Cplex_.setParam(IloCplex::Param::Preprocessing::Presolve, 0);
        Cplex_.setOut(env_.getNullStream());
        solveTime_->start();
        Cplex_.solve();
        solveTime_->stop();
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

        if (routeSolution.size() != pInst->nbVehicles_)
            myTools::throwError("Number of routes in the solution does not match with the vehicles!!!");

        // fixed the values on integer solution to get duals for MIP
        if (pInst->parameters_->mainAlgorithm_ == MP_MIP) {
            IloInt incomID = Cplex_.getIncumbentNode();
            Cplex_.solveFixed(incomID);

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

                for (auto &vehicleObj: pInst->vehicles_) {
                    vehicleDuals_[vehicleObj->vehicleID_] = Cplex_.getDual(vehicleConst_[vehicleObj->vehicleID_]);
//            std::cout << "vehicle " << vehicleObj->vehicleID_ << " dual == " << vehicleObj->dual_;
                    vehicleObj->dual_ = vehicleDuals_[vehicleObj->vehicleID_];
                    vehicleObj->CPDual_ = vehicleDuals_[vehicleObj->vehicleID_];
//            std::cout << " --> " << vehicleObj->dual_ << std:: endl;
                }
            }
        }
        convR.end();
        convZ.end();
        Cplex_.clearModel();
    }
    catch (IloException& e) {
        std::cout << e << std::endl;
    }
}

