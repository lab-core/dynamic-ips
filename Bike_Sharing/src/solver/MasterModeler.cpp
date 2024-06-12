//
// Created by Elahe Amiri on 2024-05-13.
//

#include "MasterModeler.h"

// Constructor and Destructor
MasterModeler::MasterModeler() {
    Model_ = IloModel(env_);
    objFunction_ = IloMaximize(env_);
//    Model_.add(objFunction_);

    taskDuals_ = IloNumArray(env_);
    vehicleDuals_ = IloNumArray(env_);

    taskRHS_ = IloNumArray(env_);
    vehicleRHS_ = IloNumArray(env_);
    solveTime_ = new myTools::Timer(); solveTime_->init();

    // defining variable
    routeVar_ = IloNumVarArray(env_, 0.0, 0.0, IloInfinity,ILOFLOAT);
    modelRoutes_.clear();
}

MasterModeler::~MasterModeler() {
    delete solveTime_;
    env_.end();
}

// Display function
std::string MasterModeler::toString() const {
    std::stringstream repStr;
    repStr << "#" << std::endl;
    repStr << "# Solution status = " << Cplex_.getStatus() << std::endl;
    repStr << "# Incumbent objective value = " << Cplex_.getObjValue() << std::endl;

    return repStr.str();
}

// function to create pattern from routes
void MasterModeler::createPattern(IloNumArray &pattern, PRoute &route) {
    for (auto & taskObj : route->assignedTasks_) {
        pattern[taskObj->locationIndex_] = 1;
    }
}

// this function initialized the model
void MasterModeler::initializeModel(PInstance &pInst, int rhs, int nbVehicles) {
// update order of requests
    nbRequestTask_ = pInst->nbTasks_;

    // define and add objective

    createIloNumArray (taskRHS_, pInst->nbTasks_, rhs);
    createIloNumArray (vehicleRHS_, nbVehicles, rhs);

    taskConst_ = IloRangeArray(env_, -IloInfinity, taskRHS_);
    vehicleConst_ = IloRangeArray(env_, vehicleRHS_, vehicleRHS_);

}

// this function adds routeVar to the model
void MasterModeler::addRouteVar(PRoute &newRoute, bool intVar) {
    IloNumArray columnVar(env_, nbRequestTask_);
    createPattern(columnVar, newRoute);
    IloNumColumn numVar;
    numVar = objFunction_(newRoute->totalBonus_) + taskConst_(columnVar)
             + vehicleConst_[newRoute->vehicleID_](1);
    if (intVar)
        routeVar_.add(IloNumVar(numVar,0,IloInfinity,ILOINT));
    else
        routeVar_.add(IloNumVar(numVar));
    //   numVar.setName(newRoute->name_);
    routeVar_[routeVar_.getSize()-1].setName(newRoute->name_);

    modelRoutes_.push_back(newRoute);
    newRoute->mpAdded_ = true;
}

void MasterModeler::updateModel() {
    // add the new column to the model
    for (auto & routeObj : routesToAdd_) {
        addRouteVar(routeObj, false);
    }
}

void MasterModeler::buildModelMP(PInstance &pInst, vector<PRoute> &routeSolution, int nbVehicles) {
    // model initialization (defining empty set of constraints and adding objective)
    int rhs = 1;
    initializeModel(pInst, rhs, nbVehicles);

    // adding route solution columns
    for (auto & routeSol : routeSolution){
        addRouteVar(routeSol, false);
    }

    //adding new route variables
    for (auto & routeObj : routesToAdd_) {
        addRouteVar(routeObj, false);
    }
    Model_.add(taskConst_);
    Model_.add(vehicleConst_);
    Model_.add(objFunction_);
}

void MasterModeler::solveModelLP(PInstance &pInst) {
    try {

        IloConversion convR = IloConversion(env_, routeVar_, ILOFLOAT);

        Model_.add(convR);

        /*std::ofstream logFile(inputPaths.output_cplexLog_, std::ofstream::app);
        logFile << "----------------------- LMP ------------------------"<< std::endl;
        std::streambuf* coutBuffer = std::cout.rdbuf();
        std::cout.rdbuf(logFile.rdbuf());*/

        Cplex_ = IloCplex(Model_);
        Cplex_.setParam(IloCplex::Param::Threads, pInst->parameters_->nbThreads_);
        Cplex_.setParam(IloCplex::Param::Preprocessing::Presolve, 0);
        Cplex_.setParam(IloCplex::Param::RootAlgorithm, 2);

        solveTime_->start();
        Cplex_.solve();
        solveTime_->stop();

        objValue_ = Cplex_.getObjValue();
        // getting dual values
        taskDuals_.clear();
        vehicleDuals_.clear();

        // define dual container size
        taskDuals_ = IloNumArray(env_, pInst->nbTasks_);
        vehicleDuals_ = IloNumArray(env_, pInst->nbVehicles_);

        for (auto &taskObj: pInst->tasks_) {
            int rowIndex = taskObj->locationIndex_;
            taskDuals_[rowIndex] = Cplex_.getDual(taskConst_[rowIndex]);
            taskObj->dual_ = taskDuals_[rowIndex];
        }

        for (auto &vehicleObj: pInst->vehicles_) {
            int index = vehicleObj->vehicleID_;
            vehicleDuals_[index] = Cplex_.getDual(vehicleConst_[index]);
            vehicleObj->dual_ = vehicleDuals_[index];
        }
        convR.end();
        /*std::cout.rdbuf(coutBuffer);
        logFile.close();*/
        Cplex_.clearModel();
    }
    catch (IloException& e) {
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}

void MasterModeler::solveModelInt(PInstance &pInst, vector<PRoute> &routeSolution, float availableTime, double preObj) {
    try {

        IloConversion convR = IloConversion(env_, routeVar_, ILOINT);

        Model_.add(convR);

        Cplex_ = IloCplex(Model_);

        /*std::ofstream logFile(inputPaths.output_cplexLog_, std::ofstream::app);
        logFile << "----------------------- MP ------------------------"<< std::endl;
        std::streambuf* coutBuffer = std::cout.rdbuf();
        std::cout.rdbuf(logFile.rdbuf());*/

        Cplex_.setParam(IloCplex::Param::Threads, pInst->parameters_->nbThreads_);
        Cplex_.setParam(IloCplex::Param::Preprocessing::Presolve, 0);
        Cplex_.setParam(IloCplex::Param::RootAlgorithm, 2);
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
            if (Cplex_.getObjValue() >= preObj) {
                objValue_ = Cplex_.getObjValue();

                // saving the result and remove out of base variables
                routeSolution.clear();

                IloNumArray zVal(env_);
                IloNumArray routeVal(env_);

                Cplex_.getValues(routeVal, routeVar_);


                for (int r = (int) routeVal.getSize() - 1; r >= 0; --r) {
                    if (routeVal[r] > 0.9) {
                        routeSolution.push_back(modelRoutes_[r]);
                        pInst->vehicles_[modelRoutes_[r]->vehicleID_]->setCurrentRoute(modelRoutes_[r]);
                    }
                }


                /*if (routeSolution.size() != pInst->nbVehicles_)
                    myTools::throwError("Number of routes in the solution does not match with the vehicles!!!");*/

            }
        }
        convR.end();
        /*std::cout.rdbuf(coutBuffer);
        logFile.close();*/
        Cplex_.clearModel();
    }
    catch (IloException& e) {
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}

void MasterModeler::solveModelLPInt(PInstance &pInst, vector<PRoute> &routeSolution, float availableTime, double preObj) {
    try {

        // Solve the Linear Relaxation
        IloConversion convR = IloConversion(env_, routeVar_, ILOFLOAT);

        Model_.add(convR);

        /*std::ofstream logFile(inputPaths.output_cplexLog_, std::ofstream::app);
        logFile << "----------------------- LMP ------------------------"<< std::endl;
        std::streambuf* coutBuffer = std::cout.rdbuf();
        std::cout.rdbuf(logFile.rdbuf());*/

        Cplex_ = IloCplex(Model_);
        Cplex_.setParam(IloCplex::Param::Threads, pInst->parameters_->nbThreads_);
        Cplex_.setParam(IloCplex::Param::Preprocessing::Presolve, 0);
        Cplex_.setParam(IloCplex::Param::RootAlgorithm, 2);
        solveTime_->start();
        Cplex_.solve();
        solveTime_->stop();

        objValue_ = Cplex_.getObjValue();
        // getting dual values
        taskDuals_.clear();
        vehicleDuals_.clear();

        // define dual container size
        taskDuals_ = IloNumArray(env_, pInst->nbTasks_);
        vehicleDuals_ = IloNumArray(env_, pInst->nbVehicles_);

        for (auto &taskObj: pInst->tasks_) {
            int rowIndex = taskObj->locationIndex_;
            taskDuals_[rowIndex] = Cplex_.getDual(taskConst_[rowIndex]);
            taskObj->dual_ = taskDuals_[rowIndex];
        }

        for (auto &vehicleObj: pInst->vehicles_) {
            int index = vehicleObj->vehicleID_;
            vehicleDuals_[index] = Cplex_.getDual(vehicleConst_[index]);
            vehicleObj->dual_ = vehicleDuals_[index];
        }
        convR.end();

        // Convert to integer
        convR = IloConversion(env_, routeVar_, ILOINT);

        Model_.add(convR);
//        logFile << "----------------------- MP ------------------------"<< std::endl;
//        std::cout.rdbuf(logFile.rdbuf());
        if (pInst->parameters_->MIPGap_ > 0.0001)
            Cplex_.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, pInst->parameters_->MIPGap_);
        Cplex_.setParam(IloCplex::Param::TimeLimit, availableTime);
        Cplex_.setParam(IloCplex::Param::RootAlgorithm, 2);
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
                routeSolution.clear();

                IloNumArray zVal(env_);
                IloNumArray routeVal(env_);

                Cplex_.getValues(routeVal, routeVar_);


                for (int r = (int) routeVal.getSize() - 1; r >= 0; --r) {
                    if (routeVal[r] > 0.9) {
                        routeSolution.push_back(modelRoutes_[r]);
                        pInst->vehicles_[modelRoutes_[r]->vehicleID_]->setCurrentRoute(modelRoutes_[r]);
                    }
                }

            }
        }

        convR.end();
        /*std::cout.rdbuf(coutBuffer);
        logFile.close();*/

        Cplex_.clearModel();

    }
    catch (IloException& e) {
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }

}


