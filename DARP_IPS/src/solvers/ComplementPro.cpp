//
// Created by Ella on 2021-11-17.
//

#include "ComplementPro.h"
ComplementPro::ComplementPro() : MasterModeler() {

    routeIncVar_ = IloNumVarArray(env_, 0.0, 0.0, IloInfinity,ILOFLOAT);
    zIncVar_ = IloNumVarArray(env_, 0.0, 0.0, IloInfinity, ILOFLOAT);
    routeSolVar_ = IloNumVarArray (env_, 0.0, 0.0, IloInfinity, ILOFLOAT);
    zSolVar_ = IloNumVarArray (env_, 0.0, 0.0, IloInfinity,ILOFLOAT);
    status_ = NOT_SOLVED;
    IncRoute_.clear();
    fractionalZ_.clear();
    fractionalRoutes_.clear();
}

// this function initialized the model and define empty set of constraints
void ComplementPro::initializeCPModel(PInstance &pInst, int nbVehicles) {
    int rhs = 0;
    initializeModel(pInst, rhs, nbVehicles);
    normalConst_ = IloRangeArray(env_,1,1.0,1.0);
//    Model_.add(normalConst_);
    IncRoute_.clear();
//    solIndexes_.clear();
}

// this function adds zVar to the model
void ComplementPro::addZVar(IloNumVarArray zVar, PRequest &request, VarSign sign) {

    if (sign == NEGATIVE)
        MasterModeler::addZVarFloat(zVar, request, sign);
    else if (sign == POSITIVE) {
        IloNumVar numVar = IloNumVar(objFunction_(request->penalty_) +
                                     requestConst_[request->taskIndex_](1) +
                                     normalConst_[0](1));
        numVar.setName(request->name_);
        zVar.add(numVar);
    }
}

// this function adds routeVar to the model
void ComplementPro::addRouteVar(IloNumVarArray routeVar, PRoute &newRoute, VarSign sign, PInstance &pInst) {

    if (sign == NEGATIVE)
        MasterModeler::addRouteVarFloat(routeVar, newRoute, sign, pInst);
    else {
        IloNumArray columnVar(env_, (signed) orderToRequest_.size());
        MasterModeler::createPattern(columnVar, newRoute, POSITIVE);
        IloNumVar numVar = IloNumVar(objFunction_(newRoute->totalDelay_) + requestConst_(columnVar)
                                     + vehicleConst_[pInst->vehicles_[newRoute->vehicleID_]->vehicleIndex_](1)
                                     + normalConst_[0](newRoute->incompatibilityDegree_));
        numVar.setName(newRoute->name_);
        routeVar.add(numVar);
        IncRoute_.push_back(newRoute);
        if (routeVar.getSize() != IncRoute_.size()){
            std::cout << "Modelling Problem -> Incompatible routeVar array" << std::endl;
            throw myTools::myException("Modelling Problem -> Incompatible routeVar array!", __LINE__);
        }
    }
}

void ComplementPro::addAuxVar(PInstance &pInst, double cost, int nbVehicles) {

    IloNumArray columnVar(env_, (signed) orderToRequest_.size());
    for (auto & requestObj : pInst->requests_)
        columnVar[requestObj->taskIndex_] = -1;

    IloNumArray vehicleVar(env_, nbVehicles);
    for (int v = 0; v < nbVehicles; ++v)
        vehicleVar[v] = -1;

    auxVar_ = IloNumVar(objFunction_(-cost) + requestConst_(columnVar)+ vehicleConst_(vehicleVar));
    auxVar_.setName("aux");
}

// this function build the model at each iteration
void ComplementPro::buildModel(PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
                               int nbVehicles) {
    // model initialization (defining empty set of constraints and adding objective)
//    ResetCPModel();
//    clearModel();
    initializeCPModel(pInst, nbVehicles);

    // adding solution route columns
    for (int r = 0; r < routeSolution.size(); r++) {
        if (pInst->vehicles_[routeSolution[r]->vehicleID_]->vehicleIndex_ > -1) {
            addRouteVar(routeSolVar_, routeSolution[r], NEGATIVE, pInst);
            routeSolution[r]->cpAdded_ = true;
//            solIndexes_.push_back(r);
        }
    }


    // adding incompatible route columns
    for (auto & routeAdd: routesToAdd_) {
        if (pInst->vehicles_[routeAdd->vehicleID_]->vehicleIndex_ > -1) {
            addRouteVar(routeIncVar_, routeAdd, POSITIVE, pInst);
            routeAdd->cpAdded_ = true;
        }
    }

    // adding z columns
    for (auto & zSol: zSolution) {
        addZVar(zSolVar_, zSol, NEGATIVE);
    }
    for (int i = 0; i < pInst->nbRequests_; ++i) {
        int flagAdd =0;
        for (auto & zSol: zSolution) {
            if (pInst->requests_[i] == zSol) {
                flagAdd = 1;
                break;
            }
        }
        if (flagAdd == 0)
            addZVar(zIncVar_, pInst->requests_[i], POSITIVE);
    }
    for (int v = 0; v < pInst->nbVehicles_; ++v) {
        if (pInst->vehicles_[v]->vehicleIndex_>-1 && !pInst->vehicles_[v]->emptyRoute_->cpAdded_ &&
            !pInst->vehicles_[v]->currentRoute_->routeRequests_.empty()) {
            addRouteVar(routeIncVar_, pInst->vehicles_[v]->emptyRoute_, POSITIVE, pInst);
            pInst->vehicles_[v]->emptyRoute_->cpAdded_ = true;
        }
    }
    Model_.add(normalConst_);
    Model_.add(requestConst_);
    Model_.add(vehicleConst_);
    Model_.add(objFunction_);
}

void ComplementPro::repairModel(PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
                               int nbVehicles) {
    routeSolVar_.endElements();
//    routeSolVar_.end();
    zSolVar_.endElements();
//    zSolVar_.end();

    // adding solution route columns
    for (int r = 0; r < routeSolution.size(); r++) {
        if (pInst->vehicles_[routeSolution[r]->vehicleID_]->vehicleIndex_ > -1) {
            addRouteVar(routeSolVar_, routeSolution[r], NEGATIVE, pInst);
            routeSolution[r]->cpAdded_ = true;
//            solIndexes_.push_back(r);
        }
    }

    // adding z columns
    for (auto & zSol: zSolution) {
        addZVar(zSolVar_, zSol, NEGATIVE);
    }
}
void ComplementPro::buildModelCP2(PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
                               int nbVehicles, double preObj) {
    // model initialization (defining empty set of constraints and adding objective)
    initializeCPModel(pInst, nbVehicles);

    // adding solution route columns
    for (int r = 0; r < routeSolution.size(); r++) {
        if (pInst->vehicles_[routeSolution[r]->vehicleID_]->vehicleIndex_ > -1) {
            addRouteVar(routeIncVar_, routeSolution[r], POSITIVE, pInst);
            routeSolution[r]->cpAdded_ = true;
        }
    }


    // adding incompatible route columns
    for (auto & routeAdd: routesToAdd_) {
        if (pInst->vehicles_[routeAdd->vehicleID_]->vehicleIndex_ > -1) {
            addRouteVar(routeIncVar_, routeAdd, POSITIVE, pInst);
            routeAdd->cpAdded_ = true;
        }
    }

    // adding z columns
    for (auto & zSol: zSolution) {
        addZVar(zIncVar_, zSol, POSITIVE);
    }
    for (int i = 0; i < pInst->nbRequests_; ++i) {
        int flagAdd =0;
        for (auto & zSol: zSolution) {
            if (pInst->requests_[i] == zSol) {
                flagAdd = 1;
                break;
            }
        }
        if (flagAdd == 0)
            addZVar(zIncVar_, pInst->requests_[i], POSITIVE);
    }
    addAuxVar(pInst, preObj, nbVehicles);
    Model_.add(normalConst_);
    Model_.add(requestConst_);
    Model_.add(vehicleConst_);
    Model_.add(objFunction_);
}

// this function update the model and variables
void ComplementPro::updateModel(PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution) {

    // adding solution route columns
    /*for (auto & routeSol: routeSolution)
        addRouteVar(routeSolVar_, routeSol, NEGATIVE);*/

    // adding incompatible route columns
    for (auto & routeAdd: routesToAdd_) {
        addRouteVar(routeIncVar_, routeAdd, POSITIVE, pInst);
        routeAdd->cpAdded_ = true;
    }

    // adding z columns
    /*for (auto & zSol: zSolution) {
        addZVar(zSolVar_, zSol, NEGATIVE);
    }
    for (int i = 0; i < pInst->nbRequests_; ++i) {
        int flagAdd =0;
        for (auto & zSol: zSolution) {
            if (pInst->requests_[i] == zSol) {
                flagAdd = 1;
                break;
            }
        }
        if (flagAdd == 0)
            addZVar(zIncVar_, pInst->requests_[i], POSITIVE);
    }*/
}

// this function solve the model

void ComplementPro::solveCPModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                                 InputPaths &inputPaths) {
    try {
        Cplex_ = IloCplex(Model_);
        Cplex_.setParam(IloCplex::Param::Threads, pInst->parameters_->nbThreads_);
        Cplex_.setParam(IloCplex::Param::RootAlgorithm, 2);
        std::ofstream logFile(inputPaths.getOutputCplexLog(), std::ofstream::app);
        logFile << "----------------------- CP ------------------------"<< std::endl;
        std::streambuf* coutBuffer = std::cout.rdbuf();
        std::cout.rdbuf(logFile.rdbuf());
        solveTime_->start();
        if (!Cplex_.solve()) {
            Cplex_.clearModel();
            solveTime_->stop();
            status_ = INFEASIBLE;
            env_.out() << Model_ << std::endl;
            std::cout << "Failed to optimize the problem" << std::endl;
//        throw myTools::myException("the Complementary model is infeasible!!!", __LINE__);
        }
        else {
            solveTime_->stop();

            // getting dual values
            requestDuals_.clear();
            vehicleDuals_.clear();

            // define dual container size
            requestDuals_ = IloNumArray(env_, pInst->nbRequests_);
            vehicleDuals_ = IloNumArray(env_, pInst->nbVehicles_);

            // get duals
            for (auto &requestObj: pInst->requests_) {
                if (requestObj->requestStatus_ == NO_ACTION) {
                    int rowIndex = requestObj->taskIndex_;
                    requestDuals_[rowIndex] = Cplex_.getDual(requestConst_[rowIndex]);
                    requestObj->dual_ = requestDuals_[rowIndex];
                    requestObj->InitialDual_ = requestDuals_[rowIndex];
                }

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

            // saving the result and remove out of base variables
            if (Cplex_.getObjValue() < -0.01) {
                status_ = NEGATIVE_VALUE;
                // check the solution to be column disjoint
                vector<PRequest> zResult;
                vector<PRoute> routeResult;

                vector<int> InRouteVar;
                vector<int> OutRouteVar;
                vector<int> InRequestVar;
                vector<int> OutRequestVar;

                // determine incoming variables
                for (int r = (int) routeIncVar_.getSize() - 1; r >= 0; --r) {
                    if (Cplex_.getValue(routeIncVar_[r]) > 0) {
                        routeResult.push_back(IncRoute_[r]);
                        InRouteVar.push_back(r);
                    }
                }
                for (int i = (int) zIncVar_.getSize() - 1; i >= 0; --i) {
                    if (Cplex_.getValue(zIncVar_[i]) > 0) {
                        zResult.push_back(pInst->nameToRequest_[zIncVar_[i].getName()]);
                        InRequestVar.push_back(i);
                    }
                }

                // determine outgoing variables
                for (int r = (int) routeSolVar_.getSize() - 1; r >= 0; --r) {
                    if (Cplex_.getValue(routeSolVar_[r]) > 0) {
                        OutRouteVar.push_back(r);
                    }
                }
                for (int i = (int) zSolVar_.getSize() - 1; i >= 0; --i) {
                    if (Cplex_.getValue(zSolVar_[i]) > 0) {
                        OutRequestVar.push_back(i);
                    }
                }
                if (isColumnDisjointBit(zResult, routeResult)) {
                    // if (isColumnDisjointBit(zResult, routeResult,pInst->nbVehicles_, pInst->requests_.size())) {
                    // remove outgoing variable
                    Cplex_.clearModel();
                    for (auto &r: OutRouteVar) {
            //            addRouteVar(routeIncVar_, routeSolution[r], POSITIVE);
                        routeSolution.erase(routeSolution.begin() + r);
                        routeSolVar_[r].end();
                        routeSolVar_.remove(r, 1);
                    }
                    for (auto &i: OutRequestVar) {
                        addZVar(zIncVar_, zSolution[i], POSITIVE);
                        zSolution.erase(zSolution.begin() + i);
                        zSolVar_[i].end();
                        zSolVar_.remove(i, 1);
                    }

                    // add incoming variables
                    for (auto &r: InRouteVar) {
                        routeSolution.push_back(IncRoute_[r]);
                        addRouteVar(routeSolVar_, routeSolution.back(), NEGATIVE, pInst);
                        pInst->vehicles_[IncRoute_[r]->vehicleID_]->setCurrentRoute(IncRoute_[r]);
                        routeIncVar_[r].end();
                        routeIncVar_.remove(r, 1);
                        IncRoute_.erase(IncRoute_.begin() + r);
                    }
                    for (auto &i: InRequestVar) {
                        zSolution.push_back(pInst->nameToRequest_[zIncVar_[i].getName()]);
                        addZVar(zSolVar_, zSolution.back(), NEGATIVE);
                        zIncVar_[i].end();
                        zIncVar_.remove(i, 1);
                    }

                    int nbRequests = 0;
                    for (auto &requestObj: pInst->requests_) {
                        if (requestObj->requestStatus_ == NO_ACTION)
                            nbRequests++;
                    }
                } else {
                    status_ = FRACTIONAL;
                    if (pInst->parameters_->useZoom_ || pInst->parameters_->useMultiStage_) {
                        fractionalRoutes_.clear();
                        fractionalZ_.clear();

                        for (auto &r: OutRouteVar)
                            fractionalRoutes_.push_back(routeSolution[r]);
                        for (auto &r: InRouteVar)
                            fractionalRoutes_.push_back(IncRoute_[r]);
                    }

                }
                routeResult.clear();
                zResult.clear();
            } else{
                Cplex_.clearModel();
                status_ = POSITIVE_VALUE;
            }
        }
        std::cout.rdbuf(coutBuffer);
        logFile.close();
    }
    catch (IloException& e) {
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
//    Cplex_.clearModel();
}

void ComplementPro::solveCP2Model(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                                 InputPaths &inputPaths) {
    try {
        Cplex_ = IloCplex(Model_);
//        env_.out() << Model_ << std::endl;
        Cplex_.setParam(IloCplex::Param::Threads, pInst->parameters_->nbThreads_);
        std::ofstream logFile(inputPaths.getOutputCplexLog(), std::ofstream::app);
        logFile << "----------------------- CP ------------------------"<< std::endl;
        std::streambuf* coutBuffer = std::cout.rdbuf();
        std::cout.rdbuf(logFile.rdbuf());

        solveTime_->start();
        if (!Cplex_.solve()) {
            Cplex_.clearModel();
            solveTime_->stop();
            status_ = INFEASIBLE;
            env_.out() << Model_ << std::endl;
            std::cout << "Failed to optimize the problem" << std::endl;
//        throw myTools::myException("the Complementary model is infeasible!!!", __LINE__);
        }
        else {
            solveTime_->stop();

            // getting dual values
            requestDuals_.clear();
            vehicleDuals_.clear();

            // define dual container size
            requestDuals_ = IloNumArray(env_, pInst->nbRequests_);
            vehicleDuals_ = IloNumArray(env_, pInst->nbVehicles_);

//        std::cout << "COMPLEMENTARY DUALS:" << std::endl;
            for (auto &requestObj: pInst->requests_) {
                if (requestObj->requestStatus_ == NO_ACTION) {
                    int rowIndex = requestObj->taskIndex_;
                    requestDuals_[rowIndex] = Cplex_.getDual(requestConst_[rowIndex]);
                    requestObj->dual_ = requestDuals_[rowIndex];
                    requestObj->InitialDual_ = requestDuals_[rowIndex];
                }

            }

//        std::cout << "VEHICLE DUALS:" << std::endl;
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


            // saving the result and remove out of base variables
            if (Cplex_.getObjValue() < -0.01) {
                status_ = NEGATIVE_VALUE;
                // check the solution to be column disjoint
                vector<PRequest> zResult;
                vector<PRoute> routeResult;

                vector<int> InRouteVar;
                vector<int> OutRouteVar;
                vector<int> InRequestVar;
                vector<int> OutRequestVar;

                // determine incoming variables
                for (int r = (int) routeIncVar_.getSize() - 1; r >= 0; --r) {
                    if (Cplex_.getValue(routeIncVar_[r]) > 0) {
//                    std::cout << routeIncVar_[r].getName() << std::endl;
                        //                   routeResult.push_back(generatedRoutes[routeIncVar_[r].getName()]);
                        routeResult.push_back(IncRoute_[r]);
                        InRouteVar.push_back(r);
                    }
                }
                for (int i = (int) zIncVar_.getSize() - 1; i >= 0; --i) {
                    if (Cplex_.getValue(zIncVar_[i]) > 0) {
//                    std::cout << zIncVar_[i].getName() << std::endl;
                        zResult.push_back(pInst->nameToRequest_[zIncVar_[i].getName()]);
                        InRequestVar.push_back(i);
                    }
                }

                Cplex_.clearModel();
                if (isColumnDisjointBit(zResult, routeResult)) {
                    //           if (isColumnDisjointBit(zResult, routeResult,pInst->nbVehicles_, pInst->requests_.size())) {
                    // remove outgoing variable
                    routeSolution = routeResult;
                    zSolution = zResult;

                    int nbRequests = 0;
                    for (auto &requestObj: pInst->requests_) {
                        if (requestObj->requestStatus_ == NO_ACTION)
                            nbRequests++;
                    }
                } else {
                    status_ = FRACTIONAL;
                    //             std::cout << "The solution is not column disjoint!!!!!!!" << std::endl;
                    /*if (pInst->parameters_->useZoom_ || pInst->parameters_->useMultiStage_) {
                        fractionalRoutes_.clear();
                        fractionalZ_.clear();
                        // add incoming variables
                        for (int r = 0; r < routeIncVar_.getSize(); ++r) {
                            if (Cplex_.getValue(routeIncVar_[r]) > 0) {
                                fractionalRoutes_.push_back(IncRoute_[r]);
                            }
                        }
                        for (int i = 0; i < zIncVar_.getSize(); ++i) {
                            if (Cplex_.getValue(zIncVar_[i]) > 0) {
                                fractionalZ_.push_back(pInst->nameToRequest_[zIncVar_[i].getName()]);
                            }
                        }
                    }*/
                    /*if (routeSolution.size() != pInst->nbVehicles_)
                        myTools::throwError("Number of routes in the solution does not match with the vehicles!!!");*/
                }
                routeResult.clear();
                zResult.clear();
            } else{
                Cplex_.clearModel();
                status_ = POSITIVE_VALUE;
            }
        }
        std::cout.rdbuf(coutBuffer);
        logFile.close();
    }
    catch (IloException& e) {
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
//    Cplex_.clearModel();
}

// this function check the situation of the CP solution to be column disjoint
bool ComplementPro::isColumnDisjoint(vector<PRequest> &zResults, vector<PRoute> &routeResults, int nbVehicle) {
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero((signed)orderToRequest_.size()+ nbVehicle, (signed)zResults.size() + (signed)routeResults.size());
    for (int r = 0; r < routeResults.size(); ++r) {
        for (int i = 0; i < routeResults[r]->routeRequests_.size(); ++i)
            A(routeResults[r]->routeRequests_[i]->taskIndex_, r) = 1;
        A((signed)orderToRequest_.size()+routeResults[r]->vehicleID_,r) = 1;
    }
    for (int i = 0; i < zResults.size(); ++i) {
        A((signed)zResults[i]->taskIndex_,(signed)routeResults.size()+i) = 1;
    }

    Eigen::MatrixXd ATA = A.transpose()*A;
    // setting diagonal elements to zero
    for (int i = 0; i < ATA.rows(); ++i) {
        ATA(i,i) = 0;
    }

    if (ATA == Eigen::MatrixXd::Zero(ATA.rows(), ATA.cols()))
        return true;
    else
        return false;
}
bool ComplementPro::isColumnDisjointBit(vector<PRequest> &zResults, vector<PRoute> &routeResults) {
    std::vector<std::bitset<MAX_SIZE>> Columns;
    std::bitset<MAX_SIZE> unions;
    std::bitset<MAX_SIZE> vehicles;
    int counts = 0;
    int veh = 0;
    for (auto & requestObj : zResults) {
        std::bitset<MAX_SIZE>  zCol;
        unions.set(requestObj->taskIndex_, true);
        counts ++;
    }
    for (auto & routeObj : routeResults) {
        unions = unions|routeObj->column_;
        counts += routeObj->column_.count();
        vehicles.set(routeObj->vehicleID_, true);
        veh ++;
    }

    if (unions.count() != counts || vehicles.count() != veh)
        return false;
    else
        return true;  // Sets are disjoint
}

// Display function
std::string ComplementPro::toString() const {
    std::stringstream repStr;
    repStr << std::endl;
    repStr << "# ====================  COMPLEMENTARY PROBLEM SOLVED  ==================== " << std::endl;
    repStr << "# COMPLEMENTARY PROBLEM SOLVED: " << std::endl;
    repStr << MasterModeler::toString();
    return repStr.str();
}

void ComplementPro::ResetCPModel() {
    try {

        bool isModelExist = false;
        if (routeSolVar_.getSize() > 0)
            isModelExist = true;
        if (isModelExist) {
            routeSolVar_.endElements();
            routeSolVar_.clear();
//        SolRoute_.clear();
            zSolVar_.endElements();
            zSolVar_.clear();
            routeIncVar_.endElements();
            routeIncVar_.clear();
            IncRoute_.clear();
//            solIndexes_.clear();
            zIncVar_.endElements();
            zIncVar_.clear();
            Model_.remove(requestConst_);
            Model_.remove(vehicleConst_);
            Model_.remove(normalConst_);
        }

    }
    catch (IloException& e) {
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}

void ComplementPro::restartCp() {
    env_.end();
    routeIncVar_ = IloNumVarArray(env_, 0.0, 0.0, IloInfinity,ILOFLOAT);
    zIncVar_ = IloNumVarArray(env_, 0.0, 0.0, IloInfinity, ILOFLOAT);
    routeSolVar_ = IloNumVarArray (env_, 0.0, 0.0, IloInfinity, ILOFLOAT);
    zSolVar_ = IloNumVarArray (env_, 0.0, 0.0, IloInfinity,ILOFLOAT);
    status_ = NOT_SOLVED;
}








