//
// Created by Ella on 2021-11-17.
//

#include "CP_Cplex.h"
#include "solvers/SolverEnv.h"

CP_Cplex::CP_Cplex() : CplexModeler() {

    routeIncVar_ = IloNumVarArray(env_, 0.0, 0.0, IloInfinity,ILOFLOAT);
    routeSolVar_ = IloNumVarArray (env_, 0.0, 0.0, IloInfinity, ILOFLOAT);

    zIncVar_ = IloNumVarArray(env_, 0.0, 0.0, IloInfinity, ILOFLOAT);
    zSolVar_ = IloNumVarArray (env_, 0.0, 0.0, IloInfinity,ILOFLOAT);
    status_ = NOT_SOLVED;
    IncRoute_.clear();
    fractionalZ_.clear();
    fractionalRoutes_.clear();
}

void CP_Cplex::initializeCP(const PInstance &pInst, bool reduced) {
    pInst_ = &pInst;
    int rhs = 0;
    nbRequestTask_ = pInst->nbTasks_;

    createIloNumArray(requestRHS_, nbRequestTask_, rhs);
    requestConst_ = IloRangeArray(env_, requestRHS_, requestRHS_);
    Model_.add(requestConst_);

    if (!reduced) {
        int nbVehicles = pInst->nbVehicles_;
        createIloNumArray(vehicleRHS_, nbVehicles, rhs);
        vehicleConst_ = IloRangeArray(env_, vehicleRHS_, vehicleRHS_);
        Model_.add(vehicleConst_);
    }

    Model_.add(objFunction_);
    Cplex_.setOut(env_.getNullStream());
    Cplex_.setParam(IloCplex::Param::Preprocessing::Reduce, 2);
    Cplex_.setParam(IloCplex::Param::Preprocessing::Aggregator, 0);
    Cplex_.setParam(IloCplex::Param::Preprocessing::BoundStrength, 0);
    Cplex_.setParam(IloCplex::Param::RootAlgorithm, 2);
    Cplex_.setParam(IloCplex::Param::Threads, pInst->parameters_->nbThreads_);
    Cplex_.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, pInst->parameters_->MIPGap_);
    Cplex_.setParam(IloCplex::Param::Preprocessing::RepeatPresolve, 0);

    normalConst_ = IloRange(env_, 1.0, 1.0);
    Model_.add(normalConst_);
    IncRoute_.clear();
}

// this function initializes the model and defines an empty set of constraints
void CP_Cplex::initializeCPModel(const PInstance &pInst, int nbVehicles) {
    initializeCP(pInst, false);
}

// this function adds zVar to the model
void CP_Cplex::addZVar(IloNumVarArray zVar, const PRequest &request, VarSign sign) {

    if (sign == NEGATIVE)
        addZVarFloat(zVar, request, sign);
    else if (sign == POSITIVE) {
        IloNumVar numVar = IloNumVar(objFunction_(request->Req_W3_ * request->penalty_) +
                                     requestConst_[request->taskIndex_](1) +
                                     normalConst_(1));
        numVar.setName(request->name_);
        zVar.add(numVar);
    }
}

// this function adds routeVar to the model
void CP_Cplex::addRouteVar(IloNumVarArray routeVar, const PRoute &newRoute, VarSign sign, const PInstance &pInst) {

    if (sign == NEGATIVE)
        addRouteVarFloat(routeVar, newRoute, sign, pInst);
    else {
        IloNumArray columnVar(env_, nbRequestTask_);
        createPattern(columnVar, newRoute, POSITIVE);
        IloNumVar numVar = IloNumVar(objFunction_(newRoute->objCoef_) + requestConst_(columnVar)
                                     + vehicleConst_[pInst->vehicles_[newRoute->vehicleID_]->vehicleIndex_](1)
                                     + normalConst_(newRoute->incompatibilityDegree_));
        numVar.setName(newRoute->name_);
        routeVar.add(numVar);
        columnVar.end();
        IncRoute_.push_back(newRoute);
    }
}


void CP_Cplex::addRouteVarReduced(IloNumVarArray &routeVar, const PRoute &newRoute, const PRoute &currRoute) {
    IloNumArray columnVar(env_, nbRequestTask_);
    int incScore = 0;
    for (auto &req : currRoute->routeRequests_) {
        if (!newRoute->column_.test(req->taskIndex_)) {
            columnVar[req->taskIndex_] = -1.0;
            incScore++;
        }
    }
    for (auto &req : newRoute->routeRequests_) {
        if (!currRoute->column_.test(req->taskIndex_)) {
            columnVar[req->taskIndex_] = 1.0;
            incScore++;
        }
    }
    newRoute->IncScore_ = incScore;
    double objCoef = newRoute->objCoef_ - currRoute->objCoef_;
    IloNumVar var = IloNumVar(objFunction_(objCoef) + requestConst_(columnVar) + normalConst_(incScore));
    var.setName(newRoute->name_);
    routeVar.add(var);
    columnVar.end();
    IncRoute_.push_back(newRoute);
    newRoute->cpAdded_ = true;
}

void CP_Cplex::buildModel_batch(const PInstance &pInst, const std::vector<PRoute> &routeSolution) {
    for (auto &routeObj : routeSolution) {
        if (pInst->vehicles_[routeObj->vehicleID_]->vehicleIndex_ > -1) {
            addRouteVar(routeSolVar_, routeObj, NEGATIVE, pInst);
            routeObj->cpAdded_ = true;
        }
    }
    for (auto &requestObj : pInst->requests_) {
        if (requestObj->solVehicleID_ == LARGE_CONSTANT)
            addZVar(zSolVar_, requestObj, NEGATIVE);
        else if (requestObj->committedPickTime_ == LARGE_CONSTANT)
            addZVar(zIncVar_, requestObj, POSITIVE);
    }
}

void CP_Cplex::buildModel_batch(const PInstance &pInst) {
    for (auto &routeObj : routesToAdd_) {
        const PRoute &curr = pInst->vehicles_[routeObj->vehicleID_]->currentRoute_;
        addRouteVarReduced(routeIncVar_, routeObj, curr);
    }
    for (auto &requestObj : pInst->requests_) {
        if (requestObj->committedPickTime_ == LARGE_CONSTANT && requestObj->solVehicleID_ < LARGE_CONSTANT)
            addZVar(zIncVar_, requestObj, POSITIVE);
    }
}

// this function builds the model at each iteration
void CP_Cplex::buildModel(const PInstance &pInst, const vector<PRequest> &zSolution, const vector<PRoute> &routeSolution,
                               int nbVehicles) {
    // model initialization (defining an empty set of constraints and adding the objective function)
    initializeCPModel(pInst, nbVehicles);

    // adding solution route columns
    for (auto & routeObj : routeSolution) {
        if (pInst->vehicles_[routeObj->vehicleID_]->vehicleIndex_ > -1) {
            addRouteVar(routeSolVar_, routeObj, NEGATIVE, pInst);
            routeObj->cpAdded_ = true;
        }
    }


    // adding solution z columns
    for (auto & zSol: zSolution) {
        addZVar(zSolVar_, zSol, NEGATIVE);
    }

    // adding z columns out of the basis
    for (int i = 0; i < pInst->nbRequests_; ++i) {
        if (pInst->requests_[i]->solVehicleID_ < LARGE_CONSTANT && pInst->requests_[i]->committedPickTime_ == LARGE_CONSTANT) {
            if (pInst->vehicles_[pInst->requests_[i]->solVehicleID_]->currentRoute_->routeRequests_.size() > 1)
                addZVar(zIncVar_, pInst->requests_[i], POSITIVE);
        }

    }
    for (int v = 0; v < pInst->nbVehicles_; ++v) {
        if (pInst->vehicles_[v]->vehicleIndex_>-1 && !pInst->vehicles_[v]->emptyRoute_->cpAdded_ &&
            !pInst->vehicles_[v]->currentRoute_->routeRequests_.empty()) {
            addRouteVar(routeIncVar_, pInst->vehicles_[v]->emptyRoute_, POSITIVE, pInst);
            pInst->vehicles_[v]->emptyRoute_->cpAdded_ = true;
        }
    }
}

void CP_Cplex::repairModel(const PInstance &pInst, const vector<PRequest> &zSolution, const vector<PRoute> &routeSolution) {
    routeSolVar_.endElements();
    zSolVar_.endElements();

    // adding solution route columns
    for (auto & routeObj : routeSolution) {
        addRouteVar(routeSolVar_, routeObj, NEGATIVE, pInst);
        routeObj->cpAdded_ = true;
    }

    // adding solution z columns
    for (auto & zSol: zSolution) {
        addZVar(zSolVar_, zSol, NEGATIVE);
    }
}


void CP_Cplex::updateModel(const PInstance &pInst) {
    for (auto &routeAdd : routesToAdd_) {
        addRouteVar(routeIncVar_, routeAdd, POSITIVE, pInst);
        routeAdd->cpAdded_ = true;
    }
}

// this function solves the model

void CP_Cplex::solveCPModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                                 InputPaths &inputPaths) {
    try {
        Cplex_.extract(Model_);
        myTools::CoutRedirector redirector(inputPaths.getOutputSolverLog(), "CP");
        solveTime_->start();
        if (!Cplex_.solve()) {
            Cplex_.clearModel();
            solveTime_->stop();
            status_ = INFEASIBLE;
            std::cout << "Failed to optimize the problem" << std::endl;
        }
        else {
            solveTime_->stop();

            // getting dual values
            requestDuals_.clear();
            vehicleDuals_.clear();

            // define dual container size
            Cplex_.getDuals(requestDuals_, requestConst_);
            Cplex_.getDuals(vehicleDuals_, vehicleConst_);

            // get duals
            for (auto &requestObj: pInst->requests_) {
                requestObj->dual_ = static_cast<float>(requestDuals_[requestObj->taskIndex_]);
            }
            for (auto &vehicleObj: pInst->vehicles_) {
                int index = vehicleObj->vehicleIndex_;
                if (index > -1) {
                    vehicleObj->dual_ = static_cast<float>(vehicleDuals_[index]);
                }
                else {
                    vehicleObj->dual_ = 0;
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
                for (IloInt r = routeIncVar_.getSize() - 1; r >= 0; --r) {
                    if (Cplex_.getValue(routeIncVar_[r]) > 0) {
                        routeResult.push_back(IncRoute_[r]);
                        InRouteVar.push_back(static_cast<int>(r));
                    }
                }
                for (IloInt i = zIncVar_.getSize() - 1; i >= 0; --i) {
                    if (Cplex_.getValue(zIncVar_[i]) > 0) {
                        zResult.push_back(pInst->nameToRequest_[zIncVar_[i].getName()]);
                        InRequestVar.push_back(static_cast<int>(i));
                    }
                }

                // determine outgoing variables
                for (IloInt r = routeSolVar_.getSize() - 1; r >= 0; --r) {
                    if (Cplex_.getValue(routeSolVar_[r]) > 0) {
                        OutRouteVar.push_back(static_cast<int>(r));
                    }
                }
                for (IloInt i = zSolVar_.getSize() - 1; i >= 0; --i) {
                    if (Cplex_.getValue(zSolVar_[i]) > 0) {
                        OutRequestVar.push_back(static_cast<int>(i));
                    }
                }
                if (isColumnDisjointFast(zResult, routeResult, pInst)) {
                    // remove outgoing variable
                    Cplex_.clearModel();
                    for (auto &r: OutRouteVar) {
                        addRouteVar(routeIncVar_, routeSolution[r], POSITIVE, pInst);
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

                } else {
                    status_ = FRACTIONAL;
                    if (pInst->parameters_->useZoom_) {
                        fractionalRoutes_.clear();
                        fractionalZ_.clear();

                        for (auto &r: OutRouteVar)
                            fractionalRoutes_.push_back(routeSolution[r]);
                        for (auto &r: InRouteVar)
                            fractionalRoutes_.push_back(IncRoute_[r]);

                        /*for (auto &i: OutRequestVar)
                            fractionalZ_.push_back(zSolution[i]);
                        for (auto &i: InRequestVar)
                            fractionalZ_.push_back(pInst->nameToRequest_[zIncVar_[i].getName()]);*/
                    }

                }
                routeResult.clear();
                zResult.clear();
            } else{
                Cplex_.clearModel();
                status_ = POSITIVE_VALUE;
            }
        }
    }
    catch (IloException& e) {
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}

void CP_Cplex::solveCPModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                                 InputPaths &inputPaths, bool update) {
    if (update) {
        solveCPModel(pInst, zSolution, routeSolution, inputPaths);
        return;
    }
    // update=false: extract solution and set current routes without touching model structure
    try {
        Cplex_.extract(Model_);
        myTools::CoutRedirector redirector(inputPaths.getOutputSolverLog(), "CP");
        solveTime_->start();
        if (!Cplex_.solve()) {
            Cplex_.clearModel();
            solveTime_->stop();
            status_ = INFEASIBLE;
            std::cout << "Failed to optimize the problem" << std::endl;
        }
        else {
            solveTime_->stop();
            requestDuals_.clear();
            vehicleDuals_.clear();
            Cplex_.getDuals(requestDuals_, requestConst_);
            Cplex_.getDuals(vehicleDuals_, vehicleConst_);
            for (auto &requestObj: pInst->requests_)
                requestObj->dual_ = static_cast<float>(requestDuals_[requestObj->taskIndex_]);
            for (auto &vehicleObj: pInst->vehicles_) {
                int index = vehicleObj->vehicleIndex_;
                vehicleObj->dual_ = (index > -1) ? static_cast<float>(vehicleDuals_[index]) : 0.0f;
            }

            if (Cplex_.getObjValue() < -0.01) {
                status_ = NEGATIVE_VALUE;
                std::vector<PRoute> routeResult;
                std::vector<PRequest> zResult;
                for (IloInt r = routeIncVar_.getSize() - 1; r >= 0; --r) {
                    if (Cplex_.getValue(routeIncVar_[r]) > 0)
                        routeResult.push_back(IncRoute_[r]);
                }
                for (IloInt i = zIncVar_.getSize() - 1; i >= 0; --i) {
                    if (Cplex_.getValue(zIncVar_[i]) > 0)
                        zResult.push_back(pInst->nameToRequest_[zIncVar_[i].getName()]);
                }
                if (isColumnDisjointFast(zResult, routeResult, pInst)) {
                    for (auto &routeObj : routeResult)
                        pInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
                    routeSolution.clear();
                    for (auto &vehicleObj : pInst->vehicles_)
                        routeSolution.push_back(vehicleObj->currentRoute_);
                    zSolution.clear();
                    for (auto &requestObj : pInst->requests_) {
                        if (requestObj->solVehicleID_ == LARGE_CONSTANT)
                            zSolution.push_back(requestObj);
                    }
                } else {
                    status_ = FRACTIONAL;
                    if (pInst->parameters_->useZoom_) {
                        fractionalRoutes_.clear();
                        fractionalZ_.clear();
                        for (IloInt r = 0; r < routeIncVar_.getSize(); ++r)
                            if (Cplex_.getValue(routeIncVar_[r]) > 0)
                                fractionalRoutes_.push_back(IncRoute_[r]);
                    }
                }
            } else {
                Cplex_.clearModel();
                status_ = POSITIVE_VALUE;
            }
        }
    }
    catch (IloException &e) {
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}

// Display function
std::string CP_Cplex::toString() const {
    std::stringstream repStr;
    repStr << std::endl;
    repStr << "# ====================  COMPLEMENTARY PROBLEM SOLVED  ==================== " << std::endl;
    repStr << "# COMPLEMENTARY PROBLEM SOLVED: " << std::endl;
    repStr << CplexModeler::toString();
    return repStr.str();
}

void CP_Cplex::ResetCPModel() {
    try {

        bool isModelExist = false;
        if (routeSolVar_.getSize() > 0)
            isModelExist = true;
        if (isModelExist) {
            routeSolVar_.endElements();
            routeSolVar_.clear();
            zSolVar_.endElements();
            zSolVar_.clear();
            routeIncVar_.endElements();
            routeIncVar_.clear();
            IncRoute_.clear();
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

void CP_Cplex::restartCp() {
    env_.end();
    routeIncVar_ = IloNumVarArray(env_, 0.0, 0.0, IloInfinity,ILOFLOAT);
    zIncVar_ = IloNumVarArray(env_, 0.0, 0.0, IloInfinity, ILOFLOAT);
    routeSolVar_ = IloNumVarArray (env_, 0.0, 0.0, IloInfinity, ILOFLOAT);
    zSolVar_ = IloNumVarArray (env_, 0.0, 0.0, IloInfinity,ILOFLOAT);
    status_ = NOT_SOLVED;
}


void CP_Cplex::solveCP2Model(const PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                                 const InputPaths &inputPaths) {
    try {
        Cplex_.extract(Model_);
        myTools::CoutRedirector redirector(inputPaths.getOutputSolverLog(), "CP");

        solveTime_->start();
        if (!Cplex_.solve()) {
            Cplex_.clearModel();
            solveTime_->stop();
            status_ = INFEASIBLE;
            env_.out() << Model_ << std::endl;
            std::cout << "Failed to optimize the problem" << std::endl;
        }
        else {
            solveTime_->stop();

            // getting dual values
            requestDuals_.clear();
            vehicleDuals_.clear();

            // define dual container size
            Cplex_.getDuals(requestDuals_, requestConst_);
            Cplex_.getDuals(vehicleDuals_, vehicleConst_);

            // get duals
            for (auto &requestObj: pInst->requests_) {
                requestObj->dual_ = static_cast<float>(requestDuals_[requestObj->taskIndex_]);
            }
            for (auto &vehicleObj: pInst->vehicles_) {
                int index = vehicleObj->vehicleIndex_;
                if (index > -1) {
                    vehicleObj->dual_ = static_cast<float>(vehicleDuals_[index]);
                }
                else {
                    vehicleObj->dual_ = 0.0f;
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
                for (IloInt r = routeIncVar_.getSize() - 1; r >= 0; --r) {
                    if (Cplex_.getValue(routeIncVar_[r]) > 0) {
                        routeResult.push_back(IncRoute_[r]);
                        InRouteVar.push_back(static_cast<int>(r));
                    }
                }
                for (IloInt i = zIncVar_.getSize() - 1; i >= 0; --i) {
                    if (Cplex_.getValue(zIncVar_[i]) > 0) {
                        zResult.push_back(pInst->nameToRequest_[zIncVar_[i].getName()]);
                        InRequestVar.push_back(static_cast<int>(i));
                    }
                }
                if (isColumnDisjointFast(zResult, routeResult, pInst)) {
                    // remove outgoing variable
                    routeSolution = routeResult;
                    zSolution = zResult;

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
    }
    catch (IloException& e) {
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}

// this function checks the situation of the CP solution to be column disjoint
/*bool CP_Cplex::isColumnDisjoint(vector<PRequest> &zResults, vector<PRoute> &routeResults, int nbVehicle) {
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nbRequestTask_ + nbVehicle, (signed)zResults.size() + (signed)routeResults.size());
    for (int r = 0; r < routeResults.size(); ++r) {
        for (int i = 0; i < routeResults[r]->routeRequests_.size(); ++i)
            A(routeResults[r]->routeRequests_[i]->taskIndex_, r) = 1;
        A(nbRequestTask_ + routeResults[r]->vehicleID_, r) = 1;
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
}*/
bool CP_Cplex::isColumnDisjointBit(const vector<PRequest> &zResults, const vector<PRoute> &routeResults, const PInstance &pInst) {
    std::vector<boost::dynamic_bitset<>> Columns;
    boost::dynamic_bitset<> unions(pInst->nbRequests_);
    boost::dynamic_bitset<> vehicles(pInst->nbVehicles_);
    int counts = 0;
    int veh = 0;
    for (auto & requestObj : zResults) {
        unions.set(requestObj->taskIndex_, true);
        counts ++;
    }
    for (auto & routeObj : routeResults) {
        unions |= routeObj->column_;
        counts += static_cast<int>(routeObj->column_.count());
        vehicles.set(routeObj->vehicleID_, true);
        veh ++;
    }

    if (unions.count() != counts || vehicles.count() != veh)
        return false;
    return true;  // Sets are disjoint
}

bool CP_Cplex::isColumnDisjointFast(const vector<PRequest>& zResults, const vector<PRoute>& routeResults,
    const PInstance &pInst) {
    boost::dynamic_bitset<> coveredRequests(pInst->nbRequests_);
    boost::dynamic_bitset<> usedVehicles(pInst->nbVehicles_);

    // mark each request index; exit on duplicate
    for (auto& requestObj : zResults) {
        if (coveredRequests.test(requestObj->taskIndex_))
            return false;
        coveredRequests.set(requestObj->taskIndex_);
    }

    // for each route: test for any overlap, then mark bits and vehicle
    for (auto& route : routeResults) {
        if ((coveredRequests & route->column_).any())
            return false;
        coveredRequests |= route->column_;

        if (usedVehicles.test(route->vehicleID_))
            return false;
        usedVehicles.set(route->vehicleID_);
    }

    return true;
}

bool CP_Cplex::isColumnDisjoint(const std::vector<PRequest> &zResults, const std::vector<PRoute> &routeResults,
const PInstance &pInst) {
    boost::dynamic_bitset<> requestUnion(pInst->nbRequests_);

    // Process individual requests
    for (const auto& request : zResults) {
        const size_t taskIndex = request->taskIndex_;
        if (requestUnion.test(taskIndex)) {
            return false; // Early exit on conflict
        }
        requestUnion.set(taskIndex);
    }

    // Process routes - BLAZING FAST with precomputed columns
    for (const auto& route : routeResults) {
        if ((requestUnion & route->column_).any()) {
            return false; // Early exit on conflict
        }
        requestUnion |= route->column_;
    }

    return true; // No conflicts found

}


void CP_Cplex::buildModelCP_improved(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                                          int nbVehicles, float preObj) {
    // model initialization (defining an empty set of constraints and adding the objective function)
    initializeCPModel(pInst, nbVehicles);

    // adding solution route columns
    for (auto & routeObj : routeSolution) {
        if (pInst->vehicles_[routeObj->vehicleID_]->vehicleIndex_ > -1) {
            addRouteVar(routeIncVar_, routeObj, POSITIVE, pInst);
            routeObj->cpAdded_ = true;
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
    // adding z columns out of the basis
    for (int i = 0; i < pInst->nbRequests_; ++i) {
        if (pInst->requests_[i]->solVehicleID_ < LARGE_CONSTANT && pInst->requests_[i]->committedPickTime_ == LARGE_CONSTANT)
            addZVar(zIncVar_, pInst->requests_[i], POSITIVE);
    }

    for (int v = 0; v < pInst->nbVehicles_; ++v) {
        if (pInst->vehicles_[v]->vehicleIndex_>-1 && !pInst->vehicles_[v]->emptyRoute_->cpAdded_ &&
            !pInst->vehicles_[v]->currentRoute_->routeRequests_.empty()) {
            addRouteVar(routeIncVar_, pInst->vehicles_[v]->emptyRoute_, POSITIVE, pInst);
            pInst->vehicles_[v]->emptyRoute_->cpAdded_ = true;
            }
    }
    addAuxVar(pInst, preObj, nbVehicles);
}

void CP_Cplex::addAuxVar(const PInstance &pInst, float cost, int nbVehicles) {

    IloNumArray columnVar(env_, nbRequestTask_);
    for (auto & requestObj : pInst->requests_)
        columnVar[requestObj->taskIndex_] = -1;

    IloNumArray vehicleVar(env_, nbVehicles);
    for (int v = 0; v < nbVehicles; ++v)
        vehicleVar[v] = -1;

    auxVar_ = IloNumVar(objFunction_(-cost) + requestConst_(columnVar)+ vehicleConst_(vehicleVar));
    auxVar_.setName("aux");
    columnVar.end();
    vehicleVar.end();
}

void CP_Cplex::resetForNextIteration() {
    for (auto &routeObj : IncRoute_)
        routeObj->cpAdded_ = false;
    IncRoute_.clear();
    fractionalRoutes_.clear();
    fractionalZ_.clear();
    routesToAdd_.clear();
    pInst_  = nullptr;
    status_ = NOT_SOLVED;

    // Nuclear env reset (debug): end the entire IloEnv and start fresh.
    // This guarantees every Concert allocation is freed, so if memory growth
    // stops we know the cause is env-pool accumulation.
    renewEnv();

    // Reinitialise CP-specific var arrays against the new env.
    routeIncVar_ = IloNumVarArray(env_, 0.0, 0.0, IloInfinity, ILOFLOAT);
    zIncVar_     = IloNumVarArray(env_, 0.0, 0.0, IloInfinity, ILOFLOAT);
    routeSolVar_ = IloNumVarArray(env_, 0.0, 0.0, IloInfinity, ILOFLOAT);
    zSolVar_     = IloNumVarArray(env_, 0.0, 0.0, IloInfinity, ILOFLOAT);
}




