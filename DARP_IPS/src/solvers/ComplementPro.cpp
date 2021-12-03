//
// Created by Ella on 2021-11-17.
//

#include "ComplementPro.h"
ComplementPro::ComplementPro() : MasterModeler() {

    routeIncVar_ = IloNumVarArray(env_, 0.0, 0.0, ILOFLOAT);
    zIncVar_ = IloNumVarArray(env_, 0.0, 0.0, ILOFLOAT);
    routeSolVar_ = IloNumVarArray (env_, 0.0, 0.0, ILOFLOAT);
    zSolVar_ = IloNumVarArray (env_, 0.0, 0.0, ILOFLOAT);
}

// this function initialized the model and define empty set of constraints
void ComplementPro::initializeCPModel(PInstance &pInst) {
    int rhs = 0;
    initializeModel(pInst, rhs);

    // defining constraints
    /*requestRHS_ = IloNumArray(env_, pInst->nbRequests_, 0);
    vehicleRHS_ = IloNumArray(env_, pInst->nbVehicles_, 0);
*/

    normalConst_ = IloRangeArray(env_,1,1.0,1.0);

    Model_.add(normalConst_);
}

// this function adds zVar to the model
void ComplementPro::addZVar(IloNumVarArray zVar, PRequest &request, VarSign sign) {
    if (sign == NEGATIVE)
        MasterModeler::addZVar(zVar, request, sign);
    else if (sign == POSITIVE) {
        IloNumVar numVar = IloNumVar(objFunction_(request->penalty_) +
                                     requestConst_[requestToOrder_[request->getRequestId()]](1) +
                                     normalConst_[0](1));
        numVar.setName(request->name_);
        zVar.add(numVar);
    }
}

// this function adds routeVar to the model
void ComplementPro::addRouteVar(IloNumVarArray routeVar, PRoute &newRoute, VarSign sign) {
    if (sign == NEGATIVE)
        MasterModeler::addRouteVar(routeVar, newRoute, sign);
    else {
        IloNumArray columnVar(env_, orderToRequest_.size());
        MasterModeler::createPattern(columnVar, newRoute, POSITIVE);
        IloNumVar numVar = IloNumVar(objFunction_(newRoute->totalDelay_) + requestConst_(columnVar)
                                     + vehicleConst_[newRoute->vehicleID_](1)
                                     + normalConst_[0](newRoute->incompatibilityDegree));
        numVar.setName(newRoute->name_);
        routeVar.add(numVar);
    }
}

// this function build the model at each iteration
void ComplementPro::buildModel(PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution) {
    // model initialization (defining empty set of constraints and adding objective)
    initializeCPModel(pInst);

    // adding solution route columns
    for (int r = 0; r < routeSolution.size(); ++r)
        addRouteVar(routeSolVar_, routeSolution[r], NEGATIVE);

    /*// adding solution z columns
    for (int i = 0; i < zSolution.size(); ++i)
        addZVar(zSolVar_, zSolution[i], NEGATIVE);*/

    // adding incompatible route columns
    for (int r = 0; r < routesToAdd_.size(); ++r)
        addRouteVar(routeIncVar_, routesToAdd_[r], POSITIVE);

    // adding z columns
    for (int i = 0; i < pInst->nbRequests_; ++i) {
        int flagAdd =0;
        for (int j = 0; j < zSolution.size(); ++j) {
            if (pInst->requests_[i] == zSolution[j]) {
                addZVar(zSolVar_, zSolution[j], NEGATIVE);
                flagAdd = 1;
                break;
            }
        }
        if (flagAdd == 0)
            addZVar(zIncVar_, pInst->requests_[i], POSITIVE);
    }
    /*std::cout << "# number of route IncColumns after build: " << routeIncVar_.getSize() << std::endl;
    std::cout << "# number of z IncColumns after build: " << zIncVar_.getSize() << std::endl;
    std::cout << "# number of route SolColumns after build: " << routeSolVar_.getSize() << std::endl;
    std::cout << "# number of z SolColumns after build: " << zSolVar_.getSize() << std::endl;*/
}

// this function solve the model
void ComplementPro::solveModel(PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
                               std::map<std::string, PRoute> &generatedRoutes) {
    int optimalFlag = 0;
    Cplex_ = IloCplex(Model_);
//    env_.out() << Model_;
    Cplex_.solve();

    // printing solution status
    std::cout << MasterModeler::toString();

    // saving the result and remove out of base variables
    if (Cplex_.getObjValue() < 0) {
        status_ = NEGATIVE_VALUE;
        // check the solution to be column disjoint
        vector<PRequest> zResult;
        vector<PRoute> routeResult;

        for (int r = 0; r < routeIncVar_.getSize(); ++r) {
            if (Cplex_.getValue(routeIncVar_[r]) > 0) {
                routeResult.push_back(generatedRoutes[routeIncVar_[r].getName()]);
            }
        }
        for (int i = 0; i < zIncVar_.getSize(); ++i) {
            if (Cplex_.getValue(zIncVar_[i]) > 0) {
                zResult.push_back(pInst->nameToRequest_[zIncVar_[i].getName()]);
            }
        }
        if (isColumnDisjoint(zResult, routeResult, requestToOrder_)) {
            optimalFlag = 1;
            // getting dual values
            requestDuals_.clear();
            vehicleDuals_.clear();

            // define dual container size
            requestDuals_ = IloNumArray(env_, pInst->nbRequests_);
            vehicleDuals_ = IloNumArray(env_, pInst->nbVehicles_);

            std::cout << "COMPLEMENTARY DUALS:" << std::endl;
            for (int r = 0; r < pInst->nbRequests_; ++r) {
                requestDuals_[r] = Cplex_.getDual(requestConst_[r]);
                std::cout << "requestDuals[" << r <<"]: " << requestDuals_[r] << std::endl;
            }
            for (int v = 0; v < pInst->nbVehicles_; ++v) {
                vehicleDuals_[v] = Cplex_.getDual(vehicleConst_[v]);
                std::cout << "vehicleDuals[" << v <<"]: " << vehicleDuals_[v] << std::endl;
            }

            // remove outgoing variable
            for (int r = routeSolVar_.getSize()-1; r >= 0; --r) {
                if  (Cplex_.getValue(routeSolVar_[r]) > 0) {
                    routeSolution.erase(routeSolution.begin()+r);
                }
            }
            for (int i = zSolVar_.getSize()-1; i >=0; --i) {
                if (Cplex_.getValue(zSolVar_[i]) > 0) {
                    zSolution.erase(zSolution.begin()+i);
                }
            }

            // add incoming variables
            for (int r = 0; r < routeIncVar_.getSize(); ++r) {
                if (Cplex_.getValue(routeIncVar_[r]) > 0) {
                    routeSolution.push_back(generatedRoutes[routeIncVar_[r].getName()]);
                    pInst->vehicles_[generatedRoutes[routeIncVar_[r].getName()]->vehicleID_]->setCurrentRoute(generatedRoutes[routeIncVar_[r].getName()]);
                }
            }
            for (int i = 0; i < zIncVar_.getSize(); ++i) {
                if (Cplex_.getValue(zIncVar_[i]) > 0) {
                    zSolution.push_back(pInst->nameToRequest_[zIncVar_[i].getName()]);
                }
            }
            std::cout << "# from " << pInst->nbRequests_ << " request, " << pInst->nbRequests_ - zSolution.size()
                      << " are selected to served." << std::endl;
            for (int r = 0; r < routeSolution.size(); ++r) {
                std::cout << routeSolution[r]->toString();
            }
        }
        else {
            status_ = FRACTIONAL;
            std::cout << "The solution is not column disjoint!!!!!!!" << std::endl;
        }
    }
    else
        status_ = POSITIVE_VALUE;
}

// this function check the situation of the CP solution to be column disjoint
bool ComplementPro::isColumnDisjoint(vector<PRequest> &zResults, vector<PRoute> &routeResults,
                                     std::map<int, int>& requestToOrder) {
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(requestToOrder.size(), zResults.size()+routeResults.size());
    for (int r = 0; r < routeResults.size(); ++r) {
        for (int i = 0; i < routeResults[r]->routeRequests.size(); ++i)
            A(requestToOrder[routeResults[r]->routeRequests[i]],r) = 1;
    }
    for (int i = 0; i < zResults.size(); ++i) {
        A(requestToOrder[zResults[i]->getRequestId()],routeResults.size()+i) = 1;
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

// Display function
std::string ComplementPro::toString() const {
    std::stringstream repStr;
    repStr << std::endl;
    repStr << "# ====================  COMPLEMENTARY PROBLEM SOLVED  ==================== " << std::endl;
    repStr << "# COMPLEMENTARY PROBLEM SOLVED: " << std::endl;
    repStr << MasterModeler::toString();
    return repStr.str();
}





