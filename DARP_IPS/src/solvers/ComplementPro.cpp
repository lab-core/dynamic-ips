//
// Created by Ella on 2021-11-17.
//

#include "ComplementPro.h"
ComplementPro::ComplementPro() : MasterModeler() {

    routeIncVar_ = IloNumVarArray(env_, 0.0, 0.0, ILOFLOAT);
    zIncVar_ = IloNumVarArray(env_, 0.0, 0.0, ILOFLOAT);
    routeSolVar_ = IloNumVarArray (env_, 0.0, 0.0, ILOFLOAT);
    zSolVar_ = IloNumVarArray (env_, 0.0, 0.0, ILOFLOAT);
    status_ = NOT_SOLVED;
}

// this function initialized the model and define empty set of constraints
void ComplementPro::initializeCPModel(PInstance &pInst) {
    int rhs = 0;
    initializeModel(pInst, rhs);
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
        IloNumArray columnVar(env_, (signed) orderToRequest_.size());
        MasterModeler::createPattern(columnVar, newRoute, POSITIVE);
        IloNumVar numVar = IloNumVar(objFunction_(newRoute->totalDelay_) + requestConst_(columnVar)
                                     + vehicleConst_[newRoute->vehicleID_](1)
                                     + normalConst_[0](newRoute->incompatibilityDegree_));
        numVar.setName(newRoute->name_);
        routeVar.add(numVar);
    }
}

// this function build the model at each iteration
void ComplementPro::buildModel(PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution) {
    // model initialization (defining empty set of constraints and adding objective)
    ResetCPModel();
//    clearModel();
    initializeCPModel(pInst);

    // adding solution route columns
    for (auto & routeSol: routeSolution)
        addRouteVar(routeSolVar_, routeSol, NEGATIVE);

    // adding incompatible route columns
    for (auto & routeAdd: routesToAdd_)
        addRouteVar(routeIncVar_, routeAdd, POSITIVE);

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
}

// this function update the model and variables
void ComplementPro::updateModel(PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution) {

    // adding solution route columns
    for (auto & routeSol: routeSolution)
        addRouteVar(routeSolVar_, routeSol, NEGATIVE);

    // adding incompatible route columns
    for (auto & routeAdd: routesToAdd_)
        addRouteVar(routeIncVar_, routeAdd, POSITIVE);

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
}


// this function solve the model
void ComplementPro::solveModel(PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
                               std::unordered_map<std::string, PRoute> &generatedRoutes) {
    Cplex_ = IloCplex(Model_);
    if ( !Cplex_.solve() ) {
        status_ = INFEASIBLE;
        std::cout << "Failed to optimize the problem" << std::endl;
//        throw Tools::myException("the Complementary model is infeasible!!!", __LINE__);
    }

    else {

        // printing solution status
        std::cout << MasterModeler::toString();

        // getting dual values
        requestDuals_.clear();
        vehicleDuals_.clear();

        // define dual container size
        requestDuals_ = IloNumArray(env_, pInst->nbRequests_);
        vehicleDuals_ = IloNumArray(env_, pInst->nbVehicles_);

        std::cout << "COMPLEMENTARY DUALS:" << std::endl;
        for (auto &requestObj: pInst->requests_) {
            if (requestObj->requestStatus_ == NO_ACTION) {
                int rowIndex = requestToOrder_[requestObj->getRequestId()];
                requestDuals_[rowIndex] = Cplex_.getDual(requestConst_[rowIndex]);
                requestObj->dual_ = requestDuals_[rowIndex];
                requestObj->CPDual_ = requestDuals_[rowIndex];
 //               std::cout << "requestDuals[" << requestObj->getRequestId() << "]: " << requestObj->dual_<< std::endl;
            }

        }

        std::cout << "VEHICLE DUALS:" << std::endl;
        for (auto &vehicleObj: pInst->vehicles_) {
            vehicleDuals_[vehicleObj->vehicleID_] = Cplex_.getDual(vehicleConst_[vehicleObj->vehicleID_]);
            vehicleObj->dual_ = vehicleDuals_[vehicleObj->vehicleID_];
            vehicleObj->CPDual_ = vehicleDuals_[vehicleObj->vehicleID_];
//            std::cout << "vehicleDuals[" << vehicleObj->vehicleID_ << "]: " << vehicleObj->dual_ << std::endl;
        }


        // saving the result and remove out of base variables
        if (Cplex_.getObjValue() < -0.01) {
            status_ = NEGATIVE_VALUE;
            // check the solution to be column disjoint
            vector<PRequest> zResult;
            vector<PRoute> routeResult;

            for (int r = 0; r < routeIncVar_.getSize(); ++r) {
                if (Cplex_.getValue(routeIncVar_[r]) > 0) {
                    std::cout << routeIncVar_[r].getName() << std::endl;
                    routeResult.push_back(generatedRoutes[routeIncVar_[r].getName()]);
                }
            }
            for (int i = 0; i < zIncVar_.getSize(); ++i) {
                if (Cplex_.getValue(zIncVar_[i]) > 0) {
                    std::cout << zIncVar_[i].getName() << std::endl;
                    zResult.push_back(pInst->nameToRequest_[zIncVar_[i].getName()]);
                }
            }

            if (isColumnDisjoint(zResult, routeResult, requestToOrder_, pInst->nbVehicles_)) {


                // remove outgoing variable
                for (int r = (int)routeSolVar_.getSize() - 1; r >= 0; --r) {
                    if (Cplex_.getValue(routeSolVar_[r]) > 0) {
                        routeSolution.erase(routeSolution.begin() + r);
                    }
                }
                for (int i = (int)zSolVar_.getSize() - 1; i >= 0; --i) {
                    if (Cplex_.getValue(zSolVar_[i]) > 0) {
                        zSolution.erase(zSolution.begin() + i);
                    }
                }

                // add incoming variables
                for (int r = 0; r < routeIncVar_.getSize(); ++r) {
                    if (Cplex_.getValue(routeIncVar_[r]) > 0) {
                        routeSolution.push_back(generatedRoutes[routeIncVar_[r].getName()]);
                        pInst->vehicles_[generatedRoutes[routeIncVar_[r].getName()]->vehicleID_]->setCurrentRoute(
                                generatedRoutes[routeIncVar_[r].getName()]);
                    }
                }
                for (int i = 0; i < zIncVar_.getSize(); ++i) {
                    if (Cplex_.getValue(zIncVar_[i]) > 0) {
                        zSolution.push_back(pInst->nameToRequest_[zIncVar_[i].getName()]);
                    }
                }
                int nbRequests = 0;
                for (auto &requestObj: pInst->requests_) {
                    if (requestObj->requestStatus_ == NO_ACTION)
                        nbRequests++;
                }
                std::cout << "# from " << nbRequests << " request, " << nbRequests - zSolution.size()
                          << " are selected to served." << std::endl;
            }
            else
            {
                status_ = FRACTIONAL;
                std::cout << "The solution is not column disjoint!!!!!!!" << std::endl;
                fractionalRoutes_.clear();
                fractionalZ_.clear();
                // add incoming variables
                for (int r = 0; r < routeIncVar_.getSize(); ++r) {
                    if (Cplex_.getValue(routeIncVar_[r]) > 0) {
 //                       std::cout << routeIncVar_[r].getName() << " : " << Cplex_.getValue(routeIncVar_[r]) << std::endl;
                        fractionalRoutes_.push_back(generatedRoutes[routeIncVar_[r].getName()]);
                    }
                }
                for (int i = 0; i < zIncVar_.getSize(); ++i) {
                    if (Cplex_.getValue(zIncVar_[i]) > 0) {
//                        std::cout << zIncVar_[i].getName() << " : " << Cplex_.getValue(zIncVar_[i]) << std::endl;
                        fractionalZ_.push_back(pInst->nameToRequest_[zIncVar_[i].getName()]);
                    }
                }
            }
        } else
            status_ = POSITIVE_VALUE;
    }
}

void ComplementPro::solveModelIndex(PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
                                    std::unordered_map<std::string, PRoute> &generatedRoutes) {
    Cplex_ = IloCplex(Model_);

    if ( !Cplex_.solve() ) {
        status_ = INFEASIBLE;
        std::cout << "Failed to optimize the problem" << std::endl;
//        throw Tools::myException("the Complementary model is infeasible!!!", __LINE__);
    }

    else {

        // printing solution status
        std::cout << MasterModeler::toString();

        // getting dual values
        requestDuals_.clear();
        vehicleDuals_.clear();

        // define dual container size
        requestDuals_ = IloNumArray(env_, pInst->nbRequests_);
        vehicleDuals_ = IloNumArray(env_, pInst->nbVehicles_);

        std::cout << "COMPLEMENTARY DUALS:" << std::endl;
        for (auto &requestObj: pInst->requests_) {
            if (requestObj->requestStatus_ == NO_ACTION) {
                int rowIndex = requestToOrder_[requestObj->getRequestId()];
                requestDuals_[rowIndex] = Cplex_.getDual(requestConst_[rowIndex]);
                requestObj->dual_ = requestDuals_[rowIndex];
                requestObj->CPDual_ = requestDuals_[rowIndex];
                //               std::cout << "requestDuals[" << requestObj->getRequestId() << "]: " << requestObj->dual_<< std::endl;
            }

        }

        std::cout << "VEHICLE DUALS:" << std::endl;
        for (auto &vehicleObj: pInst->vehicles_) {
            vehicleDuals_[vehicleObj->vehicleID_] = Cplex_.getDual(vehicleConst_[vehicleObj->vehicleID_]);
            vehicleObj->dual_ = vehicleDuals_[vehicleObj->vehicleID_];
            vehicleObj->CPDual_ = vehicleDuals_[vehicleObj->vehicleID_];
//            std::cout << "vehicleDuals[" << vehicleObj->vehicleID_ << "]: " << vehicleObj->dual_ << std::endl;
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
            for (int r = (int)routeIncVar_.getSize() - 1; r >= 0; --r) {
                if (Cplex_.getValue(routeIncVar_[r]) > 0) {
                    std::cout << routeIncVar_[r].getName() << std::endl;
                    routeResult.push_back(generatedRoutes[routeIncVar_[r].getName()]);
                    InRouteVar.push_back(r);
                }
            }
            for (int i = (int)zIncVar_.getSize() - 1; i >= 0; --i) {
                if (Cplex_.getValue(zIncVar_[i]) > 0) {
                    std::cout << zIncVar_[i].getName() << std::endl;
                    zResult.push_back(pInst->nameToRequest_[zIncVar_[i].getName()]);
                    InRequestVar.push_back(i);
                }
            }

            // determine outgoing variables
            for (int r = (int)routeSolVar_.getSize() - 1; r >= 0; --r) {
                if (Cplex_.getValue(routeSolVar_[r]) > 0) {
                    OutRouteVar.push_back(r);
                }
            }
            for (int i = (int)zSolVar_.getSize() - 1; i >= 0; --i) {
                if (Cplex_.getValue(zSolVar_[i]) > 0) {
                    OutRequestVar.push_back(i);
                }
            }

            if (isColumnDisjoint(zResult, routeResult, requestToOrder_, pInst->nbVehicles_)) {
                // remove outgoing variable
                for (auto & r : OutRouteVar) {
                    addRouteVar(routeIncVar_, routeSolution[r], POSITIVE);
                    routeSolution.erase(routeSolution.begin() + r);
                    routeSolVar_[r].end();
                    routeSolVar_.remove(r,1);
                }

                for (auto & i : OutRequestVar){
                    addZVar(zIncVar_, zSolution[i], POSITIVE);
                    zSolution.erase(zSolution.begin() + i);
                    zSolVar_[i].end();
                    zSolVar_.remove(i,1);
                }

                // add incoming variables
                for (auto & r : InRouteVar){
                    routeSolution.push_back(generatedRoutes[routeIncVar_[r].getName()]);
                    addRouteVar(routeSolVar_, routeSolution.back(), NEGATIVE);
                    pInst->vehicles_[generatedRoutes[routeIncVar_[r].getName()]->vehicleID_]->setCurrentRoute(
                            generatedRoutes[routeIncVar_[r].getName()]);
                    routeIncVar_[r].end();
                    routeIncVar_.remove(r,1);
                }
                for (auto & i : InRequestVar){
                    zSolution.push_back(pInst->nameToRequest_[zIncVar_[i].getName()]);
                    addZVar(zSolVar_, zSolution.back(), NEGATIVE);
                    zIncVar_[i].end();
                    zIncVar_.remove(i,1);
                }

                int nbRequests = 0;
                for (auto &requestObj: pInst->requests_) {
                    if (requestObj->requestStatus_ == NO_ACTION)
                        nbRequests++;
                }
                std::cout << "# from " << nbRequests << " request, " << nbRequests - zSolution.size()
                          << " are selected to served." << std::endl;
            }
            else
            {
                status_ = FRACTIONAL;
                std::cout << "The solution is not column disjoint!!!!!!!" << std::endl;
                fractionalRoutes_.clear();
                fractionalZ_.clear();
                // add incoming variables
                for (int r = 0; r < routeIncVar_.getSize(); ++r) {
                    if (Cplex_.getValue(routeIncVar_[r]) > 0) {
                        fractionalRoutes_.push_back(generatedRoutes[routeIncVar_[r].getName()]);
                    }
                }
                for (int i = 0; i < zIncVar_.getSize(); ++i) {
                    if (Cplex_.getValue(zIncVar_[i]) > 0) {
                        fractionalZ_.push_back(pInst->nameToRequest_[zIncVar_[i].getName()]);
                    }
                }
            }
        } else
            status_ = POSITIVE_VALUE;
    }
}

// this function check the situation of the CP solution to be column disjoint
bool ComplementPro::isColumnDisjoint(vector<PRequest> &zResults, vector<PRoute> &routeResults,
                                     std::unordered_map<unsigned int, int>& requestToOrder, int nbVehicle) {
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero((signed)requestToOrder.size()+ nbVehicle, (signed)zResults.size() + (signed)routeResults.size());
    for (int r = 0; r < routeResults.size(); ++r) {
        for (int i = 0; i < routeResults[r]->routeRequests_.size(); ++i)
            A(requestToOrder[routeResults[r]->routeRequests_[i]], r) = 1;
        A((signed)requestToOrder.size()+routeResults[r]->vehicleID_,r) = 1;
    }
    for (int i = 0; i < zResults.size(); ++i) {
        A((signed)requestToOrder[zResults[i]->getRequestId()],(signed)routeResults.size()+i) = 1;
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

void ComplementPro::ResetCPModel() {
    try {
        /*int modelExist = 0;
        for (int r = (int)routeSolVar_.getSize() - 1; r >= 0; --r) {
            routeSolVar_[r].end();
            routeSolVar_.remove(r, 1);
            modelExist = 1;
        }
        for (int i = (int)zSolVar_.getSize() - 1; i >= 0; --i) {
            zSolVar_[i].end();
            zSolVar_.remove(i, 1);
            modelExist = 1;
        }
        for (int r = (int)routeIncVar_.getSize() - 1; r >= 0; --r) {
            routeIncVar_[r].end();
            routeIncVar_.remove(r, 1);
            modelExist = 1;
        }
        for (int i = (int)zIncVar_.getSize() - 1; i >= 0; --i) {
            zIncVar_[i].end();
            zIncVar_.remove(i, 1);
            modelExist = 1;
        }
        if (modelExist == 1) {
            Model_.remove(requestConst_);
            Model_.remove(vehicleConst_);
            Model_.remove(normalConst_);
        }*/


        routeSolVar_.endElements();
        routeSolVar_.clear();
        zSolVar_.endElements();
        zSolVar_.clear();
        routeIncVar_.endElements();
        routeIncVar_.clear();
        zIncVar_.endElements();
        zIncVar_.clear();
        Model_.remove(requestConst_);
        Model_.remove(vehicleConst_);
        Model_.remove(normalConst_);
    }
    catch (IloException& e) {
        std::cout << e << std::endl;
    }

}








