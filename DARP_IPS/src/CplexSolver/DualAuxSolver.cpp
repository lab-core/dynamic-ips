//
// Created by Elahe Amiri on 2025-02-01.
//

#include "DualAuxSolver.h"
#include "data/Instance.h"


DualAuxSolver::DualAuxSolver(int nbRequestTask, int nbVehicles) : Model_(env_), nbRequestTask_(nbRequestTask),
nbVehicles_(nbVehicles), routeConst_(IloRangeArray(env_)), zConst_(IloRangeArray(env_)), objConst_(IloRangeArray(env_)){
    solveTime_ = new myTools::Timer(); solveTime_->init();
    objValue_ = 0;
}

DualAuxSolver::~DualAuxSolver() {
    delete solveTime_;
    env_.end();
}

// this function initialized the model
void DualAuxSolver::initializeModel(const PInstance &pInst) {

    // define variables
    requestDuals_ = IloNumVarArray(env_, nbRequestTask_, -IloInfinity, IloInfinity);
    vehicleDuals_ = IloNumVarArray(env_, nbVehicles_, -IloInfinity, 0.0);

    // Naming variables
    for (auto & requestObj : pInst->requests_) {
        std::ostringstream rName;
        rName << "R_" << requestObj->taskIndex_;
        requestDuals_[requestObj->taskIndex_].setName(rName.str().c_str());
    }

    for (auto & vehicleObj : pInst->vehicles_) {
        std::ostringstream vName;
        vName << "V_" << vehicleObj->vehicleID_;
        vehicleDuals_[vehicleObj->vehicleID_].setName(vName.str().c_str());
    }
}

void DualAuxSolver::addRouteExpr(const PRoute &route) {
    IloExpr expr(env_);
    for (auto & requestObj : route->routeRequests_) {
        expr += requestDuals_[requestObj->taskIndex_];
    }
    expr += vehicleDuals_[route->vehicleID_];
    routeExpr_.emplace_back(std::move(expr));
}

void DualAuxSolver::buildModel(vector<PRoute> &RMPRoutes, vector<PRequest> &Requests) {
    Model_.end();
    Model_ = IloModel(env_);
    Cplex_ = IloCplex(Model_);
    routeConst_ = IloRangeArray(env_);
    zConst_ = IloRangeArray(env_);
    objConst_ = IloRangeArray(env_);
    routeExpr_.clear();
    zExpr_.clear();
    objExpr_.clear();

    epsilonVar_ = IloNumVarArray(env_,  static_cast<int>(RMPRoutes.size()), 0.0, IloInfinity);
    deltaVar_ = IloNumVarArray(env_, static_cast<int>(Requests.size()), 0.0, IloInfinity);

    IloExpr obj_p1(env_);
    for (int i = 0; i < nbRequestTask_; ++i)
        obj_p1 += requestDuals_[i];
    for (int v = 0; v < nbVehicles_; ++v)
        obj_p1 += vehicleDuals_[v];
    objExpr_.push_back(obj_p1);

    IloExpr obj_p2(env_);

    for (int r = 0; r < RMPRoutes.size(); ++r) {
        addRouteExpr(RMPRoutes[r]);
        obj_p2 += epsilonVar_[r];

        std::ostringstream eName;
        eName << "e_" << r;
        epsilonVar_[r].setName(eName.str().c_str());
    }
    zExpr_.resize(Requests.size());
    for (int i = 0; i < Requests.size(); ++i) {
        if (Requests[i]->committedPickTime_ == LARGE_CONSTANT) {
            zExpr_[Requests[i]->taskIndex_] = requestDuals_[Requests[i]->taskIndex_];
            obj_p2 += deltaVar_[i];

            std::ostringstream eName;
            eName << "d_" << i;
            deltaVar_[i].setName(eName.str().c_str());
        }
    }
    objExpr_.push_back(obj_p2);
    /*Model_.add(IloMinimize(env_, std::move(obj)));
    obj.end();*/
}

void DualAuxSolver::solveModel(const PInstance &pInst, const InputPaths &inputPaths) {
    solveTime_->start();
    try {
        Model_.add(IloMaximize(env_, objExpr_[0] - 100 *objExpr_[1]));
        Model_.add(routeConst_);
        Model_.add(zConst_);
        Cplex_.extract(Model_);

        myTools::CoutRedirector redirector(inputPaths.getOutputSolverLog(), "AUX Dual");
        Cplex_.setParam(IloCplex::Param::Threads, pInst->parameters_->nbThreads_);
        Cplex_.setParam(IloCplex::Param::RootAlgorithm, 2);
        Cplex_.setParam(IloCplex::Param::NodeAlgorithm, 2);
        Cplex_.setParam(IloCplex::Param::Preprocessing::Presolve, 0);
        if (!Cplex_.solve()) {
            solveTime_->stop();
            env_.out() << Model_ << std::endl;
            std::cout << "Failed to optimize the AUX" << std::endl;
            // Check if the model is infeasible
            if (Cplex_.getStatus() == IloAlgorithm::Infeasible) {
                std::cout << "Model is infeasible. Running conflict refinement..." << std::endl;

                // Prepare arrays for conflict refinement
                IloRangeArray infeasConstraints(env_);
                IloNumArray preferences(env_);

                for (int i = 0; i < routeConst_.getSize(); ++i) {
                    infeasConstraints.add(routeConst_[i]); // Ensure constraints are added
                    preferences.add(1.0);
                }

                for (int i = 0; i < zConst_.getSize(); ++i) {
                    infeasConstraints.add(zConst_[i]); // Ensure constraints are added
                    preferences.add(1.0);
                }

                // Attempt to refine conflict
                bool conflictFound = Cplex_.refineConflict(infeasConstraints, preferences);

                if (conflictFound) {
                    std::cout << "Conflicting constraints detected:" << std::endl;
                    IloCplex::ConflictStatusArray conflict(env_, infeasConstraints.getSize());
                    conflict = Cplex_.getConflict(infeasConstraints);
                    for (int i = 0; i < infeasConstraints.getSize(); ++i) {
                        if (conflict[i] == IloCplex::ConflictMember)
                            std::cout << infeasConstraints[i] << std::endl;
                    }
                } else {
                    std::cout << "No specific conflict found, infeasibility remains unclear." << std::endl;
                }
            }
        }
        else {
            solveTime_->stop();
            objValue_ = 0;

            IloNumArray piVal(env_);
            IloNumArray sigmaVal(env_);
            IloNumArray eVal(env_);
            IloNumArray dVal(env_);

            Cplex_.getValues(requestDuals_, piVal);
            Cplex_.getValues(vehicleDuals_, sigmaVal);
            Cplex_.getValues(epsilonVar_, eVal);
            Cplex_.getValues(deltaVar_, dVal);

            for (auto & requestObj : pInst->requests_) {
            //    if (requestObj->dual_ != piVal[requestObj->taskIndex_])
            //        std::cout << requestObj->dual_ << "  :  " << piVal[requestObj->taskIndex_] << std::endl;
                requestObj->dual_ = static_cast<float>(piVal[requestObj->taskIndex_]);
                objValue_ += static_cast<float>(piVal[requestObj->taskIndex_]);
            }
            for (auto & vehicleObj : pInst->vehicles_) {
            //    if (vehicleObj->dual_ != sigmaVal[vehicleObj->vehicleID_])
            //        std::cout << vehicleObj->dual_ << "  :  " << sigmaVal[vehicleObj->vehicleID_] << std::endl;
                vehicleObj->dual_ = static_cast<float>(sigmaVal[vehicleObj->vehicleID_]);
                objValue_ += static_cast<float>(sigmaVal[vehicleObj->vehicleID_]);
            }
            // for (int r = 0; r < (int) epsilonVar_.getSize(); ++r) {
            //     if (eVal[r] != 0)
            //         std::cout << "epsilon " << r << " : " << eVal[r] << std::endl;
            // }
            // for (int i = 0; i < (int) deltaVar_.getSize(); ++i) {
            //     if (dVal[i] != 0)
            //         std::cout << "delta " << i << " : " << dVal[i] << std::endl;
            // }
        }
        // std::cout << "Dual objective: " << objValue_ << std::endl;

        Cplex_.clearModel();

        Model_.remove(routeConst_);
        Model_.remove(zConst_);
        Model_.remove(objConst_);
        routeConst_.end();
        zConst_.end();
        objConst_.end();

    }
    catch (IloException& e) {
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
}
