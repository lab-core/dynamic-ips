//
// Created by Ella on 2021-10-15.
//

#include "MIPSolver.h"
//-----------------------------------------------------------------------------
//  Contains MIP formulation to solve with Cplex
//  This is just for testing the results and comparison
//-----------------------------------------------------------------------------

MIPSolver::MIPSolver() : Model_(env_), objFunction_(IloMinimize(env_)), constraints_(env_) {
    solveTime_ = new myTools::Timer(); solveTime_->init();
    objValue_ = 0;
}

MIPSolver::~MIPSolver() {
    delete solveTime_;
    env_.end();
}


// this function initialized the model
void MIPSolver::initializeModel(PInstance &pInst) {
    pInst->setNodeIndices();
    // update order of requests
    nbNodes_ = pInst->instGraph_->nbNodes_;
    nbVehicles_ = pInst->nbVehicles_;
    nbRequests_ = pInst->nbTasks_;

    // define variables
    Z_ = IloNumVarArray(env_, nbRequests_, 0.0, 1.0,ILOINT);
    X_ = IloNumVar3D(env_, nbVehicles_);
    U_ = IloNumVar2D(env_, nbVehicles_);
    W_ = IloNumVar2D(env_, nbVehicles_);

    // Naming and adding Z variables
    for (int i = 0; i < nbRequests_; ++i)
        Z_[i].setName(pInst->instGraph_->pickNodes_[i]->related_Request_->name_);

    for (auto & vehicleObj : pInst->vehicles_) {
        int v = vehicleObj->vehicleID_;
        int nbNodes = nbRequests_ * 2 + 2 + vehicleObj->onboards_.size();
        U_[v] = IloNumVarArray (env_, nbNodes, 0, IloInfinity, ILOFLOAT);
        W_[v] = IloNumVarArray (env_, nbNodes, 0, IloInfinity, ILOFLOAT);
        X_[v] = IloNumVar2D(env_, nbNodes);
        for (int i = 0; i < nbNodes; ++i) {

            // Generate a name like "U_v_i"
            std::ostringstream uName;
            uName << "U_v" << v << "_i" << i;
            U_[v][i].setName(uName.str().c_str());

            // Generate a name like "W_v_i"
            std::ostringstream wName;
            wName << "W_v" << v << "_i" << i;
            W_[v][i].setName(wName.str().c_str());

            X_[v][i] = IloNumVarArray(env_, nbNodes, 0.0, 1.0, ILOINT);
            for (int j = 0; j < nbNodes; ++j) {
                // Generate a name like "X_v_i_j"
                std::ostringstream xName;
                xName << "X_v" << v << "_i" << i << "_j" << j;
                X_[v][i][j].setName(xName.str().c_str());
            }
        }
    }
}

void MIPSolver::buildModel(PInstance &pInst){
    solveTime_->start();
    int i,j,v,i_n;
    std::vector<PNode> nodes = concatenateVectors(pInst->instGraph_->pickNodes_, pInst->instGraph_->dropNodes_);

    // define objective function
    IloExpr objExpr(env_);
    for (auto & pickNode : pInst->instGraph_->pickNodes_) {
        i = pickNode->related_Request_->taskIndex_;
        objExpr += Z_[i] * pickNode->related_Request_->penalty_;
        for (auto & vehicleObj : pInst->vehicles_) {
            objExpr += (U_[vehicleObj->vehicleID_][pickNode->xIndex_] - pickNode->readyTime_);
        }
    }
//    Model_.add(IloMinimize(env_, objExpr));

    // -----------------------------------------
    // defining constraints
    // -----------------------------------------

    // constraints 1b -------------------
    for (auto & pickNode : pInst->instGraph_->pickNodes_) {
        IloExpr expr_1b(env_);
        i = pickNode->xIndex_;
        expr_1b += Z_[pickNode->related_Request_->taskIndex_];
        for (auto & vehicleObj : pInst->vehicles_) {

            // other pick/drop nodes
            for (auto & otherNode : nodes) {
                if (otherNode->xIndex_ != pickNode->xIndex_)
                    expr_1b += X_[vehicleObj->vehicleID_][i][otherNode->xIndex_];
            }

            // vehicle onboards
            for (auto & nodeID : vehicleObj->onboards_) {
                expr_1b += X_[vehicleObj->vehicleID_][i][pInst->instGraph_->nodes_[nodeID]->xIndex_];
            }

            // vehicle sink node
            expr_1b += X_[vehicleObj->vehicleID_][i][vehicleObj->sinkNode_->xIndex_];
        }
        IloRange c_1b = (expr_1b == 1);
        Model_.add(c_1b);
        constraints_.add(c_1b);
    }

    IloExpr expr_extract(env_);

    for (auto & vehicleObj : pInst->vehicles_) {
        int sourceIndex = vehicleObj->departNode_->xIndex_;
        int sinkIndex = vehicleObj->sinkNode_->xIndex_;

        v = vehicleObj->vehicleID_;
        std::vector<PNode> nodesWithOnboards = nodes;
        for (auto & nodeID : vehicleObj->onboards_)
            nodesWithOnboards.push_back(pInst->instGraph_->nodes_[nodeID]);
        std::vector<PNode> allNodes = nodesWithOnboards;
        allNodes.push_back(vehicleObj->sinkNode_);
        allNodes.push_back(vehicleObj->departNode_);

        // constraints 1d 1e -------------------
        IloExpr expr_1d(env_);
        IloExpr expr_1e(env_);
        for (auto & otherNode : nodesWithOnboards) {
            expr_1d += X_[v][sourceIndex][otherNode->xIndex_];
            expr_1e += X_[v][otherNode->xIndex_][sinkIndex];

            expr_extract += X_[v][otherNode->xIndex_][sourceIndex];
            expr_extract += X_[v][sinkIndex][otherNode->xIndex_];
        }
        expr_1d += X_[v][sourceIndex][sinkIndex];
        expr_1e += X_[v][sourceIndex][sinkIndex];
        expr_extract += X_[v][sinkIndex][sourceIndex] + X_[v][sinkIndex][sinkIndex] +
            X_[v][sourceIndex][sourceIndex];

        IloRange c_1d = (expr_1d == 1);
        IloRange c_1e = (expr_1e == 1);

        Model_.add(c_1d);
        Model_.add(c_1e);
        constraints_.add(c_1d);
        constraints_.add(c_1e);

        // constraints 1i , 1j -------------------
        IloRange c_1i = (U_[v][sourceIndex] >= vehicleObj->departTime_);
        IloRange c_1j = (U_[v][sinkIndex] <= vehicleObj->endTime_);
        Model_.add(c_1i);
        Model_.add(c_1j);
        constraints_.add(c_1i);
        constraints_.add(c_1j);


        for (auto & pickNode : pInst->instGraph_->pickNodes_) {
            i = pickNode->xIndex_;
            i_n = pickNode->pairNode_->xIndex_;
            // constraints 1f -------------------
            IloExpr expr_1f(env_);
            for (auto & otherNode : nodesWithOnboards) {
                j = otherNode->xIndex_;
                expr_1f += (X_[v][i][j]  - X_[v][i_n][j]);
            }
            expr_1f +=  (X_[v][i][sinkIndex] - X_[v][i_n][sinkIndex]);
            IloRange c_1f = (expr_1f == 0);
            Model_.add(c_1f);
            constraints_.add(c_1f);

            // constraints 1k -------------------
            IloRange c_1k = (U_[v][i] >= pickNode->readyTime_);
            Model_.add(c_1k);
            constraints_.add(c_1k);

            IloRange c_1k_1 = (U_[v][i] <= pickNode->related_Request_->latestPickup_);
            Model_.add(c_1k_1);
            constraints_.add(c_1k_1);

            IloRange c_1k_2 = (U_[v][i_n] >= (pickNode->readyTime_ +
                pickNode->related_Request_->minTravelTime_ + pickNode->serviceTime_));
            Model_.add(c_1k_2);
            constraints_.add(c_1k_2);

            IloRange c_1k_3 = (U_[v][i_n] <= (pickNode->related_Request_->latestPickup_ +
                pickNode->related_Request_->maxTravelTime_ + pickNode->serviceTime_));
            Model_.add(c_1k_3);
            constraints_.add(c_1k_3);


            // constraints 1l -------------------
            IloExpr expr_1l(env_);
            expr_1l = U_[v][i_n] - U_[v][i] - pickNode->serviceTime_;
            IloRange c_1l_1 = (expr_1l <= pickNode->related_Request_->maxTravelTime_);
            Model_.add(c_1l_1);
            constraints_.add(c_1l_1);

            IloRange c_1l_2 = (expr_1l >= pickNode->related_Request_->minTravelTime_);
            Model_.add(c_1l_2);
            constraints_.add(c_1l_2);

            expr_extract += X_[v][i_n][i] + X_[v][i][i] + X_[v][i_n][i_n];
        }

        for (auto & onboardID : vehicleObj->onboards_) {
            // constraints 1g -------------------
            IloExpr expr_1g(env_);
            PNode onNode = pInst->instGraph_->nodes_[onboardID];
            j = onNode->xIndex_;
            for (auto & otherNode : nodesWithOnboards) {
                if (otherNode->xIndex_ != j)
                    expr_1g += X_[v][otherNode->xIndex_][j];
            }
            expr_1g += X_[v][sourceIndex][j];
            IloRange c_1g = (expr_1g == 1);
            Model_.add(c_1g);
            constraints_.add(c_1g);

            // constraints 1m -------------------
            IloExpr expr_1m(env_);
            expr_1m = U_[v][j] -
                    onNode->related_Request_->pickTime_ -
                    onNode->related_Request_->serviceTime_;
            IloRange c_1m_1 = (expr_1m <= onNode->related_Request_->maxTravelTime_);
            Model_.add(c_1m_1);
            constraints_.add(c_1m_1);

            IloRange c_1m_2 = (expr_1m >= onNode->related_Request_->minTravelTime_);
            Model_.add(c_1m_2);
            constraints_.add(c_1m_2);

            expr_extract += X_[v][j][j];

            IloRange c_1k_4 = (U_[v][j] >= (onNode->related_Request_->pickTime_ +
                onNode->related_Request_->minTravelTime_ + onNode->related_Request_->serviceTime_));
            Model_.add(c_1k_4);
            constraints_.add(c_1k_4);

            IloRange c_1k_5 = (U_[v][j] <= (onNode->related_Request_->latestPickup_ +
                onNode->related_Request_->maxTravelTime_ + onNode->serviceTime_));
            Model_.add(c_1k_5);
            constraints_.add(c_1k_5);

        }

        // constraints 1c -------------------
        for (auto & node : nodesWithOnboards) {
            i = node->xIndex_;
            IloExpr expr_1c(env_);
            for (auto & otherNode : nodesWithOnboards) {
                j = otherNode->xIndex_;
                if (i != j) {
                    expr_1c += (X_[v][i][j] - X_[v][j][i]);
                }
            }
            expr_1c += (X_[v][i][sinkIndex] - X_[v][sourceIndex][i]);
            IloRange c_1c = (expr_1c == 0);
            Model_.add(c_1c);
            constraints_.add(c_1c);
        }


        for (auto & node : allNodes) {
            i = node->xIndex_;

            // constraints 1o -------------------
            IloRange c_1o_1 = (W_[v][i] <= vehicleObj->capacity_);
            Model_.add(c_1o_1);
            constraints_.add(c_1o_1);

            IloRange c_1o_2 = (W_[v][i] >= 0);
            Model_.add(c_1o_2);
            constraints_.add(c_1o_2);

            for (auto & otherNode : allNodes) {
                j = otherNode->xIndex_;
                if (i != j) {
                    // constraints 1h -------------------
                    IloExpr expr_1h(env_);
                    expr_1h = U_[v][i] + node->serviceTime_ + durationMatrix_[node->locationID_][otherNode->locationID_]
                              - U_[v][j] - 61200 * (1 - X_[v][i][j]);
                    IloRange c_1h = (expr_1h <= 0);
                    Model_.add(c_1h);
                    constraints_.add(c_1h);

                    // constraints 1n -------------------
                    IloExpr expr_1n(env_);
                    expr_1n = W_[v][i] + otherNode->nbPassengers_ - W_[v][j] - vehicleObj->capacity_ * (1 - X_[v][i][j]);
                    IloRange c_1n = (expr_1n <= 0);
                    Model_.add(c_1n);
                    constraints_.add(c_1n);
                    objExpr += 0 * X_[v][i][j] * durationMatrix_[node->locationID_][otherNode->locationID_];
                }
            }
        }

        // add this constraint for re-optimization when the vehicle is not idle
        // at the start and have some passengers onboard
        IloRange c = (W_[v][sinkIndex] >= vehicleObj->numPassengers_);
        Model_.add(c);
        constraints_.add(c);
    }
    Model_.add(IloMinimize(env_, objExpr));

    IloRange extarct = (expr_extract == 0);
    Model_.add(extarct);
    constraints_.add(extarct);
    solveTime_->stop();
}

void MIPSolver::solveModel(PInstance &pInst, InputPaths &inputPaths) {
    solveTime_->start();
    try {

        Cplex_ = IloCplex(Model_);

        // std::ofstream logFile(inputPaths.getOutputCplexLog(), std::ofstream::app);
        // logFile << "----------------------- MIP SOlver ------------------------" << std::endl;
        // std::streambuf *coutBuffer = std::cout.rdbuf();
        // std::cout.rdbuf(logFile.rdbuf());

     //   Cplex_.setParam(IloCplex::Param::TimeLimit, 3600);
     //   Cplex_.setParam(IloCplex::Param::Threads, pInst->parameters_->nbThreads_);
    //    Cplex_.setParam(IloCplex::Param::Preprocessing::Presolve, 0);
        Cplex_.setParam(IloCplex::Param::RootAlgorithm, 2);
        Cplex_.setParam(IloCplex::Param::NodeAlgorithm, 2);
        Cplex_.setParam(IloCplex::Param::MIP::Limits::RepairTries, 10);
        Cplex_.setParam(IloCplex::Param::MIP::PolishAfter::Time, 50);


    //    Cplex_.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, pInst->parameters_->MIPGap_);

        // Export model for manual inspection
    //    Cplex_.exportModel("model.lp");

        // Try solving the model
        if (!Cplex_.solve()) {
            std::cout << "Failed to optimize the MIP." << std::endl;

            // Check if the model is infeasible
            if (Cplex_.getStatus() == IloAlgorithm::Infeasible) {
                std::cout << "Model is infeasible. Running conflict refinement..." << std::endl;

                // Prepare arrays for conflict refinement
                IloRangeArray infeasConstraints(env_);
                IloNumArray preferences(env_);

                if (constraints_.getSize() > 0) {
                    for (int i = 0; i < constraints_.getSize(); ++i) {
                        infeasConstraints.add(constraints_[i]); // Ensure constraints are added
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
                } else {
                    std::cout << "No constraints available for conflict refinement." << std::endl;
                }
            }
        }
        else {
            objValue_ = Cplex_.getObjValue();
            std::cout << "The objective value: " << objValue_ << std::endl;

            routeSolution_.clear();
            zSolution_.clear();

            IloNumArray zVal(env_);
            Cplex_.getValues(zVal, Z_);

            for (auto & vehicleObj : pInst->vehicles_) {
                int v = vehicleObj->vehicleID_;
                IloNumArray uVal(env_);
                IloNumArray wVal(env_);

                int nbNodes = nbRequests_ * 2 + 2 + vehicleObj->onboards_.size();
                IloNum2D xVal(env_, nbNodes);

                Cplex_.getValues(uVal, U_[v]);
                Cplex_.getValues(wVal, W_[v]);

                for (int i = 0; i < nbNodes; ++i) {
                    xVal[i] = IloNumArray(env_);
                    Cplex_.getValues(xVal[i], X_[v][i]);
                }

                // creating the route
                PRoute newRoute = std::make_shared<Route>(vehicleObj->vehicleID_);

                newRoute->addSource(vehicleObj->departNode_,vehicleObj->departTime_,
                                    vehicleObj->numPassengers_);

                int currentNodeIndex = vehicleObj->departNode_->xIndex_;

                while (currentNodeIndex != vehicleObj->sinkNode_->xIndex_) {
                    for (int i = 0; i < nbNodes_; ++i) {
                        if (xVal[currentNodeIndex][i] > 0.5) {
                            PNode nodeSelected = pInst->instGraph_->nodes_[getNode(pInst, v,i, nbRequests_)];
                            if (nodeSelected->xIndex_ != vehicleObj->sinkNode_->xIndex_)
                                newRoute->addNode(nodeSelected);
                            currentNodeIndex = i;
                            break;
                        }
                    }
                }
                vehicleObj->setCurrentRoute(newRoute);
                routeSolution_.push_back(std::move(newRoute));
            }

            for (int i = (int) zVal.getSize() - 1; i >= 0; --i) {
                if (zVal[i] > 0.9) {
                    zSolution_.push_back(pInst->nameToRequest_[Z_[i].getName()]);
                }
            }
        }
        /*std::cout.rdbuf(coutBuffer);
        logFile.close();*/
        Cplex_.clearModel();
    }
    catch (IloException& e) {
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
    solveTime_->stop();
}

void MIPSolver::SolveMIP(PInstance &pInst, InputPaths &inputPaths) {
    initializeModel(pInst);
    buildModel(pInst);
    solveModel(pInst, inputPaths);
}


