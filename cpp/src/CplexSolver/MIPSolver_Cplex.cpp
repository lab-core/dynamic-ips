//
// Created by Ella on 2021-10-15.
//

#include "MIPSolver_Cplex.h"
//-----------------------------------------------------------------------------
//  Contains MIP formulation to solve with Cplex
//  This is just for testing the results and comparison
//-----------------------------------------------------------------------------

namespace {
    constexpr double kBinaryThreshold      = 0.5;  // round x_ij in integer solution
    constexpr int    kDualSimplex          = 2;    // CPLEX RootAlgorithm/NodeAlgorithm value
    constexpr int    kPolishAfterSeconds   = 50;
    constexpr int    kRepairTries          = 10;
}

MIPSolver_Cplex::MIPSolver_Cplex() : Model_(env_), constraints_(env_) {
    solveTime_ = new myTools::Timer(); solveTime_->init();
    objValue_ = 0;
}

MIPSolver_Cplex::~MIPSolver_Cplex() {
    delete solveTime_;
    env_.end();
}


// this function initialized the model
void MIPSolver_Cplex::initializeModel(PInstance &pInst) {
    pInst->setNodeIndices();
    nbNodes_     = pInst->instGraph_->nbNodes_;
    nbVehicles_  = pInst->nbVehicles_;
    nbRequests_  = pInst->nbTasks_;

    // -----------------------------------------
    // Define variables
    // -----------------------------------------
    Z_ = IloNumVarArray(env_, nbRequests_, 0.0, 1.0, ILOBOOL);
    X_ = IloNumVar3D(env_, nbVehicles_);
    U_ = IloNumVar2D(env_, nbVehicles_);
    W_ = IloNumVar2D(env_, nbVehicles_);

    // Name Z and register the whole array with the model so every z_i
    // is extracted by CPLEX even if it never appears in a constraint.
    for (int i = 0; i < nbRequests_; ++i)
        Z_[i].setName(pInst->instGraph_->pickNodes_[i]->related_Request_->name_);
    Model_.add(Z_);

    // -----------------------------------------
    // Per-vehicle variables
    // -----------------------------------------
    for (auto & vehicleObj : pInst->vehicles_) {
        int v           = vehicleObj->vehicleID_;
        int nbNodes     = nbRequests_ * 2 + 2 + vehicleObj->onboards_.size();
        int sourceIndex = vehicleObj->departNode_->nodeIndex_;
        int sinkIndex   = vehicleObj->sinkNode_->nodeIndex_;

        U_[v] = IloNumVarArray(env_, nbNodes, 0, IloInfinity, ILOFLOAT);
        W_[v] = IloNumVarArray(env_, nbNodes, 0, IloInfinity, ILOFLOAT);
        X_[v] = IloNumVar2D(env_, nbNodes);

        for (int i = 0; i < nbNodes; ++i) {
            // Names for U and W
            std::ostringstream uName, wName;
            uName << "U_v" << v << "_i" << i;
            wName << "W_v" << v << "_i" << i;
            U_[v][i].setName(uName.str().c_str());
            W_[v][i].setName(wName.str().c_str());

            // X row
            X_[v][i] = IloNumVarArray(env_, nbNodes, 0.0, 1.0, ILOINT);
            for (int j = 0; j < nbNodes; ++j) {
                std::ostringstream xName;
                xName << "X_v" << v << "_i" << i << "_j" << j;
                X_[v][i][j].setName(xName.str().c_str());

                // Fix logically forbidden arcs at variable creation.
                if (i == j)                  X_[v][i][j].setBounds(0.0, 0.0); // no self-loop
                else if (j == sourceIndex)   X_[v][i][j].setBounds(0.0, 0.0); // nothing -> source
                else if (i == sinkIndex)     X_[v][i][j].setBounds(0.0, 0.0); // sink   -> nothing
            }
        }

        // Forbid each fresh dropoff -> its matching pickup (i_n -> i).
        for (auto & pickNode : pInst->instGraph_->pickNodes_) {
            int p   = pickNode->nodeIndex_;
            int p_n = pickNode->pairNode_->nodeIndex_;
            X_[v][p_n][p].setBounds(0.0, 0.0);
        }

        // Register per-vehicle variables with the model so CPLEX extracts
        // them even if a particular index isn't touched by any constraint.
        Model_.add(U_[v]);
        Model_.add(W_[v]);
        for (int i = 0; i < nbNodes; ++i)
            Model_.add(X_[v][i]);
    }
}

void MIPSolver_Cplex::buildModel(PInstance &pInst){
    solveTime_->start();

    auto addCons = [&](IloRange r) { Model_.add(r); constraints_.add(r); };
    std::vector<PNode> nodes = concatenateVectors(pInst->instGraph_->pickNodes_, pInst->instGraph_->dropNodes_);

    // define objective function
    IloExpr objExpr(env_);
    for (auto & pickNode : pInst->instGraph_->pickNodes_) {
        const int i = pickNode->related_Request_->taskIndex_;
        objExpr += Z_[i] * pickNode->related_Request_->Req_W3_ * pickNode->related_Request_->penalty_;
        for (auto & vehicleObj : pInst->vehicles_) {
            objExpr += (U_[vehicleObj->vehicleID_][pickNode->nodeIndex_] - pickNode->readyTime_);
        }
    }

    // defining constraints

    // -----------------------------------------------------------------------
    // (1b) Each pickup is served by exactly one vehicle, or z_i = 1.
    // -----------------------------------------------------------------------
    for (auto & pickNode : pInst->instGraph_->pickNodes_) {
        const int i = pickNode->nodeIndex_;

        IloExpr expr_1b(env_);
        expr_1b += Z_[pickNode->related_Request_->taskIndex_];
        for (auto & vehicleObj : pInst->vehicles_) {
            const int v = vehicleObj->vehicleID_;

            // outgoing to other fresh pickup/dropoff nodes
            for (auto & otherNode : nodes) {
                if (otherNode->nodeIndex_ != pickNode->nodeIndex_)
                    expr_1b += X_[v][i][otherNode->nodeIndex_];
            }
            // outgoing to this vehicle's onboard dropoffs
            for (auto & nodeID : vehicleObj->onboards_) {
                expr_1b += X_[v][i][pInst->instGraph_->nodes_[nodeID]->nodeIndex_];
            }
            // outgoing to this vehicle's sink
            expr_1b += X_[v][i][vehicleObj->sinkNode_->nodeIndex_];
        }
        addCons(expr_1b == 1);
    }

    // -----------------------------------------------------------------------
    // Per-vehicle constraints
    // -----------------------------------------------------------------------
    for (auto & vehicleObj : pInst->vehicles_) {
        const int v = vehicleObj->vehicleID_;
        const int sourceIndex = vehicleObj->departNode_->nodeIndex_;
        const int sinkIndex = vehicleObj->sinkNode_->nodeIndex_;

        std::vector<PNode> nodesWithOnboards = nodes;
        for (auto & nodeID : vehicleObj->onboards_)
            nodesWithOnboards.push_back(pInst->instGraph_->nodes_[nodeID]);

        std::vector<PNode> allNodes = nodesWithOnboards;
        allNodes.push_back(vehicleObj->sinkNode_);
        allNodes.push_back(vehicleObj->departNode_);

        // ----- flow constraints (1c, 1d, 1e) and operating window (1i, 1j) -----

        // (1d) one arc out of source; (1e) one arc into sink
        IloExpr expr_1d(env_);
        IloExpr expr_1e(env_);
        for (auto & otherNode : nodesWithOnboards) {
            expr_1d += X_[v][sourceIndex][otherNode->nodeIndex_];
            expr_1e += X_[v][otherNode->nodeIndex_][sinkIndex];

        }
        expr_1d += X_[v][sourceIndex][sinkIndex];
        expr_1e += X_[v][sourceIndex][sinkIndex];

        addCons(expr_1d == 1);
        addCons(expr_1e == 1);

        // (1c) flow balance at every non-source / non-sink node
        for (auto & node : nodesWithOnboards) {
            const int i = node->nodeIndex_;
            IloExpr expr_1c(env_);
            for (auto & otherNode : nodesWithOnboards) {
                const int j = otherNode->nodeIndex_;
                if (i != j) {
                    expr_1c += (X_[v][i][j] - X_[v][j][i]);
                }
            }
            expr_1c += (X_[v][i][sinkIndex] - X_[v][sourceIndex][i]);
            addCons(expr_1c == 0);
        }

        // (1i) (1j) vehicle operating window
        addCons(U_[v][sourceIndex] >= vehicleObj->departTime_);
        addCons(U_[v][sinkIndex] <= vehicleObj->endTime_);

        for (auto & pickNode : pInst->instGraph_->pickNodes_) {
            const int i = pickNode->nodeIndex_;
            const int i_n = pickNode->pairNode_->nodeIndex_;
            const auto req = pickNode->related_Request_;
            const double e_i= pickNode->readyTime_;
            const double Delta_i = pickNode->serviceTime_;
            const double t_i_min = req->minTravelTime_;
            const double t_i_max = req->maxTravelTime_;
            const double latestPick = req->latestPickup_;

            // (1f) pickup and matching dropoff use the same vehicle
            IloExpr expr_1f(env_);
            for (auto & otherNode : nodesWithOnboards) {
                const int j = otherNode->nodeIndex_;
                expr_1f += (X_[v][i][j]  - X_[v][i_n][j]);
            }
            expr_1f +=  (X_[v][i][sinkIndex] - X_[v][i_n][sinkIndex]);
            addCons(expr_1f == 0);

            // (1k) pickup window
            addCons(U_[v][i] >= e_i);
            addCons(U_[v][i] <= latestPick);

            // Implied dropoff time window (valid inequalities)
            addCons(U_[v][i_n] >= e_i + t_i_min + Delta_i);
            addCons(U_[v][i_n] <= latestPick + t_i_max + Delta_i);

            // (1l) ride-time deviation bounds
            addCons(U_[v][i_n] - U_[v][i] - Delta_i <= t_i_max);
            addCons(U_[v][i_n] - U_[v][i] - Delta_i >= t_i_min);
        }

        // ----- per-onboard constraints (1g, 1m) -----
        for (auto & onboardID : vehicleObj->onboards_) {
            PNode onNode = pInst->instGraph_->nodes_[onboardID];
            const int j = onNode->nodeIndex_;
            const auto req= onNode->related_Request_;
            const double u_P = req->pickTime_;
            const double Delta_j = req->serviceTime_;
            const double t_j_min = req->minTravelTime_;
            const double t_j_max = req->maxTravelTime_;
            const double latestPick = req->latestPickup_;

            // (1g) onboard dropoff is entered from exactly one node
            IloExpr expr_1g(env_);
            for (auto & otherNode : nodesWithOnboards) {
                if (otherNode->nodeIndex_ != j)
                    expr_1g += X_[v][otherNode->nodeIndex_][j];
            }
            expr_1g += X_[v][sourceIndex][j];
            addCons(expr_1g == 1);

            // (1m) ride-time deviation for onboard passenger
            addCons(U_[v][j] - u_P - Delta_j <= t_j_max);
            addCons(U_[v][j] - u_P - Delta_j >= t_j_min);

            // Implied onboard dropoff time window (valid inequalities)
            addCons(U_[v][j] >= u_P + t_j_min + Delta_j);
            addCons(U_[v][j] <= latestPick + t_j_max + onNode->serviceTime_);
        }

        // ----- arc-level constraints: time (1h), load (1n), capacity (1o) -----
        for (auto & node : allNodes) {
            const int i = node->nodeIndex_;
            const double Delta_i = node->serviceTime_;

            // (1o) load is in [0, Q_v]
            addCons(W_[v][i] <= vehicleObj->capacity_);
            addCons(W_[v][i] >= 0);

            for (auto & otherNode : allNodes) {
                const int j = otherNode->nodeIndex_;
                if (i == j) continue;
                const double t_ij = durationMatrix_[node->locationID_][otherNode->locationID_];

                // (1h) Big-M time propagation
                const double Mij = vehicleObj->endTime_ + Delta_i + t_ij;
                addCons(U_[v][i] + Delta_i + t_ij - U_[v][j] - Mij * (1 - X_[v][i][j]) <= 0);

                // (1n) Big-M load propagation; M = 2*Q_v safely covers q_j up to Q_v
                addCons(W_[v][i] + otherNode->nbPassengers_ - W_[v][j] -
                    2 * vehicleObj->capacity_ * (1 - X_[v][i][j]) <= 0);
            }
        }

        // Initialize load at depart node to reflect currently onboard passengers
        // (needed for re-optimization when the vehicle is not idle)
        addCons(W_[v][sourceIndex] >= vehicleObj->numPassengers_);
    }
    Model_.add(IloMinimize(env_, objExpr));
    solveTime_->stop();
}

// -----------------------------------------------------------------------------
// CPLEX configuration
// -----------------------------------------------------------------------------
void MIPSolver_Cplex::configureCplex() {
    Cplex_.setParam(IloCplex::Param::RootAlgorithm,            kDualSimplex);
    Cplex_.setParam(IloCplex::Param::NodeAlgorithm,            kDualSimplex);
    Cplex_.setParam(IloCplex::Param::MIP::Limits::RepairTries, kRepairTries);
    Cplex_.setParam(IloCplex::Param::MIP::PolishAfter::Time,   kPolishAfterSeconds);
    /*Cplex_.setParam(IloCplex::Param::TimeLimit, 3600);
    Cplex_.setParam(IloCplex::Param::Threads, pInst->parameters_->nbThreads_);
    Cplex_.setParam(IloCplex::Param::Preprocessing::Presolve, 0);
    Cplex_.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, pInst->parameters_->MIPGap_);*/
}

void MIPSolver_Cplex::solveModel(PInstance &pInst, InputPaths &inputPaths) {
    solveTime_->start();
    try {

        Cplex_ = IloCplex(Model_);
        configureCplex();

        // Try solving the model
        if (!Cplex_.solve()) {
            std::cout << "Failed to optimize the MIP." << std::endl;

            // Check if the model is infeasible
            if (Cplex_.getStatus() == IloAlgorithm::Infeasible)
                diagnoseInfeasibility();
        } else {
            objValue_ = Cplex_.getObjValue();
            std::cout << "The objective value: " << objValue_ << std::endl;
            extractSolution(pInst);
        }
        Cplex_.clearModel();
    }
    catch (IloException& e) {
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
    solveTime_->stop();
}

// -----------------------------------------------------------------------------
// Infeasibility diagnosis via conflict refinement
// -----------------------------------------------------------------------------
void MIPSolver_Cplex::diagnoseInfeasibility() {
    std::cout << "Model is infeasible. Running conflict refinement..." << std::endl;

    if (constraints_.getSize() == 0) {
        std::cout << "No constraints available for conflict refinement." << std::endl;
        return;
    }

    IloRangeArray infeasConstraints(env_);
    IloNumArray   preferences(env_);
    for (int i = 0; i < constraints_.getSize(); ++i) {
        infeasConstraints.add(constraints_[i]);
        preferences.add(1.0);
    }

    if (!Cplex_.refineConflict(infeasConstraints, preferences)) {
        std::cout << "No specific conflict found, infeasibility remains unclear." << std::endl;
        return;
    }

    std::cout << "Conflicting constraints detected:" << std::endl;
    IloCplex::ConflictStatusArray conflict(env_, infeasConstraints.getSize());
    conflict = Cplex_.getConflict(infeasConstraints);
    for (int i = 0; i < infeasConstraints.getSize(); ++i) {
        if (conflict[i] == IloCplex::ConflictMember)
            std::cout << infeasConstraints[i] << std::endl;
    }
}

// -----------------------------------------------------------------------------
// Solution extraction
// -----------------------------------------------------------------------------
void MIPSolver_Cplex::extractSolution(PInstance &pInst) {
    routeSolution_.clear();
    zSolution_.clear();

    IloNumArray zVal(env_);
    Cplex_.getValues(zVal, Z_);

    for (auto & vehicleObj : pInst->vehicles_) {
        int v = vehicleObj->vehicleID_;
        IloNumArray uVal(env_);
        IloNumArray wVal(env_);

        int nbNodes = nbRequests_ * 2 + 2 + vehicleObj->onboards_.size();
        IloArray<IloNumArray> xVal(env_, nbNodes);

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

        int currentNodeIndex = vehicleObj->departNode_->nodeIndex_;

        const int maxHops = nbNodes + 1;
        int hops = 0;
        while (currentNodeIndex != vehicleObj->sinkNode_->nodeIndex_ && hops++ < maxHops) {
            int next = -1;
            for (int i = 0; i < nbNodes_; ++i) {
                if (xVal[currentNodeIndex][i] > kBinaryThreshold) { next = i; break; }
            }
            if (next < 0) {
                std::cout << "Warning: no outgoing arc from node " << currentNodeIndex
                          << " for vehicle " << v << "; stopping route reconstruction.\n";
                break;
            }
            PNode nodeSelected = pInst->instGraph_->nodes_[getNode(pInst, v, next, nbRequests_)];
            if (nodeSelected->nodeIndex_ != vehicleObj->sinkNode_->nodeIndex_)
                newRoute->addNode(nodeSelected);
            currentNodeIndex = next;
        }
        newRoute->calculateTripDelay(pInst->parameters_->Wait_W1_, pInst->parameters_->Ride_W2_);
        vehicleObj->setCurrentRoute(newRoute);
        routeSolution_.push_back(std::move(newRoute));
    }

    for (int i = (int) zVal.getSize() - 1; i >= 0; --i) {
        if (zVal[i] > kBinaryThreshold) {
            zSolution_.push_back(pInst->nameToRequest_[Z_[i].getName()]);
        }
    }
}

void MIPSolver_Cplex::SolveMIP(PInstance &pInst, InputPaths &inputPaths) {
    initializeModel(pInst);
    buildModel(pInst);
    solveModel(pInst, inputPaths);
}


