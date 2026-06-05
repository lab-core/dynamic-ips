//
// Gurobi port of MIPSolver_Cplex
//

#include "MIPSolver_Gurobi.h"
#include "solvers/SolverEnv.h"
#include <sstream>

//-----------------------------------------------------------------------------
//  Contains MIP formulation to solve with Gurobi.
//  Mirrors MIPSolver_Cplex constraint-for-constraint; only the API differs.
//-----------------------------------------------------------------------------

namespace {
    constexpr double kBinaryThreshold      = 0.5;   // round x_ij in integer solution
    constexpr int    kDualSimplex          = 1;     // Gurobi: 0=primal, 1=dual, 2=barrier
    constexpr double kImproveStartTime     = 50.0;  // equivalent of CPLEX PolishAfter::Time
}

// ----------------------------------------------------------------------------
// Constructor / destructor
// ----------------------------------------------------------------------------
MIPSolver_Gurobi::MIPSolver_Gurobi() : env_(solverEnv::gurobi()), Model_(env_) {
    solveTime_ = new myTools::Timer();
    solveTime_->init();
    objValue_  = 0;
}

MIPSolver_Gurobi::~MIPSolver_Gurobi() {
    delete solveTime_;
}

// ----------------------------------------------------------------------------
// Variable creation + statically forbidden arcs
// ----------------------------------------------------------------------------
void MIPSolver_Gurobi::initializeModel(PInstance &pInst) {
    pInst->setNodeIndices();
    nbNodes_    = pInst->instGraph_->nbNodes_;
    nbVehicles_ = pInst->nbVehicles_;
    nbRequests_ = pInst->nbTasks_;

    // -----------------------------------------
    // Z variables (request unserved indicators)
    // -----------------------------------------
    Z_.clear();
    Z_.reserve(nbRequests_);
    for (int i = 0; i < nbRequests_; ++i) {
        std::string name = pInst->instGraph_->pickNodes_[i]->related_Request_->name_;
        Z_.push_back(Model_.addVar(0.0, 1.0, 0.0, GRB_BINARY, name));
    }

    // -----------------------------------------
    // Per-vehicle variables
    // -----------------------------------------
    X_.assign(nbVehicles_, std::vector<std::vector<GRBVar>>());
    U_.assign(nbVehicles_, std::vector<GRBVar>());
    W_.assign(nbVehicles_, std::vector<GRBVar>());

    for (auto & vehicleObj : pInst->vehicles_) {
        const int v           = vehicleObj->vehicleID_;
        const int nbNodes     = nbRequests_ * 2 + 2 + vehicleObj->onboards_.size();
        const int sourceIndex = vehicleObj->departNode_->nodeIndex_;
        const int sinkIndex   = vehicleObj->sinkNode_->nodeIndex_;

        U_[v].reserve(nbNodes);
        W_[v].reserve(nbNodes);
        X_[v].assign(nbNodes, std::vector<GRBVar>());

        for (int i = 0; i < nbNodes; ++i) {
            std::ostringstream uName, wName;
            uName << "U_v" << v << "_i" << i;
            wName << "W_v" << v << "_i" << i;
            U_[v].push_back(Model_.addVar(0.0, GRB_INFINITY, 0.0,
                                          GRB_CONTINUOUS, uName.str()));
            W_[v].push_back(Model_.addVar(0.0, GRB_INFINITY, 0.0,
                                          GRB_CONTINUOUS, wName.str()));

            X_[v][i].reserve(nbNodes);
            for (int j = 0; j < nbNodes; ++j) {
                std::ostringstream xName;
                xName << "X_v" << v << "_i" << i << "_j" << j;
                X_[v][i].push_back(Model_.addVar(0.0, 1.0, 0.0,
                                                 GRB_BINARY, xName.str()));
            }
        }
    }
    Model_.update();   // make new variables visible before bounds edits

    // -----------------------------------------
    // Fix logically forbidden arcs
    // -----------------------------------------
    for (auto & vehicleObj : pInst->vehicles_) {
        const int v           = vehicleObj->vehicleID_;
        const int nbNodes     = nbRequests_ * 2 + 2 + vehicleObj->onboards_.size();
        const int sourceIndex = vehicleObj->departNode_->nodeIndex_;
        const int sinkIndex   = vehicleObj->sinkNode_->nodeIndex_;

        for (int i = 0; i < nbNodes; ++i) {
            for (int j = 0; j < nbNodes; ++j) {
                const bool forbidden = (i == j)
                                    || (j == sourceIndex)   // nothing -> source
                                    || (i == sinkIndex);    // sink   -> nothing
                if (forbidden) {
                    X_[v][i][j].set(GRB_DoubleAttr_LB, 0.0);
                    X_[v][i][j].set(GRB_DoubleAttr_UB, 0.0);
                }
            }
        }

        // Forbid each fresh dropoff -> its matching pickup
        for (auto & pickNode : pInst->instGraph_->pickNodes_) {
            const int p   = pickNode->nodeIndex_;
            const int p_n = pickNode->pairNode_->nodeIndex_;
            X_[v][p_n][p].set(GRB_DoubleAttr_UB, 0.0);
        }
    }

    Model_.update();
}

// ----------------------------------------------------------------------------
// Build constraints and objective
// ----------------------------------------------------------------------------
void MIPSolver_Gurobi::buildModel(PInstance &pInst) {
    solveTime_->start();

    auto addCons = [&](const GRBTempConstr &c) {
        constraints_.push_back(Model_.addConstr(c));
    };

    std::vector<PNode> nodes = concatenateVectors(pInst->instGraph_->pickNodes_,
                                                  pInst->instGraph_->dropNodes_);

    // -----------------------------------------------------------------------
    // Objective (1a) + Req_W3_-weighted penalty (local addition, not in paper)
    // -----------------------------------------------------------------------
    const double wait_W1 = pInst->parameters_->Wait_W1_;
    const double ride_W2 = pInst->parameters_->Ride_W2_;
    GRBLinExpr objExpr;
    for (auto & pickNode : pInst->instGraph_->pickNodes_) {
        const auto   req     = pickNode->related_Request_;
        const int    iReq    = req->taskIndex_;
        const int    pick    = pickNode->nodeIndex_;
        const int    drop    = pickNode->pairNode_->nodeIndex_;
        const double e_i     = pickNode->readyTime_;
        const double Delta_i = pickNode->serviceTime_;
        const double t_i_min = req->minTravelTime_;
        const double w_coef  = wait_W1 * req->Req_W3_ / req->Relative_W5_;
        const double r_coef  = ride_W2 * req->Req_W3_ / req->Relative_W5_;

        objExpr += Z_[iReq] * req->Req_W3_ * req->penalty_;
        for (auto & vehicleObj : pInst->vehicles_) {
            const int v = vehicleObj->vehicleID_;
            objExpr += w_coef * (U_[v][pick] - e_i);
            objExpr += r_coef * (U_[v][drop] - U_[v][pick] - Delta_i - t_i_min + req->Ride_W4_);
        }
    }

    // -----------------------------------------------------------------------
    // (1b) Each pickup is served by exactly one vehicle, or z_i = 1.
    // -----------------------------------------------------------------------
    for (auto & pickNode : pInst->instGraph_->pickNodes_) {
        const int i = pickNode->nodeIndex_;

        GRBLinExpr expr_1b;
        expr_1b += Z_[pickNode->related_Request_->taskIndex_];

        for (auto & vehicleObj : pInst->vehicles_) {
            const int v = vehicleObj->vehicleID_;

            // outgoing to other fresh pickup/dropoff nodes
            for (auto & otherNode : nodes) {
                if (otherNode->nodeIndex_ != i)
                    expr_1b += X_[v][i][otherNode->nodeIndex_];
            }
            // outgoing to this vehicle's onboard dropoffs
            for (auto & nodeID : vehicleObj->onboards_) {
                expr_1b += X_[v][i][pInst->instGraph_->nodes_[nodeID]->nodeIndex_];
            }
            // outgoing to this vehicle's sink
            expr_1b += X_[v][i][vehicleObj->sinkNode_->nodeIndex_];
        }
        addCons(expr_1b == 1.0);
    }

    // -----------------------------------------------------------------------
    // Per-vehicle constraints
    // -----------------------------------------------------------------------
    for (auto & vehicleObj : pInst->vehicles_) {
        const int    v           = vehicleObj->vehicleID_;
        const int    sourceIndex = vehicleObj->departNode_->nodeIndex_;
        const int    sinkIndex   = vehicleObj->sinkNode_->nodeIndex_;

        std::vector<PNode> nodesWithOnboards = nodes;
        for (auto & nodeID : vehicleObj->onboards_)
            nodesWithOnboards.push_back(pInst->instGraph_->nodes_[nodeID]);

        std::vector<PNode> allNodes = nodesWithOnboards;
        allNodes.push_back(vehicleObj->sinkNode_);
        allNodes.push_back(vehicleObj->departNode_);

        // ----- flow constraints (1c, 1d, 1e) and operating window (1i, 1j) -----

        // (1d) one arc out of source; (1e) one arc into sink
        GRBLinExpr expr_1d, expr_1e;
        for (auto & otherNode : nodesWithOnboards) {
            expr_1d += X_[v][sourceIndex][otherNode->nodeIndex_];
            expr_1e += X_[v][otherNode->nodeIndex_][sinkIndex];
        }
        expr_1d += X_[v][sourceIndex][sinkIndex];
        expr_1e += X_[v][sourceIndex][sinkIndex];
        addCons(expr_1d == 1.0);
        addCons(expr_1e == 1.0);

        // (1c) flow balance at every non-source / non-sink node
        for (auto & node : nodesWithOnboards) {
            const int i = node->nodeIndex_;
            GRBLinExpr expr_1c;
            for (auto & otherNode : nodesWithOnboards) {
                const int j = otherNode->nodeIndex_;
                if (i != j) expr_1c += (X_[v][i][j] - X_[v][j][i]);
            }
            expr_1c += (X_[v][i][sinkIndex] - X_[v][sourceIndex][i]);
            addCons(expr_1c == 0.0);
        }

        // (1i) (1j) vehicle operating window
        addCons(U_[v][sourceIndex] >= vehicleObj->departTime_);
        addCons(U_[v][sinkIndex]   <= vehicleObj->endTime_);

        // ----- per-pickup constraints (1f, 1k, 1l) -----
        for (auto & pickNode : pInst->instGraph_->pickNodes_) {
            const int    i          = pickNode->nodeIndex_;
            const int    i_n        = pickNode->pairNode_->nodeIndex_;
            const auto   req        = pickNode->related_Request_;
            const double e_i        = pickNode->readyTime_;
            const double Delta_i    = pickNode->serviceTime_;
            const double t_i_min    = req->minTravelTime_;
            const double t_i_max    = req->maxTravelTime_;
            const double latestPick = req->latestPickup_;

            // (1f) pickup and matching dropoff use the same vehicle
            GRBLinExpr expr_1f;
            for (auto & otherNode : nodesWithOnboards) {
                const int j = otherNode->nodeIndex_;
                expr_1f += (X_[v][i][j] - X_[v][i_n][j]);
            }
            expr_1f += (X_[v][i][sinkIndex] - X_[v][i_n][sinkIndex]);
            addCons(expr_1f == 0.0);

            // (1k) pickup window
            //   latestPickup_ upper bound is a local addition, not in the paper.
            addCons(U_[v][i] >= e_i);
            addCons(U_[v][i] <= latestPick);

            // Implied dropoff time window (valid inequalities)
            addCons(U_[v][i_n] >= e_i        + t_i_min + Delta_i);
            addCons(U_[v][i_n] <= latestPick + t_i_max + Delta_i);

            // (1l) ride-time deviation bounds
            addCons(U_[v][i_n] - U_[v][i] - Delta_i <= t_i_max);
            addCons(U_[v][i_n] - U_[v][i] - Delta_i >= t_i_min);
        }

        // ----- per-onboard constraints (1g, 1m) -----
        for (auto & onboardID : vehicleObj->onboards_) {
            PNode        onNode     = pInst->instGraph_->nodes_[onboardID];
            const int    j          = onNode->nodeIndex_;
            const auto   req        = onNode->related_Request_;
            const double u_P        = req->pickTime_;
            const double Delta_j    = req->serviceTime_;
            const double t_j_min    = req->minTravelTime_;
            const double t_j_max    = req->maxTravelTime_;
            const double latestPick = req->latestPickup_;

            // (1g) onboard dropoff is entered from exactly one node
            GRBLinExpr expr_1g;
            for (auto & otherNode : nodesWithOnboards) {
                if (otherNode->nodeIndex_ != j)
                    expr_1g += X_[v][otherNode->nodeIndex_][j];
            }
            expr_1g += X_[v][sourceIndex][j];
            addCons(expr_1g == 1.0);

            // (1m) ride-time deviation for onboard passenger
            addCons(U_[v][j] - u_P - Delta_j <= t_j_max);
            addCons(U_[v][j] - u_P - Delta_j >= t_j_min);

            // Implied onboard dropoff time window
            addCons(U_[v][j] >= u_P        + t_j_min + Delta_j);
            addCons(U_[v][j] <= latestPick + t_j_max + onNode->serviceTime_);

            // ride delay for onboard passenger
            objExpr += (ride_W2 * req->Req_W3_ / req->Relative_W5_) * (U_[v][j] - u_P - Delta_j - t_j_min + req->Ride_W4_);
        }

        // ----- arc-level constraints: time (1h), load (1n), capacity (1o) -----
        for (auto & node : allNodes) {
            const int    i       = node->nodeIndex_;
            const double Delta_i = node->serviceTime_;

            // (1o) load is in [0, Q_v]
            addCons(W_[v][i] <= vehicleObj->capacity_);
            addCons(W_[v][i] >= 0);

            for (auto & otherNode : allNodes) {
                const int j = otherNode->nodeIndex_;
                if (i == j) continue;
                const double t_ij = durationMatrix_[node->locationID_][otherNode->locationID_];

                // (1h) Big-M time propagation; safe per-arc M = T^E_v + Delta_i + t_ij
                const double Mij = vehicleObj->endTime_ + Delta_i + t_ij;
                addCons(U_[v][i] + Delta_i + t_ij - U_[v][j]
                        - Mij * (1 - X_[v][i][j]) <= 0);

                // (1n) Big-M load propagation; M = 2*Q_v safely covers q_j up to Q_v
                addCons(W_[v][i] + otherNode->nbPassengers_ - W_[v][j]
                        - 2 * vehicleObj->capacity_ * (1 - X_[v][i][j]) <= 0);
            }
        }

        // Initialize load at depart node to reflect currently onboard passengers
        // (needed for re-optimization when the vehicle is not idle)
        addCons(W_[v][sourceIndex] >= vehicleObj->numPassengers_);
    }

    Model_.setObjective(objExpr, GRB_MINIMIZE);
    Model_.update();
    solveTime_->stop();
}

// ----------------------------------------------------------------------------
// Gurobi configuration
// ----------------------------------------------------------------------------
void MIPSolver_Gurobi::configureGurobi() {
    // Algorithm: dual simplex at root and at non-root nodes
    Model_.set(GRB_IntParam_Method,     kDualSimplex);
    Model_.set(GRB_IntParam_NodeMethod, kDualSimplex);

    // ImproveStartTime is Gurobi's equivalent of CPLEX's PolishAfter::Time:
    // after this many seconds, switch effort toward improving the incumbent.
    Model_.set(GRB_DoubleParam_ImproveStartTime, kImproveStartTime);

    // Other useful knobs (commented out, mirroring the CPLEX original):
    if (timeLimit_ > 0)
        Model_.set(GRB_DoubleParam_TimeLimit, static_cast<double>(timeLimit_));
    // Model_.set(GRB_IntParam_Threads,      pInst->parameters_->nbThreads_);
    // Model_.set(GRB_IntParam_Presolve,     0);
    // Model_.set(GRB_DoubleParam_MIPGap,    pInst->parameters_->MIPGap_);
}

// ----------------------------------------------------------------------------
// Solve and dispatch to extraction or infeasibility diagnosis
// ----------------------------------------------------------------------------
void MIPSolver_Gurobi::solveModel(PInstance &pInst, InputPaths &inputPaths) {
    solveTime_->start();
    try {
        configureGurobi();
        Model_.optimize();

        const int status = Model_.get(GRB_IntAttr_Status);

        // Anything other than OPTIMAL or SUBOPTIMAL is "no usable solution"
        const bool haveSolution = (Model_.get(GRB_IntAttr_SolCount) > 0)
                               && (status == GRB_OPTIMAL
                                || status == GRB_SUBOPTIMAL
                                || status == GRB_TIME_LIMIT);

        if (!haveSolution) {
            std::cout << "Failed to optimize the MIP." << std::endl;
            if (status == GRB_INFEASIBLE)
                diagnoseInfeasibility();
        } else {
            objValue_ = Model_.get(GRB_DoubleAttr_ObjVal);
            std::cout << "The objective value: " << objValue_ << std::endl;
            extractSolution(pInst);
        }
    }
    catch (GRBException &e) {
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e.getMessage() << " (code " << e.getErrorCode() << ")" << std::endl;
    }
    solveTime_->stop();
}

// ----------------------------------------------------------------------------
// Infeasibility diagnosis via Irreducible Inconsistent Subsystem (IIS)
// ----------------------------------------------------------------------------
void MIPSolver_Gurobi::diagnoseInfeasibility() {
    std::cout << "Model is infeasible. Running IIS computation..." << std::endl;

    if (constraints_.empty()) {
        std::cout << "No constraints available for conflict refinement." << std::endl;
        return;
    }

    try {
        Model_.computeIIS();

        std::cout << "Conflicting constraints detected:" << std::endl;
        bool anyConflict = false;
        for (auto & c : constraints_) {
            if (c.get(GRB_IntAttr_IISConstr) == 1) {
                std::cout << c.get(GRB_StringAttr_ConstrName) << std::endl;
                anyConflict = true;
            }
        }
        if (!anyConflict)
            std::cout << "No specific conflict found, infeasibility remains unclear." << std::endl;
    }
    catch (GRBException &e) {
        std::cout << "Error during IIS: " << e.getMessage() << std::endl;
    }
}

// ----------------------------------------------------------------------------
// Solution extraction
// ----------------------------------------------------------------------------
void MIPSolver_Gurobi::extractSolution(PInstance &pInst) {
    routeSolution_.clear();
    zSolution_.clear();

    // Cache z values
    std::vector<double> zVal(Z_.size());
    for (size_t i = 0; i < Z_.size(); ++i)
        zVal[i] = Z_[i].get(GRB_DoubleAttr_X);

    for (auto & vehicleObj : pInst->vehicles_) {
        const int v       = vehicleObj->vehicleID_;
        const int nbNodes = nbRequests_ * 2 + 2 + vehicleObj->onboards_.size();

        // Cache U, W values for this vehicle
        std::vector<double> uVal(nbNodes), wVal(nbNodes);
        for (int i = 0; i < nbNodes; ++i) {
            uVal[i] = U_[v][i].get(GRB_DoubleAttr_X);
            wVal[i] = W_[v][i].get(GRB_DoubleAttr_X);
        }

        // Cache X values for this vehicle
        std::vector<std::vector<double>> xVal(nbNodes, std::vector<double>(nbNodes, 0.0));
        for (int i = 0; i < nbNodes; ++i) {
            for (int j = 0; j < nbNodes; ++j) {
                xVal[i][j] = X_[v][i][j].get(GRB_DoubleAttr_X);
            }
        }

        // Build the route by walking from depart to sink
        PRoute newRoute = std::make_shared<Route>(vehicleObj->vehicleID_);
        newRoute->addSource(vehicleObj->departNode_,
                            vehicleObj->departTime_,
                            vehicleObj->numPassengers_);

        int       currentNodeIndex = vehicleObj->departNode_->nodeIndex_;
        const int sinkIndex        = vehicleObj->sinkNode_->nodeIndex_;
        const int maxHops          = nbNodes + 1;
        int       hops             = 0;

        while (currentNodeIndex != sinkIndex && hops++ < maxHops) {
            int next = -1;
            for (int i = 0; i < nbNodes_; ++i) {
                if (xVal[currentNodeIndex][i] > kBinaryThreshold) {
                    next = i;
                    break;
                }
            }
            if (next < 0) {
                std::cout << "Warning: no outgoing arc from node " << currentNodeIndex
                          << " for vehicle " << v
                          << "; stopping route reconstruction.\n";
                break;
            }
            PNode nodeSelected = pInst->instGraph_->nodes_[
                getNode(pInst, v, next, nbRequests_)];
            if (nodeSelected->nodeIndex_ != sinkIndex)
                newRoute->addNode(nodeSelected);
            currentNodeIndex = next;
        }

        newRoute->calculateTripDelay(pInst->parameters_->Wait_W1_,
                                     pInst->parameters_->Ride_W2_);
        vehicleObj->setCurrentRoute(newRoute);
        routeSolution_.push_back(std::move(newRoute));
    }

    // Collect unserved requests (z_i ~ 1)
    for (int i = static_cast<int>(zVal.size()) - 1; i >= 0; --i) {
        if (zVal[i] > kBinaryThreshold) {
            const std::string name = Z_[i].get(GRB_StringAttr_VarName);
            zSolution_.push_back(pInst->nameToRequest_[name]);
        }
    }
}

// ----------------------------------------------------------------------------
// Orchestrator
// ----------------------------------------------------------------------------
void MIPSolver_Gurobi::SolveMIP(PInstance &pInst, InputPaths &inputPaths) {
    initializeModel(pInst);
    buildModel(pInst);
    solveModel(pInst, inputPaths);
}
