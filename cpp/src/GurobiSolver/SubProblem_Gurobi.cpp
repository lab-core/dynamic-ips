//
// Gurobi port of SubProblem_Cplex
//

#include "SubProblem_Gurobi.h"
#include "solvers/BaseSolver.h"
#include "solvers/SolverEnv.h"
#include <sstream>
#include <algorithm>

//-----------------------------------------------------------------------------
//  SubProblem_Gurobi
//  Pricing subproblem (Figure 3 of the paper) implemented with Gurobi.
//-----------------------------------------------------------------------------

namespace {
    constexpr double kBinaryThreshold        = 0.5;
    constexpr double kNegativeCostTolerance  = -0.001; // reduced cost < this is "negative enough"
    constexpr int    kDualSimplex            = 1;      // Gurobi: 0=primal, 1=dual, 2=barrier
    constexpr int    kMainTimeLimit          = 1000;
    constexpr int    kPoolTimeLimit          = 500;
    constexpr int    kThreads                = 8;
    constexpr int    kPoolCapacity           = 20;
    constexpr int    kPoolSearchMode         = 2;      // find the n best solutions
    constexpr double kPoolRelGap             = 0.5;
}

// ----------------------------------------------------------------------------
// Constructor / destructor
// ----------------------------------------------------------------------------
SubProblem_Gurobi::SubProblem_Gurobi(PVehicle &vehicle)
    : SubproModeler(vehicle),
      env_(solverEnv::gurobi()),
      SubProModel_(env_)
{
    nbGenerated_        = 0;
    nbNodes_            = 0;
    nbRequests_         = 0;
    bestObjectiveValue_ = 0.0;
    solutionFound_      = false;
}

// ----------------------------------------------------------------------------
// Solver configuration
// ----------------------------------------------------------------------------
void SubProblem_Gurobi::configureGurobi() {
    SubProModel_.set(GRB_IntParam_OutputFlag,  0);            // silent
    SubProModel_.set(GRB_DoubleParam_TimeLimit, kMainTimeLimit);
    SubProModel_.set(GRB_DoubleParam_MIPGap,    0.01);

    // Algorithm choice: dual simplex at root and at nodes
    SubProModel_.set(GRB_IntParam_Method,     kDualSimplex);
    SubProModel_.set(GRB_IntParam_NodeMethod, kDualSimplex);

    SubProModel_.set(GRB_IntParam_Threads, kThreads);
    SubProModel_.set(GRB_IntParam_Cuts,    1);   // moderate cuts

    // MIPFocus: 0=balanced, 1=feasibility, 2=optimality, 3=bound
    // SubProModel_.set(GRB_IntParam_MIPFocus, 2);
}

void SubProblem_Gurobi::configurePool() {
    SubProModel_.set(GRB_IntParam_PoolSolutions,    kPoolCapacity);
    SubProModel_.set(GRB_IntParam_PoolSearchMode,   kPoolSearchMode);
    SubProModel_.set(GRB_DoubleParam_PoolGap,       kPoolRelGap);
    SubProModel_.set(GRB_DoubleParam_TimeLimit,     kPoolTimeLimit);
}

// ----------------------------------------------------------------------------
// Variable creation + statically forbidden arcs
// ----------------------------------------------------------------------------
void SubProblem_Gurobi::initializeModel(PInstance &pInst) {
    setNodeIndices();
    nbNodes_    = subGraph_->nbNodes_;
    nbRequests_ = subGraph_->pickNodes_.size();

    const int sourceIndex = Vehicle_->departNode_->nodeIndex_;
    const int sinkIndex   = Vehicle_->sinkNode_->nodeIndex_;

    // ------ Allocate variable containers ------
    X_.assign(nbNodes_, std::vector<GRBVar>());
    U_.clear(); U_.reserve(nbNodes_);
    W_.clear(); W_.reserve(nbNodes_);

    // ------ Create variables ------
    for (int i = 0; i < nbNodes_; ++i) {
        std::ostringstream uName, wName;
        uName << "U_i" << i;
        wName << "W_i" << i;

        U_.push_back(SubProModel_.addVar(0.0, GRB_INFINITY, 0.0,
                                         GRB_CONTINUOUS, uName.str()));
        W_.push_back(SubProModel_.addVar(0.0, GRB_INFINITY, 0.0,
                                         GRB_CONTINUOUS, wName.str()));

        X_[i].reserve(nbNodes_);
        for (int j = 0; j < nbNodes_; ++j) {
            std::ostringstream xName;
            xName << "X_i" << i << "_j" << j;
            X_[i].push_back(SubProModel_.addVar(0.0, 1.0, 0.0,
                                                GRB_BINARY, xName.str()));
        }
    }
    SubProModel_.update();   // make the new variables visible before bounds edits

    // ------ Statically forbidden arcs ------
    // Equivalent to expr_extract == 0 in the original CPLEX code.
    for (int i = 0; i < nbNodes_; ++i) {
        for (int j = 0; j < nbNodes_; ++j) {
            const bool forbidden = (i == j)
                                || (j == sourceIndex)   // nothing -> source
                                || (i == sinkIndex);    // sink   -> nothing
            if (forbidden) {
                X_[i][j].set(GRB_DoubleAttr_LB, 0.0);
                X_[i][j].set(GRB_DoubleAttr_UB, 0.0);
            }
        }
    }

    // Forbid each fresh dropoff -> its matching pickup
    for (auto & pickNode : subGraph_->pickNodes_) {
        const int p   = pickNode->nodeIndex_;
        const int p_n = pickNode->pairNode_->nodeIndex_;
        X_[p_n][p].set(GRB_DoubleAttr_UB, 0.0);
    }

    SubProModel_.update();
}

// ----------------------------------------------------------------------------
// Build constraints and objective
// ----------------------------------------------------------------------------
void SubProblem_Gurobi::buildModel(PInstance &pInst) {
    std::vector<PNode> nodes = concatenateVectors(subGraph_->pickNodes_,
                                                  subGraph_->dropNodes_);

    const int    sourceIndex = Vehicle_->departNode_->nodeIndex_;
    const int    sinkIndex   = Vehicle_->sinkNode_->nodeIndex_;
    const double Qv          = Vehicle_->capacity_;

    std::vector<PNode> nodesWithOnboards = nodes;
    for (auto & nodeID : Vehicle_->onboards_)
        nodesWithOnboards.push_back(subGraph_->nodes_[nodeID]);

    std::vector<PNode> allNodes = nodesWithOnboards;
    allNodes.push_back(Vehicle_->sinkNode_);
    allNodes.push_back(Vehicle_->departNode_);

    // -----------------------------------------------------------------------
    // Objective (4a): reduced cost = wait time - sum_i pi_i * served(i) - sigma_v
    // -----------------------------------------------------------------------
    GRBLinExpr objExpr;
    for (auto & pickNode : subGraph_->pickNodes_) {
        const int    i   = pickNode->nodeIndex_;
        const double e_i = pickNode->readyTime_;
        const double pi  = pickNode->related_Request_->dual_;

        objExpr += (U_[i] - e_i);
        // sum_j x_{ji} = served(i) by flow conservation on pickup nodes
        for (int j = 0; j < nbNodes_; ++j)
            objExpr -= X_[j][i] * pi;
    }
    objExpr -= Vehicle_->dual_;

    // -----------------------------------------------------------------------
    // Pruning Theorem: arcs where the smallest wait exceeds the
    // penalty are suboptimal. Fix them at 0 via setBounds rather than
    // adding an aggregating equality.
    // -----------------------------------------------------------------------
    for (auto & nodeObj : nodesWithOnboards) {
        if (nodeObj->type_ == SINK) continue;
        const int fromIndex = nodeObj->nodeIndex_;

        for (auto & pickNodeObj : subGraph_->pickNodes_) {
            if (nodeObj->nodeID_ == pickNodeObj->nodeID_) continue;
            const int toIndex = pickNodeObj->nodeIndex_;

            const double minWait = Vehicle_->departTime_
                                 + nodeObj->travelTimeFromSource_
                                 + nodeObj->serviceTime_
                                 + durationMatrix_[nodeObj->locationID_][pickNodeObj->locationID_]
                                 - pickNodeObj->readyTime_;

            if (minWait > pickNodeObj->related_Request_->penalty_)
                X_[fromIndex][toIndex].set(GRB_DoubleAttr_UB, 0.0);
        }
    }

    // -----------------------------------------------------------------------
    // (1d) (1e) source/sink flow
    // -----------------------------------------------------------------------
    {
        GRBLinExpr expr_1d, expr_1e;
        for (auto & otherNode : nodesWithOnboards) {
            expr_1d += X_[sourceIndex][otherNode->nodeIndex_];
            expr_1e += X_[otherNode->nodeIndex_][sinkIndex];
        }
        expr_1d += X_[sourceIndex][sinkIndex];
        expr_1e += X_[sourceIndex][sinkIndex];
        SubProModel_.addConstr(expr_1d == 1.0);
        SubProModel_.addConstr(expr_1e == 1.0);
    }

    // -----------------------------------------------------------------------
    // (1c) flow balance at non-source / non-sink nodes
    // -----------------------------------------------------------------------
    for (auto & node : nodesWithOnboards) {
        const int i = node->nodeIndex_;
        GRBLinExpr expr_1c;
        for (auto & otherNode : nodesWithOnboards) {
            const int j = otherNode->nodeIndex_;
            if (i != j) expr_1c += (X_[i][j] - X_[j][i]);
        }
        expr_1c += (X_[i][sinkIndex] - X_[sourceIndex][i]);
        SubProModel_.addConstr(expr_1c == 0.0);
    }

    // -----------------------------------------------------------------------
    // (1i) (1j) vehicle operating window
    // -----------------------------------------------------------------------
    SubProModel_.addConstr(U_[sourceIndex] >= Vehicle_->departTime_);
    SubProModel_.addConstr(U_[sinkIndex]   <= Vehicle_->endTime_);

    // -----------------------------------------------------------------------
    // Per-pickup constraints (1f, 1k, 1l)
    // -----------------------------------------------------------------------
    for (auto & pickNode : subGraph_->pickNodes_) {
        const int    i          = pickNode->nodeIndex_;
        const int    i_n        = pickNode->pairNode_->nodeIndex_;
        const auto   req        = pickNode->related_Request_;
        const double e_i        = pickNode->readyTime_;
        const double Delta_i    = pickNode->serviceTime_;
        const double t_i_min    = req->minTravelTime_;
        const double t_i_max    = req->maxTravelTime_;
        const double latestPick = req->latestPickup_;

        // (1f) pickup and dropoff use the same vehicle
        GRBLinExpr expr_1f;
        for (auto & otherNode : nodesWithOnboards) {
            const int j = otherNode->nodeIndex_;
            expr_1f += (X_[i][j] - X_[i_n][j]);
        }
        expr_1f += (X_[i][sinkIndex] - X_[i_n][sinkIndex]);
        SubProModel_.addConstr(expr_1f == 0.0);

        // (1k) pickup window
        //   latestPickup_ upper bound is a local addition, not in the paper.
        SubProModel_.addConstr(U_[i] >= e_i);
        SubProModel_.addConstr(U_[i] <= latestPick);

        // Implied dropoff time window (valid inequalities)
        SubProModel_.addConstr(U_[i_n] >= e_i        + t_i_min + Delta_i);
        SubProModel_.addConstr(U_[i_n] <= latestPick + t_i_max + Delta_i);

        // (1l) ride-time deviation bounds
        SubProModel_.addConstr(U_[i_n] - U_[i] - Delta_i <= t_i_max);
        SubProModel_.addConstr(U_[i_n] - U_[i] - Delta_i >= t_i_min);
    }

    // -----------------------------------------------------------------------
    // Per-onboard constraints (1g, 1m)
    // -----------------------------------------------------------------------
    for (auto & onboardID : Vehicle_->onboards_) {
        PNode        onNode     = subGraph_->nodes_[onboardID];
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
                expr_1g += X_[otherNode->nodeIndex_][j];
        }
        expr_1g += X_[sourceIndex][j];
        SubProModel_.addConstr(expr_1g == 1.0);

        // (1m) ride-time deviation for onboard passenger
        SubProModel_.addConstr(U_[j] - u_P - Delta_j <= t_j_max);
        SubProModel_.addConstr(U_[j] - u_P - Delta_j >= t_j_min);

        // Implied onboard dropoff time window
        SubProModel_.addConstr(U_[j] >= u_P        + t_j_min + Delta_j);
        SubProModel_.addConstr(U_[j] <= latestPick + t_j_max + onNode->serviceTime_);
    }

    // -----------------------------------------------------------------------
    // Arc-level constraints: time (1h), load (1n), capacity (1o)
    // -----------------------------------------------------------------------
    for (auto & node : allNodes) {
        const int    i       = node->nodeIndex_;
        const double Delta_i = node->serviceTime_;

        // (1o) load in [0, Q_v], with tighter bounds where possible
        SubProModel_.addConstr(W_[i] <= Qv);
        SubProModel_.addConstr(W_[i] >= 0);

        if (node->type_ == PICKUP) {
            W_[i].set(GRB_DoubleAttr_LB, node->nbPassengers_);
            W_[i].set(GRB_DoubleAttr_UB, Qv);
        } else if (node->type_ == DROPOFF) {
            W_[i].set(GRB_DoubleAttr_LB, 0.0);
            W_[i].set(GRB_DoubleAttr_UB, Qv);
        }

        // Tighter U bounds at pickup nodes
        if (node->type_ == PICKUP) {
            const double minTime = std::max(node->readyTime_,
                Vehicle_->departTime_ +
                durationMatrix_[Vehicle_->departNode_->locationID_][node->locationID_]);
            const double maxTime = std::min(node->related_Request_->latestPickup_,
                Vehicle_->endTime_ - node->serviceTime_);
            if (minTime < maxTime) {
                U_[i].set(GRB_DoubleAttr_LB, minTime);
                U_[i].set(GRB_DoubleAttr_UB, maxTime);
            }
        }

        for (auto & otherNode : allNodes) {
            const int j = otherNode->nodeIndex_;
            if (i == j) continue;
            const double t_ij = durationMatrix_[node->locationID_][otherNode->locationID_];

            // (1h) Big-M time propagation; safe per-arc M = T^E_v + Delta_i + t_ij
            const double Mij = Vehicle_->endTime_ + Delta_i + t_ij;
            SubProModel_.addConstr(U_[i] + Delta_i + t_ij - U_[j]
                                   - Mij * (1 - X_[i][j]) <= 0);

            // (1n) Big-M load propagation; M = 2*Q_v safely covers q_j up to Q_v
            SubProModel_.addConstr(W_[i] + otherNode->nbPassengers_ - W_[j]
                                   - 2 * Qv * (1 - X_[i][j]) <= 0);
        }
    }

    // Initialize load at depart node to reflect currently onboard passengers
    // (needed for re-optimization when the vehicle is not idle).
    SubProModel_.addConstr(W_[sourceIndex] >= Vehicle_->numPassengers_);

    // Set the objective
    SubProModel_.setObjective(objExpr, GRB_MINIMIZE);
    SubProModel_.update();
}

// ----------------------------------------------------------------------------
// Solve and extract solutions
// ----------------------------------------------------------------------------
void SubProblem_Gurobi::solveModel(PInstance &pInst, std::vector<PRoute> &availableRoutes) {
    bestObjectiveValue_ = 1e9;
    solutionFound_      = false;

    try {
        configureGurobi();
        configurePool();

        SubProModel_.optimize();

        const int status   = SubProModel_.get(GRB_IntAttr_Status);
        const int solCount = SubProModel_.get(GRB_IntAttr_SolCount);

        if (solCount == 0) {
            std::cout << "Failed to optimize subproblem for vehicle "
                      << Vehicle_->vehicleID_
                      << " (status: " << status << ")" << std::endl;
            return;
        }

        bestObjectiveValue_         = SubProModel_.get(GRB_DoubleAttr_ObjVal);
        Vehicle_->bestReducedCost_  = bestObjectiveValue_;
        solutionFound_              = true;

        std::cout << "Vehicle " << Vehicle_->vehicleID_
                  << " - Best objective: " << bestObjectiveValue_
                  << " (Status: " << status << ")" << std::endl;

        // Extract incumbent
        extractSingleSolution(pInst, availableRoutes, -1);

        // Extract additional pool solutions only if the incumbent has
        // negative reduced cost (otherwise no useful column to generate).
        if (bestObjectiveValue_ < kNegativeCostTolerance) {
            extractPoolSolutions(pInst, availableRoutes);
        }
    }
    catch (GRBException& e) {
        std::cout << "Error in solveModel: " << e.getMessage()
                  << " (code " << e.getErrorCode() << ")" << std::endl;
    }
}

void SubProblem_Gurobi::solveSP(PInstance &pInst, std::vector<PRoute> &availableRoutess) {
    subproTime_->start();

    initializeModel(pInst);
    buildModel(pInst);

    std::vector<PRoute> newRoutes;
    solveModel(pInst, newRoutes);

    for (auto & route : newRoutes) {
        if (route->reducedCost_ < -1e-6) {
            availableRoutess.push_back(route);
            nbNegativeColumns_++;
        }
        nbGenerated_++;
    }

    if (!newRoutes.empty()) {
        float bestCost = newRoutes[0]->reducedCost_;
        for (auto & route : newRoutes)
            bestCost = std::min(bestCost, route->reducedCost_);
        Vehicle_->bestReducedCost_ = bestCost;
    }

    subproTime_->stop();
}

// ----------------------------------------------------------------------------
// Extraction
// ----------------------------------------------------------------------------
void SubProblem_Gurobi::extractPoolSolutions(PInstance &pInst, std::vector<PRoute> &availableRoutes) {
    const int maxRoutes      = 10;
    int       routesExtracted = 1;            // incumbent already extracted

    const int nSolutions = SubProModel_.get(GRB_IntAttr_SolCount);

    for (int s = 0; s < nSolutions && routesExtracted < maxRoutes; ++s) {
        SubProModel_.set(GRB_IntParam_SolutionNumber, s);
        const double objValue = SubProModel_.get(GRB_DoubleAttr_PoolObjVal);

        // Only extract solutions with negative reduced cost that are
        // distinct from the incumbent.
        if (objValue < kNegativeCostTolerance
            && objValue > bestObjectiveValue_ + 0.001) {
            extractSingleSolution(pInst, availableRoutes, s);
            routesExtracted++;
        }
    }
}

void SubProblem_Gurobi::extractSingleSolution(PInstance &pInst,
                                             std::vector<PRoute> &availableRoutes,
                                             int solutionIndex) {
    try {
        double objValue;
        if (solutionIndex >= 0) {
            SubProModel_.set(GRB_IntParam_SolutionNumber, solutionIndex);
            objValue = SubProModel_.get(GRB_DoubleAttr_PoolObjVal);
        } else {
            objValue = SubProModel_.get(GRB_DoubleAttr_ObjVal);
        }

        // Helper: get x value for the current solution context
        auto getX = [&](const GRBVar& v) -> double {
            return solutionIndex >= 0 ? v.get(GRB_DoubleAttr_Xn)
                                      : v.get(GRB_DoubleAttr_X);
        };

        // Build the route
        PRoute newRoute = std::make_shared<Route>(Vehicle_->vehicleID_);
        newRoute->reducedCost_ = objValue;
        newRoute->addSource(Vehicle_->departNode_, Vehicle_->departTime_,
                            Vehicle_->numPassengers_);

        int       currentNodeIndex = Vehicle_->departNode_->nodeIndex_;
        const int sinkIndex        = Vehicle_->sinkNode_->nodeIndex_;
        const int maxHops          = nbNodes_ + 1;
        int       hops             = 0;

        while (currentNodeIndex != sinkIndex && hops++ < maxHops) {
            int next = -1;
            for (int i = 0; i < nbNodes_; ++i) {
                if (getX(X_[currentNodeIndex][i]) > kBinaryThreshold) {
                    next = i;
                    break;
                }
            }
            if (next < 0) {
                std::cout << "Warning: no outgoing arc from node " << currentNodeIndex
                          << " (vehicle " << Vehicle_->vehicleID_ << ")\n";
                return;
            }
            PNode nodeSelected = pInst->instGraph_->nodes_[
                getNode(pInst, Vehicle_->vehicleID_, next, nbRequests_)];
            if (nodeSelected->nodeIndex_ != sinkIndex)
                newRoute->addNode(nodeSelected);
            currentNodeIndex = next;
        }

        if (hops >= maxHops) {
            std::cout << "Warning: route reconstruction hit hop limit for vehicle "
                      << Vehicle_->vehicleID_ << std::endl;
            return;
        }

        newRoute->calculateTripDelay(pInst->parameters_->Wait_W1_,
                                     pInst->parameters_->Ride_W2_);
        availableRoutes.push_back(newRoute);
    }
    catch (GRBException& e) {
        std::cout << "Error in solution extraction: " << e.getMessage()
                  << " (code " << e.getErrorCode() << ")" << std::endl;
    }
}

// ----------------------------------------------------------------------------
// Display
// ----------------------------------------------------------------------------
std::string SubProblem_Gurobi::toString() const {
    // const_cast needed because Gurobi's get() on the model is not declared const
    GRBModel& M = const_cast<GRBModel&>(SubProModel_);

    std::stringstream repStr;
    repStr << "#\n";
    repStr << "# ------------------------------------------------------------\n";
    repStr << "# SUB PROBLEM SOLUTION RESULT FOR VEHICLE: " << Vehicle_->vehicleID_ << "\n";
    repStr << "#\n";
    repStr << "# Solution status = "          << M.get(GRB_IntAttr_Status)     << "\n";
    repStr << "# Incumbent objective value = "<< M.get(GRB_DoubleAttr_ObjVal)  << "\n";

    const int nSolutions = M.get(GRB_IntAttr_SolCount);
    repStr << "# The solution pool contains = " << nSolutions << " solutions.\n";

    int nbValidSolution = 0;
    for (int s = 0; s < nSolutions; ++s) {
        M.set(GRB_IntParam_SolutionNumber, s);
        if (M.get(GRB_DoubleAttr_PoolObjVal) < 0)
            ++nbValidSolution;
    }
    repStr << "# The solution pool contains = " << nbValidSolution
           << " solutions with negative reduced cost.\n";
    return repStr.str();
}

void RuntimeMetrics::updateSubproblemMetrics(const PGurobiSubPro &subProblem) {
    nbNegativeFound_ += subProblem->nbNegativeColumns_;
    nbGenerated_ += subProblem->nbGenerated_;
}