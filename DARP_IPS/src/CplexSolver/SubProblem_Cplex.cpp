//
// Created by Ella on 2021-10-13.
//

#include "SubProblem_Cplex.h"
#include "solvers/BaseSolver.h"

//-----------------------------------------------------------------------------
//  SubProblem_Cplex class
//  Algorithms for solving the sub problems and getting results
//-----------------------------------------------------------------------------

// Constructor and Destructor
SubProblem_Cplex::SubProblem_Cplex(PVehicle &vehicle) : SubproModeler(vehicle){
    SubProModel_ = IloModel(env_);
    nbGenerated_ = 0;
}
SubProblem_Cplex::~SubProblem_Cplex() {
    env_.end();
}
namespace {
    constexpr double kBinaryThreshold        = 0.5;
    constexpr double kNegativeCostTolerance  = -0.001;  // reduced cost < this counts as "negative enough"
    constexpr int    kDualSimplex            = 2;
    constexpr int    kMainTimeLimit          = 1000;
    constexpr int    kPoolTimeLimit          = 500;
    constexpr int    kThreads                = 8;
    constexpr int    kPoolCapacity           = 20;
    constexpr double kPoolRelGap             = 0.5;
}

void SubProblem_Cplex::configureCplex() {
    // Output settings
    Cplex_.setOut(env_.getNullStream());

    Cplex_.setParam(IloCplex::Param::TimeLimit,                     kMainTimeLimit);
    Cplex_.setParam(IloCplex::Param::MIP::Tolerances::MIPGap,       0.01); // 0.01% gap for near-optimal

    // Algorithm settings
    Cplex_.setParam(IloCplex::Param::RootAlgorithm,                 kDualSimplex);
    Cplex_.setParam(IloCplex::Param::NodeAlgorithm,                 kDualSimplex);
    Cplex_.setParam(IloCplex::Param::Threads,                       kThreads);
    Cplex_.setParam(IloCplex::Param::MIP::Cuts::Gomory,             1);  // Cuts - moderate settings
    Cplex_.setParam(IloCplex::Param::MIP::Strategy::NodeSelect,     2);  // best estimate

    // Emphasis on optimality for column generation
    // Cplex_.setParam(IloCplex::Param::Emphasis::MIP, 2);  // Emphasize optimality

    // Preprocessing
    // Cplex_.setParam(IloCplex::Param::Preprocessing::Reduce, 3);  // Aggressive
    // Cplex_.setParam(IloCplex::Param::MIP::Strategy::Probe, 3);   // Aggressive probing
}

void SubProblem_Cplex::configurePool() {
    // Solution pool settings for multiple solutions
    Cplex_.setParam(IloCplex::Param::MIP::Pool::Capacity,   kPoolCapacity);
    Cplex_.setParam(IloCplex::Param::MIP::Pool::RelGap,     kPoolRelGap);
    Cplex_.setParam(IloCplex::Param::MIP::Pool::Intensity,  2);
    Cplex_.setParam(IloCplex::Param::MIP::Pool::Replace,    1);
    Cplex_.setParam(IloCplex::Param::MIP::Limits::Populate, 100);

    // Shorter time limit for populate
    Cplex_.setParam(IloCplex::Param::TimeLimit,             kPoolTimeLimit);
}

//************************************************************************
// Build the SUb Problem model with CPLEX
//************************************************************************
void SubProblem_Cplex::initializeModel(PInstance &pInst) {
    setNodeIndices();
    nbNodes_    = subGraph_->nbNodes_;
    nbRequests_ = subGraph_->pickNodes_.size();

    const int sourceIndex = Vehicle_->departNode_->nodeIndex_;
    const int sinkIndex   = Vehicle_->sinkNode_->nodeIndex_;

    // -----------------------------------------
    // Define variables
    // -----------------------------------------
    X_ = IloNumVar2D(env_, nbNodes_);
    U_ = IloNumVarArray(env_, nbNodes_, 0, IloInfinity, ILOFLOAT);
    W_ = IloNumVarArray(env_, nbNodes_, 0, IloInfinity, ILOFLOAT);

    for (int i = 0; i < nbNodes_; ++i) {
        std::ostringstream uName, wName;
        uName << "U_i" << i;
        wName << "W_i" << i;
        U_[i].setName(uName.str().c_str());
        W_[i].setName(wName.str().c_str());

        X_[i] = IloNumVarArray(env_, nbNodes_, 0.0, 1.0, ILOINT);
        for (int j = 0; j < nbNodes_; ++j) {
            std::ostringstream xName;
            xName << "X_i" << i << "_j" << j;
            X_[i][j].setName(xName.str().c_str());

            // Statically forbid arcs that can never appear in a valid route.
            if (i == j)                X_[i][j].setBounds(0.0, 0.0); // self-loop
            else if (j == sourceIndex) X_[i][j].setBounds(0.0, 0.0); // nothing -> source
            else if (i == sinkIndex)   X_[i][j].setBounds(0.0, 0.0); // sink   -> nothing
        }
    }

    // Forbid each fresh dropoff -> its matching pickup.
    for (auto & pickNode : subGraph_->pickNodes_) {
        const int p   = pickNode->nodeIndex_;
        const int p_n = pickNode->pairNode_->nodeIndex_;
        X_[p_n][p].setBounds(0.0, 0.0);
    }

    // Register variables with the model so CPLEX extracts them
    // even if a particular index is unreferenced by constraints.
    SubProModel_.add(U_);
    SubProModel_.add(W_);
    for (int i = 0; i < nbNodes_; ++i)
        SubProModel_.add(X_[i]);
}

void SubProblem_Cplex::buildModel(PInstance &pInst){
    auto addCons = [&](IloRange r) { SubProModel_.add(r); };
    std::vector<PNode> nodes = concatenateVectors(subGraph_->pickNodes_, subGraph_->dropNodes_);

    const int    sourceIndex = Vehicle_->departNode_->nodeIndex_;
    const int    sinkIndex   = Vehicle_->sinkNode_->nodeIndex_;

    std::vector<PNode> nodesWithOnboards = nodes;
    for (auto & nodeID : Vehicle_->onboards_)
        nodesWithOnboards.push_back(subGraph_->nodes_[nodeID]);

    std::vector<PNode> allNodes = nodesWithOnboards;
    allNodes.push_back(Vehicle_->sinkNode_);
    allNodes.push_back(Vehicle_->departNode_);

    // -----------------------------------------------------------------------
    // Objective (4a): reduced cost = wait time - sum_i pi_i * served(i) - sigma_v
    // -----------------------------------------------------------------------
    IloExpr objExpr(env_);
    for (auto & pickNode : subGraph_->pickNodes_) {
        const int i = pickNode->nodeIndex_;
        const double e_i = pickNode->readyTime_;

        objExpr += (U_[i] - e_i);
        for (int j = 0; j < subGraph_->nbNodes_; ++j) {
            objExpr -= X_[j][i]* pickNode->related_Request_->dual_;
        }
    }

    objExpr -= Vehicle_->dual_;

    // -----------------------------------------------------------------------
    //  Pruning Theorem: arcs where the smallest possible wait exceeds
    // the penalty are suboptimal. Fix them at 0 via setBounds.
    // -----------------------------------------------------------------------

    for (auto & nodeObj : nodesWithOnboards) {
        if (nodeObj->type_ == SINK) continue;
        const int fromIndex = nodeObj->nodeIndex_;

        for (auto &pickNodeObj : subGraph_->pickNodes_) {
            if (nodeObj->nodeID_ != pickNodeObj->nodeID_) {
                const int toIndex = pickNodeObj->nodeIndex_;

                // Apply the same pruning logic from sortSuccessors
                const double minWait = Vehicle_->departTime_
                                 + nodeObj->travelTimeFromSource_
                                 + nodeObj->serviceTime_
                                 + durationMatrix_[nodeObj->locationID_][pickNodeObj->locationID_]
                                 - pickNodeObj->readyTime_;

                if (minWait > pickNodeObj->related_Request_->penalty_) {
                    // Mark this arc as pruned
                    X_[fromIndex][toIndex].setBounds(0.0, 0.0);
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // (4b-equivalent: 1d, 1e) source/sink flow
    // -----------------------------------------------------------------------
    IloExpr expr_1d(env_);
    IloExpr expr_1e(env_);
    for (auto & otherNode : nodesWithOnboards) {
        expr_1d += X_[sourceIndex][otherNode->nodeIndex_];
        expr_1e += X_[otherNode->nodeIndex_][sinkIndex];

    }
    expr_1d += X_[sourceIndex][sinkIndex];
    expr_1e += X_[sourceIndex][sinkIndex];
    addCons(expr_1d == 1);
    addCons(expr_1e == 1);

    // -----------------------------------------------------------------------
    // (1c) flow balance at non-source / non-sink nodes
    // -----------------------------------------------------------------------
    for (auto & node : nodesWithOnboards) {
        const int i = node->nodeIndex_;
        IloExpr expr_1c(env_);
        for (auto & otherNode : nodesWithOnboards) {
            const int j = otherNode->nodeIndex_;
            if (i != j) expr_1c += (X_[i][j] - X_[j][i]);
        }
        expr_1c += (X_[i][sinkIndex] - X_[sourceIndex][i]);
        addCons(expr_1c == 0);
    }

    // -----------------------------------------------------------------------
    // (1i) (1j) vehicle operating window
    // -----------------------------------------------------------------------
    addCons(U_[sourceIndex] >= Vehicle_->departTime_);
    addCons(U_[sinkIndex] <= Vehicle_->endTime_);

    // -----------------------------------------------------------------------
    // Per-pickup constraints (1f, 1k, 1l)
    // -----------------------------------------------------------------------
    for (auto & pickNode : subGraph_->pickNodes_) {
        const int i = pickNode->nodeIndex_;
        const int i_n = pickNode->pairNode_->nodeIndex_;
        const auto req = pickNode->related_Request_;
        const double e_i = pickNode->readyTime_;
        const double Delta_i = pickNode->serviceTime_;
        const double t_i_min = req->minTravelTime_;
        const double t_i_max = req->maxTravelTime_;
        const double latestPick = req->latestPickup_;

        // (1f) pickup and matching dropoff use the same vehicle
        IloExpr expr_1f(env_);
        for (auto & otherNode : nodesWithOnboards) {
            const int j = otherNode->nodeIndex_;
            expr_1f += (X_[i][j] - X_[i_n][j]);
        }
        expr_1f +=  (X_[i][sinkIndex] - X_[i_n][sinkIndex]);
        addCons(expr_1f == 0);

        // (1k) pickup window
        //   latestPickup_ upper bound is a local addition, not in the paper.
        addCons(U_[i] >= e_i);
        addCons(U_[i] <= latestPick);

        // Implied dropoff time window (valid inequalities)
        addCons(U_[i_n] >= e_i        + t_i_min + Delta_i);
        addCons(U_[i_n] <= latestPick + t_i_max + Delta_i);


        // (1l) ride-time deviation bounds
        addCons(U_[i_n] - U_[i] - Delta_i <= t_i_max);
        addCons(U_[i_n] - U_[i] - Delta_i >= t_i_min);
    }

    // -----------------------------------------------------------------------
    // Per-onboard constraints (1g, 1m)
    // -----------------------------------------------------------------------
    for (auto & onboardID : Vehicle_->onboards_) {
        PNode onNode = subGraph_->nodes_[onboardID];
        const int j = onNode->nodeIndex_;
        const auto req = onNode->related_Request_;
        const double u_P = req->pickTime_;
        const double Delta_j = req->serviceTime_;
        const double t_j_min = req->minTravelTime_;
        const double t_j_max = req->maxTravelTime_;
        const double latestPick = req->latestPickup_;

        // (1g) onboard dropoff is entered from exactly one node
        IloExpr expr_1g(env_);
        for (auto & otherNode : nodesWithOnboards) {
            if (otherNode->nodeIndex_ != j)
                expr_1g += X_[otherNode->nodeIndex_][j];
        }
        expr_1g += X_[sourceIndex][j];
        addCons(expr_1g == 1);

        // (1m) ride-time deviation for onboard passenger
        addCons(U_[j] - u_P - Delta_j <= t_j_max);
        addCons(U_[j] - u_P - Delta_j >= t_j_min);

        // Implied onboard dropoff time window
        addCons(U_[j] >= u_P        + t_j_min + Delta_j);
        addCons(U_[j] <= latestPick + t_j_max + onNode->serviceTime_);
    }

    // -----------------------------------------------------------------------
    // Arc-level constraints: time (1h), load (1n), capacity (1o)
    // -----------------------------------------------------------------------
    for (auto & node : allNodes) {
        const int i = node->nodeIndex_;
        const double Delta_i = node->serviceTime_;

        // (1o) load in [0, Q_v], with tighter bounds where possible
        addCons(W_[i] <= Vehicle_->capacity_);
        addCons(W_[i] >= 0);

        // Tighten capacity bounds
        if (node->type_ == PICKUP)       W_[i].setBounds(node->nbPassengers_, Vehicle_->capacity_);
        else if (node->type_ == DROPOFF) W_[i].setBounds(0.0, Vehicle_->capacity_);

        // Tighten U bounds for pickup nodes
        if (node->type_ == PICKUP) {
            // Tighten U bounds for pickup nodes
            const double minTime = std::max(node->readyTime_,
                                     Vehicle_->departTime_ +
                                     durationMatrix_[Vehicle_->departNode_->locationID_][node->locationID_]);
            const double maxTime = std::min(node->related_Request_->latestPickup_,
                                     Vehicle_->endTime_ - node->serviceTime_);

            if (minTime < maxTime) U_[i].setBounds(minTime, maxTime);
        }

        for (auto & otherNode : allNodes) {
            const int j = otherNode->nodeIndex_;
            if (i == j) continue;

            const double t_ij = durationMatrix_[node->locationID_][otherNode->locationID_];

            // (1h) Big-M time propagation;
            const double Mij = Vehicle_->endTime_ + Delta_i + t_ij;
            addCons(U_[i] + Delta_i + t_ij - U_[j] - Mij * (1 - X_[i][j]) <= 0);

            // (1n) Big-M load propagation; M = 2*Q_v safely covers q_j up to Q_v
            addCons(W_[i] + otherNode->nbPassengers_ - W_[j] - 2 * Vehicle_->capacity_ * (1 - X_[i][j]) <= 0);
        }
    }

    // Initialize load at depart node to reflect currently onboard passengers
    // add this constraint for re-optimization when the vehicle is not idle
    addCons(W_[sourceIndex] >= Vehicle_->numPassengers_);
    SubProModel_.add(IloMinimize(env_, objExpr));
}

// Updated solveModel method
void SubProblem_Cplex::solveModel(PInstance &pInst, std::vector<PRoute> &availableRoutes) {
    // Initialize tracking variables
    bestObjectiveValue_ = 1e9;  // Large positive value
    solutionFound_ = false;
    try {
        Cplex_ = IloCplex(SubProModel_);

        configureCplex();

        // Solve for optimal/near-optimal solution first
        if (!Cplex_.solve()) {
            std::cout << "Failed to optimize the subproblem for vehicle "
                      << Vehicle_->vehicleID_ << std::endl;
            std::cout << "Status: " << Cplex_.getStatus() << std::endl;
            Cplex_.clearModel();
            return;
        }

        // Store the best objective value
        bestObjectiveValue_ = Cplex_.getObjValue();
        Vehicle_->bestReducedCost_ = bestObjectiveValue_;
        solutionFound_ = true;

        std::cout << "Vehicle " << Vehicle_->vehicleID_
                  << " - Best objective: " << bestObjectiveValue_
                  << " (Status: " << Cplex_.getStatus() << ")" << std::endl;

        // Extract the optimal solution
        extractSingleSolution(pInst, availableRoutes, -1);

        // Only look for additional solutions if we found negative reduced cost
        if (bestObjectiveValue_ < kNegativeCostTolerance) {
            configurePool();

            try {
                Cplex_.populate();
                extractPoolSolutions(pInst, availableRoutes);
            } catch (IloException& e) {
                std::cout << "Warning: Could not populate solution pool: " << e << std::endl;
            }
        }

        Cplex_.clearModel();
    }
    catch (IloException& e) {
        std::cout << "Error in solveModel: " << e << std::endl;
    }
}

void SubProblem_Cplex::solveSP(PInstance &pInst, std::vector<PRoute> &availableRoutess) {
    subproTime_->start();
    // Initialize the CPLEX model
    initializeModel(pInst);

    // Build the optimization model
    buildModel(pInst);

    // Solve the subproblem
    std::vector<PRoute> newRoutes;
    solveModel(pInst, newRoutes);

    // Store generated routes
    for (auto &route : newRoutes) {
        if (route->reducedCost_ < -1e-6) { // Negative reduced cost threshold
            availableRoutess.push_back(route);
            nbNegativeColumns_++;
        }
        nbGenerated_++;
    }

    // Update vehicle's best reduced cost
    if (!newRoutes.empty()) {
        float bestCost = newRoutes[0]->reducedCost_;
        for (auto &route : newRoutes) {
            bestCost = std::min(bestCost, route->reducedCost_);
        }
        Vehicle_->bestReducedCost_ = bestCost;
    }
    subproTime_->stop();
}


//************************************************************************
// Display function
//************************************************************************
std::string SubProblem_Cplex::toString() const {
    std::stringstream repStr;
    repStr << "#" << std::endl;
    repStr << "# ------------------------------------------------------------" << std::endl;
    repStr << "# SUB PROBLEM SOLUTION RESULT FOR VEHICLE: " << (Vehicle_)->vehicleID_ << std::endl;
    repStr << "#" << std::endl;
    repStr << "# Solution status = " << Cplex_.getStatus() << std::endl;
    repStr << "# Incumbent objective value = " << Cplex_.getObjValue() << std::endl;
    repStr << "# The solution pool contains = " << Cplex_.getSolnPoolNsolns() << " solutions." << std::endl;
    repStr << "# " << Cplex_.getSolnPoolNreplaced() << " solutions were replaced in the solution pool " << std::endl;

    repStr << "# In total, " << Cplex_.getSolnPoolNsolns() + Cplex_.getSolnPoolNreplaced() <<
    " solutions were generated." << std::endl;

    int nbValidSolution = 0;
    for (int i = 0; i < Cplex_.getSolnPoolNsolns(); ++i) {
        if (Cplex_.getObjValue(i) < 0)
            nbValidSolution ++;
    }
    repStr << "# The solution pool contains = " << nbValidSolution << " solutions with negative reduced cost." << std::endl;
    return repStr.str();
}


// Method to extract pool solutions
void SubProblem_Cplex::extractPoolSolutions(PInstance &pInst, std::vector<PRoute> &availableRoutes) {
    int maxRoutes = 10;
    int routesExtracted = 1;  // Already extracted optimal

    for (int s = 0; s < Cplex_.getSolnPoolNsolns() && routesExtracted < maxRoutes; ++s) {
        double objValue = Cplex_.getObjValue(s);

        // Only extract solutions with negative reduced cost
        if (objValue < -0.001 && objValue > bestObjectiveValue_ + 0.001) {
            extractSingleSolution(pInst, availableRoutes, s);
            routesExtracted++;
        }
    }

}

// Updated extraction method
void SubProblem_Cplex::extractSingleSolution(PInstance &pInst, std::vector<PRoute> &availableRoutes, int solutionIndex) {
    try {
        IloNumArray uVal(env_), wVal(env_);

        const int nbNodes = nbRequests_ * 2 + 2 + Vehicle_->onboards_.size();
        IloArray<IloNumArray> xVal(env_, nbNodes);

        double objValue;

        // Fetch variable values for either the incumbent (-1) or a pool solution
        if (solutionIndex == -1) {  // Incumbent solution
            Cplex_.getValues(uVal, U_);
            Cplex_.getValues(wVal, W_);
            objValue = Cplex_.getObjValue();

            for (int i = 0; i < nbNodes; ++i) {
                xVal[i] = IloNumArray(env_);
                Cplex_.getValues(xVal[i], X_[i]);
            }
        } else {  // Solution from pool
            Cplex_.getValues(uVal, U_, solutionIndex);
            Cplex_.getValues(wVal, W_, solutionIndex);
            objValue = Cplex_.getObjValue(solutionIndex);

            for (int i = 0; i < nbNodes; ++i) {
                xVal[i] = IloNumArray(env_);
                Cplex_.getValues(xVal[i], X_[i], solutionIndex);
            }
        }

        // Create route
        PRoute newRoute = std::make_shared<Route>(Vehicle_->vehicleID_);
        newRoute->reducedCost_ = objValue;

        // Build the route
        newRoute->addSource(Vehicle_->departNode_, Vehicle_->departTime_,
                           Vehicle_->numPassengers_);

        int currentNodeIndex = Vehicle_->departNode_->nodeIndex_;
        const int sinkIndex = Vehicle_->sinkNode_->nodeIndex_;
        const int maxHops = nbNodes_ + 1;
        int hops = 0;

        while (currentNodeIndex != sinkIndex && hops++ < maxHops) {
            int next = -1;
            for (int i = 0; i < nbNodes_; ++i) {
                if (xVal[currentNodeIndex][i] > kBinaryThreshold) { next = i; break; }
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
        newRoute->calculateTripDelay(pInst->parameters_->Wait_W1_, pInst->parameters_->Ride_W2_);

        availableRoutes.push_back(newRoute);

        // Clean up
        for (int i = 0; i < nbNodes; ++i) {
            xVal[i].end();
        }
        uVal.end();
        wVal.end();
    }
    catch (IloException& e) {
        std::cout << "Error in solution extraction: " << e << std::endl;
    }
}

// CPLEX-specific overload of RuntimeMetrics::updateSubproblemMetrics. Lives
// here (next to PCplexSubPro) so the metric overload is colocated with the
// subproblem type it consumes and is naturally only built when USE_CPLEX is ON.
void RuntimeMetrics::updateSubproblemMetrics(const PCplexSubPro &subProblem) {
    nbNegativeFound_ += subProblem->nbNegativeColumns_;
    nbGenerated_ += subProblem->nbGenerated_;
}
