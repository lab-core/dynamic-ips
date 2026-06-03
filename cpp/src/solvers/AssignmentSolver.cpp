//
// Created by Elahe Amiri on 2026-05-25.
//

#include "AssignmentSolver.h"

// ─────────────────────────────────────────────
//  Constructor
// ─────────────────────────────────────────────
#ifdef DARP_USE_GUROBI
AssignmentSolver::AssignmentSolver(const GRBEnv& env)
    : GRBenv_(env) {
}
#endif
#ifdef DARP_USE_CPLEX
AssignmentSolver::AssignmentSolver(const IloEnv& env)
    : CPXenv_(env) {
}
#endif


// ─────────────────────────────────────────────
//  Shared: build cost matrix
// ─────────────────────────────────────────────
void AssignmentSolver::buildInput(
    const PInstance& EpochInst,
    const std::vector<PVehicle>& idleVehicles,
    const std::vector<std::vector<float>>& durationMatrix)
{
    nV_   = idleVehicles.size();
    nR_  = EpochInst->lastCommittedRequests_.size();
    assignNeed_ = static_cast<int>(std::min(nV_, nR_));
 //   nbThreads_ = EpochInst->parameters_->nbThreads_;
    nbThreads_ = 1;

    cost_.clear();
    cost_.resize(nV_, std::vector<float>(nR_));
    for (std::size_t v = 0; v < nV_; ++v) {
        int vehLoc = idleVehicles[v]->departNode_->locationID_;
        for (std::size_t r = 0; r < nR_; ++r) {
            int reqLoc = EpochInst->lastCommittedRequests_[r]->PickUpID_;
            // Block same-zone assignments by setting infinite cost
            if (idleVehicles[v]->departNode_->zoneID_ ==
                EpochInst->lastCommittedRequests_[r]->pickZoneID_)
                cost_[v][r] = 1e7f;   // large penalty — avoids ∞ coefficients in solvers
            else
                cost_[v][r] = durationMatrix[vehLoc][reqLoc];
        }
    }
}

// ─────────────────────────────────────────────
//  Shared: write assignments back to routes
// ─────────────────────────────────────────────
void AssignmentSolver::applyAssignments(
    const PInstance& EpochInst,
    std::vector<PVehicle>& idleVehicles,
    const std::vector<std::pair<int,int>>& assignments,
    float elapsedTime,
    myTools::Timer* simulationTime)
{
    for (auto& [v, r] : assignments) {
        std::cout << "Vehicle " << idleVehicles[v]->departNode_->zoneID_
                  << " to Request " << EpochInst->lastCommittedRequests_[r]->pickZoneID_
                  << std::endl;

        PNode sinkNode = std::make_shared<Node>(idleVehicles[v]->sinkNode_);
        sinkNode->zoneID_     = EpochInst->lastCommittedRequests_[r]->pickZoneID_;
        sinkNode->locationID_ = EpochInst->lastCommittedRequests_[r]->PickUpID_;
        idleVehicles[v]->currentRoute_->addSink(sinkNode);

        if (EpochInst->parameters_->solutionMode_ == ANYTIME)
            idleVehicles[v]->updateCurrentRoute(
                EpochInst->simulationStartTime_ + elapsedTime + simulationTime->dSinceStart().count(),
                EpochInst->parameters_->Wait_W1_,
                EpochInst->parameters_->Ride_W2_);
    }
}

// ─────────────────────────────────────────────
//  TO_SOURCE policy: send each idle vehicle to its depot
// ─────────────────────────────────────────────
void AssignmentSolver::returnToSource(
    const PInstance& EpochInst,
    std::vector<PVehicle>& idleVehicles,
    float elapsedTime,
    myTools::Timer* simulationTime)
{
    for (auto& vehicleObj : idleVehicles) {
        if (vehicleObj->currentRoute_->routeNodes_.back()->locationID_ ==
            vehicleObj->sinkNode_->locationID_)
            continue;

        PNode sinkNode = std::make_shared<Node>(vehicleObj->sinkNode_);
        vehicleObj->currentRoute_->addSink(sinkNode);

        if (EpochInst->parameters_->solutionMode_ == ANYTIME)
            vehicleObj->updateCurrentRoute(EpochInst->simulationStartTime_ + elapsedTime +
            simulationTime->dSinceStart().count(),EpochInst->parameters_->Wait_W1_,EpochInst->parameters_->Ride_W2_);
    }
}

// ─────────────────────────────────────────────
//  Main dispatch
// ─────────────────────────────────────────────
void AssignmentSolver::solve(
    const PInstance& EpochInst,
    std::vector<PVehicle>& idleVehicles,
    const std::vector<std::vector<float>>& durationMatrix,
    float elapsedTime,
    myTools::Timer* simulationTime)
{
    buildInput(EpochInst, idleVehicles, durationMatrix);

    std::vector<std::pair<int,int>> assignments;

#if defined(DARP_USE_GUROBI)
    assignments = solveGurobi();
#elif defined(DARP_USE_CPLEX)
    assignments = solveCplex();
#endif

    applyAssignments(EpochInst, idleVehicles, assignments, elapsedTime, simulationTime);
}

// ─────────────────────────────────────────────
//  Gurobi backend
// ─────────────────────────────────────────────
#ifdef DARP_USE_GUROBI
std::vector<std::pair<int,int>> AssignmentSolver::solveGurobi() const
{
    std::vector<std::pair<int,int>> result;
    try {
        GRBModel model(GRBenv_);
        model.set(GRB_IntParam_Threads, nbThreads_);

        // Variables — continuous in [0, 1]; same-zone blocking is encoded
        // via +infinity cost in the objective (see buildInput).
        std::vector<std::vector<GRBVar>> y(nV_, std::vector<GRBVar>(nR_));
        for (std::size_t v = 0; v < nV_; ++v)
            for (std::size_t r = 0; r < nR_; ++r)
                y[v][r] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS);

        // Objective
        GRBLinExpr obj = 0;
        for (std::size_t v = 0; v < nV_; ++v)
            for (std::size_t r = 0; r < nR_; ++r)
                obj += cost_[v][r] * y[v][r];
        model.setObjective(obj, GRB_MINIMIZE);

        // (1) at most one request per vehicle
        for (std::size_t v = 0; v < nV_; ++v) {
            GRBLinExpr sum = 0;
            for (std::size_t r = 0; r < nR_; ++r) sum += y[v][r];
            model.addConstr(sum <= 1);
        }
        // (2) at most one vehicle per request
        for (std::size_t r = 0; r < nR_; ++r) {
            GRBLinExpr sum = 0;
            for (std::size_t v = 0; v < nV_; ++v) sum += y[v][r];
            model.addConstr(sum <= 1);
        }
        // (3) use exactly need assignments
        GRBLinExpr tot = 0;
        for (std::size_t v = 0; v < nV_; ++v)
            for (std::size_t r = 0; r < nR_; ++r)
                tot += y[v][r];
        model.addConstr(tot == assignNeed_);

        model.optimize();

        if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL)
            for (std::size_t v = 0; v < nV_; ++v)
                for (std::size_t r = 0; r < nR_; ++r)
                    if (y[v][r].get(GRB_DoubleAttr_X) > 0.5)
                        result.emplace_back(v, r);
    }
    catch (const GRBException& e) {
        std::cerr << "Gurobi error in assignment: " << e.getMessage() << std::endl;
    }
    return result;
}
#endif

// ─────────────────────────────────────────────
//  CPLEX backend
// ─────────────────────────────────────────────
#ifdef DARP_USE_CPLEX
std::vector<std::pair<int,int>> AssignmentSolver::solveCplex() const
{
    std::vector<std::pair<int,int>> result;
    // Use a local env so all Concert objects are freed when env.end() is called.
    // Using the shared CPXenv_ would accumulate allocations across calls.
    IloEnv env;
    try {
        IloModel model(env);
        IloArray<IloBoolVarArray> y(env, nV_);
        for (std::size_t v = 0; v < nV_; ++v) {
            y[v] = IloBoolVarArray(env, nR_);
            for (std::size_t r = 0; r < nR_; ++r)
                y[v][r] = IloBoolVar(env);
        }

        // Objective
        IloExpr obj(env);
        for (std::size_t v = 0; v < nV_; ++v)
            for (std::size_t r = 0; r < nR_; ++r)
                obj += cost_[v][r] * y[v][r];
        model.add(IloMinimize(env, obj));
        obj.end();

        // (1) at most one request per vehicle
        for (std::size_t v = 0; v < nV_; ++v) {
            IloExpr sum(env);
            for (std::size_t r = 0; r < nR_; ++r) sum += y[v][r];
            model.add(sum <= 1);
            sum.end();
        }
        // (2) at most one vehicle per request
        for (std::size_t r = 0; r < nR_; ++r) {
            IloExpr sum(env);
            for (std::size_t v = 0; v < nV_; ++v) sum += y[v][r];
            model.add(sum <= 1);
            sum.end();
        }
        // (3) use exactly need assignments
        IloExpr tot(env);
        for (std::size_t v = 0; v < nV_; ++v)
            for (std::size_t r = 0; r < nR_; ++r)
                tot += y[v][r];
        model.add(tot == assignNeed_);
        tot.end();

        IloCplex cplex(model);
        cplex.setOut(env.getNullStream());
        cplex.setParam(IloCplex::Param::RootAlgorithm, 2);
        cplex.setParam(IloCplex::Param::Threads, nbThreads_);
        cplex.solve();

        if (cplex.getStatus() == IloAlgorithm::Optimal) {
            for (std::size_t v = 0; v < nV_; ++v)
                for (std::size_t r = 0; r < nR_; ++r)
                    if (cplex.getValue(y[v][r]) > 0.5)
                        result.emplace_back(v, r);
        } else {
            std::cerr << "CPLEX assignment: no optimal solution (status="
                      << cplex.getStatus() << "), saving model to assignment_infeasible.lp"
                      << std::endl;
            cplex.exportModel("assignment_infeasible.lp");
        }
    }
    catch (const IloException& e) {
        std::cerr << "CPLEX error in assignment: " << e.getMessage() << std::endl;
    }
    env.end();   // frees all Concert objects allocated in this call
    return result;
}
#endif