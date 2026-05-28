//
// Created by Elahe Amiri on 2026-05-25.
//

#ifndef ASSIGNMENTSOLVER_H
#define ASSIGNMENTSOLVER_H


#include "data/Instance.h"

#ifdef DARP_USE_GUROBI
#include "GurobiSolver/GurobiModeler.h"
#endif
#ifdef DARP_USE_CPLEX
#include "CplexSolver/CplexModeler.h"
#endif


class AssignmentSolver {
public:
    std::size_t nV_;                        // Number of idle vehicles
    std::size_t nR_;                        // Number of committed requests
    int assignNeed_;
    int nbThreads_;                         // Solver thread budget (from EpochInst parameters)
    std::vector<std::vector<float>> cost_;  // cost[v][r]

#ifdef DARP_USE_GUROBI
    explicit AssignmentSolver(const GRBEnv& env);
#endif
#ifdef DARP_USE_CPLEX
    explicit AssignmentSolver(const IloEnv& env);
#endif

    // Main entry point — dispatches to the right backend
    void solve(const PInstance& EpochInst,
               std::vector<PVehicle>& idleVehicles,
               const std::vector<std::vector<float>>& durationMatrix,
               float elapsedTime,
               myTools::Timer* simulationTime);

    // Return idle vehicles to their depot (TO_SOURCE policy)
    void returnToSource(const PInstance& EpochInst,
                        std::vector<PVehicle>& idleVehicles,
                        float elapsedTime,
                        myTools::Timer* simulationTime);

private:
    // Shared: builds cost matrix and need count
    void buildInput(const PInstance& EpochInst,
                    const std::vector<PVehicle>& idleVehicles,
                    const std::vector<std::vector<float>>& durationMatrix);

    // Shared: writes (v, r) assignments back to vehicle routes
    void applyAssignments(const PInstance& EpochInst,
                          std::vector<PVehicle>& idleVehicles,
                          const std::vector<std::pair<int,int>>& assignments,
                          float elapsedTime,
                          myTools::Timer* simulationTime);

#ifdef DARP_USE_GUROBI
    const GRBEnv& GRBenv_;
    std::vector<std::pair<int,int>> solveGurobi() const;
#endif
#ifdef DARP_USE_CPLEX
    const IloEnv& CPXenv_;
    std::vector<std::pair<int,int>> solveCplex() const;
#endif

};


#endif //ASSIGNMENTSOLVER_H
