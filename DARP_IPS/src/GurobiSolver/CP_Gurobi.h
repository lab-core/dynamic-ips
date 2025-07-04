//
// Created by Elahe Amiri on 2025-06-26.
//

#ifndef CP_GUROBI_H
#define CP_GUROBI_H
#include <gurobi_c++.h>
#include <string>
#include <vector>

#include "GurobiModeler.h"
#include "CplexSolver/CplexModeler.h"
#include "data/Instance.h"
#include "data/Request.h"
#include "data/Route.h"
#include "utilities/InputPaths.h"
#include "utilities/Types.h"


class CP_Gurobi : public GurobiModeler{
public:
// Variable arrays
    std::vector<GRBVar> routeIncVar_;    // Incompatible route variables
    std::vector<GRBVar> routeSolVar_;    // Solution route variables
    std::vector<GRBVar> zIncVar_;        // Incompatible z variables
    std::vector<GRBVar> zSolVar_;        // Solution z variables

    // Constraint
    GRBConstr normalConst_;

    // Data structures
    std::vector<PRoute> IncRoute_;
    std::vector<PRoute> fractionalRoutes_;
    std::vector<PRequest> fractionalZ_;

    // Status
    SolutionStatus status_;

    // Helper functions
    void addZVar(const PRequest& request, VarSign sign);
    void addRouteVar(const PRoute& newRoute, VarSign sign, const PInstance& pInst);

    // Constructor and Destructor
    explicit CP_Gurobi(std::string outputLog);

    void resetForNextIteration();

    // Model initialization
    void initializeCPModel(const PInstance& pInst, int nbVehicles);

    // Build model
    void buildModel(const PInstance& pInst, const std::vector<PRequest>& zSolution,
                    const std::vector<PRoute>& routeSolution, int nbVehicles);

    // Repair model
    void repairModel(const PInstance& pInst, const std::vector<PRequest>& zSolution,
                     const std::vector<PRoute>& routeSolution);

    // Update model
    void updateModel(const PInstance& pInst);

void updateNormalCoefficients();

void updateModel_Batch(const PInstance& pInst);

    // Solve model
    void solveCPModel(PInstance& pInst, std::vector<PRequest>& zSolution,
                      std::vector<PRoute>& routeSolution, InputPaths& inputPaths);

    void solveCPModelFresh(PInstance& pInst, std::vector<PRequest>& zSolution,
                      std::vector<PRoute>& routeSolution, InputPaths& inputPaths);

    bool isColumnDisjointFast(const std::vector<PRequest>& zResults,
                                         const std::vector<PRoute>& routeResults,
                                         const PInstance& pInst);

    bool isColumnDisjointBit(const vector<PRequest> &zResults, const vector<PRoute> &routeResults, const PInstance &pInst);

    bool isColumnDisjoint(const std::vector<PRequest>& zResults,
                                         const std::vector<PRoute>& routeResults,
                                         const PInstance& pInst);
    // Display
    std::string toString() const;

    // Getters
    SolutionStatus getStatus() const { return status_; }
    const std::vector<PRoute>& getFractionalRoutes() const { return fractionalRoutes_; }
    const std::vector<PRequest>& getFractionalZ() const { return fractionalZ_; }
};



#endif //CP_GUROBI_H
