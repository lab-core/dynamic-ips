//
// Created by Elahe Amiri on 2025-09-16.
//

#ifndef DARP_IPS_CPMODELER_H
#define DARP_IPS_CPMODELER_H

#include <gurobi_c++.h>
#include <string>
#include <vector>
#include "data/Route.h"
#include "data/Request.h"
#include "utilities/InputPaths.h"
#include "utilities/Types.h"


class CPModeler {
public:
    GRBEnv env_;
    GRBModel* model_;
    std::string outputLog_;

    // Constraints
    std::vector<GRBConstr> requestConstr_;
    std::vector<GRBConstr> vehicleConstr_;
    GRBConstr normalConst_;

    // RHS values
    std::vector<double> requestRHS_;
    std::vector<double> vehicleRHS_;

    // Variable arrays
    std::vector<GRBVar> routeIncVar_;    // Incompatible route variables
    std::vector<GRBVar> routeSolVar_;    // Solution route variables
    std::vector<GRBVar> zIncVar_;        // Incompatible z variables
    std::vector<GRBVar> zSolVar_;        // Solution z variables

    // Data structures
    std::vector<PRoute> IncRoute_;
    std::vector<PRoute> fractionalRoutes_;
    std::vector<PRequest> fractionalZ_;

    SolutionStatus status_;

    int nbRequestTask_;
    std::vector<PRoute> routesToAdd_;
    myTools::Timer* solveTime_;

    // Constructor and Destructor
    explicit CPModeler(std::string outputLog);
    ~CPModeler();

    // Model initialization
    void initializeCP(const PInstance& pInst, bool reduced);

    // Helper functions
    GRBColumn createRouteColumn(const PRoute& newRoute, const PRoute &currentVehicleRoute);
    GRBColumn createRouteColumn(const PRoute& newRoute, VarSign sign);

    // add route variables
    void addRouteVar(const PRoute& newRoute, const PRoute &currentVehicleRoute);
    void addRouteVar(const PRoute& newRoute, VarSign sign);

    // add z variables
    void addZVar(const PRequest &request);
    void addZVar(const PRequest& request, VarSign sign);

    // add group variables
    void addRouteVarBatch(const PInstance &pInst);
    void addRouteSolVarBatch(const std::vector<PRoute>& routeSolution);
    void addRouteIncVarBatch();

    // only for out of basis z variables
    void addZVarBatch(const PInstance &pInst);

    // Build model
    void buildModel(const PInstance& pInst);
    void buildModel(const PInstance& pInst, const std::vector<PRoute>& routeSolution);
    void buildModel_batch(const PInstance& pInst);
    void buildModel_batch(const PInstance& pInst, const std::vector<PRoute>& routeSolution);

    void repairModel(const PInstance& pInst, const std::vector<PRoute>& routeSolution);
    // Adding incompatible route columns
    void updateModel();
    void updateNormalCoefficients();
    void resetForNextIteration();

    // Solve model
    void getDuals(const PInstance& pInst);
    int getStatus() const;
    double getObjValue() const;
    double getVarValue(const GRBVar& var) const;

    void saveModelBeforeSolve(const std::string &prefix);

    void printBasisVariables(const std::string &outputFile);

    void saveBasisModel(const std::string &prefix);

    bool verifyBasisSolution(const PInstance &pInst);

    void solveCPModel(PInstance& pInst, std::vector<PRequest>& zSolution,
                      std::vector<PRoute>& routeSolution, InputPaths& inputPaths);
    void solveCPModel(PInstance& pInst, std::vector<PRequest>& zSolution,
                      std::vector<PRoute>& routeSolution, InputPaths& inputPaths, bool update);

    void solveCPDual(PInstance& pInst, InputPaths& inputPaths);

    bool isColumnDisjoint(const std::vector<PRoute>& routeResults, int nbRequests, int nbVehicles);

    bool isColumnDisjointFast(const std::vector<PRequest> &zResults, const std::vector<PRoute> &routeResults,
                              const PInstance &pInst);

    bool isColumnDisjointBit(const vector<PRequest> &zResults, const vector<PRoute> &routeResults,
                             const PInstance &pInst);

    bool isColumnDisjoint(const std::vector<PRequest> &zResults, const std::vector<PRoute> &routeResults,
                          const PInstance &pInst);
    void dump_gurobi();
};

#endif //DARP_IPS_CPMODELER_H
