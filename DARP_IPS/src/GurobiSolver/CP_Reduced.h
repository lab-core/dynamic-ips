//
// Created by Elahe Amiri on 2025-07-13.
//

#ifndef CP_REDUCED_H
#define CP_REDUCED_H

#include <string>
#include <vector>

#include "GurobiModeler.h"
#include "data/Instance.h"
#include "data/Request.h"
#include "data/Route.h"
#include "utilities/InputPaths.h"
#include "utilities/Types.h"

class CP_Reduced : public GurobiModeler {
public:
    // Variable arrays
    std::vector<GRBVar> routeIncVar_;    // Incompatible route variables
    std::vector<GRBVar> zIncVar_;        // Incompatible z variables

    // Constraint
    GRBConstr normalConst_;

    // Data structures
    std::vector<PRoute> IncRoute_;
    std::vector<PRoute> fractionalRoutes_;
    std::vector<PRequest> fractionalZ_;

    // Status
    SolutionStatus status_;

    // Helper functions
    void addRouteVar(const PRoute& newRoute, const PRoute &currentVehicleRoute);

    void addRouteVarBatch(const PInstance &pInst, bool greedyBase);

    void addZVar(const PRequest &request);

    void addZVarBatch(const PInstance &pInst);

    // Helper function for creating column
    GRBColumn createCPColumnM1(const PRoute& newRoute, const PRoute &currentVehicleRoute);

    // Constructor and Destructor
    explicit CP_Reduced(const std::string &outputLog)
        : GurobiModeler(outputLog) {}

    void resetForNextIteration();

    // Model initialization
    void initializeCPModel(const PInstance& pInst);

    // Build model
    void buildModel(const PInstance& pInst, bool greedyBase);
    void buildModel_batch(const PInstance& pInst, bool greedyBase);

    void updateNormalCoefficients();

    // Solve model
    void solveCPModel(PInstance& pInst, std::vector<PRequest>& zSolution,
                      std::vector<PRoute>& routeSolution, InputPaths& inputPaths);

    void solveCPDual(PInstance& pInst, InputPaths& inputPaths);

    bool isColumnDisjoint(const std::vector<PRoute>& routeResults, int nbRequests, int nbVehicles);
};



#endif //CP_REDUCED_H
