//
// Created by Elahe Amiri on 2025-07-13.
//

#ifndef CP_REDUCED_H
#define CP_REDUCED_H

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

    // Constructor and Destructor
    explicit CP_Reduced(const std::string &outputLog)
        : GurobiModeler(outputLog) {}

    // Model initialization
    void initializeCPModel(const PInstance& pInst, int nbVehicles, int nbPenalties);

    // Build model
    void buildModel(const PInstance& pInst);
};



#endif //CP_REDUCED_H
