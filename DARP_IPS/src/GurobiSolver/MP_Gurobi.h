//
// Created by Elahe Amiri on 2025-06-25.
//

#ifndef MP_GUROBI_H
#define MP_GUROBI_H

#include "GurobiModeler.h"
#include "data/Instance.h"
#include <vector>

#include "RP_Gurobi.h"

class MP_Gurobi : public RP_Gurobi {
public:
    MP_Gurobi(std::string outputLog);

    // Build the master problem model
    void buildModelMP(PInstance& pInst, std::vector<PRoute>& routeSolution, int nbVehicles);

    // Update model with new routes
    void updateModel(PInstance& pInst);

    // Getters
    const std::vector<PRoute>& getCompRoutes() const { return compRoutes_; }

};



#endif //MP_GUROBI_H
