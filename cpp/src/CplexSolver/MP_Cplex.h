//
// Created by Elahe Amiri on 2023-04-22.
//

#ifndef DARP_IPS_MIPMASTERPROBLEM_H
#define DARP_IPS_MIPMASTERPROBLEM_H

#include "RP_Cplex.h"

class MP_Cplex : public RP_Cplex {
public:

    // Constructor and Destructor
    MP_Cplex();

    // this function builds the model at the start of each epoch
    void buildModelMP(PInstance &pInst, std::vector<PRoute> &routeSolution,
                      int nbVehicles);

    // this function updates the model and adds column with negative reduce costs
    void updateModel(PInstance &pInst);
};


#endif //DARP_IPS_MIPMASTERPROBLEM_H
