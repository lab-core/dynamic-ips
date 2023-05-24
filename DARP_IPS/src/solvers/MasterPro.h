//
// Created by Elahe Amiri on 2023-04-22.
//

#ifndef DARP_IPS_MASTERPRO_H
#define DARP_IPS_MASTERPRO_H

#include "solvers/ReducedProblem.h"

class MasterPro : public ReducedProblem {
public:
    double objValue_;
    // Constructor and Destructor
    MasterPro();

    // this function build the model at the start of each epoch
    void buildModelMP(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution);

    // this function update the model and add column with negative reduce costs
    void updateModel();

    void solveModelLP(PInstance &pInst, InputPaths &inputPaths);
    void solveModelInt(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                       InputPaths &inputPaths);
    void solveModelLPInt(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                       InputPaths &inputPaths);
};


#endif //DARP_IPS_MASTERPRO_H
