//
// Created by Elahe Amiri on 2025-11-02.
//

#ifndef CP_GUROBI_CPP_BATCHSOLVER_H
#define CP_GUROBI_CPP_BATCHSOLVER_H
#include "BaseSolver.h"

//---------------------------------------------------------------------------------------------
//  BatchSolver class
//  This class implements the fixed rolling horizon simulation where requests are batched at fixed
//  intervals and solved together.
//---------------------------------------------------------------------------------------------

class BatchSolver : public BaseSolver {
public:
    // Constructor and Destructor
    BatchSolver(const PInstance &mainInst, const InputPaths &inputPaths)
        : BaseSolver(mainInst, inputPaths) {}
    ~BatchSolver() override = default;

    // Main function to perform the batch rolling horizon simulation
    void BatchHorizon(PInstance &mainInst, InputPaths &inputPaths, bool middleSave, float saveTime);

    // Override the pure virtual function from BaseSolver
    void doSimulation(PInstance &mainInst, InputPaths &inputPaths, bool middleSave, float saveTime) override;
};


#endif //CP_GUROBI_CPP_BATCHSOLVER_H