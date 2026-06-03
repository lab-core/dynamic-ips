//
// Created by Elahe Amiri on 2025-11-02.
//

#ifndef CP_GUROBI_CPP_ANYTIMESOLVER_H
#define CP_GUROBI_CPP_ANYTIMESOLVER_H
#include "BaseSolver.h"

//---------------------------------------------------------------------------------------------
//  AnytimeSolver class
//  This class implements the flexible rolling horizon simulation where the size of epochs can
//  vary based on the computational time available.
//---------------------------------------------------------------------------------------------

class AnytimeSolver : public BaseSolver {
public:
    // Constructor and Destructor
    AnytimeSolver(const PInstance &mainInst, const InputPaths &inputPaths)
        : BaseSolver(mainInst, inputPaths) {}

    ~AnytimeSolver() override = default;

    // Function to perform the anytime horizon simulation
    void AnytimeHorizon(PInstance &mainInst, InputPaths &inputPaths, bool middleSave, float saveTime);

    // Override the pure virtual function from BaseSolver
    void doSimulation(PInstance &mainInst, InputPaths &inputPaths, bool middleSave, float saveTime) override;
};


#endif //CP_GUROBI_CPP_ANYTIMESOLVER_H