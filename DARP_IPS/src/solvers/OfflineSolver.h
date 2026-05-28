//
// Created by Elahe Amiri on 2025-11-02.
//

#ifndef CP_GUROBI_CPP_OFFLINESOLVER_H
#define CP_GUROBI_CPP_OFFLINESOLVER_H
#include "BaseSolver.h"

#ifdef DARP_USE_GUROBI
#include "GurobiSolver/MIPSolver_Gurobi.h"
#endif
#ifdef DARP_USE_CPLEX
#include "CplexSolver/MIPSolver_Cplex.h"
#endif
//---------------------------------------------------------------------------------------------
//  OfflineSolver class
//  This class implements the offline simulation where all requests are known in advance.
//---------------------------------------------------------------------------------------------

class OfflineSolver : public BaseSolver {
public:
    PMIPSolver MIPModel_;
    // Constructor and Destructor
    OfflineSolver(const PInstance &mainInst, const InputPaths &inputPaths)
        : BaseSolver(mainInst, inputPaths) {}
    ~OfflineSolver() override = default;

    // Function to perform the offline simulation
    void staticSolver(PInstance & mainInst, InputPaths &inputPaths, bool middleSave, float saveTime);

    // Override the pure virtual function from BaseSolver
    void doSimulation(PInstance &mainInst, InputPaths &inputPaths, bool middleSave, float saveTime) override;
};

#endif //CP_GUROBI_CPP_OFFLINESOLVER_H
