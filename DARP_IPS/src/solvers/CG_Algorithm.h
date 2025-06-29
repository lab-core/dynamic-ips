//
// Created by Elahe Amiri on 2025-06-26.
//

#ifndef CG_ALGORITHM_H
#define CG_ALGORITHM_H
#include "MasterAlgorithm.h"


class CG_Algorithm : public MasterAlgorithm{
public:
    // CPLEX solver
    PMasterPro MasterPro_;
    PDualAuxSolver DualAuxSolver_;

    // Gurobi solver
    PMP_Gurobi MPGurobiPro_;

    // Constructor and Destructor
    explicit CG_Algorithm(const InputPaths &inputPaths, ModelSOLVER modelSolver);

    void initializationCPLEX(PInstance &pInst, const InputPaths &inputPaths, const PGreedyModeler &GreedyModel);
    void initializationGurobi(PInstance &pInst, const InputPaths &inputPaths, const PGreedyModeler &GreedyModel);

    // Solve Linear relaxation of Restricted MP in CG
    void solveRMP_IP(const PInstance &pInst, int epoch, const InputPaths &inputPaths, float subProTime);
    void solveRMP_LP(PInstance &pInst, int epoch, const InputPaths &inputPaths, float subProTime);

    void solveMP_LP_CPLEX(PInstance &pInst, const InputPaths &inputPaths, int epoch, float subProTime);
    void solveMP_LP_Gurobi(PInstance &pInst, const InputPaths &inputPaths, int epoch, float subProTime);

    void solveMP_CG_CPLEX(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime);
    void solveMP_CG_Gurobi(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime);
    void solveMP_Gurobi_tune(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime);
    void solveMP_CG(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime);

    void solveCP_CPLEX(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime);
    void solveCP_Gurobi(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime);

};
#endif //CG_ALGORITHM_H
