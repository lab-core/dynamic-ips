//
// Created by Elahe Amiri on 2025-06-26.
//

#ifndef ICG_ALGORITHM_H
#define ICG_ALGORITHM_H
#include "MasterAlgorithm.h"


class ISUD_Algorithm : public MasterAlgorithm {
public:
    // CPLEX solver
    PComplementPro CompPro_;
    PReducedProblem ReducedPro_;

    // Gurobi solver
    PRP_Gurobi RPGurobiPro_;
    PCPModeler CPGurobiPro_;

    int maxIncDegree_;

    bool CPBuilt_;                          // check whether CP model is built or not (one time during each iteration CP is built)


    // Constructor and Destructor
    explicit ISUD_Algorithm(const InputPaths &inputPaths, ModelSOLVER modelSolver);

    void initializationCPLEX(PInstance &pInst, InputPaths &inputPaths, int epoch, const PGreedyModeler &GreedyModel);
    void initializationGurobi(PInstance &pInst, InputPaths &inputPaths, int epoch, const PGreedyModeler &GreedyModel);
    void epochInitialization(PInstance &pInst, InputPaths &inputPaths, int epoch, const PGreedyModeler &GreedyModel) override;

    int solveRP_CPLEX(PInstance &pInst, int compDegree, const InputPaths &inputPaths);
    int solveRP_Gurobi(PInstance &pInst, int compDegree, const InputPaths &inputPaths);
    int solveRP_Pivot(PInstance &pInst, int compDegree, const InputPaths &inputPaths);
    void solveRP(PInstance &pInst, const InputPaths &inputPaths, int epoch, float subProTime);

    void solveISUD_CPLEX(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime);
    void solveISUD_Gurobi(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime);
    void solveISUD_Gurobi2(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime);
    void solveISUD_Gurobi3(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime);

    void solve(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime) override;
    void resetModels() override;
    bool shouldTerminate(const PInstance &pInst, float previousObj, float previousLpObj, int iter) override;
    void getIPSolution(const PInstance &pInst, int epoch, const InputPaths &inputPaths, float subProTime) override;
};



#endif //ICG_ALGORITHM_H
