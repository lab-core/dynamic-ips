//
// Created by Elahe Amiri on 2025-06-26.
//

#ifndef CG_ALGORITHM_H
#define CG_ALGORITHM_H
#include "MasterAlgorithm.h"


class CG_Algorithm : public MasterAlgorithm{
public:
    PMaster MasterProblem_;
    void initializations(PInstance &pInst, InputPaths &inputPaths, int epoch, const PGreedyModeler &GreedyModel);
    void solveMP_LP(PInstance &pInst, const InputPaths &inputPaths, int epoch, float subProTime);
    void solveMP_CG(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime);

    // Constructor and Destructor
    explicit CG_Algorithm(const InputPaths &inputPaths);
    void epochInitialization(PInstance &pInst, InputPaths &inputPaths, int epoch, const PGreedyModeler &GreedyModel) override;
    // Solve Linear relaxation of Restricted MP in CG
    void solveRMP_IP(const PInstance &pInst, int epoch, const InputPaths &inputPaths, float subProTime);
    void solveRMP_LP(PInstance &pInst, int epoch, const InputPaths &inputPaths, float subProTime);
    void solve(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime) override;
    void getIPSolution(const PInstance &pInst, int epoch, const InputPaths &inputPaths, float subProTime) override;
    bool shouldTerminate(const PInstance &pInst, float previousObj, float previousLpObj, int iter) override;
    void resetModels() override;
};
#endif //CG_ALGORITHM_H
