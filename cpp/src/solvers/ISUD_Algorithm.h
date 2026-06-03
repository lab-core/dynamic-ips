//
// Created by Elahe Amiri on 2025-06-26.
//

#ifndef ICG_ALGORITHM_H
#define ICG_ALGORITHM_H
#include "MasterAlgorithm.h"


class ISUD_Algorithm : public MasterAlgorithm {
public:
    PReducedPro ReducedPro_;
    int maxIncDegree_;
    bool CPBuilt_;                          // true while CP model is live within an ISUD outer iteration

    // Constructor and Destructor
    explicit ISUD_Algorithm(const InputPaths &inputPaths);

    void epochInitialization(PInstance &pInst, InputPaths &inputPaths, int epoch, const PGreedyModeler &GreedyModel) override;
    void initializations(PInstance &pInst, InputPaths &inputPaths, int epoch, const PGreedyModeler &GreedyModel);
    void solveRP(PInstance &pInst, const InputPaths &inputPaths, int epoch, float subProTime);
    int solveReducedPro(PInstance &pInst, int compDegree, const InputPaths &inputPaths);
    int solveRP_Pivot(PInstance &pInst, int compDegree, const InputPaths &inputPaths);
    void solveISUD_MIP(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime);
    void solveISUD_Pivot(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime);
    void solve(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime) override;
    void resetModels() override;
    bool shouldTerminate(const PInstance &pInst, float previousObj, float previousLpObj, int iter) override;
    void getIPSolution(const PInstance &pInst, int epoch, const InputPaths &inputPaths, float subProTime) override;
};



#endif //ICG_ALGORITHM_H
