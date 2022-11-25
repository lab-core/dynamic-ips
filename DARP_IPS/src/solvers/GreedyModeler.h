//
// Created by Ella on 5/25/2022.
//

#ifndef GREEDYMODELER_H
#define GREEDYMODELER_H


#include "data/Instance.h"
#include "utilities/InputPaths.h"
#include "data/Greedy.h"

class GreedyModeler {
public:
    std::vector<PLinkedGreedyLabels> solutionList_;
    myTools::Timer *greedyTime_;
    myTools::Timer *greedySolveTime_;
    myTools::Timer *greedySolTime_;
    std::vector<int> selectedVehicles_;
    std::vector<PGreedyLabel> removedLabels_;
    std::vector<PInsertPosition> positionList_;

    //Constructor
    GreedyModeler();
    virtual ~GreedyModeler();

    void initialization(PInstance &PInst);
    void initializationFast(PInstance &PInst);
    void solve(PInstance &PInst);
    void solveInsertion(PInstance &PInst);
    void solutionToRoute(PInstance &PInst);
    void GreedySolver(PInstance &PInst);
    void solveInsertionFast(PInstance &PInst);
    void GreedySolverFast(PInstance &PInst);
};

// this function just assign requests to vehicles based on the minimum delay possible and do not consider ride-sharing
// any pick up is followed by the drop-off
void GreedySolver_noShare(PInstance& PInst);

#endif //GREEDYMODELER_H
