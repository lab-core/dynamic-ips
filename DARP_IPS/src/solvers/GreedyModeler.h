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
    std::vector<PGreedyRoute> solutionList_;
    myTools::Timer *greedyTime_;
    myTools::Timer *greedyAssignTime_;
    std::vector<int> selectedVehicles_;
    std::vector<PStopLabel> greedyLabelPool_;
    std::vector<PInsertPosition> positionList_;
    double objValue_;

    //Constructor
    GreedyModeler();
    virtual ~GreedyModeler();

    void initialization(PInstance &PInst);
    void solutionToRoute(PInstance &PInst);
    // Greedy solver
    void GreedySolver(PInstance &PInst);
    void GreedyAssignment(PInstance &PInst);
    void solveInsertion(PInstance &PInst);
    void solveAssignment(PInstance &PInst);
    void setObjValue();
};

// this function just assign requests to vehicles based on the minimum delay possible and do not consider ride-sharing
// any pick up is followed by the drop-off
void GreedySolver_noShare(PInstance& PInst);

#endif //GREEDYMODELER_H
