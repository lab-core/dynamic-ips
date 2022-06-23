//
// Created by Ella on 5/25/2022.
//

#ifndef _GREEDYMODELER_H
#define _GREEDYMODELER_H


#include "data/Instance.h"
#include "utilities/InputPaths.h"
#include "data/Greedy.h"

class GreedyModeler {
    std::vector<PLinkedGreedyLabels> solutionList_;

    //Constructor
public:
    GreedyModeler();

    void initialization(PInstance &PInst);
    void solve(PInstance &PInst);
    void solveInsertion(PInstance &PInst);
    void solutionToRoute(PInstance &PInst);
};

// this function just assign requests to vehicles based on the minimum delay possible and do not consider ride-sharing
// any pick up is followed by the drop off
void GreedySolver_noShare(PInstance& PInst);

#endif //_GREEDYMODELER_H
