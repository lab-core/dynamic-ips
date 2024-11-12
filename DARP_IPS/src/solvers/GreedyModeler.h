//
// Created by Ella on 5/25/2022.
//

#ifndef GREEDY_MODELLER_H
#define GREEDY_MODELLER_H


#include "data/Instance.h"
#include "utilities/InputPaths.h"
#include "data/Greedy.h"

//-----------------------------------------------------------------------------
// GreedyModeler class
// functions to solve instance with greedy approach based on cheapest insertion
//-----------------------------------------------------------------------------

class GreedyModeler {
public:
    std::vector<PGreedyRoute> greedyRouteList_;
    myTools::Timer *greedyTime_;                            // time to solve the problem with greedy
    myTools::Timer *greedyAssignTime_;                      // time to solve the assignment problem for dynamic pro.
//    std::vector<int> selectedVehicles_;                     // list of the vehicles selected to solve sub problem
    std::vector<PStopLabel> greedyLabelPool_;               // pool of greedy labels to re-use
    std::vector<PInsertPosition> positionList_;
    float objValue_;

    //Constructor
    GreedyModeler();
    virtual ~GreedyModeler();

    void initialization(PInstance &PInst);
    // this function convert GreedyRoute to Route
    void solutionToRoute(PInstance &PInst);
    void GreedySolver(PInstance &PInst);
    void GreedyAssignment(PInstance &PInst, int select);
    void solveInsertion(PInstance &PInst);
    void solveAssignment(PInstance &PInst,int select);
    void setObjValue();
};

// this function just assign requests to vehicles based on the minimum delay possible and do not consider ride-sharing
// any pick up is followed by the drop-off
void GreedySolver_noShare(PInstance& PInst);

#endif //GREEDY_MODELLER_H
