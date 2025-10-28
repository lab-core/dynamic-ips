//
// Created by Ella on 5/25/2022.
//

#ifndef GREEDY_MODELLER_H
#define GREEDY_MODELLER_H


#include "data/Instance.h"

//-----------------------------------------------------------------------------
// GreedyModeler class
// functions to solve instance with greedy approach based on cheapest insertion
//-----------------------------------------------------------------------------

class GreedyModeler {
public:
    std::vector<PGreedyRoute> greedyRouteList_;
    myTools::Timer *greedyTime_;                            // time to solve the problem with greedy
    myTools::Timer *greedyAssignTime_;                      // time to solve the assignment problem for dynamic pro.
//    std::vector<int> selectedVehicles_;                   // list of the vehicles selected to solve subproblem
    std::vector<PStopLabel> greedyLabelPool_;               // pool of greedy labels to re-use
    std::vector<PInsertPosition> positionList_;
    float objValue_;

    //Constructor
    GreedyModeler();
    virtual ~GreedyModeler();

    void initialization(PInstance &PInst);
    // this function converts GreedyRoute to Route
    void solutionToRoute(const PInstance &PInst);
    float createUpperbound(float wait_W1, float ride_W2);
    void GreedySolver(PInstance &PInst);
    float GreedyUpperbound(PInstance &PInst);
    void solveInsertion(const PInstance &PInst);
    void setObjValue();
};

#endif //GREEDY_MODELLER_H
