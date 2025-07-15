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
    float createUpperbound(const PInstance &PInst);
    void GreedySolver(PInstance &PInst);
    void GreedyAssignment(PInstance &PInst, int select);
    void solveInsertion(const PInstance &PInst);
    void solveAssignment(const PInstance &PInst,int select);
    void setObjValue();
};

// this function assigns requests to vehicles based on the minimum delay possible and do not consider ride-sharing
// any pickup is followed by the drop-off
void GreedySolver_noShare(const PInstance& PInst);

#endif //GREEDY_MODELLER_H
