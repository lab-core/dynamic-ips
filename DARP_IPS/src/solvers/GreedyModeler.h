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
    std::vector<PGreedyRoute> greedyRouteList_;             // list of greedy routes as linked list
    myTools::Timer *greedyTime_;                            // time to solve the problem with greedy
    std::vector<PStopLabel> greedyLabelPool_;               // pool of greedy labels to re-use
    std::vector<PInsertPosition> positionList_;             // list of possible insertion positions   
    float objValue_;                                        // objective value of the greedy solution
    float totalWaitTime_;                                   // total waiting time without penalties
    float totalTripDelay_;                                  // total trip delay
    std::vector<PRequest> zSolution_;                       // list of unserved requests in the greedy solution
    std::vector<PRoute> generatedRoute_;                    // list of generated route

    //Constructor and Destructor
    GreedyModeler();
    virtual ~GreedyModeler();

    // Step 1: function to initialize the greedy modeller and create linked list of greedy routes
    void initialization(PInstance &PInst);

    // Step 2: function to solve the insertion problem for all requests
    void solveInsertion(const PInstance &PInst);

    // Step 3-1: function to convert the greedy solution to Route
    void solutionToRoute(const PInstance &PInst);

    // Step 3-2: function to convert GreedyRoute to Route serving as an upper bound
    float createUpperbound(float wait_W1, float ride_W2);

    // greedy solver to solve the instance with greedy approach from initialization to the end
    void GreedySolver(PInstance &PInst);

    // greedy solver to solve the instance with greedy approach from initialization to the end only to create upper bound
    float GreedyUpperbound(PInstance &PInst);

    // function to set the objective value of the greedy solution
    void setObjValue(float wait_W1, float ride_W2);
};

#endif //GREEDY_MODELLER_H
