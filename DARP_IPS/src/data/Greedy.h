//
// Created by Ella on 5/20/2022.
//

#ifndef _GREEDY_H
#define _GREEDY_H

#include "data/Instance.h"
#include "utilities/InputPaths.h"

class GreedyLabel {
public:
    PNode currentNode_;
    PGreedyLabel parent_;
    PGreedyLabel child_;
    PGreedyLabel pair_;
    float reachTime_;
    int nbPassengers_;
    float travelResource_;

    // Constructor
    GreedyLabel(const PNode &currentNode, float reachTime, int nbPassengers);
};

class LinkedGreedyLabels {
public:
    PVehicle* Vehicle_;
    PGreedyLabel head_;         // Last pickup node in the partial path
    PGreedyLabel tail_;         // Last node in the partial path
    PGreedyLabel source_;       // starting node of the route
    float totalDelay_;
    float departTime_;
    double idleTime_;

    // Constructor
public:
    LinkedGreedyLabels(PVehicle &vehicle, PInstance &pInst);
    void insertionAfter(PGreedyLabel preLabel, PNode &insetNode);
    PGreedyLabel findInsertPosition(PNode &pickNode, PNode &dropNode, float maxDuration);
    void insertRequest(PGreedyLabel &preLabel, PNode &pickNode, PNode &dropNode, float maxDuration);
    PRoute greedyLabelToRoute();
};


#endif //_GREEDY_H
