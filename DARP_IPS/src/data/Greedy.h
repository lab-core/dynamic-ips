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
    float departTime_;
    int nbPassengers_;
    float travelResource_;

    // Constructor
    GreedyLabel(PNode currentNode, float reachTime, int nbPassengers);
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
    LinkedGreedyLabels(const LinkedGreedyLabels &label);
//    void insertionAfter(PGreedyLabel preLabel, PNode &insetNode);
    PGreedyLabel findInsertPosition(PNode &pickNode, PNode &dropNode, float maxDuration) const;
    void insertRequest(PGreedyLabel &preLabel, PNode &pickNode, PNode &dropNode, float maxDuration);
    void insertNode(PGreedyLabel &preLabel, PNode &newNode);
    void removeLabel(PGreedyLabel &Label, float deltaT, float deltaDelay);
    void insertRequest(PInsertPosition &position, PNode &pickNode, PNode &dropNode, float maxDuration);
    PRoute greedyLabelToRoute() const;
    PInsertPosition findInsertPlace(PNode &pickNode, PNode &dropNode, float maxDuration);
    bool isInsertPossible (PGreedyLabel &preLabel, PNode & newNode) const;
    bool isDropPossible (PGreedyLabel &preDrop, PGreedyLabel &pickLabel, PNode & dropNode, float maxDuration) const;
};

struct insertPosition {
    PGreedyLabel prePickup_;
    PGreedyLabel preDrop_;
    float deltaDelay_;
    float deltaLength_;

    // Constructor
    insertPosition(PGreedyLabel prePickup, PGreedyLabel preDrop, float deltaDelay, float deltaLength);
    void updatePosition (const PGreedyLabel &prePickup, const PGreedyLabel &preDrop, float deltaDelay, float deltaLength);
};

#endif //_GREEDY_H
