//
// Created by Ella on 5/20/2022.
//

#ifndef GREEDY_H
#define GREEDY_H

#include "data/Instance.h"
#include "utilities/InputPaths.h"

//---------------------------------------------------------------------------------------------
//  GreedyLabel class
//  for Greedy method I use linked list for and I linked GreedyLabel to create routes and insertions
//---------------------------------------------------------------------------------------------
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
    // this function is for reusing the previous labels
    void setValues(PNode currentNode, float reachTime, int nbPassengers);
};

class LinkedGreedyLabels {
public:
    PVehicle* Vehicle_;
    PGreedyLabel head_;         // Last pickup node in the partial path
    PGreedyLabel tail_;         // Last node in the partial path
    PGreedyLabel source_;       // starting node of the route
    float totalDelay_;
    float departTime_;
    float idleTime_;

    // Constructor
public:
    LinkedGreedyLabels(PVehicle &vehicle, PInstance &pInst);
    LinkedGreedyLabels(PVehicle &vehicle, PInstance &pInst, std::vector<PGreedyLabel> &removedLabels);
    LinkedGreedyLabels(const LinkedGreedyLabels &label);

    virtual ~LinkedGreedyLabels();
    void resetLinkedGreedyLabels(std::vector<PGreedyLabel> &removedLabels);

    // this function find a position to insert pickup point and add drop off point at the end
    PGreedyLabel findInsertPosition(PNode &pickNode, PNode &dropNode, float maxDuration) const;
    // this function insert a request based on the pick-up position and add drop off at the end
    void insertRequest(PGreedyLabel &preLabel, PNode &pickNode, PNode &dropNode, float maxDuration);

    /********************* New approach **********************/
    bool isInsertPossible (PGreedyLabel &preLabel, PNode & newNode) const;
    bool isDropPossible (PGreedyLabel &preDrop, PGreedyLabel &pickLabel, PNode & dropNode, float maxDuration) const;
    PInsertPosition findInsertPlace(PNode &pickNode, PNode &dropNode, float maxDuration);
    PInsertPosition findInsertPlace(PNode &pickNode, PNode &dropNode, float maxDuration, std::vector<PGreedyLabel> &removedLabels);
    void findInsertPlace(PNode &pickNode, PNode &dropNode, float maxDuration, std::vector<PGreedyLabel> &removedLabels,
                         PInsertPosition & position);
    void insertNode(PGreedyLabel &preLabel, PNode &newNode);
    void insertNode(PGreedyLabel &preLabel, PNode &newNode, std::vector<PGreedyLabel> &removedLabels);

    // This function remove the Label from the list and update the data based on that
    void removeLabel(PGreedyLabel &label);
    void removeLabel(PGreedyLabel &label, std::vector<PGreedyLabel> &removedLabels);
    void insertRequest(PInsertPosition &position, PNode &pickNode, PNode &dropNode, float maxDuration);
    void insertRequest(PInsertPosition &position, PNode &pickNode, PNode &dropNode, float maxDuration,
                       std::vector<PGreedyLabel> &removedLabels);
    // this function calculate the reachTime from a Label to a node
    static float labelToNodeReachTime(PGreedyLabel &preLabel, PNode &Node) ;

    // this function calculate the reachTime from a node to a Label
    static float nodeToLabelReachTime(float nodeReachTime, PNode &preNode, PGreedyLabel &nextLabel) ;

    // this function starts from a label in the list and update reachTimes and departTimes afterwards to the tail
    void updateReachTimes(PGreedyLabel &preLabel);

    // this function convert a greedyLabel list to a route
    PRoute greedyLabelToRoute() const;
    // Display function
    std::string toString() const;
};

struct insertPosition {
    PGreedyLabel prePickup_;
    PGreedyLabel preDrop_;
    float deltaDelay_;
    float deltaLength_;

    // Constructor
    insertPosition();

    insertPosition(PGreedyLabel prePickup, PGreedyLabel preDrop, float deltaDelay, float deltaLength);
    void updatePosition (const PGreedyLabel &prePickup, const PGreedyLabel &preDrop, float deltaDelay, float deltaLength);
};

#endif //GREEDY_H
