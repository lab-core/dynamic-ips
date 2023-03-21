//
// Created by Ella on 5/20/2022.
//

#ifndef GREEDY_H
#define GREEDY_H

#include "data/Instance.h"
#include "utilities/InputPaths.h"

//---------------------------------------------------------------------------------------------
//  StopLabel class
//  for Greedy method I use linked list for and I linked StopLabel to create routes and insertions
//---------------------------------------------------------------------------------------------
class StopLabel {
public:
    PNode currentNode_;
    PStopLabel parent_;
    PStopLabel child_;
    PStopLabel pair_;
    float reachTime_;
    float leaveTime_;
    int nbPassengers_;
    float travelResource_;

    // Constructor
    StopLabel(PNode currentNode, float reachTime, float departTime, int nbPassengers);
    // this function is for reusing the previous labels
    void setValues(PNode currentNode, float reachTime, float departTime, int nbPassengers);
};

class GreedyRoute {
public:
    PVehicle* Vehicle_;
    PStopLabel PCurrentStop_;         // Last pickup node in the partial path
    PStopLabel PLastStop_;    // Last node in the partial path
    PStopLabel PInitialStop_;       // starting node of the route
    float totalDelay_;
    float departureTime_;
    float preDepartTime_;
    float idleTime_;
    bool selected_;
    bool idle_;
    // Constructor
public:
    GreedyRoute(PVehicle &vehicle, PInstance &pInst);
    GreedyRoute(PVehicle &vehicle, PInstance &pInst, std::vector<PStopLabel> &greedyLabelPool, bool greedyReOptimize);
    GreedyRoute(const GreedyRoute &label);

    virtual ~GreedyRoute();
    void resetLinkedGreedyLabels(std::vector<PStopLabel> &greedyLabelPool);

    /********************* New approach **********************/
    bool isInsertPossible (PStopLabel &preLabel, PNode & newNode) const;
    bool isDropPossible (PStopLabel &preDrop, PStopLabel &pickLabel, PNode & dropNode, float maxDuration) const;
//    bool isDropPossible1 (PStopLabel &preDrop, PStopLabel &pickLabel, PNode & dropNode, float maxDuration) const;
    // this function find a position to insert pickup point and add drop off point at the end
    void findInsertPlace(PNode &pickNode, PNode &dropNode, float maxDuration, std::vector<PStopLabel> &removedLabels,
                         PInsertPosition & position);

    void findInsertPlace1(PNode &pickNode, PNode &dropNode, float maxDuration, std::vector<PStopLabel> &greedyLabelPool,
                         PInsertPosition & position);

    void findInsertPlace2(PNode &pickNode, PNode &dropNode, float maxDuration, std::vector<PStopLabel> &greedyLabelPool,
                          PInsertPosition & position);

    void insertNode(PStopLabel &preLabel, PNode &newNode, std::vector<PStopLabel> &greedyLabelPool);
    void insertNode1(PStopLabel &preLabel, PNode &newNode, std::vector<PStopLabel> &greedyLabelPool);

    // This function remove the Label from the list and update the data based on that
    void removeLabel(PStopLabel &label, std::vector<PStopLabel> &greedyLabelPool);
    // this function insert a request based on the pick-up position and add drop off at the end
    void insertRequest(PInsertPosition &position, PNode &pickNode, PNode &dropNode, float maxDuration,
                       std::vector<PStopLabel> &greedyLabelPool);
    // this function calculate the reachTime from a Label to a node
    static float labelToNodeReachTime(PStopLabel &preLabel, PNode &Node) ;
//    static float labelToNodeReachTime1(PStopLabel &preLabel, PNode &Node) ;

    // this function calculate the reachTime from a node to a Label
    static float nodeToLabelReachTime(float nodeReachTime, PNode &preNode, PStopLabel &nextLabel) ;
    static float nodeToLabelReachTime1(float nodeReachTime, PNode &preNode, PStopLabel &nextLabel) ;

    // this function starts from a label in the list and update reachTimes and departTimes afterwards to the tail
    void updateReachTimes(PStopLabel &preLabel);
    void updateReachTimes1(PStopLabel &preLabel);

    // this function convert a greedyLabel list to a route
    PRoute greedyLabelToRoute(bool update) const;
    // Display function
    std::string toString() const;
    void findAssignedPlace(PNode &pickNode, PNode &dropNode, float maxDuration, std::vector<PStopLabel> &removedLabels,
                         PInsertPosition & position);
    void findAssignedPlace1(PNode &pickNode, PNode &dropNode, float maxDuration, std::vector<PStopLabel> &removedLabels,
                           PInsertPosition & position);

    void updateTailDepart();
    void updateTailDepart1();

    bool isAnyViolation(PStopLabel &startLabel);
};

struct insertPosition {
    PStopLabel prePickup_;
    PStopLabel preDrop_;
    float deltaDelay_;
    float deltaLength_;

    // Constructor
    insertPosition();

    insertPosition(PStopLabel prePickup, PStopLabel preDrop, float waitIncrease, float lengthIncrease);
    void updatePosition (const PStopLabel &prePickup, const PStopLabel &preDrop, float waitIncrease, float lengthIncrease);
};

#endif //GREEDY_H
