//
// Created by Ella on 5/20/2022.
//

#ifndef GREEDY_H
#define GREEDY_H

#include "data/Instance.h"

//---------------------------------------------------------------------------------------------
//  StopLabel class
//  for Greedy method I use linked list for and I linked StopLabel to create routes and insertions
//---------------------------------------------------------------------------------------------


// ----------------------------------------------------------------------------------------
//  Objective audit helpers (debugging)
//  Recompute totals from the linked-list representation and compare with stored accumulators.
// ----------------------------------------------------------------------------------------
struct ObjectiveAudit {
    double totalWait = 0.0;
    double totalTripDelay = 0.0;
    double totalObjective = 0.0;
    int pickups = 0;
    int dropoffs = 0;
};

class StopLabel {
public:
    PNode currentNode_;             // Pointer to the current node
    PStopLabel parent_;             // Pointer to the parent stop label
    PStopLabel child_;              // Pointer to the child stop label
    PStopLabel pair_;               // Pointer to the paired stop label
    float reachTime_;               // Time when this stop is reached
    float leaveTime_;               // Time when this stop is left
    int nbPassengers_;              // Number of passengers at this stop
    float travelResource_;          // Resource used for travel (e.g., distance, fuel)

    // Constructor
    StopLabel(PNode currentNode, float reachTime, float departTime, int nbPassengers);

    // this function is for reusing the previous labels
    void setValues(PNode currentNode, float reachTime, float departTime, int nbPassengers);
};

//---------------------------------------------------------------------------------------------
//  GreedyRoute class
//  This class is used to define greedy labels as linked list
//---------------------------------------------------------------------------------------------

class GreedyRoute {
public:
    PVehicle* Vehicle_;             // Pointer to the corresponding vehicle
    PStopLabel lastStop_;           // Last node in the partial path
    PStopLabel initialStop_;        // starting node of the route
    PStopLabel initialDepartStop_;  // initial feasible stop label to re-optimize the route after that
    float totalWait_;               // total delay of the corresponding vehicle
    float totalTripDelay_;          // total trip delay of the corresponding vehicle
    float totalObjective_;
    float departureTime_;           // departure time of the corresponding vehicle
    float idleTime_;                // idle time of the corresponding vehicle (used in static solver)

    
    // Constructor and Destructor
    GreedyRoute(PVehicle &vehicle, const PInstance &pInst, std::vector<PStopLabel> &greedyLabelPool, bool greedyReOptimize);
    GreedyRoute(const GreedyRoute &label);
    virtual ~GreedyRoute();

    // Display function
    [[nodiscard]] std::string toString() const;

    // this function resets the greedy route to recycle labels
    void resetGreedyRoute(std::vector<PStopLabel> &greedyLabelPool) const;

    // this function finds a position to insert pickup point and add drop off point at the end
    void findInsertPlace(PNode &pickNode, PNode &dropNode, float maxDuration, std::vector<PStopLabel> &greedyLabelPool,
                         const PInsertPosition & position, float wait_W1, float ride_W2);

    // a helper function that finds the last possible position to insert drop-off based on capacity feasibility
    [[nodiscard]] PStopLabel findCapacityLimit(const PStopLabel &startLabel) const;

    // a helper function tries to find the best possible insertion for pick-up and drop-off nodes in the middle of the route
    void tryInsertionAt(PStopLabel &prePick, PNode &pickNode, PNode &dropNode, float maxDuration,
                        std::vector<PStopLabel> &greedyLabelPool, const PInsertPosition &position, float wait_W1,
                        float ride_W2);

    // This function inserts the newNode after the preLabel in the linked list
    void insertNode(const PStopLabel &preLabel, PNode &newNode, std::vector<PStopLabel> &greedyLabelPool,
        PStopLabel &pickLabel, float wait_W1,float ride_W2);

    // This function removes the Label from the list and updates the data based on that
    void removeLabel(PStopLabel &label, std::vector<PStopLabel> &greedyLabelPool, PStopLabel &pickLabel, float wait_W1,
        float ride_W2);

    // this function inserts a request based on the pick-up position and adds drop-off at the end
    void insertRequest(const PInsertPosition &position, PNode &pickNode, PNode &dropNode, float maxDuration,
                       std::vector<PStopLabel> &greedyLabelPool, float wait_W1, float ride_W2);

    // this function calculates the reachTime from a Label to a node
    static float labelToNodeReachTime(const PStopLabel &preLabel, const PNode &Node) ;

    // this function starts from a label in the list and updates reachTimes and departTimes up to the tail
    void updateReachTimes(const PStopLabel &preLabel, float wait_W1, float ride_W2);

    // this function converts a greedyLabel list to a route
    [[nodiscard]] PRoute greedyLabelToRoute(bool update) const;

    // this function updates the departure times for the last label of the path
    void updateTailDepart();
    
    // function to check the feasibility of the insertion based on the vehicle capacity and trip duration constraints
    [[nodiscard]] bool isAnyViolation(const PStopLabel &startLabel) const;

    [[nodiscard]] ObjectiveAudit recomputeObjectiveAudit(float wait_W1, float ride_W2) const;

    // Throws if any mismatch exceeds tolerances. Useful to call after insert/remove during debugging.
    void debugCheckObjective(float wait_W1, float ride_W2,
                             double absTol = 1e-3, double relTol = 1e-6,
                             bool verbose = true) const;

};

//---------------------------------------------------------------------------------------------
//  INSERT POSITION STRUCT
//  This structure is used to determine the best position to insert requests in a route.
//  For each request, the best position with the potential increase in delay and length is determined
//  then the best route and position to insert is selected
//---------------------------------------------------------------------------------------------

struct insertPosition {
    PStopLabel prePickup_;          // best stopLabel to insert pick new after
    PStopLabel preDrop_;            // best stopLabel to insert drop new after
    float deltaWait_{};             // potential increase in wait time after insertion
    float deltaTripDelay_{};        // potential increase in length after insertion
    float deltaObjective_{};        // potential increase in objective (based on wait_W1 and ride_W2) after insertion

    // Constructor
    insertPosition();

    // function to update the position data (we re-used the structures)
    void updatePosition (const PStopLabel &prePickup, const PStopLabel &preDrop, float waitIncrease,
        float tripDelayIncrease, float objectIncrease, float wait_W1, float ride_W2);
};



#endif //GREEDY_H
