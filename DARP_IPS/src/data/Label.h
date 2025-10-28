//
// Created by Ella on 2/22/2022.
//

#ifndef LABEL_H
#define LABEL_H


#include "utilities/MyTools.h"
#include "data/Graph.h"



//enum LabelStatus { ACTIVE = 0, DOMINATED = 1, INACTIVE = 2, OUTBOUND = 3, TERMINATED = 4};

//-----------------------------------------------------------------------------
// Label class
// object use to save the information of partial path in label setting
//-----------------------------------------------------------------------------

class Label {
private:
    const unsigned int labelID_;                    // label ID
public:
    static unsigned int labelCount_;                // Counter the number of requests
    const char* name_;
    float passedTime_;                              // accumulated time of the path up to last node departure
    float reachedTime_;                             // accumulated time of the path up to reaching the last node
    int load_;                                      // consume capacity of the vehicle
    std::vector<float> travelResources_;            // travel time resource for controlling the trip delay
    std::vector<Node*> openNode_;                   // it makes sure that all the picked requests are dropped
    boost::dynamic_bitset<> completeRequests_;    // keep track of completed tasks
    int numCompleted_;                              //
    boost::dynamic_bitset<> openRequests_;        // used to check feasibility and domination
    std::vector<Node*> pathNode_;                   // order of nodes in the path
    int nbCommitted_;
    float reducedCost_;
    float totalDelay_;
    float totalTripDelay_;
    LabelStatus status_;
    int nbPickUp_;                                  // the number of time the vehicle visit pick up points
    boost::dynamic_bitset<> extendCheck_;         // check the elementary condition of the path
    int numExtendCheck_;                            // used in pulling strategy to determine treated labels
    bool isDropped_;                                // used in pushing for not extending a label to pick after a drop
    bool isDropExtend_;                             // used in pulling to check if a label is extended to onboards before
    float createTime_;                             // the time that label is created
    float labelScore_;                             // it is calculated based on reducedCost_/nbPickUp_
    float lambdaScore_;

    // Constructor and Destructor
    Label(const Vehicle *vehicle, PNode &source, int labelSize);
    Label(const Label &label);
    void copyLabel(const Label &label);
    void copyLabel(const Vehicle *vehicle, PNode &source, int labelSize);

    virtual ~Label();
    // Getters and Setters
    unsigned int getLabelId() const;

    bool operator() (const Label &rhs) const;
    bool checkSubsetOpen(const PLabel &otherLabel) const;
    bool checkSubsetComplete(const PLabel &otherLabel) const;

    void extend(Node *outNode, bool isDropPickPossible, float wait_W1, float ride_W2);
    // this function checks the feasibility of the label before extension
    bool isExtendFeasible(const Node *outNode, int maxPickUp, bool discardSuboptimalPath, int capacity, int &nbPrunedPath,
                          int &nbEliminated, int &nbPrunedArcs);

    bool isTravelTimeFeasible(const Node *outNode, int &nbEliminated) const;
    bool isDominated(const PLabel &otherLabel, const PSolverOption &solverOption) const;
    // this function examines the label to be sure that it leads to a route with negative reduced cost
    bool isEliminated() const;

//    PRoute labelToRoute(PVehicle &vehicle);
    PRoute labelToRoute(const PVehicle &vehicle, const PInstance & pInst) const;

    // Display function
    std::string toString() const;

    bool haveLessTravelResource(const PLabel &otherLabel) const;

};

inline bool operator < (const PLabel &lhs, const PLabel &rhs) {return (lhs->pathNode_.size() < rhs->pathNode_.size()); }

#endif //LABEL_H
