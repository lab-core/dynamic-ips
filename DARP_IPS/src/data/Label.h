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
    int load_;                                      // consume capacity of the vehicle
    float passedTime_;                              // accumulated time of the path
    float reachedTime_;                             //
    std::vector<float> travelResources_;
    std::vector<Node*> openNode_;                   // it makes sure that all the picked requests are dropped
    std::bitset<MAX_BIT_SIZE> completeRequests_;    // keep track of completed tasks
    int numCompleted_;
    std::vector<int> openRequests_;                 // used to check feasibility and domination
    std::vector<Node*> pathNode_;                   // order of nodes in the path

    double reducedCost_;
    float totalDelay_;
    LabelStatus status_;
    int nbPickUp_;                                  // the number of time the vehicle visit pick up points
 //   int nbPickMove_;                                // this value if for the times that vehicle change location in pickups
    std::bitset<MAX_BIT_SIZE> extendCheck_;         // check the elementary condition of the path
    int numExtendCheck_;                            // used in pulling strategy to determine treated labels
    bool isDropped_;                                // used in pushing for not extending a label to pick after a drop
    bool isDropExtend_;                             // used in pulling to check if a label is extended to onboards before
    double createTime_;                             // the time that label is created
    double labelScore_;                             // it is calculated based on reducedCost_/nbPickUp_

    // Constructor and Destructor
    Label(Vehicle *vehicle, PNode &source);
    Label(const Label &label);
    void copyLabel(const Label &label);

    virtual ~Label();
    // Getters and Setters
    unsigned int getLabelId() const;

    bool operator() (const Label &rhs) const;

    void extend(Node *outNode);
    // this function check the feasibility of the label before extension
    bool isExtendFeasible(Node *outNode, int maxPickUp, bool usePick, int capacity);
    bool isDominated(PLabel &otherLabel, PSolverOption &solverOption) const;
    // this function examine the label to be sure that it leads to a route with negative reduced cost
    bool isEliminated();

//    PRoute labelToRoute(PVehicle &vehicle);
    PRoute labelToRoute(PVehicle &vehicle, PInstance & pInst);
    // Display function
    std::string toString() const;

};

inline bool operator < (const PLabel &lhs, const PLabel &rhs) {return (lhs->pathNode_.size() < rhs->pathNode_.size()); }

#endif //LABEL_H
