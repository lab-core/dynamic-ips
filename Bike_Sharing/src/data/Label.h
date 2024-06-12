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
    std::bitset<MAX_BIT_SIZE> completeTasks_;    // keep track of completed tasks
    int numCompleted_;
    std::vector<Node*> pathNode_;                   // order of nodes in the path

    double reducedCost_;
    float totalBonus_;
    LabelStatus status_;
    int nbStops_;                                  // the number of time the vehicle visit pick up points
    std::bitset<MAX_BIT_SIZE> extendCheck_;         // check the elementary condition of the path
    int numExtendCheck_;                            // used in pulling strategy to determine treated labels
    bool isDropped_;                                // used in pushing for not extending a label to pick after a drop
    double labelScore_;                             // it is calculated based on reducedCost_/nbStops_

    // Constructor and Destructor
    Label(Vehicle *vehicle, PNode &source);
    Label(const Label &label);
    void copyLabel(const Label &label);

    virtual ~Label();
    // Getters and Setters
    unsigned int getLabelId() const;

    bool operator() (const Label &rhs) const;

    void extend(Node *outNode, vector2D<float>& durationMatrix);
    // this function check the feasibility of the label before extension
    bool isExtendFeasible(Node *outNode, int maxStops, int capacity);
    bool isDominated(PLabel &otherLabel, PSolverOption &solverOption) const;

    PRoute labelToRoute(PVehicle &vehicle, vector2D<float>& durationMatrix);

    // Display function
    std::string toString() const;

};

inline bool operator < (const PLabel &lhs, const PLabel &rhs) {return (lhs->pathNode_.size() < rhs->pathNode_.size()); }

#endif //LABEL_H
