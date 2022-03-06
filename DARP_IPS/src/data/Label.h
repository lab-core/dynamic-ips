//
// Created by Ella on 2/22/2022.
//

#ifndef _LABEL_H
#define _LABEL_H

#include "utilities/MyTools.h"
#include "data/Graph.h"

enum LabelStatus { ACTIVE = 0, DOMINATED = 1, INACTIVE = 2, OUTBOUND = 3 };

class Label {
private:
    const unsigned int labelID_;      // request ID
public:
    static unsigned int labelCount_;                        // Counter the number of requests
    const char* name_;
    int load_;                                              // consume capacity of the vehicle
    float passedTime_;                                      // accumulated time of the path
    PVehicle *vehicle_;                                     // the vehicle for which the route has created
    PLabel parent_;                                        // a pointer to the parent of the label
    std::unordered_map<std::string, float> travelResource_; // time that vehicle is planned to reach each node
    std::set<PNode> openNodes_;                             // set of requests that have been started but not completed
    std::set<PRequest> completedRequests_;                  // set of completed requests
    vector<PNode> pathNodes_;                               // list of nodes in the path of the vehicle
    double reducedCost_;
    PNode currentNode_;
    double totalDelay_;
    LabelStatus status_;
    int nbPickUp_;
    std::set<std::string> extendCheck_;
    bool isDropped_;



    // Constructor and Destructor
    Label(PVehicle *vehicle, double reducedCost, PNode source);
    Label(const Label &label);
    virtual ~Label();
    // Getters and Setters
    const unsigned int getLabelId() const;

    bool operator() (const Label &rhs) const;

    void extend(PNode &outNode);
    // this function check the feasibility of the label before extension
    bool isExtendFeasible(PNode &outNode, int maxPickUp);
    bool isDominated(PLabel &otherLabel, PSolverOption &solverOption);
    // this function examine the label to be sure that it leads to a route with negative reduced cost
    bool isEliminated(int maxPickUp, PGraph &graph);
    // this function check whether the label is originated from a dominated parent or not
    bool haveDominatedParent();
    PRoute labelToRoute(PVehicle &vehicle);
    // Display function
    std::string toString() const;

};

inline bool operator < (const PLabel &lhs, const PLabel &rhs) {return (lhs->pathNodes_.size() < rhs->pathNodes_.size()); }

#endif //DARP_IPS_LABEL_H
