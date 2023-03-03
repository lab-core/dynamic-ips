//
// Created by Ella on 2/22/2022.
//

#ifndef LABEL_H
#define LABEL_H


#include "utilities/MyTools.h"
#include "data/Graph.h"


enum LabelStatus { ACTIVE = 0, DOMINATED = 1, INACTIVE = 2, OUTBOUND = 3, TERMINATED = 4, MIDDLE = 5};

class Label {
private:
    const unsigned int labelID_;      // request ID
public:
    static unsigned int labelCount_;                        // Counter the number of requests
    const char* name_;
    int load_;                                              // consume capacity of the vehicle
    float passedTime_;                                      // accumulated time of the path
    float reachedTime_;
//    Vehicle *vehicle_;                                     // the vehicle for which the route has created
    std::vector<float> travelResources_;
    std::vector<Node*> openNode_;
     std::shared_ptr<myTools::BitVector> completeRequests_;
    int numCompleted_;
    std::vector<int> openRequests_;
//    std::vector<std::string> pathNodes_;
    std::vector<Node*> pathNode_;

    double reducedCost_;
//    PNode* currentNode_;
    float totalDelay_;
    LabelStatus status_;
    int nbPickUp_;                      // the number of time the vehicle visit pick up points
    int nbPickMove_;                    // this value if for the times that vehicle change location in pickups
    std::shared_ptr<myTools::BitVector> extendCheck_;
    int numExtendCheck_;
    bool isDropped_;                    // used in pushing for not extending a label to pick after a drop
    bool isDropExtend_;                 // used in pulling to check if a label is extended to onboards before
    double createTime_;

    // Constructor and Destructor
    Label(Vehicle *vehicle, PNode &source, int numRequests);
    Label(const Label &label);
    void copyLabel(const Label &label);

    virtual ~Label();
    // Getters and Setters
    unsigned int getLabelId() const;

    bool operator() (const Label &rhs) const;

    void extend(Node *outNode);
    void extend1(Node *outNode);
    // this function check the feasibility of the label before extension
    bool isExtendFeasible(Node *outNode, int maxPickUp, bool usePick, int capacity);
    bool isExtendFeasible1(Node *outNode, int maxPickUp, bool usePick, int capacity);
    bool isDominated(PLabel &otherLabel, PSolverOption &solverOption) const;
    // this function examine the label to be sure that it leads to a route with negative reduced cost
    bool isEliminated();
    // this function check whether the label is originated from a dominated parent or not
    static bool haveDominatedParent() ;
//    PRoute labelToRoute(PVehicle &vehicle);
    PRoute labelToRoute(PVehicle &vehicle, PInstance & pInst);
    // Display function
    std::string toString() const;

};

inline bool operator < (const PLabel &lhs, const PLabel &rhs) {return (lhs->pathNode_.size() < rhs->pathNode_.size()); }

#endif //LABEL_H
