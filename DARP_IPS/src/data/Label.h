//
// Created by Ella on 2/22/2022.
//

#ifndef LABEL_H
#define LABEL_H


#include "utilities/MyTools.h"
#include "data/Graph.h"


//-----------------------------------------------------------------------------
// Label class
// object use to save the information of partial path in label setting
//-----------------------------------------------------------------------------

class Label {
private:
    const unsigned int labelID_;                    // Unique ID for each label instance
public:
    static unsigned int labelCount_;                // Counter the number of requests
    const char* name_;
    float passedTime_;                              // accumulated time of the path up to last node departure
    float reachedTime_;                             // accumulated time of the path up to reaching the last node
    int load_;                                      // consume capacity of the vehicle
    std::vector<float> travelResources_;            // travel time resource for controlling the trip delay
    std::vector<Node*> openNode_;                   // it makes sure that all the picked requests are dropped
    boost::dynamic_bitset<> completeRequests_;      // keep track of completed tasks
    int numCompleted_;                              // the number of completed requests
    boost::dynamic_bitset<> openRequests_;          // keep track of opened tasks used to check feasibility and domination
    std::vector<Node*> pathNode_;                   // order of nodes in the partial path
    int nbCommitted_;                               // the number of committed requests in the partial path
    
    float totalWait_;                               // total wait time of the partial path
    float totalTripDelay_;                          // total trip delay of the partial path 
    LabelStatus status_;                            // status of the label: active, dominated, inactive, outbound, terminated  
    int nbPickUp_;                                  // the number of times the vehicle visits pick up points
    boost::dynamic_bitset<> extendCheck_;           // check the elementary condition of the path
    int numExtendCheck_;                            // used in pulling strategy to determine treated labels
    bool isDropped_;                                // used in pushing for not extending a label to pick after a drop
    bool isDropExtend_;                             // used in pulling to check if a label is extended to onboards before
    float createTime_;                              // the time that the label is created
    float reducedCost_;                             // reduced cost of the partial path
    float labelScore_;                              // normalized reduced cost used as the sorting criteria (reducedCost_/nbPickUp_)
    float lambdaScore_;                             // lambda score value used as the sorting criteria
    

    // Constructor and Destructor
    Label(const Vehicle *vehicle, PNode &source, int labelSize);
    Label(const Label &label);
    void copyLabel(const Label &label);
    void copyLabel(const Vehicle *vehicle, PNode &source, int labelSize);
    virtual ~Label();

    // Getters and Setters
    unsigned int getLabelId() const;

    // Display function
    std::string toString() const;

    // this function defines the resource extension functions
    void extend(Node *outNode, bool isDropPickPossible, float wait_W1, float ride_W2);

    //This function checks the feasibility of the label before extension
    bool isExtendFeasible(const Node *outNode, int maxPickUp, bool discardSuboptimalPath, int capacity, int &nbPrunedPath,
                          int &nbEliminated, int &nbPrunedArcs);

    // helper function in feasibility check, checks whether extending to outNode violates travel time resources
    bool isTravelTimeFeasible(const Node *outNode, int &nbEliminated) const;
    
    // helper function in dominance check, checks whether the current label has fewer travel time resources than otherLabel
    bool haveLessTravelResource(const PLabel &otherLabel) const;

    // this function checks if the current label is dominated by otherLabel
    bool isDominated(const PLabel &otherLabel, const PSolverOption &solverOption) const;

    // helper functions in dominance check, checks subset conditions for open requests
    bool checkSubsetOpen(const PLabel &otherLabel) const;

    // helper functions in dominance check, checks subset conditions for completed requests
    bool checkSubsetComplete(const PLabel &otherLabel) const;

    // this function examines the label to be sure that it leads to a route with a negative reduced cost
    bool isEliminated() const;

    //This function converts the label to a route once the labelling process is completed
    PRoute labelToRoute(const PVehicle &vehicle, const PInstance & pInst) const;

};

// Comparator for labels based on the number of nodes in the path
inline bool operator < (const PLabel &lhs, const PLabel &rhs) {return (lhs->pathNode_.size() < rhs->pathNode_.size()); }

// Helper function to create a key for a label based on the total wait time and trip delay
static std::string makeKey(const Label& r, float wait_W1, float ride_W2) {
    float obj = wait_W1 * r.totalWait_ + ride_W2 * r.totalTripDelay_;
    std::string key = std::to_string(obj) + "|";
    std::vector<int> ids;
    for (auto & node : r.pathNode_) {
        if (node->type_ == PICKUP)
            ids.push_back(node->related_Request_->getRequestId());
    }
    for (size_t i = 0; i < ids.size(); ++i) {
        if (i) key += ",";
        key += std::to_string(ids[i]);
    }
    return key;
}





#endif //LABEL_H
