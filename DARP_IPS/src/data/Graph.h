//
// Created by Ella on 2021-10-04.
//

#ifndef GRAPH_H
#define GRAPH_H

#include "data/Instance.h"
#include "data/Label.h"

//-----------------------------------------------------------------------------
//  Node class
//  Define pickup or drop off nodes for each request
//-----------------------------------------------------------------------------


// Different node status and their names
//enum NodeStatus { DEFINED = 0, PLANNED = 1, DONE = 2 , COMMITTED = 3};

// Defining the properties of the graph nodes
class Node {
public:
    string nodeID_;                         // node ID
    PRequest related_Request_;              // pointer to its request
    Node *pairNode_;                        // pointer to the pair node
    int locationID_;                        // node location ID
    int zoneID_;                            // zone ID related to the location ID of the node
    NodeType type_;                         // node type: pick up, drop off, source, sink
    NodeType initialType_;                  // initial type (the type maybe change to source)
    float reachTime_;                       // the time that vehicle reach to the node
    float departTime_;                      // the time that vehicle reach to the node
    int nbPassengers_;                      // number of passengers to pick up or drop off
    float serviceTime_;                     // time to perform pick up or drop off
    NodeStatus nodeStatus_;                 // status of the node: no action, planned, completed
    float readyTime_;                     // earliest possible pick up time for the request (request time)
    float bestLabelReduceCost_;            // smallest reduced cost af active vehicles
    int nbActiveLabels_;                    // Number of active labels in labeling approach
    std::vector<Node *> successors_;        // List of nodes sorted based on distance from the current node
    std::bitset<MAX_BIT_SIZE> prunedArces_;
    float travelTimeFromSource_;            // is used in labeling for sorting successors_
    int nodeIndex_;                         // index used to define variables in CPLEXSubProblem / MIPSolver
    std::vector<PLabel> activeLabels_;      // list of active labels of the node (non-extended labels)

    // Constructor and Destructor
    Node(string nodeId, const PRequest &relatedRequest, NodeType type);
    Node(int locationID, NodeType type, int vehicleID, int zoneID);

    explicit Node(const PNode& oldNode);

};


//-----------------------------------------------------------------------------
//  Graph class
//-----------------------------------------------------------------------------

class Graph {
public:
    int nbNodes_;
    std::vector<PNode> pickNodes_;          // list of pickup nodes
    std::vector<PNode> dropNodes_;          // list of drop off nodes
    std::vector<PNode> onboards_;           // list of requests that are already picked up
    std::vector<PNode> sourceNodes_;        // list of source nodes (vehicle start nodes)
    std::vector<PNode> sinkNodes_;          // list of sink nodes (vehicle end nodes)
    std::unordered_map<std::string,PNode> nodes_;     // map of node objects to nodeIDs

    // Constructor and Destructor
    Graph();
    Graph(PNode &source, PNode &sink);

    // function for adding node to graph
    void addNewNode(const PNode &node);

    // function for updating the graph and adding new request
    void addRequestToMainGraph(PNode & pickNode, PNode & dropNode);
};


class Zone {
public:
    const unsigned int zoneID_;
    int centerLocationID_;
    std::vector<PVehicle> zoneVehicles_;
    std::vector<Zone *> successors_;
    int travelToZone_;
    int nbVehiclesRef_;
    int nbVehicles_;
    bool underCapacity_;
    bool highDemandZone_;

    // Constructor and Destructor
    Zone(const unsigned int zoneId, int centerLocationId);
};


#endif //GRAPH_H
