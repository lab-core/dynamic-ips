//
// Created by Ella on 2021-10-04.
//

#ifndef GRAPH_H
#define GRAPH_H

#include "data/Instance.h"

//-----------------------------------------------------------------------------
//  Node class
//  Define pickup or drop off nodes for each request
//-----------------------------------------------------------------------------

// Defining the properties of the graph nodes
class Node {
public:
    string nodeID_;                         // node ID
    PRequest related_Request_;              // pointer to its related request
    Node *pairNode_;                        // pointer to the pair node
    int locationID_;                        // node location ID
    int zoneID_;                            // zone ID related to the location ID of the node
    NodeType type_;                         // node type: pick up, drop off, source, sink
    NodeType initialType_;                  // initial type (the type maybe change to source)
    float reachTime_;                       // the time that a vehicle reaches to the node
    float nodeDepartTime_;                      // the time that a vehicle departs the node
    float serviceTime_;                     // time to perform pick up or drop off
    int nbPassengers_;                      // number of passengers to pick up or drop off
    float readyTime_;                       // earliest possible pickup time for the request (request time)
    float initialReadyTime_;                // initial earliest possible pickup time for the request (before commit)
    NodeStatus nodeStatus_;                 // status of the node: no action, planned, completed
    float bestLabelReduceCost_;             // smallest reduced cost of active vehicles
    int nbActiveLabels_;                    // Number of active labels in labeling approach
    std::vector<Node *> successors_;        // List of nodes sorted based on distance from the current node
    boost::dynamic_bitset<> prunedArcs_;    // used to mark the pruned arcs from this node
    float travelTimeFromSource_;            // is used in labeling for sorting successors_
    int nodeIndex_;                         // index used to define variables in SubProblem_Cplex / MIPSolver_Cplex
    std::vector<PLabel> activeLabels_;      // is used in labeling list of active labels (non-extended labels)

    // Constructor and Destructor
    Node(string nodeId, const PRequest &relatedRequest, NodeType type);
    Node(int locationID, NodeType type, int vehicleID, int zoneID);

    explicit Node(const PNode& oldNode);

};


//-----------------------------------------------------------------------------
//  Graph class
//  Define the main graph containing all nodes and requests
//-----------------------------------------------------------------------------

class Graph {
public:
    int nbNodes_;
    std::vector<PNode> pickNodes_;                  // list of available pickup nodes
    std::vector<PNode> newPickNodes_;               // list of new arrival pickup nodes
    std::vector<PNode> dropNodes_;                  // list of associated drop-off nodes
    std::vector<PNode> onboards_;                   // list of requests that are already picked up
    std::vector<PNode> sourceNodes_;                // list of source nodes (vehicle start nodes)
    std::vector<PNode> sinkNodes_;                  // list of sink nodes (vehicle end nodes)
    std::unordered_map<std::string,PNode> nodes_;   // map of node objects to nodeIDs

    // Constructor and Destructor
    Graph();

    // function for adding a node to the graph
    void addNewNode(const PNode &node);

    // function for updating the graph and adding a new request
    void addRequestToMainGraph(const PNode & pickNode, const PNode & dropNode);
};


//-----------------------------------------------------------------------------
//  Zone class
//  Define zones in the network for vehicle repositioning
//-----------------------------------------------------------------------------

class Zone {
public:
    const unsigned int zoneID_;                     // Unique ID for each zone instance
    int centerLocationID_;                          // location ID of the center of the zone
    std::vector<PVehicle> zoneVehicles_;            // list of vehicles in the zone
    std::vector<Zone *> successors_;                // list of successor zones based on distance
    float travelToZone_;                            // travel time to the center of the zone
    int nbVehiclesRef_;                             // reference number of vehicles in the zone
    int nbVehicles_;                                // current number of vehicles in the zone
    bool underCapacity_;                            // is the zone under capacity (not enough vehicles in the zone)
    bool highDemandZone_;                           // is the zone a high demand zone
    int nbUnserved_;                                // number of unserved requests in the zone

    // Constructor and Destructor
    Zone(unsigned int zoneId, int centerLocationId);
};


#endif //GRAPH_H
