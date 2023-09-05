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
enum NodeStatus { DEFINED = 0, PLANNED = 1, DONE = 2 , COMMITTED = 3};

// Defining the properties of the graph nodes
class Node {
public:
    string nodeID_;                     // node ID
    PRequest related_Request_;          // pointer to its request
    Node *pairNode_;                   // pointer to the pair node
    int locationID_;                    // node location ID
    NodeType type_;                     // node type: pick up, drop off, source, sink
    NodeType initialType_;              // initial type (the type maybe change to source)
    float reachTime_;                   // the time that vehicle reach to the node
    float departTime_;                   // the time that vehicle reach to the node
    int nbPassengers_;                  // number of passengers to pick up or drop off
    float serviceTime_;                 // time to perform pick up or drop off
    NodeStatus nodeStatus_;             // status of the node: no action, planned, completed
    float requestTime_;                 // earliest possible pick up time for the request (request time)
    double bestLabelReduceCost_;        // smallest reduced cost af active vehicles
    int nbActiveLabels_;                // Number of active labels in labeling approach
    std::vector<Node *> successors_;
    std::vector<Node *> closeSuccessors_;
    float travelTimeFromNode_;          // is used in labeling for sorting successors_
    float travelTimeFromSource_;          // is used in labeling for sorting successors_
    int nodeIndex_;
    double maxLabelReducedCost_;
    int zoneID_;

    std::vector<PLabel> activeLabels_;

    // Constructor and Destructor
    Node(string nodeId, PRequest &relatedRequest, NodeType type);
    Node(int locationID, NodeType type);
    Node(int locationID, NodeType type, int vehicleID, int zoneID);

    explicit Node(const PNode& oldNode);

    // this function return the index in of the first label in the active labels of the node whose reduced cost
    // is grater than the newLabel
    unsigned int getLabelListIndex(PLabel &newLabel);
};


//-----------------------------------------------------------------------------
//  Graph class
//-----------------------------------------------------------------------------

class Graph {
public:
    int nbNodes_;
    std::vector<PNode> pickNodes_;
    std::vector<PNode> dropNodes_;
    std::vector<PNode> onboards_;
    std::vector<PNode> sourceNodes_;
    std::vector<PNode> sinkNodes_;
    std::map<std::string,PNode> nodes_;
    std::vector<std::string> intToNodeID_;


    // Constructor and Destructor
    Graph();
    Graph(PNode &source, PNode &sink);

    // function for adding node to graph
    void addNewNode(const PNode &node);

    // function for updating the graph and adding new request
    void addRequestToGraph(PRequest &newRequest);
    void addRequestToMainGraph(PNode & pickNode, PNode & dropNode);
};



typedef std::shared_ptr<Graph> PGraph;

// function to calculate travel time between two node
//double calcTravelTime(PNode startNode, PNode endNode);

// function to calculate travel time of the fastest route between two node
//float queryTravelTime(PNode startNode, PNode endNode);


#endif //GRAPH_H
