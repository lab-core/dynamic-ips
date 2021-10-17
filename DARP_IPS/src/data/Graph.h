//
// Created by Ella on 2021-10-04.
//

#ifndef _GRAPH_H
#define _GRAPH_H

#include "utilities/MyTools.h"
#include "data/Instance.h"


//-----------------------------------------------------------------------------
//  Node class
//  Define pickup or drop off nodes for each request
//-----------------------------------------------------------------------------

// Different node types and their names
enum NodeType { SOURCE, SINK, PICKUP, DROPOFF };
static const std::vector<std::string> nodeTypeName = {
        "SOURCE_NODE ", "SINK_NODE   ", "PICKUP_NODE ", "DROPOFF_NODE"
};

// Different node status and their names
enum NodeStatus { DEFINED = 0, PLANNED = 1, DONE = 2 };
static const std::vector<std::string> nodeStatusName = {
        "NO_ACTION", "PLANNED  ", "COMPLETED"
};

// Defining the properties of the graph nodes
class Node {
public:
    string nodeID_;                 // node ID
    PRequest* related_Request_;     // pointer to its request
//    Node* pairNode_;              // related pickup/  drop off
    float locLatitude_;             // node location latitude
    float locLongitude_;            // node location longitude
    NodeType type_;                 // node type: pick up, drop off, source, sink
    float reachTime_;               // the time that vehicle reach to the node
    int nbPassengers_;              // number of passengers to pick up or drop off
    float deltaTime_;               // time to perform pick up or drop off
    int nodeStatus_;                // status of the node 0:no action 2:complete

    // Constructor and Destructor
    Node(string nodeId, PRequest &relatedRequest, NodeType type);
    Node(float locLatitude, float locLongitude, NodeType type);

    virtual ~Node();
};
typedef std::shared_ptr<Node> PNode;

//-----------------------------------------------------------------------------
//  Graph class
//-----------------------------------------------------------------------------

class Graph {
public:
    int nbNodes_;
    std::map<std::string,PNode> nodes_;

    // Constructor and Destructor
    Graph();
    Graph(PNode source, PNode sink);

    // function for updating the graph and adding new request
    void addNewRequests(std::vector<PRequest> &newRequests);
};
typedef std::shared_ptr<Graph> PGraph;

#endif //_GRAPH_H
