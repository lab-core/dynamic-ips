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


// Different node status and their names
enum NodeStatus { DEFINED = 0, PLANNED = 1, COMMITTED = 2 };
static const std::vector<std::string> nodeStatusName = {
        "NO_ACTION", "PLANNED  ", "COMMITTED"
};



// Defining the properties of the graph nodes
class Node {
public:
    string nodeID_;                 // node ID
    PRequest* related_Request_;     // pointer to its request
    string pairNodeID_;             // related pickup/  drop off
    float locLatitude_;             // node location latitude
    float locLongitude_;            // node location longitude
    NodeType type_;                 // node type: pick up, drop off, source, sink
    float reachTime_;               // the time that vehicle reach to the node
    int nbPassengers_;              // number of passengers to pick up or drop off
    float deltaTime_;               // time to perform pick up or drop off
    NodeStatus nodeStatus_;         // status of the node: no action, planned, completed
    float requestTime_;             // earliest possible pick up time for the request (request time)
//    float penalty_;               // penalty of not serving the related request at current period

    // Constructor and Destructor
    Node(string nodeId, PRequest &relatedRequest, NodeType type, string pairNodeID);
    Node(float locLatitude, float locLongitude, NodeType type);

    virtual ~Node();

    // Setters
    void setPairNodeId(const string &pairNodeId);

    void setType(NodeType type);

//    void setPenalty(int epoch);
};


//-----------------------------------------------------------------------------
//  Graph class
//-----------------------------------------------------------------------------

class Graph {
public:
    int nbNodes_;
    std::map<std::string,PNode> nodes_;
//    std::vector<PNode> nodes_;
    std::map<std::string, int> nodeIDToInt_;
    std::vector<std::string> intToNodeID_;


    // Constructor and Destructor
    Graph();
    Graph(PNode source, PNode sink);

    // function for adding node to graph
    void addNewNode(PNode node);

    // function for updating the graph and adding new request
    void addNewRequests(std::vector<PRequest> &newRequests);
};
typedef std::shared_ptr<Graph> PGraph;

// function to calculate travel time between two node
double calcTravelTime(PNode startNode, PNode endNode);

#endif //_GRAPH_H
