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
    PTask relatedTask_;                     // pointer to its task
    NodeType type_;                         // node type: station, source, sink
    int locationIndex_;                        // node location ID
    int load_;
    double bestLabelReduceCost_;            // smallest reduced cost af active vehicles
    int nbActiveLabels_;                    // Number of active labels in labeling approach
    std::vector<Node *> successors_;        // List of nodes sorted based on distance from the current node
    float travelTimeFromNode_;              // is used in labeling for sorting successors_
    float travelTimeFromSource_;            // is used in labeling for sorting successors_
    std::vector<PLabel> activeLabels_;      // list of active labels

    // Constructor and Destructor
    Node(int locationIndex, const PTask &relatedTask, NodeType type);
    Node(int locationIndex, NodeType type);
    explicit Node(const PNode& oldNode);

};


//-----------------------------------------------------------------------------
//  Graph class
//-----------------------------------------------------------------------------

class Graph {
public:
    int nbNodes_;
    std::vector<PNode> taskNodes_;          // list of nodes to perform
    std::vector<PNode> departNodes_;        // depart node of the vehicles
    PNode sink_;                            // sink node of the vehicles


    // Constructor and Destructor
    Graph();
    Graph(PNode &source, PNode &sink);

    // function for adding node to graph
    void addNewNode(const PNode &node);

};



#endif //GRAPH_H
