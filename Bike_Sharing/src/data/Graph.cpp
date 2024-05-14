//
// Created by Ella on 2021-10-04.
//

#include "Graph.h"

#include <utility>

//-----------------------------------------------------------------------------
//  Node class
//  Define pickup or drop off nodes for each request
//-----------------------------------------------------------------------------

using std::to_string;
// Constructor and Destructor

Node::Node(int locationIndex, const PTask &relatedTask, NodeType type) : relatedTask_(relatedTask), type_(type),
                                                                         locationIndex_(locationIndex){
    load_ = relatedTask->nbTransfer_;
    locationIndex_ = locationIndex;
    nbActiveLabels_ = 0;
    travelTimeFromNode_ = 0;
    travelTimeFromSource_ = 0;
    bestLabelReduceCost_ = INFINITY;
}

Node::Node(int locationIndex, NodeType type) : type_(type),locationIndex_(locationIndex){
    relatedTask_ = nullptr;
    locationIndex_ = locationIndex;
    nbActiveLabels_ = 0;
    load_ = 0;
    travelTimeFromNode_ = 0;
    travelTimeFromSource_ = 0;
    bestLabelReduceCost_ = INFINITY;
}
// copy constructor
Node::Node(const PNode &oldNode) {
    relatedTask_ = oldNode->relatedTask_;
    locationIndex_ = oldNode->locationIndex_;
    type_ = oldNode->type_;
    load_ = oldNode->load_;
    nbActiveLabels_ = 0;
    travelTimeFromNode_ = 0;
    travelTimeFromSource_ = 0;
    bestLabelReduceCost_ = INFINITY;
}



//-----------------------------------------------------------------------------
//  Graph class
//-----------------------------------------------------------------------------

// Constructor and Destructor
Graph::Graph() {
    nbNodes_ = 0;
}

Graph::Graph(PNode &source, PNode &sink) {
    nbNodes_ = 0;
    addNewNode(source);
    addNewNode(sink);
}

// function for adding node to graph
void Graph::addNewNode(const PNode &node) {
    nbNodes_++;

    switch(node->type_) {
        case TASK_STATION:
            taskNodes_.push_back(node);
            break;
        case SINK_NODE :
            sink_ = node;
            break;
        case DEPART_NODE :
            departNodes_.push_back(node);
            break;
    }
}
