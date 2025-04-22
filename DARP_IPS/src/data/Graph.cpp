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
Node::Node(int locationID, NodeType type, int vehicleID, int zoneID) : locationID_(locationID), zoneID_(zoneID), type_(type) {

    related_Request_ = nullptr;
    reachTime_ = 0;
    departTime_ = 0;
    nbPassengers_ = 0;
    serviceTime_ = 0;
    readyTime_ = 0;
    initialReadyTime_ = 0;
    nodeStatus_ = DEFINED;
    bestLabelReduceCost_ = INFINITY;
    nbActiveLabels_ = 0;
    travelTimeFromSource_ = 0;
    pairNode_ = nullptr;
    initialType_ = type;
    nodeIndex_ = -1;

    if (type == SOURCE)
        nodeID_ = myTools::createSourceID(vehicleID, SOURCE);
    else if (type == SINK)
        nodeID_ = myTools::createSourceID(vehicleID, SINK);
}

Node::Node(const PNode &oldNode) {
    nodeID_ = oldNode->nodeID_;
    related_Request_ = oldNode->related_Request_;
    locationID_ = oldNode->locationID_;
    type_ = oldNode->type_;
    initialType_ = oldNode->initialType_;
    reachTime_ = oldNode->reachTime_;
    departTime_ = oldNode->departTime_;
    nbPassengers_ = oldNode->nbPassengers_;
    serviceTime_ = oldNode->serviceTime_;
    nodeStatus_ = oldNode->nodeStatus_;
    readyTime_ = oldNode->readyTime_;
    initialReadyTime_ = oldNode->initialReadyTime_;
    nbActiveLabels_ = 0;
    travelTimeFromSource_ = 0;
    bestLabelReduceCost_ = INFINITY;
    pairNode_ = nullptr;
    zoneID_ = oldNode->zoneID_;
    nodeIndex_ = oldNode->nodeIndex_;
}

Node::Node(string nodeId, const PRequest &relatedRequest, NodeType type) : nodeID_(std::move(nodeId)),
                                                                                  related_Request_(relatedRequest),
                                                                                  type_(type) {
    reachTime_ = 0;
    departTime_ = 0;
    serviceTime_ = relatedRequest->serviceTime_;
    nodeStatus_ = DEFINED;
    readyTime_ = relatedRequest->earlyPick_;
    initialReadyTime_ = related_Request_->initialEarlyPick_;
    pairNode_ = nullptr;
    initialType_ = type;

    if (type == PICKUP){
        locationID_ = relatedRequest->PickUpID_;
        nbPassengers_ = relatedRequest->nbPassengers_;
    }
    else if (type == DROPOFF){
        locationID_ = relatedRequest->DropOffID_;
        nbPassengers_ = (-1) * relatedRequest->nbPassengers_;
    }
    nbActiveLabels_ = 0;
    travelTimeFromSource_ = 0;
    bestLabelReduceCost_ = INFINITY;
    zoneID_ = -1;
    nodeIndex_ = -1;
}



//-----------------------------------------------------------------------------
//  Graph class
//-----------------------------------------------------------------------------

// Constructor and Destructor
Graph::Graph() {
    nbNodes_ = 0;
}

// function for adding node to graph
void Graph::addNewNode(const PNode &node) {
    nodes_.emplace(std::pair<std::string, PNode> (node->nodeID_, node));
    nbNodes_++;
    switch(node->type_) {
        case SOURCE :
            sourceNodes_.push_back(node);
            break;
        case SINK :
            sinkNodes_.push_back(node);
            break;
        case PICKUP :
            pickNodes_.push_back(node);
            break;
        case DROPOFF :
            dropNodes_.push_back(node);
            break;
    }
}

void Graph::addRequestToMainGraph(const PNode & pickNode, const PNode & dropNode) {
    // create pickup node and drop off nodes
    addNewNode(pickNode);
    addNewNode(dropNode);
    nodes_[pickNode->nodeID_]->pairNode_ = &(*nodes_[dropNode->nodeID_]);
    nodes_[dropNode->nodeID_]->pairNode_ = &(*nodes_[pickNode->nodeID_]);
}

Zone::Zone(unsigned int zoneId, int centerLocationId) : zoneID_(zoneId), centerLocationID_(centerLocationId) {
    travelToZone_ = 0;
    nbVehiclesRef_ = 0;
    nbVehicles_ = 0;
    highDemandZone_ = true;
    underCapacity_ = false;
}
