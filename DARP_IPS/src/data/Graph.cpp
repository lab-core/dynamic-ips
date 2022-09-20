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

Node::Node(string nodeId, PRequest &relatedRequest, NodeType type, string pairNodeID) : nodeID_(std::move(nodeId)),
                                                                                        related_Request_(relatedRequest),
                                                                                        type_(type), pairNodeID_(std::move(pairNodeID)) {
    reachTime_ = 0;
    departTime_ = 0;
    nbActiveLabels_ = 0;
    travelTimeFromNode_ = 0;
    deltaTime_ = relatedRequest->deltaTime_;
    nodeStatus_ = DEFINED;
    requestTime_ = relatedRequest->earlyPick_;
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
    bestLabelReduceCost_ = INFINITY;
}

Node::Node(int locationID, NodeType type) : locationID_(locationID), type_(type) {
    related_Request_ = nullptr;
    reachTime_ = 0;
    departTime_ = 0;
    nbPassengers_ = 0;
    deltaTime_ = 0;
    requestTime_ = 0;
    nodeStatus_ = DEFINED;
    bestLabelReduceCost_ = INFINITY;
    nbActiveLabels_ = 0;
    travelTimeFromNode_ = 0;
    pairNode_ = nullptr;
    initialType_ = type;

    if (type == SOURCE)
        nodeID_ = Tools::createNodeID(0, SOURCE);
    else if (type == SINK)
        nodeID_ = Tools::createNodeID(0, SINK);
}

Node::Node(int locationID, NodeType type, int vehicleID) : locationID_(locationID), type_(type) {

    related_Request_ = nullptr;
    reachTime_ = 0;
    departTime_ = 0;
    nbPassengers_ = 0;
    deltaTime_ = 0;
    requestTime_ = 0;
    nodeStatus_ = DEFINED;
    bestLabelReduceCost_ = INFINITY;
    nbActiveLabels_ = 0;
    travelTimeFromNode_ = 0;
    pairNode_ = nullptr;
    initialType_ = type;

    if (type == SOURCE)
        nodeID_ = Tools::createSourceID(vehicleID, SOURCE);
    else if (type == SINK)
        nodeID_ = Tools::createSourceID(vehicleID, SINK);
}

// this function return the index in of the first label in the active labels of the node whose reduced cost
// is grater than the newLabel
unsigned int Node::getLabelListIndex(PLabel &newLabel) {
    if (activeLabels_.empty())
        return 0;
    else {
        for (int i = 0; i < activeLabels_.size(); ++i) {
            if ((newLabel->reducedCost_ < activeLabels_[i]->reducedCost_)||
               ((newLabel->reducedCost_ == activeLabels_[i]->reducedCost_)&&(newLabel->passedTime_ <= activeLabels_[i]->passedTime_)))
                return i;
        }
        return activeLabels_.size();
    }
}



Node::~Node() = default;


//-----------------------------------------------------------------------------
//  Graph class
//-----------------------------------------------------------------------------

// Constructor and Destructor
Graph::Graph() {
    nbNodes_ = 0;
};

Graph::Graph(PNode &source, PNode &sink) {
//    nodes_.emplace_back(source);
//    nodes_.emplace_back(sink);
    nbNodes_ = 0;
    addNewNode(source);
    addNewNode(sink);
}

// function for adding node to graph
void Graph::addNewNode(const PNode &node) {
 //   if (node->nodeStatus_ != COMMITTED) {
        nodes_.insert(std::pair<std::string, PNode> (node->nodeID_, node));
        nodeIDToInt_[node->nodeID_] = nbNodes_;
        intToNodeID_.push_back(node->nodeID_);
        nbNodes_++;
//    }
}

void Graph::addRequestToGraph(PRequest &newRequest) {
    // create pickup node and drop off nodes

    std::string pickID = Tools::createNodeID(newRequest->getRequestId(), PICKUP);
    std::string dropID = Tools::createNodeID(newRequest->getRequestId(), DROPOFF);

    addNewNode(std::make_shared<Node>(pickID, newRequest, PICKUP, dropID));
    addNewNode(std::make_shared<Node>(dropID, newRequest, DROPOFF, pickID));
    nodes_[pickID]->pairNode_ = & nodes_[dropID];
    nodes_[dropID]->pairNode_ = & nodes_[pickID];
}

/*void Graph::addNewRequestToGraph(PInstance &pInstance) {
// create pickup node and drop off nodes

    std::string pickID = Tools::createNodeID(pInstance->requests_.back()->getRequestId(), PICKUP);
    std::string dropID = Tools::createNodeID(pInstance->requests_.back()->getRequestId(), DROPOFF);

    addNewNode(std::make_shared<Node>(pickID, pInstance->requests_.back(), PICKUP, dropID));
    addNewNode(std::make_shared<Node>(dropID, pInstance->requests_.back(), DROPOFF, pickID));
    nodes_[pickID]->pairNode_ = & nodes_[dropID];
    nodes_[dropID]->pairNode_ = & nodes_[pickID];
}*/

/*double calcTravelTime(PNode startNode, PNode endNode) {
    double dist = Tools::calcDistance(startNode->locLatitude_, startNode->locLongitude_,
                                      endNode->locLatitude_, endNode->locLongitude_);
    return dist * TimePerMile;
}*/

// function to calculate travel time of the fastest route between two node
/*float queryTravelTime(PNode startNode, PNode endNode) {
    double travelTime = Tools::queryTravelTime(startNode->locLatitude_, startNode->locLongitude_,
                                      endNode->locLatitude_, endNode->locLongitude_);
    return travelTime;
}*/
