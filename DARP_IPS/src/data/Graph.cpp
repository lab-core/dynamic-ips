//
// Created by Ella on 2021-10-04.
//

#include "Graph.h"

//-----------------------------------------------------------------------------
//  Node class
//  Define pickup or drop off nodes for each request
//-----------------------------------------------------------------------------

using std::to_string;
// Constructor and Destructor

Node::Node(string nodeId, PRequest &relatedRequest, NodeType type, string pairNodeID) : nodeID_(nodeId),
                                                                                        related_Request_(relatedRequest),
                                                                                        type_(type), pairNodeID_(pairNodeID) {
    reachTime_ = 0;
    deltaTime_ = relatedRequest->deltaTime_;
    nodeStatus_ = DEFINED;
    requestTime_ = relatedRequest->earlyPick_;

    if (type == PICKUP){
        /*locLatitude_ = relatedRequest->PickUpLatitude_;
        locLongitude_ = relatedRequest->PickUpLongitude_;*/
        locationID_ = relatedRequest->PickUpID_;
        nbPassengers_ = relatedRequest->nbPassengers_;
    }
    else if (type == DROPOFF){
        /*locLatitude_ = relatedRequest->DropOffLatitude_;
        locLongitude_ = relatedRequest->DropOffLongitude_;*/
        locationID_ = relatedRequest->DropOffID_;
        nbPassengers_ = (-1) * relatedRequest->nbPassengers_;
    }
    bestLabelReduceCost_ = INFINITY;
}

Node::Node(int locationID, NodeType type) : locationID_(locationID), type_(type) {
    related_Request_ = nullptr;
    reachTime_ = 0;
    nbPassengers_ = 0;
    deltaTime_ = 0;
    requestTime_ = 0;
    nodeStatus_ = DEFINED;
    bestLabelReduceCost_ = INFINITY;

    if (type == SOURCE)
        nodeID_ = Tools::createNodeID(0, SOURCE);
    else if (type == SINK)
        nodeID_ = Tools::createNodeID(0, SINK);
}

Node::Node(int locationID, NodeType type, int vehicleID) : locationID_(locationID), type_(type) {

    related_Request_ = nullptr;
    reachTime_ = 0;
    nbPassengers_ = 0;
    deltaTime_ = 0;
    requestTime_ = 0;
    nodeStatus_ = DEFINED;
    bestLabelReduceCost_ = INFINITY;

    if (type == SOURCE)
        nodeID_ = Tools::createSourceID(vehicleID, SOURCE);
    else if (type == SINK)
        nodeID_ = Tools::createSourceID(vehicleID, SINK);
}


void Node::setPairNodeId(const string &pairNodeId) {
    pairNodeID_ = pairNodeId;
}

void Node::setType(NodeType type) {
    type_ = type;
}
// this function return the index in of the first label in the active labels of the node whose reduced cost
// is grater than the newLabel
int Node::getLabelListIndex(PLabel newLabel) {
    if (activeLabels_.empty())
        return 0;
    else {
        for (int i = 0; i < activeLabels_.size(); ++i) {
            if (newLabel->reducedCost_ < activeLabels_[i]->reducedCost_)
                return i;
            else if ((newLabel->reducedCost_ == activeLabels_[i]->reducedCost_)&&(newLabel->passedTime_ <= activeLabels_[i]->passedTime_))
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

Graph::Graph(PNode source, PNode sink) {
//    nodes_.emplace_back(source);
//    nodes_.emplace_back(sink);
    nbNodes_ = 0;
    addNewNode(source);
    addNewNode(sink);
}

// function for adding node to graph
void Graph::addNewNode(PNode node) {
    nodes_.insert(std::pair<std::string, PNode> (node->nodeID_, node));
//    node->setPenalty(0);
    nodeIDToInt_[node->nodeID_] = nbNodes_;
    intToNodeID_.push_back(node->nodeID_);
    nbNodes_++;
}
// function for updating the graph and adding new request
void Graph::addRequestsToGraph(PInstance &pInstance) {
    for (auto & requestObj : pInstance->requests_) {
        // create pickup node and drop off nodes
        requestObj->setPenalty(0, pInstance->parameters_, pInstance->simulationStartTime_);

        std::string pickID = Tools::createNodeID(requestObj->getRequestId(), PICKUP);
        std::string dropID = Tools::createNodeID(requestObj->getRequestId(), DROPOFF);

        addNewNode(std::make_shared<Node>(pickID, requestObj, PICKUP, dropID));
        addNewNode(std::make_shared<Node>(dropID, requestObj, DROPOFF, pickID));
        nodes_[pickID]->pairNode_ = & nodes_[dropID];
        nodes_[dropID]->pairNode_ = & nodes_[pickID];
    }
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

void Graph::addNewRequestToGraph(PInstance &pInstance) {
// create pickup node and drop off nodes

    std::string pickID = Tools::createNodeID(pInstance->requests_.back()->getRequestId(), PICKUP);
    std::string dropID = Tools::createNodeID(pInstance->requests_.back()->getRequestId(), DROPOFF);

    addNewNode(std::make_shared<Node>(pickID, pInstance->requests_.back(), PICKUP, dropID));
    addNewNode(std::make_shared<Node>(dropID, pInstance->requests_.back(), DROPOFF, pickID));
    nodes_[pickID]->pairNode_ = & nodes_[dropID];
    nodes_[dropID]->pairNode_ = & nodes_[pickID];
}

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
