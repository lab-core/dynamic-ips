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
            related_Request_(&relatedRequest), type_(type), pairNodeID_(pairNodeID) {
    reachTime_ = 0;
    deltaTime_ = relatedRequest->deltaTime_;
    nodeStatus_ = DEFINED;
    requestTime_ = relatedRequest->earlyPick_;

    if (type == PICKUP){
        locLatitude_ = relatedRequest->PickUpLatitude_;
        locLongitude_ = relatedRequest->PickUpLongitude_;
        nbPassengers_ = relatedRequest->nbPassengers_;
    }
    else if (type == DROPOFF){
        locLatitude_ = relatedRequest->DropOffLatitude_;
        locLongitude_ = relatedRequest->DropOffLongitude_;
        nbPassengers_ = (-1) * relatedRequest->nbPassengers_;
    }
}

Node::Node(float locLatitude, float locLongitude, NodeType type) : locLatitude_(locLatitude),
                                                                   locLongitude_(locLongitude),
                                                                   type_(type) {
    related_Request_ = nullptr;
    reachTime_ = 0;
    nbPassengers_ = 0;
    deltaTime_ = 0;
    requestTime_ = 0;
    nodeStatus_ = DEFINED;

    if (type == SOURCE)
        nodeID_ = Tools::createNodeID(0, SOURCE);
    else if (type == SINK)
        nodeID_ = Tools::createNodeID(0, SINK);
}

void Node::setPairNodeId(const string &pairNodeId) {
    pairNodeID_ = pairNodeId;
}

void Node::setType(NodeType type) {
    type_ = type;
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
void Graph::addNewRequests(std::vector<PRequest> &newRequests) {
    for (int r = 0; r < newRequests.size(); ++r) {
        // create pickup node and drop off nodes
        newRequests[r]->setPenalty(0);

        std::string pickID = Tools::createNodeID(newRequests[r]->getRequestId(), PICKUP);
        std::string dropID = Tools::createNodeID(newRequests[r]->getRequestId(), DROPOFF);

        addNewNode(std::make_shared<Node>(pickID, newRequests[r], PICKUP, dropID));
        addNewNode(std::make_shared<Node>(dropID, newRequests[r], DROPOFF, pickID));

    }
}


double calcTravelTime(PNode startNode, PNode endNode) {
    double dist = Tools::calcDistance(startNode->locLatitude_, startNode->locLongitude_,
                                      endNode->locLatitude_, endNode->locLongitude_);
    return dist * TimePerMile;
}

// function to calculate travel time of the fastest route between two node
float queryTravelTime(PNode startNode, PNode endNode) {
    double travelTime = Tools::queryTravelTime(startNode->locLatitude_, startNode->locLongitude_,
                                      endNode->locLatitude_, endNode->locLongitude_);
    return travelTime;
}
