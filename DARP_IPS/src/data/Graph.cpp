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

Node::Node(string nodeId, PRequest &relatedRequest, NodeType type) : nodeID_(nodeId),
                                                                related_Request_(&relatedRequest), type_(type) {
    reachTime_ = 0;
    deltaTime_ = relatedRequest->deltaTime_;
    nodeStatus_ = DEFINED;

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
    nodeStatus_ = DEFINED;

    if (type == SOURCE)
        nodeID_ = "SO_0";
    else if (type == SINK)
        nodeID_ = "SI_0";
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
    nbNodes_ = 2;
    nodes_.insert(std::pair<std::string, PNode> (source->nodeID_, source));
    nodes_.insert(std::pair<std::string, PNode> (sink->nodeID_, sink));
}

// function for updating the graph and adding new request
void Graph::addNewRequests(std::vector<PRequest> &newRequests) {
    string ID;
    for (int r = 0; r < newRequests.size(); ++r) {
        // create pickup node
        ID = "PI_" + to_string(newRequests[r]->requestID_);
        nodes_.insert(std::pair<std::string,PNode> (ID, std::make_shared<Node>(ID, newRequests[r], PICKUP)));

        // create drop off node
        ID = "DR_" + to_string(newRequests[r]->requestID_);
        nodes_.insert(std::pair<std::string,PNode> (ID, std::make_shared<Node>(ID, newRequests[r], DROPOFF)));
        nbNodes_ = nbNodes_ + 2;
    }
}



