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
    deltaTime_ = relatedRequest->deltaTime_;
    nodeStatus_ = 0;
}

Node::~Node() = default;


//-----------------------------------------------------------------------------
//  Graph class
//-----------------------------------------------------------------------------

// Constructor and Destructor
Graph::Graph() = default;

void Graph::addNewRequests(std::vector<PRequest> &newRequests) {
    string ID;
    for (int r = 0; r < newRequests.size(); ++r) {
        // create pickup node
        ID = "Pi_" + to_string(newRequests[r]->requestID_);
        nodes_.insert(std::pair<std::string,PNode> (ID, std::make_shared<Node>(ID, newRequests[r], PICKUP)));

        // create drop off node
        ID = "Dr_" + to_string(newRequests[r]->requestID_);
        nodes_.insert(std::pair<std::string,PNode> (ID, std::make_shared<Node>(ID, newRequests[r], DROPOFF)));
    }
}

