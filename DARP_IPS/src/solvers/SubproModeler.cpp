//
// Created by Ella on 2/22/2022.
//

#include "SubproModeler.h"

SubproModeler::SubproModeler(PVehicle &vehicle) : Vehicle_(&vehicle) {
    subGraph_ = std::make_shared<Graph>();
    bestReducedCost_ = 9999;
    nbNegativeColumns_ = 0;
}

SubproModeler::~SubproModeler() {
    for (auto & node : subGraph_->nodes_){
 //       node.second->predecessor_.clear();
        node.second->successors_.clear();
        node.second->generatedLabel_.clear();
        node.second->activeLabels_.clear();
    }
    subGraph_->nodes_.clear();
    subGraph_.reset();
}

// calculation of penalties and initialization of the subgraph
void SubproModeler::initSubGraph(PInstance &pInst) {
    nbTotalRequest_ = pInst->nbRequests_;
// create graph with source and sink
    subGraph_ = std::make_shared<Graph>(pInst->instGraph_->nodes_[(*Vehicle_)->departID_],
                                        pInst->instGraph_->nodes_[(*Vehicle_)->sinkID_]);

    // adding onboard nodes to the graph
    for (auto & nodeID: (*Vehicle_)->onboards_) {
        subGraph_->addNewNode(pInst->instGraph_->nodes_[nodeID]);
//        onboardRequests_.insert(pInst->instGraph_->nodes_[nodeID]->related_Request_);
    }

    // adding available nodes based on the penalty
    for (auto & requestObj : pInst->requests_) {
//        if ((requestObj->requestStatus_ == NO_ACTION)&&(requestObj->selectStatus_ == NOT_SELECTED)) {
        if (requestObj->requestStatus_ == NO_ACTION) {
            float minWait = (*Vehicle_)->departTime_ +
                            durationMatrix_[pInst->instGraph_->nodes_[(*Vehicle_)->departID_]->locationID_]
                            [pInst->instGraph_->nodes_[myTools::createNodeID(requestObj->getRequestId(), PICKUP)]->locationID_]
                            - requestObj->earlyPick_;
            if (minWait <= requestObj->penalty_) {
                subGraph_->addNewNode(pInst->instGraph_->nodes_[myTools::createNodeID(requestObj->getRequestId(), PICKUP)]);
                subGraph_->addNewNode(pInst->instGraph_->nodes_[myTools::createNodeID(requestObj->getRequestId(), DROPOFF)]);

                // adding available requests
                subRequests_.push_back(requestObj);
            }
        }
    }
}

void SubproModeler::initSubGraph2(PInstance &pInst) {
    // adding source and sink
    nbTotalRequest_ = pInst->nbRequests_;
    subGraph_->addNewNode(std::make_shared<Node>(pInst->instGraph_->nodes_[(*Vehicle_)->departID_]));
    subGraph_->addNewNode(std::make_shared<Node>(pInst->instGraph_->nodes_[(*Vehicle_)->sinkID_]));

    // adding onboard nodes to the graph
    for (auto & nodeID: (*Vehicle_)->onboards_) {
        subGraph_->addNewNode(std::make_shared<Node>(pInst->instGraph_->nodes_[nodeID]));
//        onboardRequests_.insert(pInst->instGraph_->nodes_[nodeID]->related_Request_);
        subGraph_->nodes_[nodeID]->pairNode_ = pInst->instGraph_->nodes_[nodeID]->pairNode_;
    }

    // adding available nodes based on the penalty
    for (auto & requestObj : pInst->requests_) {
        if (requestObj->requestStatus_ == NO_ACTION) {
            float minWait = (*Vehicle_)->departTime_ +
                            durationMatrix_[pInst->instGraph_->nodes_[(*Vehicle_)->departID_]->locationID_]
                            [pInst->instGraph_->nodes_[myTools::createNodeID(requestObj->getRequestId(), PICKUP)]->locationID_]
                            - requestObj->earlyPick_;
            if (minWait <= requestObj->penalty_) {
                std::string pickID = myTools::createNodeID(requestObj->getRequestId(), PICKUP);
                std::string dropID = myTools::createNodeID(requestObj->getRequestId(), DROPOFF);
                subGraph_->addNewNode(std::make_shared<Node>(pInst->instGraph_->nodes_[pickID]));
                subGraph_->addNewNode(std::make_shared<Node>(pInst->instGraph_->nodes_[dropID]));
                subGraph_->nodes_[pickID]->pairNode_ = & subGraph_->nodes_[dropID];
                subGraph_->nodes_[dropID]->pairNode_ = & subGraph_->nodes_[pickID];
                // adding available requests
                subRequests_.push_back(requestObj);
            }
        }
    }
}


