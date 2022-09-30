//
// Created by Ella on 2/22/2022.
//

#include "SubproModeler.h"

SubproModeler::SubproModeler(PVehicle &vehicle) : Vehicle_(&vehicle) {
//    subGraph_ = std::make_shared<Graph>();
    bestReducedCost_ = 9999;
    nbNegativeColumns_ = 0;
}

SubproModeler::~SubproModeler() {
    subGraph_.reset();
}

// calculation of penalties and initialization of the subgraph
void SubproModeler::initSubGraph(PInstance &pInst) {
// create graph with source and sink
    subGraph_ = std::make_shared<Graph>(pInst->instGraph_->nodes_[(*Vehicle_)->departID_],
                                        pInst->instGraph_->nodes_[(*Vehicle_)->sinkID_]);

    // adding onboard nodes to the graph
    for (auto & nodeID: (*Vehicle_)->onboards_) {
        subGraph_->addNewNode(pInst->instGraph_->nodes_[nodeID]);
        onboardRequests_.insert(pInst->instGraph_->nodes_[nodeID]->related_Request_);
    }

    // adding available nodes based on the penalty
    for (auto & requestObj : pInst->requests_) {
//        if ((requestObj->requestStatus_ == NO_ACTION)&&(requestObj->selectStatus_ == NOT_SELECTED)) {
        if (requestObj->requestStatus_ == NO_ACTION) {
            float minWait = (*Vehicle_)->departTime_ +
                            durationMatrix_[pInst->instGraph_->nodes_[(*Vehicle_)->departID_]->locationID_]
                            [pInst->instGraph_->nodes_[Tools::createNodeID(requestObj->getRequestId(), PICKUP)]->locationID_]
                            - requestObj->earlyPick_;
            if (minWait <= requestObj->penalty_) {
                subGraph_->addNewNode(pInst->instGraph_->nodes_[Tools::createNodeID(requestObj->getRequestId(), PICKUP)]);
                subGraph_->addNewNode(pInst->instGraph_->nodes_[Tools::createNodeID(requestObj->getRequestId(), DROPOFF)]);

                // adding available requests
                subRequests_.push_back(requestObj);
            }
        }
    }
}


