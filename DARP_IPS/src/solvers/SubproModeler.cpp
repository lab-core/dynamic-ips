//
// Created by Ella on 2/22/2022.
//

#include "SubproModeler.h"

SubproModeler::SubproModeler(PVehicle &vehicle) : Vehicle_(&(*vehicle)) {
    subGraph_ = std::make_shared<Graph>();
//    nbNodes_ = 0;
//    bestReducedCost_ = INFINITY;
    score_ = vehicle->score_;
    nbNegativeColumns_ = 0;
    nbTotalRequest_ = 0;
}

SubproModeler::~SubproModeler() {
    /*for (auto & node : nodes_){
        node.second->successors_.clear();
        node.second->activeLabels_.clear();
        node.second->generatedLabel_.clear();
    }
*/
//    subGraph_->nodes_.clear();
//    subGraph_.reset();
}

// calculation of penalties and initialization of the subgraph
/*void SubproModeler::initSubGraph(PInstance &pInst) {
    nbTotalRequest_ = pInst->nbRequests_;
// create graph with source and sink
    subGraph_ = std::make_shared<Graph>((*Vehicle_)->departNode_,pInst->instGraph_->nodes_[(*Vehicle_)->sinkID_]);

    // adding onboard nodes to the graph
    for (auto & nodeID: (*Vehicle_)->onboards_) {
        subGraph_->addNewNode(pInst->instGraph_->nodes_[nodeID]);
//        onboardRequests_.insert(pInst->instGraph_->nodes_[nodeID]->related_Request_);
    }

    // adding available nodes based on the penalty
    for (auto & requestObj : pInst->requests_) {
//        if ((requestObj->requestStatus_ == NO_ACTION)&&(requestObj->selectStatus_ == NOT_SELECTED)) {
        if (requestObj->requestStatus_ == NO_ACTION) {
            float minWait = (*Vehicle_)->departureTime_ +
                            durationMatrix_[(*Vehicle_)->departNode_->locationID_]
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
}*/

void SubproModeler::initSubGraph2(PInstance &pInst) {
    // adding source and sink
    Vehicle_->graphRequests_.reset();
    nbTotalRequest_ = pInst->nbRequests_;
    subGraph_->addNewNode(std::make_shared<Node>((Vehicle_)->departNode_));
    subGraph_->addNewNode(std::make_shared<Node>(pInst->instGraph_->sinkNodes_[(Vehicle_)->vehicleID_]));

    // adding onboard nodes to the graph
    if ((Vehicle_)->currentRoute_->routeSize_ > 1) {
        for (int i = 1; i < (Vehicle_)->currentRoute_->routeSize_; ++i) {
            if (((Vehicle_)->currentRoute_->routeNodes_[i]->nodeStatus_ == PLANNED)||
                ((Vehicle_)->currentRoute_->routeNodes_[i]->nodeStatus_ == COMMITTED && (Vehicle_)->currentRoute_->routeNodes_[i]->initialType_ == DROPOFF)){
                subGraph_->onboards_.emplace_back(std::make_shared<Node>((Vehicle_)->currentRoute_->routeNodes_[i]));
                subGraph_->nodes_[subGraph_->onboards_.back()->nodeID_]  = subGraph_->onboards_.back();
                subGraph_->onboards_.back()->travelTimeFromSource_ = durationMatrix_[(subGraph_->sourceNodes_[0])->locationID_][subGraph_->onboards_.back()->locationID_];
                subGraph_->onboards_.back()->pairNode_ = (Vehicle_)->currentRoute_->routeNodes_[i]->pairNode_;
            }
        }
    }

    // adding available nodes based on the penalty
    for (int i = 0; i < pInst->requests_.size(); i++) {
        if (pInst->requests_[i]->requestStatus_ == NO_ACTION) {
            float minWait = (Vehicle_)->departTime_ +
                            durationMatrix_[(Vehicle_)->departNode_->locationID_]
                            [pInst->instGraph_->pickNodes_[i]->locationID_]- pInst->requests_[i]->earlyPick_;
            if (minWait <= pInst->requests_[i]->penalty_) {
                subRequests_.push_back(pInst->requests_[i]);
                subGraph_->addNewNode(std::make_shared<Node>(pInst->instGraph_->pickNodes_[i]));
                subGraph_->addNewNode(std::make_shared<Node>(pInst->instGraph_->dropNodes_[i]));
                subGraph_->pickNodes_.back()->travelTimeFromSource_ = durationMatrix_[subGraph_->sourceNodes_[0]->locationID_][subGraph_->pickNodes_.back()->locationID_];
                subGraph_->dropNodes_.back()->travelTimeFromSource_ = durationMatrix_[subGraph_->sourceNodes_[0]->locationID_][subGraph_->dropNodes_.back()->locationID_];
                Vehicle_->graphRequests_.set(pInst->requests_[i]->taskIndex_, true);
            }
        }
    }
    /*std::stable_sort(subRequests_.begin(),subRequests_.end(),[](const PRequest &lhs, const PRequest &rhs){
        return lhs->dual_ > rhs->dual_;});
    for (auto & requestObj : subRequests_){
        subGraph_->addNewNode(std::make_shared<Node>(pInst->instGraph_->pickNodes_[requestObj->getRequestId()]));
        subGraph_->addNewNode(std::make_shared<Node>(pInst->instGraph_->dropNodes_[requestObj->getRequestId()]));
        subGraph_->pickNodes_.back()->travelTimeFromSource_ = durationMatrix_[subGraph_->sourceNodes_[0]->locationID_][subGraph_->pickNodes_.back()->locationID_];
        subGraph_->dropNodes_.back()->travelTimeFromSource_ = durationMatrix_[subGraph_->sourceNodes_[0]->locationID_][subGraph_->dropNodes_.back()->locationID_];
    }*/
    for (int i=0; i < subGraph_->pickNodes_.size(); i++){
        subGraph_->pickNodes_[i]->pairNode_ = &(*subGraph_->dropNodes_[i]);
        subGraph_->dropNodes_[i]->pairNode_ = &(*subGraph_->pickNodes_[i]);
    }
    subGraph_->nbNodes_ = subGraph_->pickNodes_.size() + subGraph_->dropNodes_.size() + subGraph_->onboards_.size() + 2;
}



