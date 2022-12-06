//
// Created by Ella on 2/22/2022.
//

#include "SubproModeler.h"

SubproModeler::SubproModeler(PVehicle &vehicle) : Vehicle_(&vehicle) {
    subGraph_ = std::make_shared<Graph>();
//    nbNodes_ = 0;
    bestReducedCost_ = INFINITY;
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

    pickNodes_.clear();
    dropNodes_.clear();
    onboards_.clear();*/
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
            float minWait = (*Vehicle_)->departTime_ +
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
//    nodes_.clear();
    nbTotalRequest_ = pInst->nbRequests_;
    subGraph_->addNewNode(std::make_shared<Node>((*Vehicle_)->departNode_));
    subGraph_->addNewNode(std::make_shared<Node>(pInst->instGraph_->nodes_[(*Vehicle_)->sinkID_]));
//    subGraph_->sourceNodes_.push_back(subGraph_->nodes_[(*Vehicle_)->departNode_->nodeID_]);
//    subGraph_->sinkNodes_.push_back(subGraph_->nodes_[(*Vehicle_)->sinkID_]);

//    subGraph_->addNewNode(departNode_);
//    subGraph_->addNewNode(sinkNode_);

    // adding onboard nodes to the graph
    for (auto & nodeID: (*Vehicle_)->onboards_) {
        onboards_.emplace_back(std::make_shared<Node>(pInst->instGraph_->nodes_[nodeID]));
        subGraph_->nodes_[nodeID]  = onboards_.back();
//        subGraph_->addNewNode(onboards_.back());
//        onboardRequests_.insert(pInst->instGraph_->nodes_[nodeID]->related_Request_);
//        subGraph_->nodes_[nodeID]->pairNode_ = pInst->instGraph_->nodes_[nodeID]->pairNode_;
    }

    // adding available nodes based on the penalty
    for (auto & requestObj : pInst->requests_) {
        if (requestObj->requestStatus_ == NO_ACTION) {
            float minWait = (*Vehicle_)->departTime_ +
                            durationMatrix_[(*Vehicle_)->departNode_->locationID_]
                            [pInst->instGraph_->nodes_[myTools::createNodeID(requestObj->getRequestId(), PICKUP)]->locationID_]
                            - requestObj->earlyPick_;
            if (minWait <= requestObj->penalty_) {
                /*std::string pickID = myTools::createNodeID(requestObj->getRequestId(), PICKUP);
                std::string dropID = myTools::createNodeID(requestObj->getRequestId(), DROPOFF);
                subGraph_->addNewNode(std::make_shared<Node>(pInst->instGraph_->nodes_[pickID]));
                subGraph_->addNewNode(std::make_shared<Node>(pInst->instGraph_->nodes_[dropID]));
                subGraph_->nodes_[pickID]->pairNode_ = & subGraph_->nodes_[dropID];
                subGraph_->nodes_[dropID]->pairNode_ = & subGraph_->nodes_[pickID];*/
                // adding available requests
                subRequests_.push_back(requestObj);
            }
        }
    }
    sort(subRequests_.begin(),subRequests_.end(),[](const PRequest &lhs, const PRequest &rhs){
        return lhs->dual_ > rhs->dual_;});
    for (auto & requestObj : subRequests_){
        std::string pickID = myTools::createNodeID(requestObj->getRequestId(), PICKUP);
        std::string dropID = myTools::createNodeID(requestObj->getRequestId(), DROPOFF);
        subGraph_->addNewNode(std::make_shared<Node>(pInst->instGraph_->nodes_[pickID]));
        subGraph_->addNewNode(std::make_shared<Node>(pInst->instGraph_->nodes_[dropID]));
        subGraph_->pickNodes_.back()->pairNode_ = &subGraph_->nodes_[dropID];
        subGraph_->dropNodes_.back()->pairNode_ = &subGraph_->nodes_[pickID];
    }
    subGraph_->nbNodes_ = subGraph_->pickNodes_.size() + subGraph_->dropNodes_.size() + onboards_.size() + 2;
}

void SubproModeler::initSubGraph(PInstance &pInst) {
// adding source and sink
    nbTotalRequest_ = pInst->nbRequests_;
    subGraph_->sourceNodes_.emplace_back(std::make_shared<Node>((*Vehicle_)->departNode_));
    subGraph_->sinkNodes_.emplace_back(std::make_shared<Node>(pInst->instGraph_->nodes_[(*Vehicle_)->sinkID_]));

    // adding onboard nodes to the graph
    for (auto & nodeID: (*Vehicle_)->onboards_) {
        onboards_.emplace_back(std::make_shared<Node>(pInst->instGraph_->nodes_[nodeID]));
        onboards_.back()->travelTimeFromSource_ = durationMatrix_[(subGraph_->sourceNodes_[0])->locationID_][onboards_.back()->locationID_];
    }

    // adding available nodes based on the penalty
    for (auto & requestObj : pInst->requests_) {
        if (requestObj->requestStatus_ == NO_ACTION) {
            float minWait = (*Vehicle_)->departTime_ +
                            durationMatrix_[(*Vehicle_)->departNode_->locationID_]
                            [pInst->instGraph_->nodes_[myTools::createNodeID(requestObj->getRequestId(), PICKUP)]->locationID_]
                            - requestObj->earlyPick_;
            if (minWait <= requestObj->penalty_) {
                subRequests_.push_back(requestObj);
            }
        }
    }
    sort(subRequests_.begin(),subRequests_.end(),[](const PRequest &lhs, const PRequest &rhs){
        return lhs->dual_ > rhs->dual_;});
    for (auto & requestObj : subRequests_){
        std::string pickID = myTools::createNodeID(requestObj->getRequestId(), PICKUP);
        std::string dropID = myTools::createNodeID(requestObj->getRequestId(), DROPOFF);
        subGraph_->pickNodes_.emplace_back(std::make_shared<Node>(pInst->instGraph_->nodes_[pickID]));
        subGraph_->dropNodes_.emplace_back(std::make_shared<Node>(pInst->instGraph_->nodes_[dropID]));
        subGraph_->pickNodes_.back()->travelTimeFromSource_ = durationMatrix_[subGraph_->sourceNodes_[0]->locationID_][subGraph_->pickNodes_.back()->locationID_];
        subGraph_->dropNodes_.back()->travelTimeFromSource_ = durationMatrix_[subGraph_->sourceNodes_[0]->locationID_][subGraph_->dropNodes_.back()->locationID_];
    }
    for (int i=0; i < subGraph_->pickNodes_.size(); i++){
        subGraph_->pickNodes_[i]->pairNode_ = &subGraph_->dropNodes_[i];
        subGraph_->dropNodes_[i]->pairNode_ = &subGraph_->pickNodes_[i];
    }
    subGraph_->nbNodes_ = subGraph_->pickNodes_.size() + subGraph_->dropNodes_.size() + onboards_.size() + 2;
}


