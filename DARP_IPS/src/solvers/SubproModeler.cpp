//
// Created by Ella on 2/22/2022.
//

#include "SubproModeler.h"

SubproModeler::SubproModeler(PVehicle &vehicle) : Vehicle_(&(*vehicle)) {
    subGraph_ = std::make_shared<Graph>();
    nbNegativeColumns_ = 0;
    nbTotalRequest_ = 0;
}

SubproModeler::~SubproModeler() {
}

// initialization of the subgraph
void SubproModeler::initSubGraph(PInstance &pInst) {
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
            int unReachable = 0;
            float reachTime = (Vehicle_)->departTime_ + durationMatrix_[(Vehicle_)->departNode_->locationID_]
                            [pInst->instGraph_->pickNodes_[i]->locationID_];
            if (reachTime <= pInst->requests_[i]->latestPickup_) {
                // check if it is reachable from at least one onboard
                if (!subGraph_->onboards_.empty()){
                    float remainedTravelTime_ = subGraph_->onboards_[0]->related_Request_->maxTravelTime_ - (Vehicle_)->departTime_ +
                            subGraph_->onboards_[0]->pairNode_->departTime_;
                    float timeToDrop = reachTime + pInst->requests_[i]->serviceTime_ +
                                       durationMatrix_[pInst->instGraph_->pickNodes_[i]->locationID_][subGraph_->onboards_[0]->locationID_];
                    if (timeToDrop - (Vehicle_)->departTime_ > remainedTravelTime_) {
                        float reachTimeAfterDrop =
                                durationMatrix_[(Vehicle_)->departNode_->locationID_][subGraph_->onboards_[0]->locationID_]
                                + (Vehicle_)->departTime_ + subGraph_->onboards_[0]->serviceTime_ +
                                durationMatrix_[subGraph_->onboards_[0]->locationID_][pInst->instGraph_->pickNodes_[i]->locationID_];
                        if (reachTimeAfterDrop > pInst->requests_[i]->latestPickup_) {
                            unReachable++;
                        }
                    }
                }
  //              if (reachTime <= pInst->requests_[i]->latestPickup_) {
                if (unReachable == 0) {
                    subRequests_.push_back(pInst->requests_[i]);
                    subGraph_->addNewNode(std::make_shared<Node>(pInst->instGraph_->pickNodes_[i]));
                    subGraph_->addNewNode(std::make_shared<Node>(pInst->instGraph_->dropNodes_[i]));
                    subGraph_->pickNodes_.back()->travelTimeFromSource_ = durationMatrix_[subGraph_->sourceNodes_[0]->locationID_][subGraph_->pickNodes_.back()->locationID_];
                    subGraph_->dropNodes_.back()->travelTimeFromSource_ = durationMatrix_[subGraph_->sourceNodes_[0]->locationID_][subGraph_->dropNodes_.back()->locationID_];
                    Vehicle_->graphRequests_.set(pInst->requests_[i]->taskIndex_, true);
                }
            }
        }
    }

    for (int i=0; i < subGraph_->pickNodes_.size(); i++){
        subGraph_->pickNodes_[i]->pairNode_ = &(*subGraph_->dropNodes_[i]);
        subGraph_->dropNodes_[i]->pairNode_ = &(*subGraph_->pickNodes_[i]);
    }
    subGraph_->nbNodes_ = subGraph_->pickNodes_.size() + subGraph_->dropNodes_.size() + subGraph_->onboards_.size() + 2;
}



