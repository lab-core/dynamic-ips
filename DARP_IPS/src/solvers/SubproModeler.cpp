//
// Created by Ella on 2/22/2022.
//

#include "SubproModeler.h"

SubproModeler::SubproModeler(const PVehicle &vehicle) : Vehicle_(&(*vehicle)) {
    subGraph_ = std::make_shared<Graph>();
    nbNegativeColumns_ = 0;
    nbTotalRequest_ = 0;
    reOptimize_ = false;
    possibleFirstInsert_ = 0;
    possibleSecondInsert_ = 0;
}

SubproModeler::~SubproModeler() = default;

// initialization of the subgraph
void SubproModeler::initSubGraph(const PInstance &pInst) {
    // adding source and sink
    labelSize_ = pInst->requests_.size() + 2 * Vehicle_->capacity_;
    Vehicle_->graphRequests_.resize(pInst->nbRequests_);
    Vehicle_->graphRequests_.reset();
    nbTotalRequest_ = pInst->nbRequests_;
    subGraph_->addNewNode(std::make_shared<Node>((Vehicle_)->departNode_));
    subGraph_->addNewNode(std::make_shared<Node>(pInst->instGraph_->sinkNodes_[(Vehicle_)->vehicleID_]));

    // adding onboard nodes to the graph
    if (Vehicle_->currentRoute_->routeSize_ > 1) {
        for (int i = 1; i < (Vehicle_)->currentRoute_->routeSize_; ++i) {
            if ((Vehicle_)->currentRoute_->routeNodes_[i]->nodeStatus_ == PLANNED){
                subGraph_->onboards_.emplace_back(std::make_shared<Node>((Vehicle_)->currentRoute_->routeNodes_[i]));
                subGraph_->nodes_[subGraph_->onboards_.back()->nodeID_]  = subGraph_->onboards_.back();
                subGraph_->onboards_.back()->travelTimeFromSource_ = durationMatrix_[(subGraph_->sourceNodes_[0])->locationID_][subGraph_->onboards_.back()->locationID_];
                subGraph_->onboards_.back()->pairNode_ = (Vehicle_)->currentRoute_->routeNodes_[i]->pairNode_;
            }
        }
    }

    // adding available nodes based on the penalty
    for (int i = 0; i < pInst->requests_.size(); i++) {
        if (pInst->requests_[i]->requestStatus_ != NO_ACTION) continue;

        bool addRequest = true;
        if (pInst->parameters_->pruneNodes_) {
            float reachTime = Vehicle_->departTime_ +
                            durationMatrix_[Vehicle_->departNode_->locationID_]
                            [pInst->instGraph_->pickNodes_[i]->locationID_];

            addRequest = (reachTime <= pInst->requests_[i]->latestPickup_);
            if (addRequest) {
                if (subGraph_->onboards_.empty())
                    possibleFirstInsert_ ++;
                else {
                    // check first insertion

                    float addedFirstTravelTime = computeDetourDelay(Vehicle_->departNode_,
                        pInst->instGraph_->pickNodes_[i], subGraph_->onboards_[0]);

                    bool possibleFirstInsert = true;
                    for (auto &nodeObj: subGraph_->onboards_) {
                        if (nodeObj->plannedReachtime_ + addedFirstTravelTime > nodeObj->related_Request_->latestDrop_) {
                            possibleFirstInsert = false;
                            break;
                        }
                    }
                    if (possibleFirstInsert)
                        possibleFirstInsert_ ++;
                    else {
                        // check second insertion
                        float reachTime2 = subGraph_->onboards_[0]->plannedReachtime_ + subGraph_->onboards_[0]->serviceTime_ +
                                    durationMatrix_[subGraph_->onboards_[0]->locationID_][pInst->instGraph_->pickNodes_[i]->locationID_];
                        if (reachTime2 <= pInst->requests_[i]->latestPickup_) {
                            if (subGraph_->onboards_.size() == 1)
                                possibleSecondInsert_ ++;
                            else {
                                float addedSecondTravelTime = computeDetourDelay(subGraph_->onboards_[0],
                                    pInst->instGraph_->pickNodes_[i], subGraph_->onboards_[1]);

                                bool possibleSecondInsert = true;
                                for (int j = 1; j < subGraph_->onboards_.size(); j++) {
                                    if (subGraph_->onboards_[j]->plannedReachtime_ + addedSecondTravelTime > subGraph_->onboards_[j]->related_Request_->latestDrop_) {
                                        possibleSecondInsert = false;
                                        break;
                                    }
                                }
                                if (possibleSecondInsert)
                                    possibleSecondInsert_ ++;
                            }
                        }
                    }
                }
            }
            if (addRequest && reOptimize_) {
                const bool isRequestUnassigned = (pInst->requests_[i]->solVehicleID_ == LARGE_CONSTANT);
                addRequest = isRequestUnassigned || pInst->requests_[i]->coveredVehicles_.test(Vehicle_->vehicleID_);

                /*switch (pInst->parameters_->labelingReOptimizeStrategy_) {
                    case BY_GRAPH: {
                        // For graph-based strategy, check if request is covered by vehicle's graph
                        const bool isRequestCovered = Vehicle_->coveredRequests.test(pInst->requests_[i]->taskIndex_);
                        addRequest = isRequestCovered || isRequestUnassigned;
                        break;
                    }
                    case BY_ROUTE: {
                        // For route-based strategy, check if request is covered by current route
                        const bool isRequestCovered = Vehicle_->currentRoute_->column_.test(pInst->requests_[i]->taskIndex_);
                        addRequest = isRequestCovered || isRequestUnassigned;
                        break;
                    }
                    default:
                        break;
                }*/
            }
        }

        if (addRequest) {
            subRequests_.push_back(pInst->requests_[i]);
            auto pickNode = std::make_shared<Node>(pInst->instGraph_->pickNodes_[i]);
            auto dropNode = std::make_shared<Node>(pInst->instGraph_->dropNodes_[i]);

            subGraph_->addNewNode(pickNode);
            subGraph_->addNewNode(dropNode);

            pickNode->travelTimeFromSource_ = durationMatrix_[subGraph_->sourceNodes_[0]->locationID_][pickNode->locationID_];
            dropNode->travelTimeFromSource_ = durationMatrix_[subGraph_->sourceNodes_[0]->locationID_][dropNode->locationID_];

            Vehicle_->graphRequests_.set(pInst->requests_[i]->taskIndex_, true);
        }
    }

    for (int i=0; i < subGraph_->pickNodes_.size(); i++){
        subGraph_->pickNodes_[i]->pairNode_ = &(*subGraph_->dropNodes_[i]);
        subGraph_->dropNodes_[i]->pairNode_ = &(*subGraph_->pickNodes_[i]);
    }
    /*for (const auto& [id, node] : subGraph_->nodes_) {
        node->prunedArcs_.resize(pInst->requests_.size() + pInst->nbOnboards_);
    }*/
    subGraph_->nbNodes_ = static_cast<int>(subGraph_->pickNodes_.size() + subGraph_->dropNodes_.size() + subGraph_->onboards_.size() + 2);
}

void SubproModeler::setNodeIndices() const {
    for (int i = 0; i < subGraph_->pickNodes_.size(); ++i) {
        PNode nodeObj = subGraph_->pickNodes_[i];
        nodeObj->nodeIndex_ = getIndex(nodeObj, i, static_cast<int>(subGraph_->pickNodes_.size()));
    }
    for (int i = 0; i < subGraph_->dropNodes_.size(); ++i) {
        PNode nodeObj = subGraph_->dropNodes_[i];
        nodeObj->nodeIndex_ = getIndex(nodeObj, i, static_cast<int>(subGraph_->dropNodes_.size()));
    }

    Vehicle_->departNode_->nodeIndex_ = getIndex(Vehicle_->departNode_, 0, static_cast<int>(Vehicle_->onboards_.size()));
    Vehicle_->sinkNode_->nodeIndex_ = getIndex(Vehicle_->sinkNode_, 0, static_cast<int>(Vehicle_->onboards_.size()));
    for (int i = 0; i < Vehicle_->onboards_.size(); ++i) {
        PNode nodeObj = subGraph_->nodes_[ Vehicle_->onboards_[i]];
        nodeObj->nodeIndex_ = getIndex(nodeObj, i, static_cast<int>(subGraph_->dropNodes_.size()));
    }
}



