//
// Created by Ella on 2/22/2022.
//

#include "SubproModeler.h"

SubproModeler::SubproModeler(const PVehicle &vehicle) : Vehicle_(&(*vehicle)) {
    subGraph_ = std::make_shared<Graph>();
    nbNegativeColumns_ = 0;
    nbTotalRequest_ = 0;
    reOptimize_ = false;
    nbOnePickGenerated_ = 0;
    nbTwoPickGenerated_ = 0;
    nbOutCover_ = 0;
    possibleInsert_ = 0;
    nbPriorCover_ = 0;
    subproTime_ = new myTools::Timer(); subproTime_->init();
}

SubproModeler::~SubproModeler(){
    delete subproTime_;
}

// initialization of the subgraph
void SubproModeler::initSubGraph(const PInstance &pInst) {
    // adding source and sink
    possibleInsert_ = 0;
    labelSize_ = pInst->requests_.size() + 2 * Vehicle_->capacity_;
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
        bool insertRequest = false;
        if (pInst->parameters_->pruneNodes_) {
            addRequest = isPickupProfitable(Vehicle_->departNode_, pInst->instGraph_->pickNodes_[i],Vehicle_->departTime_);
        }

        if (pInst->parameters_->LabelingStrategy_ == RE_PULLING && Vehicle_->currentRoute_->routeRequests_.empty()) {
            float reachTime = Vehicle_->departTime_ + durationMatrix_[Vehicle_->departNode_->locationID_][pInst->instGraph_->pickNodes_[i]->locationID_];
            if (reachTime - pInst->instGraph_->pickNodes_[i]->initialReadyTime_ - pInst->requests_[i]->dual_ > 1 ) {
                addRequest = false;
            }
        }

        if (addRequest) {
            if (pInst->requests_[i]->coveredVehicles_.test(Vehicle_->vehicleID_))
                nbPriorCover_ ++;

            const bool isRequestUnassigned = (pInst->requests_[i]->solVehicleID_ == LARGE_CONSTANT);
            if (isRequestUnassigned || pInst->requests_[i]->committedPickTime_ < LARGE_CONSTANT) {
                possibleInsert_ ++;
                insertRequest = true;
                pInst->requests_[i]->insertedVehicles_.set(Vehicle_->vehicleID_,1);
            }
            else {
                if (!Vehicle_->stateChanged_ || !Vehicle_->removeDrop_) {
                    if (pInst->requests_[i]->coveredVehicles_.test(Vehicle_->vehicleID_)) {
                        possibleInsert_ ++;
                        insertRequest = true;
                        pInst->requests_[i]->insertedVehicles_.set(Vehicle_->vehicleID_,1);
                    }
                }
                else if (checkInsertionPossibility(pInst->instGraph_->pickNodes_[i])) {
                    possibleInsert_ ++;
                    insertRequest = true;
                    pInst->requests_[i]->insertedVehicles_.set(Vehicle_->vehicleID_,1);
                }
            }
        }
        if (reOptimize_)
            addRequest = addRequest && insertRequest;

        if (addRequest) {
            subRequests_.push_back(pInst->requests_[i]);
            auto pickNode = std::make_shared<Node>(pInst->instGraph_->pickNodes_[i]);
            auto dropNode = std::make_shared<Node>(pInst->instGraph_->dropNodes_[i]);

            subGraph_->addNewNode(pickNode);
            subGraph_->addNewNode(dropNode);

            pickNode->travelTimeFromSource_ = durationMatrix_[subGraph_->sourceNodes_[0]->locationID_][pickNode->locationID_];
            dropNode->travelTimeFromSource_ = durationMatrix_[subGraph_->sourceNodes_[0]->locationID_][dropNode->locationID_];
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

bool SubproModeler::checkInsertionPossibility(PNode &pick) {
    if (subGraph_->onboards_.empty())
        return true;
    for (size_t j = 0; j < Vehicle_->emptyRoute_->routeNodes_.size(); j++) {
        PNode before = Vehicle_->emptyRoute_->routeNodes_[j];
        float beforeDepartTime = Vehicle_->emptyRoute_->plannedDepartTime_[j];
        int beforeLoad = Vehicle_->emptyRoute_->plannedPassengers_[j];
        if (beforeLoad + pick->related_Request_->nbPassengers_ <= Vehicle_->capacity_ &&
            isPickupProfitable(before, pick, beforeDepartTime)) {
            if (j < Vehicle_->emptyRoute_->routeNodes_.size()-1) {
                float detourDelay = computeDetourDelay(before, pick,Vehicle_->emptyRoute_->routeNodes_[j+1]);
                bool dropFeasible = true;
                for (size_t k = j+1; k < Vehicle_->emptyRoute_->routeNodes_.size(); k++) {
                    if (Vehicle_->emptyRoute_->plannedReachTime_[k] + detourDelay >
                        Vehicle_->emptyRoute_->routeNodes_[k]->related_Request_->latestDrop_) {
                        dropFeasible = false;
                        break;
                    }
                }
                if (dropFeasible) return true;
            }
            else
                return true;
        }
    }
    return false;
}
