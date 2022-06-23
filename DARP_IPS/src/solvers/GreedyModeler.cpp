//
// Created by Ella on 5/25/2022.
//

#include "GreedyModeler.h"

GreedyModeler::GreedyModeler() {}

void GreedyModeler::initialization(PInstance &PInst) {
    for (auto & vehicleObj : PInst->vehicles_) {
        solutionList_.emplace_back(std::make_shared<LinkedGreedyLabels>(vehicleObj, PInst));
    }
}

void GreedyModeler::solve(PInstance &PInst) {
    std::vector<float> possibleDelay;
    std::vector<PGreedyLabel> preLabelList;
    for (auto & requestObj : PInst->requests_) {
        if (requestObj->requestStatus_ == NO_ACTION) {
            possibleDelay.clear();
            preLabelList.clear();

            std::string pickID = Tools::createNodeID(requestObj->getRequestId(), PICKUP);
            std::string dropID = Tools::createNodeID(requestObj->getRequestId(), DROPOFF);

            for (auto & GreedyObj : solutionList_){
                // if a vehicle is idle before arrival of a request, its departure time should be after the request time
                // after arrival of each request, the vehicle positions should be updated
                if (GreedyObj->departTime_ < requestObj->earlyPick_) {
                    while ((GreedyObj->head_->child_ != nullptr) && (GreedyObj->departTime_ < requestObj->earlyPick_)) {
                        GreedyObj->head_ = GreedyObj->head_->child_;
                        GreedyObj->departTime_ = GreedyObj->head_->reachTime_ + GreedyObj->head_->currentNode_->deltaTime_;
                    }
                    if (GreedyObj->departTime_ < requestObj->earlyPick_) {
                        GreedyObj->idleTime_ += requestObj->earlyPick_ - GreedyObj->departTime_;
                        GreedyObj->departTime_ = requestObj->earlyPick_;
                    }
                }
                preLabelList.push_back(GreedyObj->findInsertPosition(PInst->instGraph_->nodes_[pickID],
                                                                     PInst->instGraph_->nodes_[dropID],
                                                                     requestObj->maxTravelTime_));
                possibleDelay.push_back(preLabelList.back()->reachTime_ + preLabelList.back()->currentNode_->deltaTime_ +
                                        durationMatrix_[preLabelList.back()->currentNode_->locationID_][PInst->instGraph_->nodes_[pickID]->locationID_]
                                        - requestObj->earlyPick_);
            }
            int vehicle_ID = std::min_element(possibleDelay.begin(), possibleDelay.end()) - possibleDelay.begin();
            solutionList_[vehicle_ID]->insertRequest(preLabelList[vehicle_ID], PInst->instGraph_->nodes_[pickID],
                                                     PInst->instGraph_->nodes_[dropID], requestObj->maxTravelTime_);
        }
    }
}

void GreedyModeler::solveInsertion(PInstance &PInst) {
    std::vector<float> possibleDelay;
    std::vector<PInsertPosition> positionList;
    for (auto & requestObj : PInst->requests_) {
        if (requestObj->requestStatus_ == NO_ACTION) {
            possibleDelay.clear();
            positionList.clear();

            std::string pickID = Tools::createNodeID(requestObj->getRequestId(), PICKUP);
            std::string dropID = Tools::createNodeID(requestObj->getRequestId(), DROPOFF);

            for (auto & GreedyObj : solutionList_){
                // if a vehicle is idle before arrival of a request, its departure time should be after the request time
                // after arrival of each request, the vehicle positions should be updated
                if (GreedyObj->departTime_ < requestObj->earlyPick_) {
                    while ((GreedyObj->head_->child_ != nullptr) && (GreedyObj->departTime_ < requestObj->earlyPick_)) {
                        GreedyObj->head_ = GreedyObj->head_->child_;
                        GreedyObj->departTime_ = GreedyObj->head_->reachTime_ + GreedyObj->head_->currentNode_->deltaTime_;
                    }
                    if (GreedyObj->departTime_ < requestObj->earlyPick_) {
                        GreedyObj->idleTime_ += requestObj->earlyPick_ - GreedyObj->departTime_;
                        GreedyObj->departTime_ = requestObj->earlyPick_;
                    }
                }
                positionList.push_back(GreedyObj->findInsertPlace(PInst->instGraph_->nodes_[pickID],
                                                                     PInst->instGraph_->nodes_[dropID],
                                                                     requestObj->maxTravelTime_));
                possibleDelay.push_back(positionList.back()->deltaDelay_);
            }
            int vehicle_ID = std::min_element(possibleDelay.begin(), possibleDelay.end()) - possibleDelay.begin();
            solutionList_[vehicle_ID]->insertRequest(positionList[vehicle_ID], PInst->instGraph_->nodes_[pickID],
                                                     PInst->instGraph_->nodes_[dropID], requestObj->maxTravelTime_);
        }
    }
}
void GreedyModeler::solutionToRoute(PInstance &PInst) {
    for (auto & greedySol : solutionList_) {
        PInst->vehicles_[(*greedySol->Vehicle_)->vehicleID_]->idleTime_ = greedySol->idleTime_;
        PInst->vehicles_[(*greedySol->Vehicle_)->vehicleID_]->currentRoute_ = greedySol->greedyLabelToRoute();
    }
}

// this function just assign requests to vehicles based on the minimum delay possible and do not consider ride-sharing
// any pick up is followed by the drop-off
void GreedySolver_noShare(PInstance &PInst) {
    std::vector<float> possibleDelay;
    for (auto & requestObj : PInst->requests_) {
        if (requestObj->requestStatus_ == NO_ACTION) {
            possibleDelay.clear();
            std::string pickID = Tools::createNodeID(requestObj->getRequestId(), PICKUP);
            std::string dropID = Tools::createNodeID(requestObj->getRequestId(), DROPOFF);

            for (auto &vehicleObj: PInst->vehicles_) {
                float requsetPickTime = vehicleObj->currentRoute_->plannedReachTime_.back() +
                                        vehicleObj->currentRoute_->routeNodes_.back()->deltaTime_ +
                                        durationMatrix_[vehicleObj->currentRoute_->routeNodes_.back()->locationID_][PInst->instGraph_->nodes_[pickID]->locationID_];
                possibleDelay.push_back(requsetPickTime);
            }
            int vehicle_ID = std::min_element(possibleDelay.begin(), possibleDelay.end()) - possibleDelay.begin();
            PInst->vehicles_[vehicle_ID]->currentRoute_->addNode(PInst->instGraph_->nodes_[pickID]);
            PInst->vehicles_[vehicle_ID]->currentRoute_->addNode(PInst->instGraph_->nodes_[dropID]);
        }
    }
}




