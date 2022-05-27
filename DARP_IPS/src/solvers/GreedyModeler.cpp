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

void GreedyModeler::solutionToRoute(PInstance &PInst) {
    for (auto & greedySol : solutionList_) {
        PInst->vehicles_[(*greedySol->Vehicle_)->vehicleID_]->currentRoute_ = greedySol->greedyLabelToRoute();
    }
}





