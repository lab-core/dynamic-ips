//
// Created by Ella on 5/5/2022.
//

#include "GreedySolver.h"

void GreedySolver(PInstance &PInst) {
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
