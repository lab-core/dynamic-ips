//
// Created by Ella on 5/5/2022.
//

#include "GreedySolver.h"


void GreedySolver::initialization(PInstance &PInst) {
    std::vector<float> possibleDelay;
    std::vector<int> RequestVector;
    routeLabels_.clear();
    requestIDToInt_.clear();

    for (auto & vehicleObj : PInst->vehicles_) {
        for (int i = 0; i < vehicleObj->onboards_.size(); ++i) {
            requestIDToInt_[PInst->instGraph_->nodes_[vehicleObj->onboards_[i]]->related_Request_->getRequestId()] = RequestVector.size();
            RequestVector.push_back(0);
        }
    }
    for (auto &requestObj : PInst->requests_) {
        requestIDToInt_[requestObj->getRequestId()] = RequestVector.size();;
        RequestVector.push_back(0);
    }

    // create an initial label for each vehicle
    for (auto & vehicleObj : PInst->vehicles_) {
        routeLabels_.emplace_back(std::make_shared<Label>(&vehicleObj, PInst->instGraph_->nodes_[vehicleObj->departID_]));
//        routeLabels_.back()->completedRequest_ = RequestVector;
        for (auto &nodeID: vehicleObj->onboards_) {
//            routeLabels_.back()->openNodes_.insert(PInst->instGraph_->nodes_[nodeID]);
//            routeLabels_.back()->completedRequest_[requestIDToInt_[PInst->instGraph_->nodes_[nodeID]->related_Request_->getRequestId()]] = 1;
            float remaindTime = PInst->instGraph_->nodes_[nodeID]->related_Request_->maxTravelTime_ -
                    vehicleObj->departTime_ + PInst->instGraph_->nodes_[nodeID]->related_Request_->pickTime_;
            routeLabels_.back()->travelResource_.insert(std::pair<std::string, float>(nodeID, remaindTime));
        }
 //       routeLabels_.back()->openRequests_ = routeLabels_.back()->completedRequest_;
    }
}

void GreedySolver::SolveRideshare(PInstance &PInst) {
    std::vector<float> possibleDelay;
    for (auto & requestObj : PInst->requests_) {
        if (requestObj->requestStatus_ == NO_ACTION) {
            possibleDelay.clear();
            std::string pickID = Tools::createNodeID(requestObj->getRequestId(), PICKUP);

            for (auto &labelObj: routeLabels_) {
                while (true) {
                    std::string dropID ="NULL";
                    float remindSource = 999999;
                    for (auto & openNodeObj: labelObj->openNodes_) {
                        float travelToDrop = labelObj->currentNode_->deltaTime_ +
                                             durationMatrix_[labelObj->currentNode_->locationID_][PInst->instGraph_->nodes_[pickID]->locationID_] +
                                             PInst->instGraph_->nodes_[pickID]->deltaTime_ +
                                             durationMatrix_[PInst->instGraph_->nodes_[pickID]->locationID_][openNodeObj->locationID_];
                        float remindValue = labelObj->travelResource_[openNodeObj->nodeID_] - travelToDrop;
                        if ((remindValue < 0)&&(remindValue < remindSource)) {
                            dropID = openNodeObj->nodeID_;
                            remindSource = labelObj->travelResource_[openNodeObj->nodeID_] - travelToDrop;
                        }
                    }
                    if (dropID != "NULL") {
                        labelObj->extend(PInst->instGraph_->nodes_[dropID]);
                    }
                    else
                        break;
                }

                while (true) {
                    if (labelObj->isExtendFeasible(PInst->instGraph_->nodes_[pickID],
                                                   PInst->nbRequests_ + PInst->nbOnboards_)) {
                        float requsetPickTime = labelObj->passedTime_ + labelObj->currentNode_->deltaTime_ +
                                                durationMatrix_[labelObj->currentNode_->locationID_][PInst->instGraph_->nodes_[pickID]->locationID_];
                        possibleDelay.push_back(requsetPickTime);
                        break;
                    }
                    else {

                    }
                }
            }
        }
    }
}

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


