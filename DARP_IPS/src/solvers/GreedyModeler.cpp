//
// Created by Ella on 5/25/2022.
//

#include "GreedyModeler.h"

GreedyModeler::GreedyModeler() {
    greedyTime_ = new myTools::Timer(); greedyTime_->init();
    greedySolveTime_ = new myTools::Timer(); greedySolveTime_->init();
    greedySolTime_ = new myTools::Timer(); greedySolTime_->init();
    float deltaDelay = INFINITY;

    // Tour length increase by adding pickup
    float pickDeltaT = INFINITY;

    // Tour length increase by adding drop off
    float dropDeltaT = INFINITY;
}

GreedyModeler::~GreedyModeler() {
    delete greedyTime_;
    delete greedySolveTime_;
    delete greedySolTime_;
    for (auto & label : removedLabels_){
        label->child_.reset();
        label->parent_.reset();
        label->pair_.reset();
    }
    removedLabels_.clear();
}

void GreedyModeler::initialization(PInstance &PInst) {
    selectedVehicles_.clear();
    greedyTime_->start();
    for (auto & vehicleObj : PInst->vehicles_) {
        solutionList_.emplace_back(std::make_shared<LinkedGreedyLabels>(vehicleObj, PInst));
    }
    greedyTime_->stop();
    selectedVehicles_.resize(PInst->nbVehicles_,0);
}

void GreedyModeler::initializationFast(PInstance &PInst) {
    selectedVehicles_.clear();
    greedyTime_->start();
    for (auto & vehicleObj : PInst->vehicles_) {
        solutionList_.emplace_back(std::make_shared<LinkedGreedyLabels>(vehicleObj, PInst, removedLabels_));
    }
    greedyTime_->stop();
    selectedVehicles_.resize(PInst->nbVehicles_,0);
    if (positionList_.empty()){
        for (auto & vehicleObj : PInst->vehicles_) {
            positionList_.emplace_back(std::make_shared<insertPosition>());
        }
    }
}


void GreedyModeler::solve(PInstance &PInst) {
    greedyTime_->start();
    std::vector<float> possibleDelay;
    std::vector<PGreedyLabel> preLabelList;

    // sort the request based on the earliest possible pickup
    sort(PInst->requests_.begin(), PInst->requests_.end(),[](const PRequest &lhs, const PRequest &rhs){
        return lhs->earlyPick_ > rhs->earlyPick_;});
    for (auto & requestObj : PInst->requests_) {
        if (requestObj->requestStatus_ == NO_ACTION) {
            possibleDelay.clear();
            preLabelList.clear();

            std::string pickID = myTools::createNodeID(requestObj->getRequestId(), PICKUP);
            std::string dropID = myTools::createNodeID(requestObj->getRequestId(), DROPOFF);

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
            unsigned int vehicle_ID = std::min_element(possibleDelay.begin(), possibleDelay.end()) - possibleDelay.begin();
            solutionList_[vehicle_ID]->insertRequest(preLabelList[vehicle_ID], PInst->instGraph_->nodes_[pickID],
                                                     PInst->instGraph_->nodes_[dropID], requestObj->maxTravelTime_);
 //           std::cout << "final insertion" << std::endl;
//            std::cout << solutionList_[vehicle_ID]->toString() << std::endl;
        }
    }
    greedyTime_->stop();
}

void GreedyModeler::solveInsertion(PInstance &PInst) {
    greedySolveTime_->start();
    greedyTime_->start();
    std::vector<float> possibleDelay;
    std::vector<PInsertPosition> positionList;
    for (auto & requestObj : PInst->requests_) {
        if (requestObj->requestStatus_ == NO_ACTION) {
            possibleDelay.clear();
            positionList.clear();

            std::string pickID = myTools::createNodeID(requestObj->getRequestId(), PICKUP);
            std::string dropID = myTools::createNodeID(requestObj->getRequestId(), DROPOFF);

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
                        GreedyObj->tail_->departTime_ = requestObj->earlyPick_;
                    }
                }
                positionList.push_back(GreedyObj->findInsertPlace(PInst->instGraph_->nodes_[pickID],
                                                                     PInst->instGraph_->nodes_[dropID],
                                                                     requestObj->maxTravelTime_));
                possibleDelay.push_back(positionList.back()->deltaDelay_);
            }
            unsigned int vehicle_ID = std::min_element(possibleDelay.begin(), possibleDelay.end()) - possibleDelay.begin();
            solutionList_[vehicle_ID]->insertRequest(positionList[vehicle_ID], PInst->instGraph_->nodes_[pickID],
                                                     PInst->instGraph_->nodes_[dropID], requestObj->maxTravelTime_);
            selectedVehicles_[vehicle_ID]++;
        }
    }
    greedyTime_->stop();
    greedySolveTime_->stop();
}
void GreedyModeler::solutionToRoute(PInstance &PInst) {
    greedyTime_->start();
    greedySolTime_->start();
    for (auto & greedySol : solutionList_) {
        PInst->vehicles_[(*greedySol->Vehicle_)->vehicleID_]->idleTime_ = greedySol->idleTime_;
        PInst->vehicles_[(*greedySol->Vehicle_)->vehicleID_]->currentRoute_.reset();
        PInst->vehicles_[(*greedySol->Vehicle_)->vehicleID_]->currentRoute_ = greedySol->greedyLabelToRoute();
        greedySol.reset();
    }
    greedyTime_->stop();
    greedySolTime_->stop();
    solutionList_.clear();
    removedLabels_.clear();
}

void GreedyModeler::GreedySolver(PInstance &PInst) {
    initialization(PInst);
    solveInsertion(PInst);
    solutionToRoute(PInst);
}

void GreedyModeler::solveInsertionFast(PInstance &PInst) {
    greedySolveTime_->start();
    greedyTime_->start();
    std::vector<float> possibleDelay;
    for (int i = 0; i < PInst->requests_.size(); i++) {
        if (PInst->requests_[i]->requestStatus_ == NO_ACTION) {
            possibleDelay.clear();

            std::string pickID = myTools::createNodeID(PInst->requests_[i]->getRequestId(), PICKUP);
            std::string dropID = myTools::createNodeID(PInst->requests_[i]->getRequestId(), DROPOFF);

            for (auto & GreedyObj : solutionList_){
                /*float minWait = (*GreedyObj->Vehicle_)->departTime_ +
                                durationMatrix_[PInst->instGraph_->nodes_[(*GreedyObj->Vehicle_)->departID_]->locationID_]
                                [PInst->instGraph_->nodes_[pickID]->locationID_]- requestObj->earlyPick_;*/
                // if a vehicle is idle before arrival of a request, its departure time should be after the request time
                // after arrival of each request, the vehicle positions should be updated
                if (GreedyObj->departTime_ < PInst->requests_[i]->earlyPick_) {
                    while ((GreedyObj->head_->child_ != nullptr) && (GreedyObj->departTime_ < PInst->requests_[i]->earlyPick_)) {
                        GreedyObj->head_ = GreedyObj->head_->child_;
                        GreedyObj->departTime_ = GreedyObj->head_->reachTime_ + GreedyObj->head_->currentNode_->deltaTime_;
                    }
                    if (GreedyObj->departTime_ < PInst->requests_[i]->earlyPick_) {
                        GreedyObj->idleTime_ += PInst->requests_[i]->earlyPick_ - GreedyObj->departTime_;
                        GreedyObj->departTime_ = PInst->requests_[i]->earlyPick_;
                        GreedyObj->tail_->departTime_ = PInst->requests_[i]->earlyPick_;
                    }
                }
 //               if (minWait <= requestObj->penalty_) {
                    GreedyObj->findInsertPlace(PInst->instGraph_->pickNodes_[i],PInst->instGraph_->dropNodes_[i],
                                               PInst->requests_[i]->maxTravelTime_, removedLabels_, positionList_[(*GreedyObj->Vehicle_)->vehicleID_]);

                    possibleDelay.push_back(positionList_[(*GreedyObj->Vehicle_)->vehicleID_]->deltaDelay_);
 //               }
 //               else{
//                    possibleDelay.push_back(INFINITY);
 //               }
            }
            unsigned int vehicle_ID = std::min_element(possibleDelay.begin(), possibleDelay.end()) - possibleDelay.begin();
            solutionList_[vehicle_ID]->insertRequest(positionList_[vehicle_ID], PInst->instGraph_->pickNodes_[i],
                                                     PInst->instGraph_->dropNodes_[i],PInst->requests_[i]->maxTravelTime_, removedLabels_);
            selectedVehicles_[vehicle_ID]++;
        }
    }
    greedyTime_->stop();
    greedySolveTime_->stop();
}

void GreedyModeler::GreedySolverFast(PInstance &PInst) {
    initializationFast(PInst);
    solveInsertionFast(PInst);
    for (auto & greedySol : solutionList_) {
        greedySol->resetLinkedGreedyLabels(removedLabels_);
        greedySol.reset();
    }
    /*for (auto & position : positionList_){
        position->preDrop_->currentNode_.reset();
        position->prePickup_->currentNode_.reset();
    }*/
    solutionList_.clear();
}


// this function just assign requests to vehicles based on the minimum delay possible and do not consider ride-sharing
// any pick up is followed by the drop-off
void GreedySolver_noShare(PInstance &PInst) {
    std::vector<float> possibleDelay;
    for (auto & requestObj : PInst->requests_) {
        if (requestObj->requestStatus_ == NO_ACTION) {
            possibleDelay.clear();
            std::string pickID = myTools::createNodeID(requestObj->getRequestId(), PICKUP);
            std::string dropID = myTools::createNodeID(requestObj->getRequestId(), DROPOFF);

            for (auto &vehicleObj: PInst->vehicles_) {
                float requestPickTime = vehicleObj->currentRoute_->plannedReachTime_.back() +
                                        vehicleObj->currentRoute_->routeNodes_.back()->deltaTime_ +
                                        durationMatrix_[vehicleObj->currentRoute_->routeNodes_.back()->locationID_][PInst->instGraph_->nodes_[pickID]->locationID_];
                possibleDelay.push_back(requestPickTime);
            }
            unsigned int vehicle_ID = std::min_element(possibleDelay.begin(), possibleDelay.end()) - possibleDelay.begin();
            PInst->vehicles_[vehicle_ID]->currentRoute_->addNode(PInst->instGraph_->nodes_[pickID]);
            PInst->vehicles_[vehicle_ID]->currentRoute_->addNode(PInst->instGraph_->nodes_[dropID]);
        }
    }
}




