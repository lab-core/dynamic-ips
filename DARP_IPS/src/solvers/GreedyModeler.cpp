//
// Created by Ella on 5/25/2022.
//

#include "GreedyModeler.h"

GreedyModeler::GreedyModeler() {
    greedyTime_ = new myTools::Timer(); greedyTime_->init();
    greedyAssignTime_ = new myTools::Timer(); greedyAssignTime_->init();
    float deltaDelay = INFINITY;

    // Tour length increase by adding pickup
    float pickDeltaT = INFINITY;

    // Tour length increase by adding drop off
    float dropDeltaT = INFINITY;
}

GreedyModeler::~GreedyModeler() {
    delete greedyTime_;
    delete greedyAssignTime_;
    for (auto & label : removedLabels_){
        label->child_.reset();
        label->parent_.reset();
        label->pair_.reset();
    }
    removedLabels_.clear();
}

void GreedyModeler::initializationFast(PInstance &PInst) {
    selectedVehicles_.clear();
    for (auto & vehicleObj : PInst->vehicles_) {
        solutionList_.emplace_back(std::make_shared<LinkedGreedyLabels>(vehicleObj, PInst, removedLabels_));
    }
    selectedVehicles_.resize(PInst->nbVehicles_,0);
    if (positionList_.empty()){
        for (auto & vehicleObj : PInst->vehicles_) {
            positionList_.emplace_back(std::make_shared<insertPosition>());
        }
    }
}


void GreedyModeler::solutionToRoute(PInstance &PInst) {
    for (auto & greedySol : solutionList_) {
        PInst->vehicles_[(*greedySol->Vehicle_)->vehicleID_]->currentRoute_.reset();
        if (PInst->parameters_->solutionMode_ == STATIC) {
            PInst->vehicles_[(*greedySol->Vehicle_)->vehicleID_]->idleTime_ += greedySol->idleTime_;
            PInst->vehicles_[(*greedySol->Vehicle_)->vehicleID_]->currentRoute_ = greedySol->greedyLabelToRoute(true);
        }
        else
            PInst->vehicles_[(*greedySol->Vehicle_)->vehicleID_]->currentRoute_ = greedySol->greedyLabelToRoute(false);
        greedySol->resetLinkedGreedyLabels(removedLabels_);
        greedySol.reset();
    }
    solutionList_.clear();
 //   removedLabels_.clear();
}

void GreedyModeler::GreedySolver(PInstance &PInst) {
    greedyTime_->start();
    initializationFast(PInst);
    solveInsertionFast(PInst);
    solutionToRoute(PInst);
    greedyTime_->stop();
}

void GreedyModeler::GreedySolver(PInstance &PInst, vector2D<PRoute> &availableRoutes, int size) {
    greedyTime_->start();
    initializationFast(PInst);
    solveInsertionFast(PInst);
    for (auto & greedySol : solutionList_) {
        availableRoutes[(*greedySol->Vehicle_)->vehicleID_].emplace_back(greedySol->greedyLabelToRoute(false));
        availableRoutes[(*greedySol->Vehicle_)->vehicleID_].back()->createColumn(size);
        greedySol->resetLinkedGreedyLabels(removedLabels_);
        greedySol.reset();
    }
    solutionList_.clear();
    greedyTime_->stop();
}

void GreedyModeler::GreedySolverFast(PInstance &PInst) {
    greedyAssignTime_->start();
    initializationFast(PInst);
    solveAssignment(PInst);
    for (auto & greedySol : solutionList_) {
        greedySol->resetLinkedGreedyLabels(removedLabels_);
        greedySol.reset();
    }
    solutionList_.clear();
    greedyAssignTime_->stop();
}

void GreedyModeler::solveInsertionFast(PInstance &PInst) {
    std::vector<float> possibleDelay;
    for (int i = 0; i < PInst->requests_.size(); i++) {
        if (PInst->requests_[i]->requestStatus_ == NO_ACTION) {
            /*if (PInst->requests_[i]->getRequestId() == 137)
                std::cout << "stop";*/
            possibleDelay.clear();
            for (auto & GreedyObj : solutionList_){
                /*if((*GreedyObj->Vehicle_)->vehicleID_ == 84)
                    std::cout << "stop";*/
                // if a vehicle is idle before arrival of a request, its departure time should be after the request time
                // after arrival of each request, the vehicle positions should be updated
                if (GreedyObj->departureTime_ < PInst->requests_[i]->earlyPick_) {
                    while ((GreedyObj->head_->child_ != nullptr) && (GreedyObj->departureTime_ < PInst->requests_[i]->earlyPick_)) {
                        GreedyObj->head_ = GreedyObj->head_->child_;
                        GreedyObj->departureTime_ = GreedyObj->head_->departTime_;
                    }
                    if (GreedyObj->departureTime_ < PInst->requests_[i]->earlyPick_) {
                        GreedyObj->idleTime_ += PInst->requests_[i]->earlyPick_ - GreedyObj->departureTime_;
                        GreedyObj->departureTime_ = PInst->requests_[i]->earlyPick_;
                        GreedyObj->tail_->departTime_ = PInst->requests_[i]->earlyPick_;
                    }
                }
                GreedyObj->findInsertPlace2(PInst->instGraph_->pickNodes_[i],PInst->instGraph_->dropNodes_[i],
                                           PInst->requests_[i]->maxTravelTime_, removedLabels_, positionList_[(*GreedyObj->Vehicle_)->vehicleID_]);

                possibleDelay.push_back(positionList_[(*GreedyObj->Vehicle_)->vehicleID_]->deltaDelay_);
            }
            unsigned int vehicle_ID = std::min_element(possibleDelay.begin(), possibleDelay.end()) - possibleDelay.begin();
            /*if (vehicle_ID == 0)
                std::cout << "stop";*/
            solutionList_[vehicle_ID]->insertRequest(positionList_[vehicle_ID], PInst->instGraph_->pickNodes_[i],
                                                     PInst->instGraph_->dropNodes_[i],PInst->requests_[i]->maxTravelTime_, removedLabels_);
            selectedVehicles_[vehicle_ID]++;
        }
    }
    for (auto & GreedyObj : solutionList_)
        GreedyObj->updateTailDepart1();
}



void GreedyModeler::solveAssignment(PInstance &PInst) {
    std::vector<float> possibleDelay;
    for (int i = 0; i < PInst->requests_.size(); i++) {
        if (PInst->requests_[i]->requestStatus_ == NO_ACTION) {
            possibleDelay.clear();

            for (auto & GreedyObj : solutionList_){
                if (!GreedyObj->selected_){
                    GreedyObj->findAssignedPlace1(PInst->instGraph_->pickNodes_[i],PInst->instGraph_->dropNodes_[i],
                                                 PInst->requests_[i]->maxTravelTime_, removedLabels_, positionList_[(*GreedyObj->Vehicle_)->vehicleID_]);

                    possibleDelay.push_back(positionList_[(*GreedyObj->Vehicle_)->vehicleID_]->deltaDelay_);
                }
                else
                    possibleDelay.push_back(INFINITY);
            }
            unsigned int vehicle_ID = std::min_element(possibleDelay.begin(), possibleDelay.end()) - possibleDelay.begin();
            solutionList_[vehicle_ID]->selected_ = true;
            selectedVehicles_[vehicle_ID]++;
        }
    }
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




