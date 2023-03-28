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
    for (auto & label : greedyLabelPool_){
        label->child_.reset();
        label->parent_.reset();
        label->pair_.reset();
    }
    greedyLabelPool_.clear();
}

void GreedyModeler::initialization(PInstance &PInst) {
    selectedVehicles_.clear();
    for (auto & vehicleObj : PInst->vehicles_) {
        solutionList_.emplace_back(std::make_shared<GreedyRoute>(vehicleObj, PInst, greedyLabelPool_, PInst->parameters_->greedyReOptimize_));
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
        greedySol->resetLinkedGreedyLabels(greedyLabelPool_);
        greedySol.reset();
    }
    solutionList_.clear();
 //   greedyLabelPool_.clear();
}

void GreedyModeler::GreedySolver(PInstance &PInst) {
    greedyTime_->start();
    initialization(PInst);
    solveInsertion(PInst);
    solutionToRoute(PInst);
    greedyTime_->stop();
}

void GreedyModeler::GreedyAssignment(PInstance &PInst) {
    greedyAssignTime_->start();
    initialization(PInst);
    solveAssignment(PInst);
    for (auto & greedySol : solutionList_) {
        greedySol->resetLinkedGreedyLabels(greedyLabelPool_);
        greedySol.reset();
    }
    solutionList_.clear();
    greedyAssignTime_->stop();
}

void GreedyModeler::solveInsertion(PInstance &PInst) {
    std::vector<float> possibleDelay;
    for (int i = 0; i < PInst->requests_.size(); i++) {
        if (PInst->requests_[i]->requestStatus_ == NO_ACTION) {
            possibleDelay.clear();
            for (auto & GRoute : solutionList_){
                GRoute->idle_ = false;
                GRoute->preDepartTime_ = GRoute->departureTime_;
                // if a vehicle is idle before arrival of a request, its departure time should be after the request time
                // after arrival of each request, the vehicle positions should be updated
                if (GRoute->departureTime_ < PInst->requests_[i]->earlyPick_ + PInst->parameters_->committedTime_) {
                    while ((GRoute->PCurrentStop_->child_ != nullptr) && (GRoute->departureTime_ < PInst->requests_[i]->earlyPick_ + PInst->parameters_->committedTime_)) {
                        GRoute->PCurrentStop_ = GRoute->PCurrentStop_->child_;
                        GRoute->departureTime_ = GRoute->PCurrentStop_->leaveTime_;
                        GRoute->preDepartTime_ = GRoute->departureTime_;
                    }
                    // if we are at the end of the route
                    if (GRoute->departureTime_ < PInst->requests_[i]->earlyPick_ + PInst->parameters_->committedTime_) {
 //                       GRoute->idleTime_ += PInst->requests_[i]->earlyPick_ + PInst->parameters_->committedTime_ - GRoute->departureTime_;
                        GRoute->departureTime_ = PInst->requests_[i]->earlyPick_ + PInst->parameters_->committedTime_;
                        GRoute->PLastStop_->leaveTime_ = PInst->requests_[i]->earlyPick_ + PInst->parameters_->committedTime_;
                        GRoute->idle_ = true;
                    }
                }
                GRoute->findInsertPlace2(PInst->instGraph_->pickNodes_[i], PInst->instGraph_->dropNodes_[i],
                                         PInst->requests_[i]->maxTravelTime_, greedyLabelPool_, positionList_[(*GRoute->Vehicle_)->vehicleID_]);

                possibleDelay.push_back(positionList_[(*GRoute->Vehicle_)->vehicleID_]->deltaDelay_);
                /*if (GRoute->idle_) {
                    GRoute->departureTime_ = GRoute->preDepartTime_;
                    GRoute->PLastStop_->leaveTime_ = GRoute->preDepartTime_;
                }*/
            }
            unsigned int vehicle_ID = std::min_element(possibleDelay.begin(), possibleDelay.end()) - possibleDelay.begin();
            if (solutionList_[vehicle_ID]->departureTime_ < PInst->requests_[i]->earlyPick_){
                // reset departure time
                solutionList_[vehicle_ID]->departureTime_ = PInst->requests_[i]->earlyPick_;
                solutionList_[vehicle_ID]->PLastStop_->leaveTime_ = PInst->requests_[i]->earlyPick_;
                solutionList_[vehicle_ID]->idleTime_ += PInst->requests_[i]->earlyPick_ - solutionList_[vehicle_ID]->departureTime_;
            }
            solutionList_[vehicle_ID]->insertRequest(positionList_[vehicle_ID], PInst->instGraph_->pickNodes_[i],
                                                     PInst->instGraph_->dropNodes_[i], PInst->requests_[i]->maxTravelTime_, greedyLabelPool_);
            solutionList_[vehicle_ID]->idle_ = false;
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
                    GreedyObj->findAssignedPlace1(PInst->instGraph_->pickNodes_[i], PInst->instGraph_->dropNodes_[i],
                                                  PInst->requests_[i]->maxTravelTime_, greedyLabelPool_, positionList_[(*GreedyObj->Vehicle_)->vehicleID_]);

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
                                        vehicleObj->currentRoute_->routeNodes_.back()->serviceTime_ +
                                        durationMatrix_[vehicleObj->currentRoute_->routeNodes_.back()->locationID_][PInst->instGraph_->nodes_[pickID]->locationID_];
                possibleDelay.push_back(requestPickTime);
            }
            unsigned int vehicle_ID = std::min_element(possibleDelay.begin(), possibleDelay.end()) - possibleDelay.begin();
            PInst->vehicles_[vehicle_ID]->currentRoute_->addNode(PInst->instGraph_->nodes_[pickID]);
            PInst->vehicles_[vehicle_ID]->currentRoute_->addNode(PInst->instGraph_->nodes_[dropID]);
        }
    }
}




