//
// Created by Ella on 5/25/2022.
//

#include "GreedyModeler.h"

GreedyModeler::GreedyModeler() {
    greedyTime_ = new myTools::Timer(); greedyTime_->init();
    greedyAssignTime_ = new myTools::Timer(); greedyAssignTime_->init();
    objValue_ = 0;
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
    if (PInst->parameters_->mainAlgorithm_ == GREEDY) {
        PInst->selectedVehicles_.clear();
        PInst->selectedVehicles_.resize(PInst->nbVehicles_, 0);
    }

    for (auto & vehicleObj : PInst->vehicles_) {
        greedyRouteList_.emplace_back(std::make_shared<GreedyRoute>(vehicleObj, PInst, greedyLabelPool_, PInst->parameters_->greedyReOptimize_));
    }

    if (positionList_.empty()){
        for (int i = 0; i < PInst->nbVehicles_; ++i)
            positionList_.emplace_back(std::make_shared<insertPosition>());
    }
}


void GreedyModeler::solutionToRoute(PInstance &PInst) {
    setObjValue();
    for (auto & greedySol : greedyRouteList_) {
        PInst->vehicles_[(*greedySol->Vehicle_)->vehicleID_]->currentRoute_.reset();
        if (PInst->parameters_->solutionMode_ == STATIC) {
            PInst->vehicles_[(*greedySol->Vehicle_)->vehicleID_]->idleTime_ += greedySol->idleTime_;
            PInst->vehicles_[(*greedySol->Vehicle_)->vehicleID_]->currentRoute_ = greedySol->greedyLabelToRoute(true);
        }
        else
            PInst->vehicles_[(*greedySol->Vehicle_)->vehicleID_]->currentRoute_ = greedySol->greedyLabelToRoute(false);
        greedySol->resetGreedyRoute(greedyLabelPool_);
        greedySol.reset();
    }
    greedyRouteList_.clear();
}

void GreedyModeler::GreedySolver(PInstance &PInst) {
    greedyTime_->start();
    initialization(PInst);
    solveInsertion(PInst);
    solutionToRoute(PInst);
    greedyTime_->stop();
}

void GreedyModeler::GreedyAssignment(PInstance &PInst, int select) {
    greedyAssignTime_->start();
    if ((select == 1 && !PInst->parameters_->addOneRequestColumn_)||(select == 2 && PInst->parameters_->addOneRequestColumn_)) {
        for (auto & greedySol : greedyRouteList_) {
            greedySol.reset();
        }
        greedyRouteList_.clear();
        initialization(PInst);
    }

    solveAssignment(PInst, select);
    /*for (auto & greedySol : greedyRouteList_) {
        greedySol.reset();
    }
    greedyRouteList_.clear();*/
    greedyAssignTime_->stop();
}

void GreedyModeler::solveInsertion(PInstance &PInst) {
    std::vector<float> possibleDelay;
    for (int i = 0; i < PInst->requests_.size(); i++) {
        if (PInst->requests_[i]->requestStatus_ == NO_ACTION) {
            if (PInst->parameters_->greedyReOptimize_ || PInst->requests_[i]->solVehicleID_ == LARGE_CONSTANT) {
                possibleDelay.clear();
                for (auto &GRoute: greedyRouteList_) {
                    GRoute->idle_ = false;
                    // if a vehicle is idle before arrival of a request, its departure time should be after the request time
                    // after arrival of each request, the vehicle positions should be updated
                    if (GRoute->departureTime_ < PInst->requests_[i]->requestTime_ + PInst->parameters_->committedTime_) {
                        while ((GRoute->PCurrentStop_->child_ != nullptr) && (GRoute->departureTime_ <
                                                                              PInst->requests_[i]->requestTime_ +
                                                                              PInst->parameters_->committedTime_)) {
                            GRoute->PCurrentStop_ = GRoute->PCurrentStop_->child_;
                            GRoute->departureTime_ = GRoute->PCurrentStop_->leaveTime_;
                        }
                        // if we are at the end of the route
                        if (GRoute->departureTime_ <
                            PInst->requests_[i]->requestTime_ + PInst->parameters_->committedTime_) {
                            //                       GRoute->idleTime_ += PInst->requests_[i]->earlyPick_ + PInst->parameters_->committedTime_ - GRoute->departureTime_;
                            GRoute->departureTime_ =
                                    PInst->requests_[i]->requestTime_ + PInst->parameters_->committedTime_;
                            GRoute->PLastStop_->leaveTime_ =
                                    PInst->requests_[i]->requestTime_ + PInst->parameters_->committedTime_;
                            GRoute->idleTime_ += PInst->requests_[i]->requestTime_ + PInst->parameters_->committedTime_ -
                                                 GRoute->departureTime_;
                            GRoute->idle_ = true;
                        }
                    }
                    GRoute->findInsertPlace(PInst->instGraph_->pickNodes_[i], PInst->instGraph_->dropNodes_[i],
                                            PInst->requests_[i]->maxTravelTime_, greedyLabelPool_,
                                            positionList_[(*GRoute->Vehicle_)->vehicleID_]);

                    possibleDelay.push_back(positionList_[(*GRoute->Vehicle_)->vehicleID_]->deltaDelay_);
                    /*if (GRoute->idle_) {
                        GRoute->departureTime_ = GRoute->preDepartTime_;
                        GRoute->PLastStop_->leaveTime_ = GRoute->preDepartTime_;
                    }*/
                }
                unsigned int vehicle_ID =
                        std::min_element(possibleDelay.begin(), possibleDelay.end()) - possibleDelay.begin();
                /*if (greedyRouteList_[vehicle_ID]->departureTime_ < PInst->requests_[i]->earlyPick_){
                    // reset departure time
                    greedyRouteList_[vehicle_ID]->departureTime_ = PInst->requests_[i]->earlyPick_;
                    greedyRouteList_[vehicle_ID]->PLastStop_->leaveTime_ = PInst->requests_[i]->earlyPick_;
                    greedyRouteList_[vehicle_ID]->idleTime_ += PInst->requests_[i]->earlyPick_ - greedyRouteList_[vehicle_ID]->departureTime_;
                }*/
                greedyRouteList_[vehicle_ID]->insertRequest(positionList_[vehicle_ID], PInst->instGraph_->pickNodes_[i],
                                                            PInst->instGraph_->dropNodes_[i],
                                                            PInst->requests_[i]->maxTravelTime_, greedyLabelPool_);
                greedyRouteList_[vehicle_ID]->idle_ = false;
                PInst->selectedVehicles_[vehicle_ID]++;
            }
        }
    }
    for (auto & GreedyObj : greedyRouteList_)
        GreedyObj->updateTailDepart();
}



void GreedyModeler::solveAssignment(PInstance &PInst, int select) {
    std::vector<float> possibleDelay;
    for (int i = 0; i < PInst->requests_.size(); i++) {
        if (PInst->requests_[i]->requestStatus_ == NO_ACTION) {
            possibleDelay.clear();

            for (auto & GreedyObj : greedyRouteList_){
                if (!GreedyObj->selected_){
                    GreedyObj->findAssignedPlace(PInst->instGraph_->pickNodes_[i], PInst->instGraph_->dropNodes_[i],
                                                 PInst->requests_[i]->maxTravelTime_, greedyLabelPool_,
                                                 positionList_[(*GreedyObj->Vehicle_)->vehicleID_]);

                    possibleDelay.push_back(positionList_[(*GreedyObj->Vehicle_)->vehicleID_]->deltaDelay_);
                }
                else
                    possibleDelay.push_back(INFINITY);
            }
            unsigned int vehicle_ID = std::min_element(possibleDelay.begin(), possibleDelay.end()) - possibleDelay.begin();
            greedyRouteList_[vehicle_ID]->selected_ = true;
            PInst->selectedVehicles_[vehicle_ID] = select;
        }
    }
}

void GreedyModeler::setObjValue() {
    objValue_ = 0.0;
    for (auto & GreedyObj : greedyRouteList_)
        objValue_ += GreedyObj->totalDelay_;
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
