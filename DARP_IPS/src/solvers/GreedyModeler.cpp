//
// Created by Ella on 5/25/2022.
//

#include "GreedyModeler.h"
#include "data/Greedy.h"

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
   // if (PInst->parameters_->mainAlgorithm_ == GREEDY) {
        PInst->selectedVehicles_.clear();
        PInst->selectedVehicles_.resize(PInst->nbVehicles_, 0);
   // }

    for (auto & vehicleObj : PInst->vehicles_) {
        greedyRouteList_.emplace_back(std::make_shared<GreedyRoute>(vehicleObj, PInst, greedyLabelPool_, PInst->parameters_->greedyReOptimize_));
    }

    if (positionList_.empty()){
        for (int i = 0; i < PInst->nbVehicles_; ++i)
            positionList_.emplace_back(std::make_shared<insertPosition>());
    }
}


void GreedyModeler::solutionToRoute(const PInstance &PInst) {
    setObjValue();
    for (auto & greedySol : greedyRouteList_) {
        PInst->vehicles_[(*greedySol->Vehicle_)->vehicleID_]->currentRoute_.reset();
        PRoute newRoute;
        if (PInst->parameters_->solutionMode_ == STATIC) {
            PInst->vehicles_[(*greedySol->Vehicle_)->vehicleID_]->idleTime_ += greedySol->idleTime_;
            newRoute = greedySol->greedyLabelToRoute(true);

        }
        else {
            newRoute = greedySol->greedyLabelToRoute(false);
        }
        newRoute->createColumn(PInst->nbRequests_);
        PInst->vehicles_[(*greedySol->Vehicle_)->vehicleID_]->setCurrentRoute(newRoute);
        greedySol->resetGreedyRoute(greedyLabelPool_);
        greedySol.reset();
    }
    greedyRouteList_.clear();
}

float GreedyModeler::createUpperbound() {
    float upperbound = 0;
    for (auto & greedySol : greedyRouteList_) {
        (*greedySol->Vehicle_)->greedyRoute_ = greedySol->greedyLabelToRoute(false);
        upperbound += (*greedySol->Vehicle_)->greedyRoute_->totalDelay_;
        greedySol->resetGreedyRoute(greedyLabelPool_);
        greedySol.reset();
    }
    greedyRouteList_.clear();
    return upperbound;
}

void GreedyModeler::GreedySolver(PInstance &PInst) {
    greedyTime_->start();
    initialization(PInst);
    solveInsertion(PInst);
    solutionToRoute(PInst);
    greedyTime_->stop();
}

void GreedyModeler::GreedyUpperbound(PInstance &PInst) {
    PInst->parameters_->greedyReOptimize_ = false;
    greedyTime_->start();
    initialization(PInst);
    solveInsertion(PInst);
    float upperbound = createUpperbound();
    greedyTime_->stop();
}

void GreedyModeler::GreedyAssignment(PInstance &PInst, int select) {
    greedyAssignTime_->start();
    if (select == 1) {
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

void GreedyModeler::solveInsertion(const PInstance &PInst) {
    std::vector<float> possibleDelay;
    for (int i = 0; i < PInst->requests_.size(); i++) {
        if (PInst->requests_[i]->requestStatus_ == NO_ACTION) {
            if (PInst->parameters_->greedyReOptimize_ || PInst->requests_[i]->solVehicleID_ == LARGE_CONSTANT) {
                possibleDelay.clear();
                for (auto &GRoute: greedyRouteList_) {

                    GRoute->findInsertPlace(PInst->instGraph_->pickNodes_[i], PInst->instGraph_->dropNodes_[i],
                                            PInst->requests_[i]->maxTravelTime_, greedyLabelPool_,
                                            positionList_[(*GRoute->Vehicle_)->vehicleID_]);

                    possibleDelay.push_back(positionList_[(*GRoute->Vehicle_)->vehicleID_]->deltaDelay_);
                }
                unsigned int vehicle_ID =
                        std::min_element(possibleDelay.begin(), possibleDelay.end()) - possibleDelay.begin();
                PInst->requests_[i]->marginalCost_ = possibleDelay[vehicle_ID];

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



void GreedyModeler::solveAssignment(const PInstance &PInst, int select) {
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


// this function assigns requests to vehicles based on the minimum delay possible and do not consider ride-sharing
// any pickup is followed by the drop-off
void GreedySolver_noShare(const PInstance &PInst) {
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
