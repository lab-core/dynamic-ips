//
// Created by Ella on 5/25/2022.
//

#include "GreedyModeler.h"
#include "data/Greedy.h"

GreedyModeler::GreedyModeler() {
    greedyTime_ = new myTools::Timer(); greedyTime_->init();
    objValue_ = 0;
}

GreedyModeler::~GreedyModeler() {
    delete greedyTime_;
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

void GreedyModeler::solveInsertion(const PInstance &PInst) {
    std::vector<float> deltaObjective;
    for (int i = 0; i < PInst->requests_.size(); i++) {
        if (PInst->requests_[i]->requestStatus_ == NO_ACTION) {
            if (PInst->parameters_->greedyReOptimize_ || PInst->requests_[i]->solVehicleID_ == LARGE_CONSTANT) {
                deltaObjective.clear();
                for (auto &GRoute: greedyRouteList_) {

                    GRoute->findInsertPlace(PInst->instGraph_->pickNodes_[i], PInst->instGraph_->dropNodes_[i],
                                            PInst->requests_[i]->maxTravelTime_, greedyLabelPool_,
                                            positionList_[(*GRoute->Vehicle_)->vehicleID_], PInst->parameters_->Wait_W1_,
                                            PInst->parameters_->Ride_W2_);

                    deltaObjective.push_back(positionList_[(*GRoute->Vehicle_)->vehicleID_]->deltaObjective_);
                }
                unsigned int vehicle_ID =
                        std::min_element(deltaObjective.begin(), deltaObjective.end()) - deltaObjective.begin();
                PInst->requests_[i]->marginalCost_ = deltaObjective[vehicle_ID];

                greedyRouteList_[vehicle_ID]->insertRequest(positionList_[vehicle_ID], PInst->instGraph_->pickNodes_[i],
                                                            PInst->instGraph_->dropNodes_[i],
                                                            PInst->requests_[i]->maxTravelTime_, greedyLabelPool_);
                PInst->selectedVehicles_[vehicle_ID]++;
            }
        }
    }
    for (auto & GreedyObj : greedyRouteList_)
        GreedyObj->updateTailDepart();
}

void GreedyModeler::solutionToRoute(const PInstance &PInst) {
    setObjValue(PInst->parameters_->Wait_W1_, PInst->parameters_->Ride_W2_);
    for (auto & greedySol : greedyRouteList_) {
        PInst->vehicles_[(*greedySol->Vehicle_)->vehicleID_]->currentRoute_.reset();
        PRoute newRoute;
        if (PInst->parameters_->solutionMode_ == STATIC) {
            PInst->vehicles_[(*greedySol->Vehicle_)->vehicleID_]->idleTime_ += greedySol->idleTime_;
            newRoute = greedySol->greedyLabelToRoute(true);
            newRoute->calculateTripDelay(PInst->parameters_->Wait_W1_, PInst->parameters_->Ride_W2_);

        }
        else {
            newRoute = greedySol->greedyLabelToRoute(false);
            newRoute->calculateTripDelay(PInst->parameters_->Wait_W1_, PInst->parameters_->Ride_W2_);
        }
        newRoute->createColumn(PInst->nbRequests_);
        PInst->vehicles_[(*greedySol->Vehicle_)->vehicleID_]->setCurrentRoute(newRoute);
        greedySol->resetGreedyRoute(greedyLabelPool_);
        greedySol.reset();
    }
    greedyRouteList_.clear();

    zSolution_.clear();
    for (auto &requestObj: PInst->requests_) {
        if (requestObj->solVehicleID_ == LARGE_CONSTANT)
            zSolution_.push_back(requestObj);
    }
}

float GreedyModeler::createUpperbound(float wait_W1, float ride_W2) {
    float upperbound = 0;
    for (auto & greedySol : greedyRouteList_) {
        (*greedySol->Vehicle_)->greedyRoute_ = greedySol->greedyLabelToRoute(false);
        (*greedySol->Vehicle_)->greedyRoute_->calculateTripDelay(wait_W1, ride_W2);
        upperbound += (*greedySol->Vehicle_)->greedyRoute_->objCoef_;
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

float GreedyModeler::GreedyUpperbound(PInstance &PInst) {
    PInst->parameters_->greedyReOptimize_ = false;
    greedyTime_->start();
    initialization(PInst);
    solveInsertion(PInst);
    float upperbound = createUpperbound(PInst->parameters_->Wait_W1_, PInst->parameters_->Ride_W2_);
    greedyTime_->stop();
    return upperbound;
}


void GreedyModeler::setObjValue(float wait_W1, float ride_W2) {
    objValue_ = 0.0;
    for (auto & GreedyObj : greedyRouteList_)
        objValue_ += wait_W1 * GreedyObj->totalWait_+ ride_W2 * GreedyObj->totalTripDelay_;
}

