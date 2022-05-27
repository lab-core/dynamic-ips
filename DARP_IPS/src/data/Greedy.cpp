//
// Created by Ella on 5/20/2022.
//

#include "Greedy.h"

GreedyLabel::GreedyLabel(const PNode &currentNode, float reachTime, int nbPassengers) : currentNode_(currentNode),
                                                                                        reachTime_(reachTime),
                                                                                        nbPassengers_(nbPassengers) {
    parent_ = nullptr;
    child_ = nullptr;
    pair_ = nullptr;
    travelResource_ = 0;
}


LinkedGreedyLabels::LinkedGreedyLabels(PVehicle &vehicle, PInstance &pInst) : Vehicle_(&vehicle)  {
    source_ = std::make_shared<GreedyLabel>(pInst->instGraph_->nodes_[(*Vehicle_)->departID_], vehicle->departTime_, vehicle->onboards_.size());
    head_ = source_;
    tail_ = source_;
    totalDelay_ = 0;
    endTime_ = vehicle->departTime_;
    for (auto &nodeID: (*Vehicle_)->onboards_) {
        float dropTime = tail_->reachTime_ + tail_->currentNode_->deltaTime_ +
                         durationMatrix_[tail_->currentNode_->locationID_][pInst->instGraph_->nodes_[nodeID]->locationID_];
        PGreedyLabel newDropLabel = std::make_shared<GreedyLabel>(pInst->instGraph_->nodes_[nodeID], dropTime,
                                                                  tail_->nbPassengers_ +
                                                                  pInst->instGraph_->nodes_[nodeID]->nbPassengers_);
        newDropLabel->parent_ = tail_;
        tail_->child_ = newDropLabel;

        newDropLabel->travelResource_ = pInst->instGraph_->nodes_[nodeID]->related_Request_->maxTravelTime_ -
                dropTime + pInst->instGraph_->nodes_[nodeID]->related_Request_->pickTime_ +
                pInst->instGraph_->nodes_[nodeID]->deltaTime_;

        tail_ = newDropLabel;
    }
}

void LinkedGreedyLabels::insertionAfter(PGreedyLabel preLabel, PNode &insetNode) {
    float reachTime = preLabel->reachTime_ + preLabel->currentNode_->deltaTime_ +
            durationMatrix_[preLabel->currentNode_->locationID_][insetNode->locationID_];
    PGreedyLabel NewGreedyLabel = std::make_shared<GreedyLabel>(insetNode, reachTime,
                                                                preLabel->nbPassengers_ + insetNode->nbPassengers_);
    if (preLabel->child_ != nullptr){
        preLabel->child_->parent_ = NewGreedyLabel;
        NewGreedyLabel->child_ = preLabel->child_;
        NewGreedyLabel->parent_ = preLabel;
        preLabel->child_= NewGreedyLabel;
    }
    float increaseTime = reachTime + insetNode->deltaTime_ +
            durationMatrix_[insetNode->locationID_][NewGreedyLabel->child_->currentNode_->locationID_] -
            NewGreedyLabel->child_->reachTime_;
    PGreedyLabel currentLabel = NewGreedyLabel;
    while (currentLabel->child_ != nullptr) {
        currentLabel->child_->reachTime_ += increaseTime;
        currentLabel->child_->nbPassengers_ += insetNode->nbPassengers_;
        currentLabel = currentLabel->child_;
    }
}


PGreedyLabel LinkedGreedyLabels::findInsertPosition(PNode &pickNode, PNode &dropNode, float maxDuration) {
    PGreedyLabel curLabel = head_;
    // find a position based on the available vehicle capacity
    while (curLabel->child_ != nullptr) {
        if (curLabel->nbPassengers_ + pickNode->nbPassengers_ > (*Vehicle_)->capacity_)
            curLabel = curLabel->child_;
        else
            break;
    }
    // find a position based on restriction of travel duration
    PGreedyLabel dropLabel = curLabel;
    while (dropLabel->child_ != nullptr) {
        float deltaT = pickNode->deltaTime_ + durationMatrix_[curLabel->currentNode_->locationID_][pickNode->locationID_] +
                       durationMatrix_[pickNode->locationID_][curLabel->child_->currentNode_->locationID_] -
                       durationMatrix_[curLabel->currentNode_->locationID_][curLabel->child_->currentNode_->locationID_];
        if (curLabel->child_->travelResource_ < deltaT)
        {
            curLabel = dropLabel->child_;
            dropLabel = dropLabel->child_;
            if (dropLabel->child_ != nullptr)
                deltaT = pickNode->deltaTime_ + durationMatrix_[curLabel->currentNode_->locationID_][pickNode->locationID_] +
                         durationMatrix_[pickNode->locationID_][curLabel->child_->currentNode_->locationID_] -
                         durationMatrix_[curLabel->currentNode_->locationID_][curLabel->child_->currentNode_->locationID_];
        }
        else
            dropLabel = dropLabel->child_;
    }
    // is it possible to drop it afterwards
    while (true) {
        if (curLabel->child_ != nullptr) {
            float pickTime = curLabel->reachTime_ + curLabel->currentNode_->deltaTime_ +
                             durationMatrix_[curLabel->currentNode_->locationID_][pickNode->locationID_];
            float deltaT =
                    pickNode->deltaTime_ + durationMatrix_[curLabel->currentNode_->locationID_][pickNode->locationID_] +
                    durationMatrix_[pickNode->locationID_][curLabel->child_->currentNode_->locationID_] -
                    durationMatrix_[curLabel->currentNode_->locationID_][curLabel->child_->currentNode_->locationID_];
            float dropTime = tail_->reachTime_ + deltaT + tail_->currentNode_->deltaTime_ +
                             durationMatrix_[tail_->currentNode_->locationID_][dropNode->locationID_];
            if (dropTime - pickTime > maxDuration)
                curLabel = curLabel->child_;
            else
                break;
        }
        else
            break;
    }
    return curLabel;
}

void LinkedGreedyLabels::insertRequest(PGreedyLabel &preLabel, PNode &pickNode, PNode &dropNode, float maxDuration) {
    float pickTime = preLabel->reachTime_ + preLabel->currentNode_->deltaTime_ +
                      durationMatrix_[preLabel->currentNode_->locationID_][pickNode->locationID_];
    if (pickTime < pickNode->requestTime_)
        pickTime = pickNode->requestTime_;
    PGreedyLabel newPickLabel = std::make_shared<GreedyLabel>(pickNode, pickTime,
                                                                preLabel->nbPassengers_ + pickNode->nbPassengers_);
    if (preLabel->child_ == nullptr){
        newPickLabel->parent_ = preLabel;
        preLabel->child_= newPickLabel;

        float dropTime = pickTime + pickNode->deltaTime_ +
                         durationMatrix_[pickNode->locationID_][dropNode->locationID_];
        PGreedyLabel newDropLabel = std::make_shared<GreedyLabel>(dropNode, dropTime,
                                                                  newPickLabel->nbPassengers_ +
                                                                  dropNode->nbPassengers_);
        head_ = newPickLabel;
        tail_ = newDropLabel;
        newPickLabel->child_ = newDropLabel;
        newDropLabel->parent_ = newPickLabel;
        newPickLabel->pair_ = newDropLabel;
        newDropLabel->pair_ = newPickLabel;
        newDropLabel->travelResource_ = maxDuration - newDropLabel->reachTime_ + newDropLabel->reachTime_ + pickNode->deltaTime_;
    }
    else {
        float deltaT = pickTime + pickNode->deltaTime_ +
                durationMatrix_[pickNode->locationID_][preLabel->child_->currentNode_->locationID_] -
                preLabel->child_->reachTime_;
        preLabel->child_->parent_ = newPickLabel;
        newPickLabel->child_ = preLabel->child_;
        newPickLabel->parent_ = preLabel;
        preLabel->child_= newPickLabel;

        PGreedyLabel currentLabel = newPickLabel;
        while (currentLabel->child_ != nullptr) {
            currentLabel->child_->reachTime_ += deltaT;
            currentLabel->child_->nbPassengers_ += pickNode->nbPassengers_;
            currentLabel->child_->travelResource_ -= deltaT;
            currentLabel = currentLabel->child_;
        }
        float dropTime = tail_->reachTime_ + tail_->currentNode_->deltaTime_ +
                         durationMatrix_[tail_->currentNode_->locationID_][dropNode->locationID_];
        PGreedyLabel newDropLabel = std::make_shared<GreedyLabel>(dropNode, dropTime,
                                                                  tail_->nbPassengers_ +
                                                                  dropNode->nbPassengers_);
        head_ = newPickLabel;
        tail_->child_ = newDropLabel;
        newDropLabel->parent_ = tail_;
        tail_ = newDropLabel;
        newPickLabel->pair_ = newDropLabel;
        newDropLabel->pair_ = newPickLabel;
        newDropLabel->travelResource_ = maxDuration - newDropLabel->reachTime_ + newDropLabel->reachTime_ + pickNode->deltaTime_;
    }
    endTime_ = tail_->reachTime_;
    totalDelay_ += (pickTime - pickNode->requestTime_);
}

PRoute LinkedGreedyLabels::greedyLabelToRoute() {
    PRoute newRoute = std::make_shared<Route>((*Vehicle_)->vehicleID_);
    newRoute->addSource(source_->currentNode_, (*Vehicle_)->departTime_, (*Vehicle_)->numPassengers_);
    PGreedyLabel  currentLabel = source_->child_;
    while (currentLabel != nullptr) {
        newRoute->addNode(currentLabel->currentNode_);
        currentLabel = currentLabel->child_;
    }
    return newRoute;
}


