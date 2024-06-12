//
// Created by Ella on 2/22/2022.
//

#include "SubproModeler.h"

SubproModeler::SubproModeler(PVehicle &vehicle) : Vehicle_(&(*vehicle)) {
    subGraph_ = std::make_shared<Graph>();
    nbNegativeColumns_ = 0;
    nbTotalRequest_ = 0;
}

SubproModeler::~SubproModeler() {
}

// initialization of the subgraph
void SubproModeler::initSubGraph(PInstance &pInst) {
    // adding source and sink
    nbTotalRequest_ = pInst->nbTasks_;
    subGraph_->addNewNode(std::make_shared<Node>((Vehicle_)->departNode_));
    subGraph_->addNewNode(std::make_shared<Node>(pInst->instGraph_->sink_));

    // adding available nodes based on the penalty
    for (int i = 0; i < pInst->tasks_.size(); i++) {
        if (pInst->tasks_[i]->nbTransfer_ != 0 && pInst->tasks_[i]->relocateBonus_ > 0) {
            subTasks_.push_back(pInst->tasks_[i]);
            subGraph_->addNewNode(std::make_shared<Node>(pInst->instGraph_->taskNodes_[i]));
            subGraph_->taskNodes_.back()->travelTimeFromSource_ = pInst->getDurationMatrix()[(Vehicle_)->departNode_->locationIndex_
            ][subGraph_->taskNodes_.back()->locationIndex_];
        }
    }

    subGraph_->nbNodes_ = subGraph_->taskNodes_.size() + 2;
}



