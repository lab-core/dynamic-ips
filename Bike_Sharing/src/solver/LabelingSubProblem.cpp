//
// Created by Ella on 1/28/2022.
//

#include "LabelingSubProblem.h"

#include <utility>



//-----------------------------------------------------------------------------
//  Labeling Sub problem class
//-----------------------------------------------------------------------------


LabelingSubProblem::LabelingSubProblem(PVehicle &vehicle, PSolverOption solverOptions) :
                                        SubproModeler(vehicle), solverOptions_(std::move(solverOptions)) {
    nbDominated_ = 0;
    nbEliminated_ = 0;
    nbGenerated_ = 0;
    nbOutputs_ = 0;
    maxPickup_ = solverOptions_->nbStop_;
}


// reset that active lists of the nodes, create the first label at the source, add onboards
void LabelingSubProblem::initialization() {


    // create the initial label at the source and add the source to the list active nodes
    PLabel initialLabel = std::make_shared<Label>(Vehicle_, Vehicle_->departNode_);
    initialNodeID_ = Vehicle_->departIndex_;

    initialLabel->pathNode_.back()->nbActiveLabels_++;
    initialLabel->pathNode_.back()->bestLabelReduceCost_ = initialLabel->reducedCost_;
    activeNodes_.clear();
    activeNodes_.push_back(initialLabel->pathNode_.back());
    initialLabel->pathNode_.back()->activeLabels_.push_back(std::move(initialLabel));
}


void LabelingSubProblem::labelExtend(PLabel &parentLabel, Node *outNode, bool Terminate) {
    PLabel newLabel;
    if (!labelPool_.empty()){
        newLabel = std::move(labelPool_.back());
        labelPool_.pop_back();
        newLabel->copyLabel(*parentLabel);
    }
    else {
        newLabel = std::make_shared<Label>(*parentLabel);
    }

    newLabel->extend(outNode);

    nbGenerated_++;
    if (!isLabelAdded(newLabel, outNode, Terminate))
        nbDominated_++;
}

bool LabelingSubProblem::isLabelAdded(PLabel &newLabel, Node *outNode, bool Terminate) {
    // check the dominance state of the new label
    for (auto & labelObj: outNode->activeLabels_){
        if (newLabel->isDominated(labelObj, this->solverOptions_)){
            newLabel->status_ = DOMINATED;
            labelPool_.push_back(std::move(newLabel));
            return false;
        }
    }
    // remove previous dominated labels
    for (int i = outNode->activeLabels_.size()-1; i >= 0; i--){
        if (outNode->activeLabels_[i]->isDominated(newLabel, this->solverOptions_)){
            if (outNode->activeLabels_[i]->status_ == ACTIVE){
                outNode->nbActiveLabels_--;
            }
            outNode->activeLabels_[i]->status_ = DOMINATED;
            labelPool_.push_back(std::move(outNode->activeLabels_[i]));
            outNode->activeLabels_.erase(outNode->activeLabels_.begin() + i);
            this->nbDominated_++;
        }
    }
    if (outNode->bestLabelReduceCost_ > newLabel->reducedCost_)
        outNode->bestLabelReduceCost_ = newLabel->reducedCost_;

    if (outNode->type_ == SINK_NODE){
        newLabel->status_ = TERMINATED;
        outNode->activeLabels_.push_back(std::move(newLabel));
    }
    else{
        outNode->activeLabels_.push_back(newLabel);
        outNode->nbActiveLabels_++;
        if (Terminate)
            labelExtend(newLabel, &(*subGraph_->sink_), false);
    }
    return true;
}

void LabelingSubProblem::solveDynamic_pulling() {
    // create initial label
    int nbActive;
    while(true) {
        // create initial label
        initialization();

        while (!activeNodes_.empty()) {
            // select a node to pull other labels to it
            for (auto &currentNode: subGraph_->taskNodes_) {
                std::stable_sort(activeNodes_.begin(),activeNodes_.end(),[](const Node *lhs, const Node *rhs){
                    return lhs->travelTimeFromSource_ > rhs->travelTimeFromSource_;});
                for (int j = activeNodes_.size()-1; j >= 0; j--) {
                    if (activeNodes_[j]->nbActiveLabels_ == 0)
                        activeNodes_.erase(activeNodes_.begin() + j);
                    else {
                        for (int l = activeNodes_[j]->activeLabels_.size() - 1; l >= 0; l--) {
                            if (activeNodes_[j]->activeLabels_[l]->status_ == ACTIVE){
                                PLabel selectedLabel = activeNodes_[j]->activeLabels_[l];
                                if (selectedLabel->numExtendCheck_ == subGraph_->taskNodes_.size() ||
                                    (selectedLabel->nbStops_ >= maxPickup_)) {
                                    selectedLabel->status_ = INACTIVE;
                                    activeNodes_[j]->nbActiveLabels_--;
                                    if (activeNodes_[j]->nbActiveLabels_ == 0) {
                                        activeNodes_.erase(activeNodes_.begin() + j);
                                        break;
                                    }
                                }
                                // pull all labels to the current node
                                else if ((!selectedLabel->extendCheck_.test(currentNode->locationIndex_)) &&
                                         (selectedLabel->isExtendFeasible(&(*currentNode), maxPickup_, Vehicle_->capacity_))) {
                                    nbActive = currentNode->nbActiveLabels_;
                                    labelExtend(selectedLabel, &(*currentNode), true);

                                    if ((currentNode->nbActiveLabels_ == 1) && (nbActive == 0)) {
                                        activeNodes_.push_back(&(*currentNode));
                                    }
                                }
                            }
                        }
                    }
                }

                // decrease the number of active labels if truncated strategy is used
                if ((solverOptions_->isTruncated_) && (currentNode->nbActiveLabels_ > solverOptions_->MaxLabel_)){
                    truncateLabelList(&(*currentNode), solverOptions_->MaxLabel_, labelPool_);
                }
            }
        }
        break;
    }
}

//***************************************************************************************//
//                           P U S H I N G  S T R A T E G Y
//***************************************************************************************//
void LabelingSubProblem::solveDynamic_pushing() {
    int nbActive;
    while(true) {
        // create initial label
        initialization();

        while (!activeNodes_.empty()) {
            std::stable_sort(activeNodes_.begin(),activeNodes_.end(),[](const Node *lhs, const Node *rhs){
                return lhs->travelTimeFromSource_ < rhs->travelTimeFromSource_;});

            // select a node to extend active labels
            Node *currentNode = activeNodes_.back();
            activeNodes_.pop_back();

            // decrease the number of active labels if truncated strategy is used
            if ((solverOptions_->isTruncated_) && (currentNode->nbActiveLabels_ > solverOptions_->MaxLabel_)){
                truncateLabelList(currentNode, solverOptions_->MaxLabel_, labelPool_);
            }
            for (int j = currentNode->activeLabels_.size()-1; j >=0; j--) {
                if (currentNode->activeLabels_[j]->status_ == ACTIVE) {
                    PLabel selectedLabel = currentNode->activeLabels_[j];
                    currentNode->nbActiveLabels_--;
                    selectedLabel->status_ = INACTIVE;

                    // push to pickup points
                    if (selectedLabel->load_ < Vehicle_->capacity_) {
                        for (auto &neighbourNode: selectedLabel->pathNode_.back()->successors_) {
                            if (selectedLabel->isExtendFeasible(neighbourNode, maxPickup_,
                                                                Vehicle_->capacity_)) {
                                nbActive = neighbourNode->nbActiveLabels_;
                                labelExtend(selectedLabel, neighbourNode, true);
                                if ((neighbourNode->nbActiveLabels_ == 1) && (nbActive == 0)) {
                                    activeNodes_.push_back(neighbourNode);
                                }
                            }
                        }
                    }
                }
            }
        }
        break;
    }
}


//***************************************************************************************//
//                           P U L L I N G  S T R A T E G Y
//***************************************************************************************//

void LabelingSubProblem::solveDynamic() {
    this->solveDynamic_pulling();

    nbNegativeColumns_ = 0;
    for (auto & labelObj : subGraph_->sink_->activeLabels_) {
        if (labelObj->reducedCost_ - (Vehicle_)->dual_ < 0)
            nbNegativeColumns_ ++;
    }
//    subproTime_->stop();
}


void LabelingSubProblem::solutionSummery(vector<int> &subProResults) {
    if (subProResults.empty()) {
        subProResults.push_back(nbGenerated_);
        subProResults.push_back(nbGenerated_);
        subProResults.push_back(nbDominated_);
        subProResults.push_back(nbEliminated_);
        subProResults.push_back(subGraph_->nbNodes_);
        subProResults.push_back(subGraph_->taskNodes_.size());
    }
    else
    {
        subProResults[0] += nbOutputs_;
        subProResults[1] += nbGenerated_;
        subProResults[2] += nbDominated_;
        subProResults[3] += nbEliminated_;
        subProResults[4] += subGraph_->nbNodes_;
        subProResults[5] += subGraph_->taskNodes_.size();
    }
}

std::string LabelingSubProblem::toString() const {
    std::stringstream repStr;
    repStr << std::endl;
    repStr << "# ------------------------------------------------------------" << std::endl;
    repStr << "# SUB PROBLEM SOLUTION RESULT FOR VEHICLE: " << (Vehicle_)->vehicleID_ << std::endl;
    repStr << "#" << std::endl;
    repStr << "# Solution status = " << "Optimal (Label Setting Algorithm)" << std::endl;
    repStr << "# In total, " << nbGenerated_ << " labels were generated." << std::endl;
    repStr << "# " << nbDominated_ << " labels were removed via Domination " << std::endl;
    repStr << "# " << nbEliminated_ << " labels were removed via Elimination " << std::endl;
    if (!subGraph_->sink_->activeLabels_.empty())
        repStr << "# best objective value = " << subGraph_->sink_->bestLabelReduceCost_ - (Vehicle_)->dual_ << std::endl;
    repStr << "# The solution pool contains = " << subGraph_->sink_->activeLabels_.size() << " solutions." << std::endl;

    repStr << "# The solution pool contains = " << nbNegativeColumns_ << " solutions with negative reduced cost." << std::endl;
    return repStr.str();
}

std::string LabelingSubProblem::toStringOut(int epoch) const {
    std::stringstream repStr;
    repStr << epoch << ",";
    repStr << (Vehicle_)->vehicleID_ << ",";
    repStr << subTasks_.size() << ",";
    repStr << subGraph_->nbNodes_-2 << ",";
    repStr << maxPickup_ << ",";
    repStr << nbGenerated_ << ",";
    repStr << nbEliminated_ << ",";
    repStr << nbDominated_ << ",";
//    repStr << nbOutputs_ << ",";
    repStr << nbOutputs_ << "\n";
    /*repStr << subproTime_->dSinceInit().count();*/
    return repStr.str();
}



void LabelingSubProblem::truncateLabelList(Node *node, int MaxLabel, std::vector<PLabel> & labelPool) {
    if (solverOptions_->pathSort_ == RD_COST) {
        std::stable_sort(node->activeLabels_.begin(), node->activeLabels_.end(),
                         [](const PLabel &lhs, const PLabel &rhs) {
                             return lhs->reducedCost_ < rhs->reducedCost_;
                         });
    }
    else {
        std::stable_sort(node->activeLabels_.begin(),node->activeLabels_.end(),[](const PLabel &lhs, const PLabel &rhs){
            return lhs->labelScore_ < rhs->labelScore_;});
    }


    for (int i = node->activeLabels_.size()-1; i >=0; i--){
        if (node->nbActiveLabels_ <= MaxLabel)
            break;
        else {
            if (node->activeLabels_[i]->status_ == ACTIVE){
                node->nbActiveLabels_--;
                node->activeLabels_[i]->status_ = DOMINATED;
                labelPool.push_back(std::move(node->activeLabels_[i]));
                node->activeLabels_.erase(node->activeLabels_.begin() + i);
            }
        }
    }
}

void LabelingSubProblem::SolutionToRoutes(PVehicle &vehicle, vector<PRoute> &availableRoutes, PInstance &pInst) {
    for (auto & labelObj : subGraph_->sink_->activeLabels_) {
        availableRoutes.emplace_back(labelObj->labelToRoute(vehicle));
        nbOutputs_++;
    }
    for (auto & nodeObj : subGraph_->taskNodes_){
        for (auto & labelObj : nodeObj->activeLabels_)
            labelPool_.push_back(std::move(labelObj));
    }
}




