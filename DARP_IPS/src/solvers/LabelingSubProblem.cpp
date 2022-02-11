//
// Created by Ella on 1/28/2022.
//

#include "LabelingSubProblem.h"
#include <set>
#include <algorithm>

unsigned int Label::labelCount_ = 0;

Label::Label(PVehicle *vehicle, float reducedCost, PNode source) : labelID_(labelCount_++), vehicle_(vehicle), reducedCost_(reducedCost) {
    char* name2 = new char[255];
    strncpy(name2, std::to_string(labelID_).c_str(), 255);
    name_ = name2;

    load_ = (*vehicle)->numPassengers_;
    passedTime_ = (*vehicle)->departTime_;
    pathNodes_.push_back(source);
//    reducedCost_ = 0;
    totalDelay_ = 0;
    status_ = ACTIVE;
    openRequests_.clear();
    completedRequests_.clear();
    currentNode_ = &pathNodes_[0];
    nbPickUp_ = 0;
    for (auto & nodeID: (*vehicle)->onboards_)
        onboards_.insert(nodeID);
}

Label::Label(const Label &label) :labelID_(labelCount_++) {
    char* name2 = new char[255];
    strncpy(name2, std::to_string(labelID_).c_str(), 255);
    name_ = name2;
    status_ = ACTIVE;
    load_ = label.load_;
    passedTime_ = label.passedTime_;
    vehicle_ = label.vehicle_;
    openNodes_ = label.openNodes_;
    openReachTime_ = label.openReachTime_;
    pathNodes_ = label.pathNodes_;
    reducedCost_ = label.reducedCost_;
    currentNode_ = label.currentNode_;
    totalDelay_ = label.totalDelay_;
    openRequests_ = label.openRequests_;
    completedRequests_ = label.completedRequests_;
    onboards_ = label.onboards_;
    nbPickUp_ = label.nbPickUp_;
}
Label::~Label() {}

const unsigned int Label::getLabelId() const {
    return labelID_;
}

bool Label::operator () (const Label &rhs) const {
    return reducedCost_ < rhs.reducedCost_;
}

void Label::extend(PNode &outNode) {
    load_ += outNode->nbPassengers_;
//    float reachTime = passedTime_ + travelMat->queryTravelTime((*currentNode_), outNode) + (*currentNode_)->deltaTime_;
    float reachTime = passedTime_ + durationMatrix_[(*currentNode_)->locationID_][outNode->locationID_]+ (*currentNode_)->deltaTime_;
    if (outNode->type_ == DROPOFF) {
        if (onboards_.count(outNode->nodeID_)) {
            passedTime_ = reachTime;
            onboards_.erase(outNode->nodeID_);
        }
        else {
            openNodes_.erase(*outNode->pairNode_);
            openReachTime_.erase((*outNode->pairNode_)->nodeID_);
            openRequests_.erase(*outNode->related_Request_);
            completedRequests_.insert(*outNode->related_Request_);
            passedTime_ = reachTime;
        }
    }
    else if (outNode->type_ == PICKUP){
        openNodes_.insert(outNode);
        openRequests_.insert(*outNode->related_Request_);
        nbPickUp_ ++;
        if (reachTime < outNode->requestTime_){
            passedTime_ = outNode->requestTime_;
        }
        else {
            passedTime_ = reachTime;
            totalDelay_ += (reachTime - outNode->requestTime_);
            reducedCost_ -= (*outNode->related_Request_)->dual_;
            reducedCost_ += (reachTime - outNode->requestTime_);
        }
        openReachTime_.insert(std::pair<std::string, float> (outNode->nodeID_, passedTime_));
    }
    pathNodes_.push_back(outNode);
    currentNode_ = &outNode;
}

// this function check the feasibility of the label before extension
bool Label::isExtendFeasible(PNode &outNode, int maxPickUp) {
    if ((load_ + outNode->nbPassengers_) > (*vehicle_)->capacity_)
        return false;
    if (outNode->type_ == PICKUP) {
        if (nbPickUp_ == maxPickUp)
            return false;
        if ((completedRequests_.count(*outNode->related_Request_))||(openRequests_.count(*outNode->related_Request_)))
            return false;
    }
    if (outNode->type_ == DROPOFF) {
        if ((openRequests_.find(*outNode->related_Request_) == openRequests_.end())&&(!onboards_.count(outNode->nodeID_)))
            return false;
    }
    if (outNode->type_ == SINK) {
        if ((openRequests_.size() > 0)||(!onboards_.empty()))
            return false;
    }
    /*float reachTime = std::max((passedTime_ + travelMat->queryTravelTime((*currentNode_), outNode) +
            (*currentNode_)->deltaTime_), outNode->requestTime_);

    for (auto & nodeObj : openNodes_) {
        float travelDuration = reachTime - openReachTime_[nodeObj->nodeID_] + outNode->deltaTime_ +
                travelMat->queryTravelTime(outNode, *nodeObj->pairNode_);
        if (travelDuration > (*nodeObj->related_Request_)->maxTravelTime_){
            return false;
        }

    }*/
    return true;
}

bool Label::isDominated(PLabel &otherLabel, SubSolveStatus status) {
    /*if((*this->currentNode_)->type_ == SINK) {
        std::cout << "NOT REMOVE ROUTE BY DOMINATION -------------" << std::endl;
        std::cout << "DOMINATED: " << std::endl;
        std::cout << this->toString() << std::endl;
        std::cout << "DOMINANT: " << std::endl;
        std::cout << otherLabel->toString() << std::endl;
    }*/
    if (this->currentNode_ = otherLabel->currentNode_) {
        if (this->passedTime_ >= otherLabel->passedTime_) {
            if (this->reducedCost_ >= otherLabel->reducedCost_) {
                if ((this->openRequests_ == otherLabel->openRequests_)&&(this->onboards_ == otherLabel->onboards_)) {
                    if ((status == H2)||(status == H1H2)) {
                        return true;
                    }
                    else if ((status == EXACT)&&(this->completedRequests_.size() >= otherLabel->completedRequests_.size())) {
                    /*else if ((status == EXACT)&&(std::includes(this->completedRequests_.begin(),this->completedRequests_.end(),
                                      otherLabel->completedRequests_.begin(),otherLabel->completedRequests_.end()))) {*/
                        /*if((*this->currentNode_)->type_ == SINK) {
                            std::cout << "REMOVE ROUTE BY DOMINATION -------------" << std::endl;
                            std::cout << "DOMINATED: " << std::endl;
                            std::cout << this->toString() << std::endl;
                            std::cout << "DOMINANT: " << std::endl;
                            std::cout << otherLabel->toString() << std::endl;
                        }*/
                        return true;
                    }
                }
            }
        }
    }
    return false;
}
// this function examine the label to be sure that it leads to a route with negative reduced cost
bool Label::isEliminated(int maxPickUp, PGraph &graph) {
    for (auto & nodeObj : openNodes_) {
        /*float travelDuration = passedTime_ - openReachTime_[nodeObj->nodeID_] + (*currentNode_)->deltaTime_ +
                               travelMat->queryTravelTime((*currentNode_), *nodeObj->pairNode_);*/
        float travelDuration = passedTime_ - openReachTime_[nodeObj->nodeID_] + (*currentNode_)->deltaTime_ +
                durationMatrix_[(*currentNode_)->locationID_][(*nodeObj->pairNode_)->locationID_];
        if (travelDuration > (*nodeObj->related_Request_)->maxTravelTime_)
            return true;
    }
    for (auto & nodeID : onboards_) {
        /*float travelDuration = passedTime_ - (*graph->nodes_[nodeID]->related_Request_)->pickTime_ + (*currentNode_)->deltaTime_ +
                               travelMat->queryTravelTime((*currentNode_), graph->nodes_[nodeID]);*/
        float travelDuration = passedTime_ - (*graph->nodes_[nodeID]->related_Request_)->pickTime_ + (*currentNode_)->deltaTime_ +
                durationMatrix_[(*currentNode_)->locationID_][graph->nodes_[nodeID]->locationID_];
        if (travelDuration > (*graph->nodes_[nodeID]->related_Request_)->maxTravelTime_)
            return true;
    }
    /*if ((*currentNode_)->type_ != SINK) {
        float predictedReducedCost = reducedCost_;
        for (auto &nodeID : graph->intToNodeID_) {
            if (graph->nodes_[nodeID]->type_ == PICKUP) {
                if ((nbPickUp_ != maxPickUp) &&(!completedRequests_.count(*graph->nodes_[nodeID]->related_Request_))
                    &&(!openRequests_.count(*graph->nodes_[nodeID]->related_Request_))) {
                    float reachTime = passedTime_ + travelMat->queryTravelTime((*currentNode_), graph->nodes_[nodeID]) +
                                      (*currentNode_)->deltaTime_;
                    float addedValue = std::max(reachTime, graph->nodes_[nodeID]->requestTime_) - graph->nodes_[nodeID]->requestTime_ -
                                       (*graph->nodes_[nodeID]->related_Request_)->dual_;
                    if (addedValue < 0)
                        predictedReducedCost += addedValue;
                }
            }
        }
        if (predictedReducedCost < 0)
            return false;
        else
            return true;
    }*/
    return false;
}

PRoute Label::labelToRoute() {
    PRoute newRoute = std::make_shared<Route>((*vehicle_)->vehicleID_);
    newRoute->reducedCost_ = reducedCost_;
    newRoute->addSource(pathNodes_[0], (*vehicle_)->departTime_, (*vehicle_)->numPassengers_);
    for (int i = 1; i < pathNodes_.size()-1; ++i)
        newRoute->addNode(pathNodes_[i]);
    return newRoute;
}

std::string Label::toString() const {
    std::stringstream repStr;

    repStr << "#" << std::left << std::endl;
    repStr << "#\t" << std::setw(24) << "- LABEL INFO" << " : " << std::endl;
    repStr << "# \t" <<"_____________________" << std::endl;
    repStr << "#\t" << std::setw(24) << "- LABEL_NUMBER" << " : " << labelID_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- CURRENT_NODE" << " : " << (*currentNode_)->nodeID_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- PASSED_TIME (seconds)" << " : " << passedTime_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- NUMBER_OF_STOPS" << " : " << pathNodes_.size() << std::endl;
    repStr << "#\t" << std::setw(24) << "- TOTAL_WAITING (seconds)" << " : " << totalDelay_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- REDUCED_COST" << " : " << reducedCost_ << std::endl;
    repStr << "#" << std::endl;
    repStr << "#\t" << std::setw(24) << "- OPEN_REQUESTS" << " : " ;

    for (auto & requestObj : openRequests_) {
        repStr << requestObj->getRequestId() << "  ";
    }
    repStr << std::endl;

    repStr << "#\t" << std::setw(24) << "- COMPLETE_REQUESTS" << " : " ;

    for (auto & requestObj : completedRequests_) {
        repStr << requestObj->getRequestId() << "   ";
    }
    repStr << std::endl;
    repStr << "# ________________________________________________________________________" << std::endl;
    return repStr.str();
}





//-----------------------------------------------------------------------------
//  Labeling Sub problem class
//-----------------------------------------------------------------------------

LabelingSubProblem::LabelingSubProblem(PVehicle &vehicle) : Vehicle_(&vehicle) {
    subGraph_ = std::make_shared<Graph>();
    nbDominated_ = 0;
    nbEliminated_ = 0;
    nbGenerated_ = 0;
}

LabelingSubProblem::~LabelingSubProblem() {}

void LabelingSubProblem::initSubGraph(PInstance &pInst) {
    // create graph with source and sink
    subGraph_ = std::make_shared<Graph>(pInst->instGraph_->nodes_[(*Vehicle_)->departID_],
                                        pInst->instGraph_->nodes_[(*Vehicle_)->sinkID_]);

    // adding onboard nodes to the graph
    for (int i = 0; i < (*Vehicle_)->onboards_.size(); ++i) {
        subGraph_->addNewNode(pInst->instGraph_->nodes_[(*Vehicle_)->onboards_[i]]);
    }

    // adding available nodes based on the penalty
    for (auto & requestObj : pInst->requests_) {
        if ((requestObj->requestStatus_ == NO_ACTION)&&(requestObj->subStatus_ == NOTSELECTED)) {
            /*float minWait = (*Vehicle_)->departTime_ +
                            travelMat->queryTravelTime(pInst->instGraph_->nodes_[(*Vehicle_)->departID_],
                                                       pInst->instGraph_->nodes_[Tools::createNodeID(requestObj->getRequestId(), PICKUP)])
                            - requestObj->earlyPick_;*/
            float minWait = (*Vehicle_)->departTime_ +
                    durationMatrix_[pInst->instGraph_->nodes_[(*Vehicle_)->departID_]->locationID_]
                    [pInst->instGraph_->nodes_[Tools::createNodeID(requestObj->getRequestId(), PICKUP)]->locationID_]
                            - requestObj->earlyPick_;
            if (minWait <= requestObj->penalty_) {
                subGraph_->addNewNode(pInst->instGraph_->nodes_[Tools::createNodeID(requestObj->getRequestId(), PICKUP)]);
                subGraph_->addNewNode(pInst->instGraph_->nodes_[Tools::createNodeID(requestObj->getRequestId(), DROPOFF)]);

                // adding available requests
                subRequests_.push_back(requestObj);
            }
        }
    }
}

// this function sort the list of nodes based of their dual values
void LabelingSubProblem::sortNodes() {
    subGraph_->nodeIDToInt_.clear();
    subGraph_->intToNodeID_.clear();

    // sorting requests based on their dual values
    sort(subRequests_.begin(),subRequests_.end(),[](const PRequest &lhs, const PRequest &rhs){
        return lhs->dual_ > rhs->dual_;});

    // add pickup nodes
    for (auto &requestObj : subRequests_) {
        std::string pickID = Tools::createNodeID(requestObj->getRequestId(), PICKUP);
        subGraph_->nodeIDToInt_[pickID] = subGraph_->intToNodeID_.size();
        subGraph_->intToNodeID_.push_back(pickID);
    }
    // add drop off points
    for (auto &requestObj : subRequests_) {
        std::string dropID = Tools::createNodeID(requestObj->getRequestId(), DROPOFF);
        subGraph_->nodeIDToInt_[dropID] = subGraph_->intToNodeID_.size();
        subGraph_->intToNodeID_.push_back(dropID);
    }

    // adding onboard nodes
    for (int i = 0; i < (*Vehicle_)->onboards_.size(); ++i) {
        subGraph_->nodeIDToInt_[(*Vehicle_)->onboards_[i]] = subGraph_->intToNodeID_.size();
        subGraph_->intToNodeID_.push_back((*Vehicle_)->onboards_[i]);
    }

    // adding sink node
    subGraph_->nodeIDToInt_[(*Vehicle_)->sinkID_] = subGraph_->intToNodeID_.size();
    subGraph_->intToNodeID_.push_back((*Vehicle_)->sinkID_);
}

void LabelingSubProblem::solveDynamic(int maxPickUp) {
    // create initial label
    SubSolveStatus status = H1H2;
    int MAXLABEL = 300;
    while(true) {
        for (auto &nodeID : subGraph_->intToNodeID_) {
            subGraph_->nodes_[nodeID]->activeLabels_.clear();
        }
        workingLabels.emplace(std::make_shared<Label>(Vehicle_,-(*Vehicle_)->dual_,
                                                      subGraph_->nodes_[(*Vehicle_)->departID_]));
        sortNodes();
        int iter = 0;
        bool safeToAdd = true;
        while (!workingLabels.empty()) {
            PLabel currentLabel = workingLabels.top();
            workingLabels.pop();
            for (auto &nodeID : subGraph_->intToNodeID_) {
                if (currentLabel->isExtendFeasible(subGraph_->nodes_[nodeID], maxPickUp)) {
                    PLabel newLabel = std::make_shared<Label>(*currentLabel);
                    newLabel->extend(subGraph_->nodes_[nodeID]);
                    nbGenerated_++;
                    if (!newLabel->isEliminated(maxPickUp, subGraph_)) {
                        safeToAdd = true;
                        if (subGraph_->nodes_[nodeID]->activeLabels_.empty()) {
                            subGraph_->nodes_[nodeID]->activeLabels_.push_back(newLabel);
                            if ((*newLabel->currentNode_)->type_ != SINK)
                                workingLabels.push(newLabel);
                            //             std::cout << newLabel->toString() << std::endl;
                        }
                        else {
                            int breakIndex = subGraph_->nodes_[nodeID]->getLabelListIndex(newLabel);
                            for (int i = 0; i < breakIndex; ++i) {
                                if (newLabel->isDominated(subGraph_->nodes_[nodeID]->activeLabels_[i], status)){
                                    nbDominated_++;
                                    safeToAdd = false;
                                    break;
                                }
                            }
                            if (safeToAdd) {
                                if ((breakIndex <= MAXLABEL-1) || (status == EXACT) || (status == H2)) {
                                    for (int i = subGraph_->nodes_[nodeID]->activeLabels_.size()-1; i >= breakIndex; --i){
                                        if (subGraph_->nodes_[nodeID]->activeLabels_[i]->isDominated(newLabel, status)) {
                                            //      dominatedLabels_.insert(subGraph_->nodes_[nodeID]->activeLabels_[i]);
                                            nbDominated_++;
                                            subGraph_->nodes_[nodeID]->activeLabels_[i]->status_ = DOMINATED;
                                            subGraph_->nodes_[nodeID]->activeLabels_.erase(subGraph_->nodes_[nodeID]->activeLabels_.begin() + i);
                                        }
                                    }
                                    subGraph_->nodes_[nodeID]->activeLabels_.insert(subGraph_->nodes_[nodeID]->activeLabels_.begin()+breakIndex, newLabel);
                                    if ((*newLabel->currentNode_)->type_ != SINK)
                                        workingLabels.push(newLabel);
                                    if ((status == H1)||(status == H1H2)) {
                                        subGraph_->nodes_[nodeID]->activeLabels_.back()->status_ = DOMINATED;
                                        subGraph_->nodes_[nodeID]->activeLabels_.pop_back();
                                    }
                                    //                    std::cout << newLabel->toString() << std::endl;
                                }

                            }
                        }
                    }
                    else
                        nbEliminated_++;
                }
            }
            /*while(!workingLabels.empty() && dominatedLabels_.count(workingLabels.top())) {
                dominatedLabels_.erase(workingLabels.top());
                workingLabels.pop();
            }*/
            while(!workingLabels.empty() && workingLabels.top()->status_ == DOMINATED) {
                workingLabels.pop();
            }
        }
        /*for (auto &nodeID : subGraph_->intToNodeID_) {
            for (auto &labelObj : subGraph_->nodes_[nodeID]->activeLabels_) {
                if (labelObj->isExtendFeasible(subGraph_->nodes_[(*Vehicle_)->sinkID_], maxPickUp)) {
                    PLabel newLabel = std::make_shared<Label>(*labelObj);
                    newLabel->extend(subGraph_->nodes_[(*Vehicle_)->sinkID_]);
                    subGraph_->nodes_[(*Vehicle_)->sinkID_]->activeLabels_.push_back(newLabel);
                }
            }
            subGraph_->nodes_[nodeID]->activeLabels_.clear();
        }*/
        // sort final labels based on reduced cost
        sort(subGraph_->nodes_[(*Vehicle_)->sinkID_]->activeLabels_.begin(),
             subGraph_->nodes_[(*Vehicle_)->sinkID_]->activeLabels_.end(),[]
                     (const PLabel &lhs, const PLabel &rhs){
                    return std::tie(lhs->reducedCost_,lhs->totalDelay_) < std::tie(rhs->reducedCost_,rhs->totalDelay_);
                });
        if (status != EXACT) {
            if (subGraph_->nodes_[(*Vehicle_)->sinkID_]->activeLabels_.empty() ||
            (subGraph_->nodes_[(*Vehicle_)->sinkID_]->activeLabels_[0]->reducedCost_ >= -0.0001)) {
                status = EXACT;
            }
            else
                break;
        }
        else
            break;

    }
    std::cout << toString() << std::endl;
}

void LabelingSubProblem::SolutionToRoutes(vector<PRoute> &availableRoutes, std::map<std::string, PRoute> &generatedRoutes) {
    for (auto & labelObj : subGraph_->nodes_[(*Vehicle_)->sinkID_]->activeLabels_) {
        if (labelObj->reducedCost_ < 0) {
            bool isRepeated = false;
            /*PRoute newRoute = std::make_shared<Route>((*Vehicle_)->vehicleID_);
            newRoute->reducedCost_ = labelObj->reducedCost_;
            newRoute->addSource(subGraph_->nodes_[(*Vehicle_)->departID_],
                                (*Vehicle_)->departTime_, (*Vehicle_)->numPassengers_);
            for (int i = 1; i < labelObj->pathNodes_.size(); ++i)
                newRoute->addNode(labelObj->pathNodes_[i]);*/
            PRoute newRoute = labelObj->labelToRoute();
            for (int r = 0; r < availableRoutes.size(); ++r) {
                if (newRoute == availableRoutes[r]) {
                    isRepeated = true;
                    break;
                }
            }
            if (!isRepeated) {
                availableRoutes.push_back(newRoute);
                generatedRoutes.insert(std::pair <std::string , PRoute> (newRoute->name_ , newRoute));
            }
        }
    }
    subGraph_->nodes_[(*Vehicle_)->sinkID_]->activeLabels_.clear();
    if (!availableRoutes.empty())
        for (auto & nodeObj: availableRoutes[0]->routeNodes_) {
            if (nodeObj->type_ == PICKUP) {
                (*nodeObj->related_Request_)->subStatus_ = SELECTED;
            }
        }
}

std::string LabelingSubProblem::toString() const {
    std::stringstream repStr;
    repStr << std::endl;
    repStr << "# ------------------------------------------------------------" << std::endl;
    repStr << "# SUB PROBLEM SOLUTION RESULT FOR VEHICLE: " << (*Vehicle_)->vehicleID_ << std::endl;
    repStr << "#" << std::endl;
    repStr << "# Solution status = " << "Optimal (Label Setting Algorithm)" << std::endl;
    repStr << "# In total, " << nbGenerated_ << " labels were generated." << std::endl;
    repStr << "# " << nbDominated_ << " labels were removed via Domination " << std::endl;
    repStr << "# " << nbEliminated_ << " labels were removed via Elimination " << std::endl;
    if (!subGraph_->nodes_[(*Vehicle_)->sinkID_]->activeLabels_.empty())
        repStr << "# best objective value = " << subGraph_->nodes_[(*Vehicle_)->sinkID_]->activeLabels_[0]->reducedCost_ << std::endl;
    repStr << "# The solution pool contains = " << subGraph_->nodes_[(*Vehicle_)->sinkID_]->activeLabels_.size() << " solutions." << std::endl;

    int nbValidSolution = 0;
    for (auto & labelObj : subGraph_->nodes_[(*Vehicle_)->sinkID_]->activeLabels_) {
        if (labelObj->reducedCost_ < 0)
            nbValidSolution ++;
    }
    repStr << "# The solution pool contains = " << nbValidSolution << " solutions with negative reduced cost." << std::endl;
    return repStr.str();
}


