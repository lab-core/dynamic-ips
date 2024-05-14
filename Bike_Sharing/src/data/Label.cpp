//
// Created by Ella on 2/22/2022.
//

#include "Label.h"

unsigned int Label::labelCount_ = 0;

Label::Label(Vehicle *vehicle, PNode &source) : labelID_(labelCount_++) {
    char* name2 = new char[255];
    strncpy(name2, std::to_string(labelID_).c_str(), 255);
    name_ = name2;

    load_ = vehicle->bikeLoad_;
    passedTime_ = vehicle->readyTime_;
    reachedTime_ = vehicle->readyTime_;
    pathNode_.push_back(&(*source));
    reducedCost_ = 0;
    labelScore_ = 0;
    totalBonus_ = 0;
    status_ = ACTIVE;

    completeTasks_.reset();
    extendCheck_.reset();
    this->numCompleted_ = 0;
    numExtendCheck_ = 0;
    nbStops_ = 0;
 //   nbPickMove_ = 0;
    isDropped_ = false;
}

Label::Label(const Label &label) :labelID_(labelCount_++) {
    char* name2 = new char[255];
    strncpy(name2, std::to_string(labelID_).c_str(), 255);
    name_ = name2;
    status_ = ACTIVE;
    load_ = label.load_;
    passedTime_ = label.passedTime_;
    reachedTime_ = label.reachedTime_;
    pathNode_ = label.pathNode_;
    reducedCost_ = label.reducedCost_;
    labelScore_ = label.labelScore_;
    totalBonus_ = label.totalBonus_;
    completeTasks_ = label.completeTasks_;
    extendCheck_ = label.completeTasks_;
    numCompleted_ = label.numCompleted_;
    numExtendCheck_ = label.numCompleted_;
    nbStops_ = label.nbStops_;
    isDropped_ = label.isDropped_;
}
void Label::copyLabel(const Label &label) {
    status_ = ACTIVE;
    load_ = label.load_;
    passedTime_ = label.passedTime_;
    reachedTime_ = label.reachedTime_;
    pathNode_ = label.pathNode_;
    reducedCost_ = label.reducedCost_;
    labelScore_ = label.labelScore_;
    totalBonus_ = label.totalBonus_;
    completeTasks_ = label.completeTasks_;
    extendCheck_ = label.completeTasks_;
    numCompleted_ = label.numCompleted_;
    numExtendCheck_ = label.numCompleted_;

    nbStops_ = label.nbStops_;
 //   nbPickMove_ = label.nbPickMove_;
    isDropped_ = label.isDropped_;
}
Label::~Label() {
    delete[] name_;
}

unsigned int Label::getLabelId() const {
    return labelID_;
}

bool Label::operator () (const Label &rhs) const {
    return reducedCost_ < rhs.reducedCost_;
}

void Label::extend(Node *outNode) {
    load_ += outNode->load_;
    float travelTime =  durationMatrix_[pathNode_.back()->locationIndex_][outNode->locationIndex_];
    reachedTime_ = passedTime_ + travelTime;
    if (outNode->load_ < 0)
        isDropped_ = true;
    completeTasks_.set(outNode->locationIndex_, true);
    numCompleted_++;
    extendCheck_.set(outNode->locationIndex_, true);
    numExtendCheck_++;

    nbStops_++;
    if (outNode->type_ != SINK_NODE){
        reducedCost_ += outNode->relatedTask_->dual_ - outNode->relatedTask_->relocateBonus_;
        totalBonus_ += (outNode->relatedTask_->relocateBonus_);
        passedTime_ = reachedTime_ + TimeToPark + abs(outNode->relatedTask_->nbTransfer_)*TimePerBike;
    }
    else
        passedTime_ = reachedTime_ + TimeToPark;

    labelScore_ = reducedCost_ / nbStops_;


    pathNode_.push_back(outNode);
}

// this function check the feasibility of the label before extension
bool Label::isExtendFeasible(Node *outNode, int maxStops, int capacity) {
    extendCheck_.set(outNode->locationIndex_, true);
    numExtendCheck_++;
    // check tha capacity of vehicle
    if ((load_ + outNode->load_) > capacity)
        return false;
    if (nbStops_ >= maxStops)
        return false;
    if (completeTasks_.test(outNode->locationIndex_))
        return false;

    return true;
}


bool Label::isDominated(PLabel &otherLabel, PSolverOption &solverOption) const {
    if (pathNode_.back() != otherLabel->pathNode_.back())
        throw myTools::myException("Label Domination error!!", __FILE__, __LINE__);

    if (this->passedTime_ >= otherLabel->passedTime_) {
        if (this->reducedCost_ >= otherLabel->reducedCost_) {
            if (this->numCompleted_ >= otherLabel->numCompleted_) {
                if ((otherLabel->completeTasks_ & this->completeTasks_) == otherLabel->completeTasks_){
                    return true;
                }
            }
        }
    }
    return false;
}

PRoute Label::labelToRoute(PVehicle &vehicle) {
    PRoute newRoute = std::make_shared<Route>(vehicle->vehicleID_);
    newRoute->reducedCost_ = reducedCost_ - vehicle->dual_;
    if (vehicle->departNode_ == nullptr)
        std::cout <<"HI";
    newRoute->addSource(vehicle->departNode_, vehicle->readyTime_, vehicle->bikeLoad_);

    for (int i = 1; i < pathNode_.size()-1; ++i) {
        newRoute->addTask(pathNode_[i]->relatedTask_);
    }

    if (!newRoute->assignedTasks_.empty())
        newRoute->score_ = reducedCost_/newRoute->assignedTasks_.size();
    else
        newRoute->score_ = 0;
    newRoute->totalDuration_ = newRoute->routeStops_.back()->departTime_ - vehicle->readyTime_;
    return newRoute;
}

std::string Label::toString() const {
    std::stringstream repStr;

    repStr << "#" << std::left << std::endl;
    repStr << "#\t" << std::setw(24) << "- LABEL INFO" << " : " << std::endl;
    repStr << "# \t" <<"_____________________" << std::endl;
    repStr << "#\t" << std::setw(24) << "- LABEL_NUMBER" << " : " << labelID_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- CURRENT_NODE" << " : " << pathNode_.back()->locationIndex_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- PASSED_TIME (seconds)" << " : " << passedTime_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- NUMBER_OF_STOPS" << " : " << pathNode_.size() << std::endl;
    repStr << "#\t" << std::setw(24) << "- TOTAL_WAITING (seconds)" << " : " << totalBonus_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- REDUCED_COST" << " : " << reducedCost_ << std::endl;
    repStr << "#" << std::endl;

    repStr << "#\t" << std::setw(24) << "- PATH_NODES" << " : ";
    for (auto &nodeObj: pathNode_) {
        repStr << (nodeObj)->locationIndex_ << "  ";
    }
    repStr << std::endl;

    repStr << std::endl;
    repStr << "# ________________________________________________________________________" << std::endl;
    return repStr.str();
}





