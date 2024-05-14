//
// Created by Ella on 2021-10-09.
//

#include "MasterAlgorithm.h"

//---------------------------------------------------------------------------------------------
//  Reduced Problem class
//  Build and solve the Reduced problem of the ISUD
//---------------------------------------------------------------------------------------------

MasterAlgorithm::MasterAlgorithm(InputPaths &inputPaths) {

    MasterPro_ = std::make_shared<MasterModeler>();
    objValue_ = 0;
    lpObjValue_ = 0;

    masterTime_ = new myTools::Timer(); masterTime_->init();
    MPBuildTime_ = new myTools::Timer(); MPBuildTime_->init();

    RMPCounter_ = 0;
    LPIter_ = 0;
    MIPIter_ = 0;
    SPIter_ = 0;

    nbRoutes_ = 0;
    minReducedCost_ = INFINITY;
    maxReducedCost_ = INFINITY;

}

MasterAlgorithm::~MasterAlgorithm() {
    delete masterTime_;
    delete MPBuildTime_;
}


void MasterAlgorithm::setObjValue() {
    objValue_ = 0.0;
    for (auto & routeObj : routeSolution_) {
        objValue_ += routeObj->totalBonus_;
    }
}

// this function create initial routes serving only one request and fill zSolution_ with available requests
// Reduced problem is also solved to initialized dual costs
void MasterAlgorithm::initialization(PInstance &pInst, InputPaths &inputPaths) {
    MPEpochSolveTime_ = 0;
    masterTime_->start();

    // Building models

    MasterPro_.reset();
    MasterPro_ = std::make_shared<MasterModeler>();
    RMPCounter_ = 0;
    SPIter_ = 0;

    for (auto &vehicleObj: pInst->vehicles_) {
        vehicleObj->setEmptyRoute();
        routeSolution_.push_back(vehicleObj->currentRoute_);
    }

    masterTime_->stop();
}


void MasterAlgorithm::updateReducedCosts(PInstance &pInst) {
    minReducedCost_ = INFINITY;
    maxReducedCost_ = INFINITY;
    for (auto & vehicleObj : pInst->vehicles_){
        for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]){
            routeObj->reducedCost_ =  vehicleObj->dual_ - routeObj->totalBonus_;
            for (auto & taskObj: routeObj->assignedTasks_){
                routeObj->reducedCost_ += taskObj->dual_;
            }
            if (minReducedCost_ > routeObj->reducedCost_)
                minReducedCost_ = routeObj->reducedCost_;
            if (!routeObj->assignedTasks_.empty())
                routeObj->score_ = routeObj->reducedCost_/routeObj->assignedTasks_.size();
            else
                routeObj->score_ = 0;
        }
        vehicleObj->bestReducedCost_ = minReducedCost_;
    }
    if (minReducedCost_ < 0)
        maxReducedCost_ = ((-0.5)*minReducedCost_);
}


void MasterAlgorithm::solveMP_CG(PInstance &pInst, int epoch, InputPaths &inputPaths, double subProTime) {
    masterTime_->start();
    setObjValue();
    double previousObj = objValue_;

    if (RMPCounter_ == 1) {
        MPBuildTime_->start();
        MasterPro_->routesToAdd_.clear();
        MasterPro_->buildModelMP(pInst, routeSolution_, pInst->nbVehicles_);
        MPBuildTime_->stop();
    }

    /************************************************************************************************/
    //                                     MASTER PROBLEM
    /************************************************************************************************/
    setAvailableTime();
    if (availableTime_ > 1) {
        double lpObj = previousObj;
        // solve RP with MIP solver
        while (true) {
            updateReducedCosts(pInst);
            setAvailableTime();
            if (minReducedCost_ >= 0 || availableTime_ < 0)
                break;

            solveMP_LP(pInst, inputPaths);
            MasterPro_->solveModelInt(pInst, routeSolution_, inputPaths, availableTime_, previousObj);
            LPIter_++;

            RMPCounter_++;

            if (lpObj < objValue_) {
                lpObj = objValue_;
            } else
                break;
        }


        setAvailableTime();
        if (availableTime_ < 1)
            availableTime_ = 2;
        // solve the model in Integer mode
        lpObjValue_ = lpObj;
        MasterPro_->solveModelInt(pInst, routeSolution_, inputPaths, availableTime_, previousObj);
        MIPIter_++;
        MPEpochSolveTime_ += MasterPro_->solveTime_->dSinceStart().count();
        setObjValue();

        RMPCounter_++;
    }

    for (auto & routeObj : routeSolution_) {
        pInst->vehicles_[routeObj->vehicleID_]->currentRoute_ = routeObj;
    }

    masterTime_->stop();
}



void MasterAlgorithm::solveMP_LP(PInstance &pInst, InputPaths &inputPaths) {
    MasterPro_->routesToAdd_.clear();

    // select routes with negative reduced costs
    updateRoutesToAdd(NR, pInst);

    if (!MasterPro_->routesToAdd_.empty()){
        MPBuildTime_->start();
        MasterPro_->updateModel();
        MPBuildTime_->stop();
        MasterPro_->solveModelLP(pInst, inputPaths);
        MPEpochSolveTime_ += MasterPro_->solveTime_->dSinceStart().count();
        objValue_ = MasterPro_->objValue_;
    }
}


// Display function
std::string MasterAlgorithm::toString() const {

    std::stringstream repStr;
    repStr << "#" << std::endl;
    repStr << std::left << std::fixed << std::setprecision(2);
    repStr << "# TOTAL OBJECTIVE (WAITING TIMES + PENALTIES) = " << objValue_ << std::endl;
    repStr << "#" << std::endl;

    repStr << "# TIME SPENT ON ISUD IMPROVEMENT              = " << masterTime_->dSinceStart().count() << " (s)" << std::endl;
    return repStr.str();
}

std::string MasterAlgorithm::toStringTimersTotal() const {
    std::stringstream repStr;
    repStr << std::left << std::fixed << std::setprecision(2);
    repStr << "#" << std::endl;
    repStr << "# -------------------   TOTAL ISUD RUN TIMES   -------------------" << std::endl;
    repStr << "#" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON ISUD IMPROVEMENT" << " = " << masterTime_->dSinceInit().count() << " (s)" << std::endl;

    return repStr.str();
}

std::string MasterAlgorithm::toStringTimersAvg(int epoch) const {
    std::stringstream repStr;
    repStr << std::left << std::fixed << std::setprecision(2);
    repStr << "#" << std::endl;
    repStr << "# -------------   AVERAGE ISUD RUN TIMES PER EPOCH   -------------" << std::endl;
    repStr << "#" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON ISUD IMPROVEMENT" << " = " << masterTime_->dSinceInit().count() / epoch << " (s)" << std::endl;

    return repStr.str();
}

void MasterAlgorithm::updateRoutesToAdd(selectionMode selectMode, PInstance &pInst){
    for (auto & vehicleObj : pInst->vehicles_) {
        std::stable_sort(availableRoutes_[vehicleObj->vehicleID_].begin(),
                         availableRoutes_[vehicleObj->vehicleID_].end(),
                         [](const PRoute &lhs, const PRoute &rhs) { return lhs->reducedCost_ < rhs->reducedCost_; });


        int numAdded = 0;
        for (auto & routeObj : availableRoutes_[vehicleObj->vehicleID_]) {
            if (!routeObj->mpAdded_ && routeObj->reducedCost_ < 0) {
                MasterPro_->routesToAdd_.push_back(routeObj);
                numAdded++;
            }
            if (numAdded > pInst->parameters_->nbColumn_)
                break;
        }
    }
}



std::string MasterAlgorithm::save_MPResults(int epoch, const std::string& model, int nbColumns, float reachTime,
                                              double subProTime, double auxObj) const {
    std::stringstream repStr;

    repStr << epoch << ",";
    repStr << RMPCounter_ << ",";
    repStr << SPIter_ << ",";
    repStr << nbRoutes_ << ",";
    repStr << nbColumns << ",";
    repStr << model << ",";
    repStr << objValue_ << ",";
    repStr << reachTime << ",";
    repStr << subProTime << ",";
    repStr << auxObj << "\n";
    return repStr.str();
}

void MasterAlgorithm::setAvailableTime() {
    availableTime_ = (int)(timeLimit_ - masterTime_->dSinceStart().count());
}
























