//
// Created by Elahe Amiri on 2022-10-13.
//

#include "Parameters.h"

//************************************************************************
//                     FUNCTIONS FOR PARAMETERS CLASS
//************************************************************************

// Constructor and Destructor
Parameters::Parameters(float alphaParam, float betaParam, float deltaPram, int epochLength, int penaltyL,
                       float committedTime, int nbThreads, InitialDual initialDual, MainAlgorithm mainAlgorithm,
                       bool oneIter, bool greedyReOptimize,
                       warmStart initialStart, int MIP_maxIncDegree, int CP_IncDegree, bool useMultiStage, float minImp,
                       bool useZoom, bool isTruncated, int maxLabel, bool isSuccessorsLimited, bool isDominanceReleased,
                       bool isDropPickPossible, SubProSolveMode subproSolveMode, LabelingStrategy LabelingStrategy,
                       subproblemAlgorithm subAlgorithm, float vehicle_portion, bool greedyPortion, bool usePick, int bigM,
                       int solveTimeLimit, int populateTimeLimit, bool addOneRequestColumn, SolutionMode solutionMode):
        alphaParam_(alphaParam), betaParam_(betaParam), deltaPram_(deltaPram), epochLength_(epochLength),
        penaltyL_(penaltyL), committedTime_(committedTime), nbThreads_(nbThreads), initialDual_(initialDual),
        mainAlgorithm_(mainAlgorithm), oneIter_(oneIter), greedyReOptimize_(greedyReOptimize),
        initialStart_(initialStart), MIP_maxIncDegree_(MIP_maxIncDegree),
        CP_IncDegree_(CP_IncDegree), useMultiStage_(useMultiStage), minImp_(minImp), useZoom_(useZoom),
        isTruncated_(isTruncated), MaxLabel_(maxLabel), isSuccessorsLimited_(isSuccessorsLimited),
        isDominanceReleased_(isDominanceReleased), isDropPickPossible_(isDropPickPossible),
        SubproSolveMode_(subproSolveMode), LabelingStrategy_(LabelingStrategy), subAlgorithm_(subAlgorithm),
        vehicle_portion_(vehicle_portion), greedyPortion_(greedyPortion), usePick_(usePick), bigM_(bigM),
        solveTimeLimit_(solveTimeLimit), populateTimeLimit_(populateTimeLimit),
        addOneRequestColumn_(addOneRequestColumn), solutionMode_(solutionMode) {
}

Parameters::~Parameters() = default;

// Display function
std::string Parameters::toString() const {
    std::stringstream repStr;
    repStr << "# MODEL PARAMETERS" << std::endl;
    repStr << "#" << std::endl;
    int setwLength = 30;
    repStr << std::left << std::fixed << std::setprecision(1) << std::boolalpha;
    repStr << std::setw(setwLength) << "# alpha Parameter " << " = " << alphaParam_ << std::endl;
    repStr << std::setw(setwLength) << "# beta Parameter " << " = " << betaParam_ << std::endl;
    repStr << std::setw(setwLength) << "# delta Parameter" << " = " << deltaPram_ << std::endl;
    if (solutionMode_ == DYNAMIC)
        repStr << std::setw(setwLength) << "# epoch Length " << " = " << epochLength_ << " (s)" << std::endl;
    if (solutionMode_ == ANYTIME)
        repStr << std::setw(setwLength) << "# time to commit stops " << " = " << committedTime_ << " (s)" << std::endl;
    repStr << std::setw(setwLength) << "# penalty epoch Length " << " = " << penaltyL_ << std::endl;
    repStr << std::setw(setwLength) << "# number of threads " << " = " << nbThreads_ << std::endl;
    repStr << std::setw(setwLength) << "# initial dual solution " << " = " << InitialDualName[initialDual_] << std::endl;
    repStr << std::setw(setwLength) << "# main algorithm " << " = " << mainAlgorithmName[mainAlgorithm_] << std::endl;
    repStr << std::setw(setwLength) << "# add columns with one request" << " = " << addOneRequestColumn_ << std::endl;
    repStr << std::setw(setwLength) << "# solution mode " << " = " << solutionModeName[solutionMode_] << std::endl;
    repStr << std::endl;

    repStr << "# ISUD PARAMETERS" << std::endl;
    repStr << "#" << std::endl;
    repStr << std::setw(setwLength) << "# warm start " << " = " << warmStartName[initialStart_] << std::endl;
    repStr << std::setw(setwLength) << "# Zoom max MIP Inc. degree " << " = " << MIP_maxIncDegree_ << std::endl;
    repStr << std::setw(setwLength) << "# max CP Inc. degree " << " = " << CP_IncDegree_ << std::endl;
    repStr << std::setw(setwLength) << "# use Multi stage " << " = " << useMultiStage_ << std::endl;
    repStr << std::setw(setwLength) << "# min ISUD improvement " << " = " << minImp_ << std::endl;
    repStr << std::setw(setwLength) << "# is Zooming used " << " = " << useZoom_ << std::endl;
    repStr << std::endl;

    repStr << "# LABEL SETTING STRATEGIES" << std::endl;
    repStr << "#" << std::endl;
    repStr << std::setw(setwLength) << "# Use Truncated Labeling " << " = " << isTruncated_ << std::endl;
    repStr << std::setw(setwLength) << "# MaxLabel in Truncating " << " = " << MaxLabel_ << std::endl;
    repStr << std::setw(setwLength) << "# Release Dominance Rule " << " = " << isDominanceReleased_ << std::endl;
    repStr << std::setw(setwLength) << "# Restrict outgoing arcs " << " = " << isSuccessorsLimited_ << std::endl;
    repStr << std::setw(setwLength) << "# Is Pickup allowd after Drop " << " = " << isDropPickPossible_ << std::endl;
    repStr << std::setw(setwLength) << "# Restrict Route Length " << " = " << SubProSolveStartName[SubproSolveMode_] << std::endl;
    repStr << std::setw(setwLength) << "# Labeling Strategy " << " = " << LabelingStrategyName[LabelingStrategy_] << std::endl;
    repStr << std::setw(setwLength) << "# SubProblem solution Method " << " = " << subAlgorithmName[subAlgorithm_] << std::endl;
    repStr << std::setw(setwLength) << "# portion of vehicles for subPro " << " = " << vehicle_portion_ << std::endl;
    repStr << std::setw(setwLength) << "# use greedy for vehicle portion " << " = " << greedyPortion_ << std::endl;
    repStr << std::setw(setwLength) << "# number of pickups is limited " << " = " << usePick_ << std::endl;
    repStr << std::endl;

    repStr << "# CPLEX PARAMETERS" << std::endl;
    repStr << "#" << std::endl;
    repStr << std::left << std::fixed << std::setprecision(0);
    repStr << std::setw(setwLength) << "# BigM value " << " = " << bigM_ << std::endl;
    repStr << std::setw(setwLength) << "# solve Time Limit " << " = " << solveTimeLimit_ << std::endl;
    repStr << std::setw(setwLength) << "# populate Time Limit " << " = " << populateTimeLimit_ << std::endl;

    return repStr.str();

}


//-----------------------------------------------------------------------------
//  Solver Option Struct
//-----------------------------------------------------------------------------

solverOption::solverOption(float maxReachTime, bool isTruncated, int maxLabel, bool isDominanceReleased,
                           bool isSuccessorsLimited, bool isDropPickPossible, LabelingStrategy labelingStrategy) :
                           maxReachTime_(maxReachTime), isTruncated_(isTruncated),
                           MaxLabel_(maxLabel), isSuccessorsLimited_(isSuccessorsLimited),
                           isDominanceReleased_(isDominanceReleased), isDropPickPossible_(isDropPickPossible),
                           LabelingStrategy_(labelingStrategy), usePick_(false) {}

solverOption::~solverOption() = default;

void solverOption::disableHeuristics() {
    isTruncated_ = false;
    isDominanceReleased_ = false;
    isSuccessorsLimited_ = false;
}

solverOption::solverOption(PParameters &MainParams) {
    isTruncated_ = MainParams->isTruncated_;
    MaxLabel_ = MainParams->MaxLabel_;
    isDominanceReleased_ = MainParams->isDominanceReleased_;
    LabelingStrategy_ = MainParams->LabelingStrategy_;
    isSuccessorsLimited_ = MainParams->isSuccessorsLimited_;
    isDropPickPossible_ = MainParams->isDropPickPossible_;
    usePick_ = MainParams->usePick_;
}

void solverOption::updateOptions(float maxReachTime) {
    maxReachTime_ = maxReachTime;
}

bool solverOption::areHeuristicsDisabled() const {
    if (isTruncated_ || isSuccessorsLimited_ || isDominanceReleased_)
        return false;
    else
        return true;
}


