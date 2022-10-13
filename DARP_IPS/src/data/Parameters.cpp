//
// Created by Elahe Amiri on 2022-10-13.
//

#include "Parameters.h"

//************************************************************************
//                     FUNCTIONS FOR PARAMETERS CLASS
//************************************************************************

// Constructor and Destructor
Parameters::Parameters(float alphaParam, float betaParam, float deltaPram, int epochLength, InitialDual initialDual,
                       MainAlgorithm mainAlgorithm, warmStart initialStart, int MIP_maxIncDegree, int CP_IncDegree,
                       float minImp, bool fixedEpoch, bool isTruncated, int maxLabel, bool isSuccessorsLimited,
                       bool isDominanceReleased, SubProSolveStart subproSolveStartState,
                       LabelingStrategy LabelingStrategy,
                       subproblemAlgorithm subAlgorithm, int bigM, int solveTimeLimit, int populateTimeLimit,
                       bool addOneRequestColumn) :
        alphaParam_(alphaParam), betaParam_(betaParam), deltaPram_(deltaPram), epochLength_(epochLength),
        initialDual_(initialDual), mainAlgorithm_(mainAlgorithm), initialStart_(initialStart),
        MIP_maxIncDegree_(MIP_maxIncDegree), CP_IncDegree_(CP_IncDegree), minImp_(minImp), fixedEpoch_(fixedEpoch),
        isTruncated_(isTruncated), MaxLabel_(maxLabel), isSuccessorsLimited_(isSuccessorsLimited),
        isDominanceReleased_(isDominanceReleased), SubproSolveStartState_(subproSolveStartState),
        LabelingStrategy_(LabelingStrategy), subAlgorithm_(subAlgorithm), bigM_(bigM),solveTimeLimit_(solveTimeLimit),
        populateTimeLimit_(populateTimeLimit), addOneRequestColumn_(addOneRequestColumn) {
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
    repStr << std::setw(setwLength) << "# epoch Length " << " = " << epochLength_ << std::endl;
    repStr << std::setw(setwLength) << "# initial dual solution " << " = " << InitialDualName[initialDual_] << std::endl;
    repStr << std::setw(setwLength) << "# main algorithm " << " = " << mainAlgorithmName[mainAlgorithm_] << std::endl;
    repStr << std::setw(setwLength) << "# add columns with one request " << " = " << addOneRequestColumn_ << std::endl;
    repStr << std::endl;

    repStr << "# ISUD PARAMETERS" << std::endl;
    repStr << "#" << std::endl;
    repStr << std::setw(setwLength) << "# warm start " << " = " << warmStartName[initialStart_] << std::endl;
    repStr << std::setw(setwLength) << "# max MIP compatibility degree " << " = " << MIP_maxIncDegree_ << std::endl;
    repStr << std::setw(setwLength) << "# max CP compatibility degree " << " = " << CP_IncDegree_ << std::endl;
    repStr << std::setw(setwLength) << "# min ISUD improvement " << " = " << minImp_ << std::endl;
    repStr << std::setw(setwLength) << "# fixed epoch " << " = " << fixedEpoch_ << std::endl;
    repStr << std::endl;

    repStr << "# LABEL SETTING STRATEGIES" << std::endl;
    repStr << "#" << std::endl;
    repStr << std::setw(setwLength) << "# Use Truncated Labeling " << " = " << isTruncated_ << std::endl;
    repStr << std::setw(setwLength) << "# MaxLabel in Truncating " << " = " << MaxLabel_ << std::endl;
    repStr << std::setw(setwLength) << "# Release Dominance Rule " << " = " << isDominanceReleased_ << std::endl;
    repStr << std::setw(setwLength) << "# Restrict outgoing arcs " << " = " << isSuccessorsLimited_ << std::endl;
    repStr << std::setw(setwLength) << "# Restrict Route Length " << " = " << SubproSolveStartState_ << std::endl;
    repStr << std::setw(setwLength) << "# Labeling Strategy " << " = " << LabelingStrategyName[LabelingStrategy_] << std::endl;
    repStr << std::setw(setwLength) << "# SubProblem solution Method " << " = " << subAlgorithmName[subAlgorithm_] << std::endl;
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

solverOption::solverOption(float maxReachTime, int maxPickup, bool isTruncated, int maxLabel, bool isDominanceReleased,
                           bool isSuccessorsLimited, LabelingStrategy labelingStrategy) : maxReachTime_(maxReachTime),
                                                                                          maxPickup_(maxPickup), isTruncated_(isTruncated), MaxLabel_(maxLabel),
                                                                                          isSuccessorsLimited_(isSuccessorsLimited), isDominanceReleased_(isDominanceReleased),
                                                                                          LabelingStrategy_(labelingStrategy) {}

solverOption::~solverOption() = default;

void solverOption::disableHeuristics() {
    isTruncated_ = false;
    isDominanceReleased_ = false;
    isSuccessorsLimited_ = false;
}

solverOption::solverOption(float maxReachTime, int maxPickup, PParameters &MainParams): maxReachTime_(maxReachTime),
                                                                                        maxPickup_(maxPickup) {
    isTruncated_ = MainParams->isTruncated_;
    MaxLabel_ = MainParams->MaxLabel_;
    isDominanceReleased_ = MainParams->isDominanceReleased_;
    LabelingStrategy_ = MainParams->LabelingStrategy_;
    isSuccessorsLimited_ = MainParams->isSuccessorsLimited_;
}

bool solverOption::areHeuristicsDisabled() const {
    if (isTruncated_ || isSuccessorsLimited_)
        return false;
    else
        return true;
}