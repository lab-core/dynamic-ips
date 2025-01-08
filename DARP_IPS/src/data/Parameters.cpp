//
// Created by Elahe Amiri on 2022-10-13.
//

#include "Parameters.h"

//************************************************************************
//                     FUNCTIONS FOR PARAMETERS CLASS
//************************************************************************

// Constructor and Destructor
Parameters::Parameters(float alphaParam, float betaParam, float deltaPram, int epochLength, int penaltyL,
                       int committedTime, int nbThreads, InitialDual initialDual, MainAlgorithm mainAlgorithm,
                       int numIter, bool greedyReOptimize, int saveScratch, bool vehicleReturn, float timeWindow,
                       warmStart initialStart, int MIP_maxIncDegree, int CP_IncDegree, bool useMultiStage,
                       float minImp, bool useZoom, int nbColumn, bool isTruncated, int maxLabel,
                       bool isSuccessorsLimited, bool pruneNodes, bool pruneArcs, bool discardSuboptimalPath,
                       bool isDominanceReleased, bool isDropPickPossible, SubProSolveMode subproSolveMode,
                       LabelingStrategy LabelingStrategy, subproblemAlgorithm subAlgorithm, bool constPortion,
                       bool vehiclePortion, bool dynamicPricing, bool partialPricing, bool routeRecycle, bool usePick,
                       int nbPick, SortPaths sortPath, SortColumns sortColumn, int bigM, int solveTimeLimit,
                       int populateTimeLimit, SolutionMode solutionMode, float MIPGap):
        alphaParam_(alphaParam), betaParam_(betaParam), deltaPram_(deltaPram), epochLength_(epochLength),
        penaltyL_(penaltyL), committedTime_(committedTime), nbThreads_(nbThreads), initialDual_(initialDual),
        mainAlgorithm_(mainAlgorithm), numIter_(numIter), greedyReOptimize_(greedyReOptimize),
        saveScratch_(saveScratch), vehicleReturn_(vehicleReturn), initialStart_(initialStart),
        MIP_maxIncDegree_(MIP_maxIncDegree), CP_IncDegree_(CP_IncDegree), useMultiStage_(useMultiStage),
        minImp_(minImp), useZoom_(useZoom), nbColumn_(nbColumn), isTruncated_(isTruncated), MaxLabel_(maxLabel),
        isSuccessorsLimited_(isSuccessorsLimited), isDominanceReleased_(isDominanceReleased),
        isDropPickPossible_(isDropPickPossible), SubproSolveMode_(subproSolveMode),
        LabelingStrategy_(LabelingStrategy), subAlgorithm_(subAlgorithm), constPortion_(constPortion),
        vehiclePortion_(vehiclePortion), dynamicPricing_(dynamicPricing), partialPricing_(partialPricing),
        routeRecycle_(routeRecycle), usePick_(usePick), nbPick_(nbPick), sortPath_(sortPath) ,
        sortColumn_(sortColumn), bigM_(bigM), solveTimeLimit_(solveTimeLimit), populateTimeLimit_(populateTimeLimit),
        solutionMode_(solutionMode), discardSuboptimalPath_(discardSuboptimalPath), MIPGap_(MIPGap),
        timeWindow_(timeWindow), pruneNodes_(pruneNodes), pruneArcs_(pruneArcs), savePartial_(false){}

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
    repStr << std::setw(setwLength) << "# solution mode " << " = " << solutionModeName[solutionMode_] << std::endl;
    repStr << std::setw(setwLength) << "# Number of iter per epoch " << " = " << numIter_ << std::endl;
    repStr << std::setw(setwLength) << "# Is Greedy Re-Optimized " << " = " << greedyReOptimize_ << std::endl;
    repStr << std::setw(setwLength) << "# drop first hour " << " = " << savePartial_ << std::endl;
    repStr << std::setw(setwLength) << "# Idle vehicles return " << " = " << vehicleReturn_ << std::endl;
    repStr << std::setw(setwLength) << "# Waiting time window " << " = " << timeWindow_ << std::endl;
    repStr << std::endl;

    repStr << "# ISUD PARAMETERS" << std::endl;
    repStr << "#" << std::endl;
    repStr << std::setw(setwLength) << "# warm start " << " = " << warmStartName[initialStart_] << std::endl;
    repStr << std::setw(setwLength) << "# Zoom max MIP Inc. degree " << " = " << MIP_maxIncDegree_ << std::endl;
    repStr << std::setw(setwLength) << "# max CP Inc. degree " << " = " << CP_IncDegree_ << std::endl;
    repStr << std::setw(setwLength) << "# use Multi stage " << " = " << useMultiStage_ << std::endl;
    repStr << std::setw(setwLength) << "# min ISUD improvement " << " = " << minImp_ << std::endl;
    repStr << std::setw(setwLength) << "# is Zooming used " << " = " << useZoom_ << std::endl;
    repStr << std::setw(setwLength) << "# Column added to MP " << " = " << nbColumn_ << std::endl;
    repStr << std::endl;

    repStr << "# LABEL SETTING STRATEGIES" << std::endl;
    repStr << "#" << std::endl;
    repStr << std::setw(setwLength) << "# Use Truncated Labeling " << " = " << isTruncated_ << std::endl;
    repStr << std::setw(setwLength) << "# MaxLabel in Truncating " << " = " << MaxLabel_ << std::endl;
    repStr << std::setw(setwLength) << "# Release Dominance Rule " << " = " << isDominanceReleased_ << std::endl;
    repStr << std::setw(setwLength) << "# Restrict outgoing arcs " << " = " << isSuccessorsLimited_ << std::endl;
    repStr << std::setw(setwLength) << "# Prune Nodes from graph " << " = " << pruneNodes_ << std::endl;
    repStr << std::setw(setwLength) << "# Prune Arcs from graph " << " = " << pruneArcs_ << std::endl;
    repStr << std::setw(setwLength) << "# Remove suboptimal labels " << " = " << discardSuboptimalPath_ << std::endl;
    repStr << std::setw(setwLength) << "# Is Pickup allowed after Drop " << " = " << isDropPickPossible_ << std::endl;
    repStr << std::setw(setwLength) << "# Restrict Route Length " << " = " << SubProSolveStartName[SubproSolveMode_] << std::endl;
    repStr << std::setw(setwLength) << "# Labeling Strategy " << " = " << LabelingStrategyName[LabelingStrategy_] << std::endl;
    repStr << std::setw(setwLength) << "# SubProblem solution Method " << " = " << subAlgorithmName[subAlgorithm_] << std::endl;
    repStr << std::setw(setwLength) << "# portion of constraints for MP " << " = " << constPortion_ << std::endl;
    repStr << std::setw(setwLength) << "# solve for vehicle portion " << " = " << vehiclePortion_ << std::endl;
    repStr << std::setw(setwLength) << "# use Dynamic Pricing " << " = " << dynamicPricing_ << std::endl;
    repStr << std::setw(setwLength) << "# use Partial Pricing " << " = " << partialPricing_ << std::endl;
    repStr << std::setw(setwLength) << "# Recycle Routes " << " = " << routeRecycle_ << std::endl;
    repStr << std::setw(setwLength) << "# number of pickups is limited " << " = " << usePick_ << std::endl;
    repStr << std::setw(setwLength) << "# number of pickups allowed " << " = " << nbPick_ << std::endl;
    repStr << std::setw(setwLength) << "# Sorting mode of paths " << " = " << SortPathsName[sortPath_] << std::endl;
    repStr << std::setw(setwLength) << "# Sorting mode of columns " << " = " << SortColumnsName[sortColumn_] << std::endl;
    repStr << std::endl;

    repStr << "# CPLEX PARAMETERS" << std::endl;
    repStr << "#" << std::endl;
    repStr << std::left << std::fixed << std::setprecision(0);
    repStr << std::setw(setwLength) << "# BigM value " << " = " << bigM_ << std::endl;
    repStr << std::setw(setwLength) << "# solve Time Limit " << " = " << solveTimeLimit_ << std::endl;
    repStr << std::setw(setwLength) << "# populate Time Limit " << " = " << populateTimeLimit_ << std::endl;
    repStr << std::setw(setwLength) << "# MIP Gap " << " = " << MIPGap_ << std::endl;

    return repStr.str();

}

std::string Parameters::toStr() const {
    std::stringstream repStr;
    repStr << alphaParam_ << ",";
    repStr << betaParam_ << ",";
    repStr << deltaPram_ << ",";
    repStr << epochLength_ << ",";
    repStr << committedTime_ << ",";
    repStr << nbThreads_ << ",";
    repStr << InitialDualName[initialDual_] << ",";
    repStr << warmStartName[initialStart_] << ",";
    repStr << mainAlgorithmName[mainAlgorithm_] << ",";
    repStr << solutionModeName[solutionMode_] << ",";
    repStr << numIter_ << ",";
    repStr << boolToString(greedyReOptimize_) << ",";
    repStr << boolToString(vehicleReturn_) << ",";
    repStr << MIP_maxIncDegree_ << ",";
    repStr << CP_IncDegree_ << ",";
    repStr << boolToString(useMultiStage_) << ",";
    repStr << boolToString(useZoom_) << ",";
    repStr << nbColumn_ << ",";
    repStr << boolToString(isTruncated_) << ",";
    repStr << MaxLabel_ << ",";
    repStr << boolToString(isDominanceReleased_) << ",";
    repStr << boolToString(isDropPickPossible_) << ",";
    repStr << boolToString(isSuccessorsLimited_) << ",";
    repStr << boolToString(pruneNodes_) << ",";
    repStr << boolToString(pruneArcs_) << ",";
    repStr << boolToString(discardSuboptimalPath_) << ",";
    repStr << LabelingStrategyName[LabelingStrategy_] << ",";
    repStr << boolToString(vehiclePortion_) << ",";
    repStr << boolToString(dynamicPricing_) << ",";
    repStr << boolToString(partialPricing_) << ",";
    repStr << boolToString(routeRecycle_) << ",";
    repStr << nbPick_ << ",";
    repStr << SortPathsName[sortPath_] << ",";
    repStr << SortColumnsName[sortColumn_] << ",";
    repStr << MIPGap_ << "\n";
    return repStr.str();
}


//-----------------------------------------------------------------------------
//  Solver Option Struct
//-----------------------------------------------------------------------------

solverOption::solverOption(bool isTruncated, int maxLabel, bool isDominanceReleased, int nbPick,
                           SortPaths pathSort, bool isSuccessorsLimited, bool pruneNodes, bool pruneArcs,
                           bool discardSuboptimalPath, bool isDropPickPossible,
                           LabelingStrategy labelingStrategy) :
        isTruncated_(isTruncated), nbPick_(nbPick), MaxLabel_(maxLabel),
        isSuccessorsLimited_(isSuccessorsLimited), discardSuboptimalPath_(discardSuboptimalPath),
        pruneNodes_(pruneNodes), pruneArcs_(pruneArcs), pathSort_(pathSort),
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
    nbPick_ = MainParams->nbPick_;
    pathSort_ = MainParams->sortPath_;
    pruneNodes_ = MainParams->pruneNodes_;
    pruneArcs_ = MainParams->pruneArcs_;
    discardSuboptimalPath_ = MainParams->discardSuboptimalPath_;
}


bool solverOption::areHeuristicsDisabled() const {
    if (isTruncated_ || isSuccessorsLimited_ || isDominanceReleased_ || !isDropPickPossible_)
        return false;
    else
        return true;
}

// Display function
std::string solverOption::toString() const {
    std::stringstream repStr;
    repStr << "# MODEL PARAMETERS" << std::endl;
    repStr << "#" << std::endl;
    int setwLength = 30;
    repStr << std::left << std::fixed << std::setprecision(1) << std::boolalpha;
    repStr << std::setw(setwLength) << "# Use Truncated Labeling " << " = " << isTruncated_ << std::endl;
    repStr << std::setw(setwLength) << "# MaxLabel in Truncating " << " = " << MaxLabel_ << std::endl;
    repStr << std::setw(setwLength) << "# Release Dominance Rule " << " = " << isDominanceReleased_ << std::endl;
    repStr << std::setw(setwLength) << "# Restrict outgoing arcs " << " = " << isSuccessorsLimited_ << std::endl;
    repStr << std::setw(setwLength) << "# Prune Nodes from graph " << " = " << pruneNodes_ << std::endl;
    repStr << std::setw(setwLength) << "# Prune Arcs from graph " << " = " << pruneArcs_ << std::endl;
    repStr << std::setw(setwLength) << "# Remove suboptimal labels " << " = " << discardSuboptimalPath_ << std::endl;
    repStr << std::setw(setwLength) << "# Is Pickup allowed after Drop " << " = " << isDropPickPossible_ << std::endl;
    repStr << std::setw(setwLength) << "# Labeling Strategy " << " = " << LabelingStrategyName[LabelingStrategy_] << std::endl;
    repStr << std::setw(setwLength) << "# number of pickups is limited " << " = " << usePick_ << std::endl;
    repStr << std::setw(setwLength) << "# number of pickups allowed " << " = " << nbPick_ << std::endl;

    return repStr.str();

}

std::string boolToString(bool value) {
    return value ? "True" : "False";
}
