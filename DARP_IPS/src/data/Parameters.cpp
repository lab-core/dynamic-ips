//
// Created by Elahe Amiri on 2022-10-13.
//

#include "Parameters.h"

//************************************************************************
//                     FUNCTIONS FOR SOLVERBASE CLASS
//************************************************************************

// Constructor
SolverBase::SolverBase(bool isTruncated, int maxLabel, int MaxCommittedLabel, bool isDominanceReleased,
                       bool pruneNodes, bool pruneArcs, bool discardSuboptimalPath, bool isDropPickPossible,
                       LabelingStrategy LabelingStrategy, LabelingReOptimizeStrategy labelingReOptimizeStrategy,
                       int nbPick, SortPaths pathSort, int newRequestLimit, float wait_W1, float ride_W2,
                       bool req_W3) :
        isTruncated_(isTruncated), MaxLabel_(maxLabel), MaxCommittedLabel_(MaxCommittedLabel), Wait_W1_(wait_W1),
        isDominanceReleased_(isDominanceReleased), pruneNodes_(pruneNodes), pruneArcs_(pruneArcs), Req_W3_(req_W3),
        discardSuboptimalPath_(discardSuboptimalPath), isDropPickPossible_(isDropPickPossible), Ride_W2_(ride_W2),
        LabelingStrategy_(LabelingStrategy), labelingReOptimizeStrategy_(labelingReOptimizeStrategy),
        nbPick_(nbPick), sortPath_(pathSort), newRequestLimit_(newRequestLimit) {}

//************************************************************************
//                     FUNCTIONS FOR PARAMETERS CLASS
//************************************************************************

// Constructor and Destructor
Parameters::Parameters(float alphaParam, float betaParam, float deltaPram, int epochLength, int penaltyL,
                       int committedTime, int nbThreads, InitialDual initialDual, DualMethod dualMethod,
                       MainAlgorithm mainAlgorithm, int numIter, bool greedyReOptimize, bool vehicleReturn,
                       float timeWindow, float WaitForReturn, int numVehicleSwitch,
                       WarmStart initialStart, int MIP_maxIncDegree, int CP_IncDegree, bool reducedCP,
                       float minImp, bool useZoom, int nbColumn, bool isTruncated, int maxLabel, int MaxCommittedLabel,
                       bool pruneNodes, bool pruneArcs, bool discardSuboptimalPath,
                       bool isDominanceReleased, bool isDropPickPossible, LabelingStrategy LabelingStrategy,
                       SubproblemAlgorithm subAlgorithm, bool constPortion, bool vehiclePortion,
                       bool dynamicPricing, bool partialPricing, bool routeRecycle, int nbPick,
                       SortPaths sortPath, SortColumns sortColumn, int bigM, int newRequestLimit, int solveTimeLimit,
                       int populateTimeLimit, SolutionMode solutionMode, float MIPGap, int informTimeLimit,
                       int pickupDeviationWindow, ReturnType returnPolicy, float maxWait, ModelSOLVER modelSolver,
                       LabelingReOptimizeStrategy labelingReOptimizeStrategy, bool smoothDual, float wait_W1,
                       float ride_W2, bool req_W3):
        SolverBase(isTruncated, maxLabel, MaxCommittedLabel, isDominanceReleased, pruneNodes, pruneArcs,
                   discardSuboptimalPath, isDropPickPossible, LabelingStrategy, labelingReOptimizeStrategy,
                   nbPick, sortPath, newRequestLimit, wait_W1, ride_W2, req_W3),
        alphaParam_(alphaParam), betaParam_(betaParam), deltaPram_(deltaPram), epochLength_(epochLength),
        penaltyL_(penaltyL), committedTime_(committedTime), nbThreads_(nbThreads), initialDual_(initialDual),
        mainAlgorithm_(mainAlgorithm), solutionMode_(solutionMode), numIter_(numIter), dualMethod_(dualMethod),
        greedyReOptimize_(greedyReOptimize), saveScratch_(0), vehicleReturn_(vehicleReturn), timeWindow_(timeWindow),
        WaitForReturn_(WaitForReturn), numVehicleSwitch_(numVehicleSwitch), informTimeLimit_(informTimeLimit),
        pickupDeviationWindow_(pickupDeviationWindow), initialStart_(initialStart), MIP_maxIncDegree_(MIP_maxIncDegree),
        CP_IncDegree_(CP_IncDegree), reducedCP_(reducedCP), minImp_(minImp), useZoom_(useZoom),
        nbColumn_(nbColumn), subAlgorithm_(subAlgorithm), smoothDual_(smoothDual),
        vehiclePortion_(vehiclePortion), dynamicPricing_(dynamicPricing), partialPricing_(partialPricing),
        routeRecycle_(routeRecycle), sortColumn_(sortColumn), bigM_(bigM), solveTimeLimit_(solveTimeLimit), modelSolver_(modelSolver),
        populateTimeLimit_(populateTimeLimit), MIPGap_(MIPGap), returnPolicy_(returnPolicy), maxWait_(maxWait) {}

Parameters::~Parameters() = default;

// Display function
std::string Parameters::toString() const {
    std::stringstream repStr;
    repStr << "# MODEL PARAMETERS" << std::endl;
    repStr << "#" << std::endl;
    int setwLength = 35;
    repStr << std::left << std::fixed << std::setprecision(1) << std::boolalpha;
    repStr << std::setw(setwLength) << "# alpha Parameter " << " = " << alphaParam_ << std::endl;
    repStr << std::setw(setwLength) << "# beta Parameter " << " = " << betaParam_ << std::endl;
    repStr << std::setw(setwLength) << "# delta Parameter" << " = " << deltaPram_ << std::endl;
    repStr << std::setw(setwLength) << "# Wait Obj. weight" << " = " << Wait_W1_ << std::endl;
    repStr << std::setw(setwLength) << "# Trip Delay Obj. weight" << " = " << Ride_W2_ << std::endl;
    repStr << std::setw(setwLength) << "# Use #Customer in Obj" << " = " << Req_W3_ << std::endl;
    if (solutionMode_ == DYNAMIC)
        repStr << std::setw(setwLength) << "# epoch Length " << " = " << epochLength_ << " (s)" << std::endl;
    if (solutionMode_ == ANYTIME)
        repStr << std::setw(setwLength) << "# time to commit stops " << " = " << committedTime_ << " (s)" << std::endl;
    repStr << std::setw(setwLength) << "# penalty epoch Length " << " = " << penaltyL_ << std::endl;
    repStr << std::setw(setwLength) << "# number of threads " << " = " << nbThreads_ << std::endl;
    repStr << std::setw(setwLength) << "# initial dual solution " << " = " << eu::toString(initialDual_) << std::endl;
    repStr << std::setw(setwLength) << "# dual method " << " = " << eu::toString(dualMethod_) << std::endl;
    repStr << std::setw(setwLength) << "# smoothing dual " << " = " << smoothDual_ << std::endl;
    repStr << std::setw(setwLength) << "# main algorithm " << " = " << eu::toString(mainAlgorithm_) << std::endl;
    repStr << std::setw(setwLength) << "# solution mode " << " = " << eu::toString(solutionMode_) << std::endl;
    repStr << std::setw(setwLength) << "# Number of iter per epoch " << " = " << numIter_ << std::endl;
    repStr << std::setw(setwLength) << "# Is Greedy Re-Optimized " << " = " << greedyReOptimize_ << std::endl;
    repStr << std::setw(setwLength) << "# Idle vehicles return " << " = " << vehicleReturn_ << std::endl;
    repStr << std::setw(setwLength) << "# vehicle return method " << " = " << eu::toString(returnPolicy_) << std::endl;
    repStr << std::setw(setwLength) << "# Waiting time window " << " = " << timeWindow_ << std::endl;
    repStr << std::setw(setwLength) << "# Wait before return time " << " = " << WaitForReturn_ << std::endl;
    repStr << std::setw(setwLength) << "# Maximum vehicle switch  " << " = " << numVehicleSwitch_ << std::endl;
    repStr << std::setw(setwLength) << "# time to inform the customer " << " = " << informTimeLimit_ << " (s)" << std::endl;
    repStr << std::setw(setwLength) << "# pickup Deviation Window " << " = " << pickupDeviationWindow_ << " (s)" << std::endl;
    repStr << std::setw(setwLength) << "# Max request wait time" << " = " << maxWait_ << " (s)" << std::endl;
    repStr << std::endl;

    repStr << "# MP PARAMETERS" << std::endl;
    repStr << "#" << std::endl;
    repStr << std::setw(setwLength) << "# warm start " << " = " << eu::toString(initialStart_) << std::endl;
    repStr << std::setw(setwLength) << "# Zoom max MIP Inc. degree " << " = " << MIP_maxIncDegree_ << std::endl;
    repStr << std::setw(setwLength) << "# max CP Inc. degree " << " = " << CP_IncDegree_ << std::endl;
    repStr << std::setw(setwLength) << "# use Reduced CP " << " = " << reducedCP_ << std::endl;
    repStr << std::setw(setwLength) << "# min ISUD improvement " << " = " << minImp_ << std::endl;
    repStr << std::setw(setwLength) << "# is Zooming used " << " = " << useZoom_ << std::endl;
    repStr << std::setw(setwLength) << "# Column added to MP " << " = " << nbColumn_ << std::endl;
    repStr << std::setw(setwLength) << "# MIP model solver " << " = " << eu::toString(modelSolver_) << std::endl;
    repStr << std::endl;

    repStr << "# LABEL SETTING STRATEGIES" << std::endl;
    repStr << "#" << std::endl;
    repStr << std::setw(setwLength) << "# Use Truncated Labeling " << " = " << isTruncated_ << std::endl;
    repStr << std::setw(setwLength) << "# MaxLabel in Truncating " << " = " << MaxLabel_ << std::endl;
    repStr << std::setw(setwLength) << "# MaxCommitLabel in Truncating " << " = " << MaxCommittedLabel_ << std::endl;
    repStr << std::setw(setwLength) << "# Release Dominance Rule " << " = " << isDominanceReleased_ << std::endl;
    repStr << std::setw(setwLength) << "# Prune Nodes from graph " << " = " << pruneNodes_ << std::endl;
    repStr << std::setw(setwLength) << "# Prune Arcs from graph " << " = " << pruneArcs_ << std::endl;
    repStr << std::setw(setwLength) << "# Remove suboptimal labels " << " = " << discardSuboptimalPath_ << std::endl;
    repStr << std::setw(setwLength) << "# Is Pickup allowed after Drop " << " = " << isDropPickPossible_ << std::endl;
    repStr << std::setw(setwLength) << "# Labeling Strategy " << " = " << eu::toString(LabelingStrategy_) << std::endl;
    repStr << std::setw(setwLength) << "# Labeling Reoptimize Strategy " << " = " << eu::toString(labelingReOptimizeStrategy_) << std::endl;
    repStr << std::setw(setwLength) << "# SubProblem solution Method " << " = " << eu::toString(subAlgorithm_) << std::endl;
    repStr << std::setw(setwLength) << "# use Dynamic Pricing " << " = " << dynamicPricing_ << std::endl;
    repStr << std::setw(setwLength) << "# use Partial Pricing " << " = " << partialPricing_ << std::endl;
    repStr << std::setw(setwLength) << "# Recycle Routes " << " = " << routeRecycle_ << std::endl;
    repStr << std::setw(setwLength) << "# number of pickups allowed " << " = " << nbPick_ << std::endl;
    repStr << std::setw(setwLength) << "# Sorting mode of paths " << " = " << eu::toString(sortPath_) << std::endl;
    repStr << std::setw(setwLength) << "# Sorting mode of columns " << " = " << eu::toString(sortColumn_) << std::endl;
    repStr << std::setw(setwLength) << "# new Request limit to recycle " << " = " << newRequestLimit_ << std::endl;
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
    repStr << eu::toString(modelSolver_) << ","
           << alphaParam_ << ","
           << betaParam_ << ","
           << deltaPram_ << ","
           << Wait_W1_ << ","
           << Ride_W2_ << ","
           << Req_W3_ << ","
           << epochLength_ << ","
           << committedTime_ << ","
           << informTimeLimit_ << ","
           << pickupDeviationWindow_ << ","
           << maxWait_ << ","
           << nbThreads_ << ","
           << eu::toString(initialDual_) << ","
           << eu::toString(dualMethod_) << ","
           << smoothDual_ << ","
           << eu::toString(initialStart_) << ","
           << eu::toString(mainAlgorithm_) << ","
           << eu::toString(solutionMode_) << ","
           << numIter_ << ","
           << boolToString(greedyReOptimize_) << ","
           << boolToString(vehicleReturn_) << ","
           << eu::toString(returnPolicy_) << ","
           << MIP_maxIncDegree_ << ","
           << CP_IncDegree_ << ","
           << boolToString(reducedCP_) << ","
           << boolToString(useZoom_) << ","
           << nbColumn_ << ","
           << boolToString(isTruncated_) << ","
           << MaxLabel_ << ","
           << MaxCommittedLabel_ << ","
           << boolToString(isDominanceReleased_) << ","
           << boolToString(isDropPickPossible_) << ","
           << boolToString(pruneNodes_) << ","
           << boolToString(pruneArcs_) << ","
           << boolToString(discardSuboptimalPath_) << ","
           << eu::toString(LabelingStrategy_) << ","
           << eu::toString(labelingReOptimizeStrategy_) << ","
           << boolToString(vehiclePortion_) << ","
           << boolToString(dynamicPricing_) << ","
           << boolToString(partialPricing_) << ","
           << boolToString(routeRecycle_) << ","
           << newRequestLimit_ << ","
           << nbPick_ << ","
           << eu::toString(sortPath_) << ","
           << eu::toString(sortColumn_) << ","
           << MIPGap_ << "\n";
    return repStr.str();
}


//-----------------------------------------------------------------------------
//  Solver Option Struct
//-----------------------------------------------------------------------------

solverOption::solverOption(bool isTruncated, int maxLabel, int MaxCommittedLabel, bool isDominanceReleased, int nbPick,
                           SortPaths pathSort, bool pruneNodes, bool pruneArcs, bool discardSuboptimalPath,
                           bool isDropPickPossible, LabelingStrategy labelingStrategy, int newRequestLimit,
                           float wait_W1, float ride_W2, bool req_W3) :
        SolverBase(isTruncated, maxLabel, MaxCommittedLabel, isDominanceReleased, pruneNodes, pruneArcs,
                   discardSuboptimalPath, isDropPickPossible, labelingStrategy, LabelingReOptimizeStrategy{},
                   nbPick, pathSort, newRequestLimit, wait_W1, ride_W2, req_W3) {}

solverOption::~solverOption() = default;

void solverOption::disableHeuristics() {
    isTruncated_ = false;
    isDropPickPossible_ = true;
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
    repStr << std::setw(setwLength) << "# Prune Nodes from graph " << " = " << pruneNodes_ << std::endl;
    repStr << std::setw(setwLength) << "# Prune Arcs from graph " << " = " << pruneArcs_ << std::endl;
    repStr << std::setw(setwLength) << "# Remove suboptimal labels " << " = " << discardSuboptimalPath_ << std::endl;
    repStr << std::setw(setwLength) << "# Is Pickup allowed after Drop " << " = " << isDropPickPossible_ << std::endl;
    repStr << std::setw(setwLength) << "# Labeling Strategy " << " = " << eu::toString(LabelingStrategy_) << std::endl;
    repStr << std::setw(setwLength) << "# number of pickups allowed " << " = " << nbPick_ << std::endl;
    repStr << std::setw(setwLength) << "# new Request limit to recycle " << " = " << newRequestLimit_ << std::endl;
    repStr << std::setw(setwLength) << "# Wait Obj. weight" << " = " << Wait_W1_ << std::endl;
    repStr << std::setw(setwLength) << "# Trip Delay Obj. weight" << " = " << Ride_W2_ << std::endl;
    repStr << std::setw(setwLength) << "# Consider Customers in Obj." << " = " << Req_W3_ << std::endl;
    return repStr.str();

}

void solverOption::enableHeuristics(const PParameters &MainParams) {
    isTruncated_ = true;
    isDropPickPossible_ = false;
}

solverOption::solverOption(const PParameters &MainParams) :
        SolverBase(MainParams->isTruncated_, MainParams->MaxLabel_, MainParams->MaxCommittedLabel_,
                   MainParams->isDominanceReleased_, MainParams->pruneNodes_, MainParams->pruneArcs_,
                   MainParams->discardSuboptimalPath_, MainParams->isDropPickPossible_,
                   MainParams->LabelingStrategy_, MainParams->labelingReOptimizeStrategy_,
                   MainParams->nbPick_, MainParams->sortPath_, MainParams->newRequestLimit_,
                   MainParams->Wait_W1_, MainParams->Ride_W2_, MainParams->Req_W3_) {}


std::string boolToString(bool value) {
    return value ? "True" : "False";
}