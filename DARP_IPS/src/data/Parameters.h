//
// Created by Elahe Amiri on 2022-10-13.
//

#ifndef PARAMETERS_H
#define PARAMETERS_H

#include "utilities/MyTools.h"
#include "utilities/ConfigParser.h"

//-----------------------------------------------------------------------------
//  Parameters Struct
//-----------------------------------------------------------------------------
struct Parameters {
public:
    // model Parameters
    float alphaParam_{};
    float betaParam_{};
    float deltaPram_{};
    float epochLength_{};
    int penaltyL_{};
    float committedTime_{};
    int nbThreads_{};
    InitialDual initialDual_;
    MainAlgorithm mainAlgorithm_;
    SolutionMode solutionMode_;     // STATIC, DYNAMIC, ANYTIME
    int numIter_;                  // solve the Master problem one time at each iteration of the CG
    bool greedyReOptimize_;         // restart greedy (re-assigning) considering the current state of the system
    int saveScratch_;              // save the results in scratch place of the server
    bool vehicleReturn_;            // determine if the idle vehicles return ti initial location or not
    float timeWindow_;
    float WaitForReturn_;             // The time that a vehicle remains idle before returning to crowded areas
    int numVehicleSwitch_;          // the number of times we are allowed to change the vehicle assigned to a customer
    float informTimeLimit_;
    float pickupDeviationWindow_;
    ReturnType returnPolicy_;
    float maxWait_;
    ModelSOLVER modelSolver_;
    Approach approach_;

    // ISUD parameters
    WarmStart initialStart_;
    int MIP_maxIncDegree_;      // max incompatibility degree for Zoom
    int CP_IncDegree_;          // max incompatibility degree for CP
    bool useMultiStage_;       // min incompatibility degree that CP starts from in multi-stage
    float minImp_;
    bool useZoom_;
    int nbColumn_;


    // label setting strategies
    bool isTruncated_{};
    int MaxLabel_{};
    int MaxCommittedLabel_{};
    bool isDominanceReleased_{};
    bool pruneNodes_{};
    bool pruneArcs_{};
    bool discardSuboptimalPath_{};
    bool isDropPickPossible_{};
    LabelingStrategy LabelingStrategy_;
    SubproblemAlgorithm subAlgorithm_;
    bool constPortion_;
    bool vehiclePortion_{};
    bool dynamicPricing_{};
    bool partialPricing_;
    bool routeRecycle_;
    bool usePick_;
    int nbPick_;
    SortPaths sortPath_;
    SortColumns sortColumn_;

    //CPLEX Parameters
    int bigM_{};
    int solveTimeLimit_{};
    int populateTimeLimit_{};
    float MIPGap_{};

    // Constructor and Destructor
    Parameters(float alphaParam, float betaParam, float deltaPram, int epochLength, int penaltyL,
               int committedTime, int nbThreads, InitialDual initialDual, MainAlgorithm mainAlgorithm, int numIter,
               bool greedyReOptimize, bool vehicleReturn, float timeWindow,
               float WaitForReturn, int numVehicleSwitch, WarmStart initialStart,
               int MIP_maxIncDegree, int CP_IncDegree, bool useMultiStage, float minImp, bool useZoom,
               int nbColumn, bool isTruncated, int maxLabel, int MaxCommittedLabel, bool pruneNodes, bool pruneArcs,
               bool discardSuboptimalPath, bool isDominanceReleased, bool isDropPickPossible,
               LabelingStrategy LabelingStrategy, SubproblemAlgorithm subAlgorithm, bool constPortion,
               bool vehiclePortion, bool dynamicPricing, bool partialPricing, bool routeRecycle,
               bool usePick, int nbPick, SortPaths sortPath, SortColumns sortColumn, int bigM,
               int solveTimeLimit, int populateTimeLimit, SolutionMode solutionMode, float MIPGap, int informTimeLimit,
               int pickupDeviationWindow, ReturnType returnPolicy, float maxWait, ModelSOLVER modelSolver);

    virtual ~Parameters();

    // Display function
    std::string toString() const;
    std::string toStr() const;
};


//-----------------------------------------------------------------------------
//  Solver Option Struct
//-----------------------------------------------------------------------------
struct solverOption {
    //   int maxPickup_;

    bool isTruncated_;
    bool isDominanceReleased_;
    bool pruneNodes_;
    bool pruneArcs_;
    bool discardSuboptimalPath_;
    bool isDropPickPossible_;
    LabelingStrategy LabelingStrategy_;
    int MaxLabel_;
    int MaxCommittedLabel_;
    bool usePick_;
    int nbPick_;
    SortPaths pathSort_;

    // Constructor and Destructor
    solverOption(bool isTruncated, int maxLabel, int MaxCommittedLabel, bool isDominanceReleased, int nbPick,
                 SortPaths pathSort, bool pruneNodes, bool pruneArcs,
                 bool discardSuboptimalPath, bool isDropPickPossible, LabelingStrategy labelingStrategy);

    explicit solverOption(const PParameters &MainParams);

    virtual ~solverOption();
    void disableHeuristics();
    // Display function
    std::string toString() const;
};

std::string boolToString(bool value);


#endif //PARAMETERS_H
