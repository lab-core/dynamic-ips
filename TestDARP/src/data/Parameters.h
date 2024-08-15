//
// Created by Elahe Amiri on 2022-10-13.
//

#ifndef PARAMETERS_H
#define PARAMETERS_H

#include "utilities/MyTools.h"

//-----------------------------------------------------------------------------
//  Parameters Struct
//-----------------------------------------------------------------------------
struct Parameters {
public:
    // model Parameters
    float alphaParam_{};
    float betaParam_{};
    float deltaPram_{};
    int epochLength_{};
    int penaltyL_{};
    float committedTime_{};
    int nbThreads_{};
    InitialDual initialDual_;
    MainAlgorithm mainAlgorithm_;
    bool addOneRequestColumn_;
    SolutionMode solutionMode_;     // STATIC, DYNAMIC, ANYTIME
    bool oneIter_;                  // solve master problem one time at each iteration of the CG
    bool greedyReOptimize_;         // restart greedy (re-assigning) considering the current state of the system
    int saveScratch_;              // save the results in scratch place of the server
    bool savePartial_;              // calculate the avg. wait time after one hour
    bool vehicleReturn_;            // determine if the idle vehicles return ti initial location or not
    float timeWindow_;

    // ISUD parameters
    warmStart initialStart_;
    int MIP_maxIncDegree_;      // max incompatibility degree for Zoom
    int CP_IncDegree_;          // max incompatibility degree for CP
    bool useMultiStage_;       // min incompatibility degree that CP starts from in multi-stage
    float minImp_;
    bool useZoom_;
    int nbColumn_;


    // label setting strategies
    bool isTruncated_{};
    int MaxLabel_{};
    bool isDominanceReleased_{};
    bool isSuccessorsLimited_{};
    bool isDropPickPossible_{};
    SubProSolveMode SubproSolveMode_;
    LabelingStrategy LabelingStrategy_;
    subproblemAlgorithm subAlgorithm_;
    bool constPortion_;
    bool vehiclePortion_{};
    bool dynamicPricing_{};
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
               float committedTime, int nbThreads, InitialDual initialDual, MainAlgorithm mainAlgorithm, bool oneIter,
               bool greedyReOptimize, int saveScratch, bool vehicleReturn, float timeWindow, warmStart initialStart,
               int MIP_maxIncDegree, int CP_IncDegree, bool useMultiStage, float minImp, bool useZoom,
               int nbColumn, bool isTruncated, int maxLabel, bool isSuccessorsLimited,
               bool isDominanceReleased, bool isDropPickPossible, SubProSolveMode subproSolveMode,
               LabelingStrategy LabelingStrategy, subproblemAlgorithm subAlgorithm, bool constPortion,
               bool vehiclePortion, bool dynamicPricing, bool usePick, int nbPick, SortPaths sortPath, SortColumns sortColumn, int bigM,
               int solveTimeLimit, int populateTimeLimit, bool addOneRequestColumn, SolutionMode solutionMode, float MIPGap);

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
    bool isSuccessorsLimited_;
    bool isDropPickPossible_;
    LabelingStrategy LabelingStrategy_;
    int MaxLabel_;
    bool usePick_;
    int nbPick_;
    SortPaths pathSort_;
    bool addOneRequestColumn_;

    // Constructor and Destructor
    solverOption(bool isTruncated, int maxLabel, bool isDominanceReleased, int nbPick, SortPaths pathSort,
                 bool isSuccessorsLimited, bool isDropPickPossible, LabelingStrategy labelingStrategy, bool addOneRequestColumn);

    explicit solverOption(PParameters &MainParams);

    virtual ~solverOption();
    void disableHeuristics();
    bool areHeuristicsDisabled() const;

    // Display function
    std::string toString() const;
};

std::string boolToString(bool value);


#endif //PARAMETERS_H
