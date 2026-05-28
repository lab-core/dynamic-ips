//
// Created by Elahe Amiri on 2022-10-13.
//

#ifndef PARAMETERS_H
#define PARAMETERS_H

#include "utilities/MyTools.h"
#include "utilities/ConfigParser.h"

//-----------------------------------------------------------------------------
//  SolverBase Class - Base class for solver-related parameters
//-----------------------------------------------------------------------------
class SolverBase {
public:
    // label setting strategies
    bool isTruncated_{};                                    // truncated or complete labeling
    int MaxLabel_{};                                        // maximum number of labels per node for truncated labeling
    int MaxCommittedLabel_{};                               // maximum number of committed labels per node for truncated labeling
    bool isDominanceReleased_{};                            // whether in domination one rule is released or not
    bool pruneNodes_{};                                     // whether to prune nodes (1st pruning policy) is used in labeling
    bool pruneArcs_{};                                      // whether to prune arcs (2nd pruning policy) is used in labeling
    bool discardSuboptimalPath_{};                          // whether to discard suboptimal paths (3rd pruning policy) in labeling
    bool isDropPickPossible_{};                             // whether pick-up is allowed when a drop-off is already visited
    LabelingStrategy LabelingStrategy_{};                   // labeling strategy: PUSHING, PULLING, RE_PULLING
    LabelingReOptimizeStrategy labelingReOptimizeStrategy_; // labeling re-optimization strategy: RE_INSERT, BY_BASIS, BY_POOL
    bool reoptimizeSP_{};                                   // whether to re-optimize subproblems or not
    int nbPick_{};                                          // number of pick-ups limits in routes, used in labeling
    SortPaths sortPath_;                                    // path sorting criteria used in truncated labeling
    int newRequestLimit_{};                                 // limit on the number of new requests accepted to decide whether to re-optimize
    float Wait_W1_;                                         // weight for wait time in the objective function
    float Ride_W2_;                                         // weight for trip delay in the objective function       
    bool Req_W3_;                                           // whether to consider request count in the objective function
    bool Ride_W4_;                                          // whether to consider ride time in the objective function
    bool Relative_W5_;                                      // whether to consider relative detour in the objective function
    bool Normal_W6_;

    // Constructor and Destructor
    SolverBase(bool isTruncated, int maxLabel, int MaxCommittedLabel, bool isDominanceReleased,
               bool pruneNodes, bool pruneArcs, bool discardSuboptimalPath, bool isDropPickPossible,
               LabelingStrategy LabelingStrategy, LabelingReOptimizeStrategy labelingReOptimizeStrategy,
               bool reoptimizeSP, int nbPick, SortPaths pathSort, int newRequestLimit, float wait_W1,
               float ride_W2, bool req_W3, bool ride_W4, bool relative_W5, bool normal_W6);

    virtual ~SolverBase() = default;

    // Pure virtual function for display
    virtual std::string toString() const = 0;
};

//-----------------------------------------------------------------------------
//  Parameters Struct
//-----------------------------------------------------------------------------
struct Parameters : public SolverBase {
public:
    // model Parameters
    float alphaParam_{};                    // parameter for defining the max ride time limit (alpha * direct travel time)
    float betaParam_{};                     // parameter for defining the max ride time limit (beta + direct travel time)
    float deltaPram_{};                     // parameter for defining penalty for unassigned requests 
    float epochLength_{};                   // length of each epoch in dynamic solution                                      
    int penaltyL_{};                        // parameter for defining penalty for unassigned requests  
    float committedTime_{};                 // length of time period that stops are frozen in anytime solution
    float informTimeLimit_;                 // time limit to inform the customers before their requested pickup time
    float pickupDeviationWindow_;           // allowable deviation window for committed pickup time

    Approach approach_;                     // solution approach: ISUD, CG, Greedy
    MainAlgorithm mainAlgorithm_;           // main algorithm: GREEDY, MIP, RT_CG, MP_ISUD, MP_MIP, MP_CP, A_CG
    SolutionMode solutionMode_;             // solution mode: STATIC, DYNAMIC, ANYTIME
    
    // CG Parameters
    InitialDual initialDual_;               // the strategy used for initializing dual variables
    DualMethod dualMethod_{};               // the method used for extracting dual variables at the end of each epoch
    bool smoothDual_;                       // whether to use smoothing dual or not
    WarmStart initialStart_;                // warm start strategy: GREEDY_START, PRE_SOLUTION, EMPTY_ROUTES
    int numIter_;                           // number of iterations for column generation at each epoch
    int nbColumn_;                          // number of columns per vehicle to add to RMP at each iteration
    SortColumns sortColumn_;                // column sorting criteria used in adding columns to RMP
    
    // Vehicle Return Parameters
    bool vehicleReturn_;                   // whether the rebalancing of the vehicles is done by returning to crowded areas
    float WaitForReturn_;                  // The time that a vehicle remains idle before returning to crowded areas
    ReturnType returnPolicy_;              // return policy for vehicles: TO_SOURCE, ASSIGN
    float maxWait_;                        // maximum wait time to define crowded zones for vehicle rebalancing
    

    // ISUD parameters
    int MIP_maxIncDegree_;                 // max incompatibility degree for Zoom
    int CP_IncDegree_;                     // max incompatibility degree for CP
    bool reducedCP_;                       // whether to use reduced CP model or not
    float minImp_;                         // minimum improvement threshold for ISUD
    bool useZoom_;                         // whether to use Zoom or not
    ISUDVariant isudVariant_;              // ISUD solve strategy: ISUD_MIP_RP or ISUD_PIVOT_RP

    // Parameters related to the subproblem
    SubproblemAlgorithm subAlgorithm_;     // subproblem algorithm: MIP_SUB, LABEL_SETTING
    bool vehiclePortion_{};                // whether to solve subproblems for a portion of vehicles or all vehicles
    bool dynamicPricing_{};                // whether to use dynamic pickup limits in labelling
    bool partialPricing_{};                // whether to use partial pickup limits in labelling
    bool routeRecycle_{};                  // whether to recycle routes from prior epoch
    

    //Solver Parameters
    int nbThreads_{};                      // number of threads used in parallel computations
    int bigM_{};                           // big M value used in MIP formulations
    int solveTimeLimit_{};                 // time limit for solving MIP models
    int populateTimeLimit_{};              // time limit for populating MIP models
    float MIPGap_{};                       // optimality gap for MIP models
    double reducedCostThreshold_{};        // threshold for pruning high-reduced-cost routes in BY_POOL reoptimization

    // other Parameters
    bool greedyReOptimize_;                 // restart greedy (re-assigning) considering the current state of the system 
    float timeWindow_;                      // time window for rejecting requests

    // Constructor and Destructor
    Parameters(float alphaParam, float betaParam, float deltaPram, int epochLength, int penaltyL,
               int committedTime, int nbThreads, InitialDual initialDual, DualMethod dualMethod,
               MainAlgorithm mainAlgorithm, int numIter, bool greedyReOptimize, bool vehicleReturn, float timeWindow,
               float WaitForReturn, WarmStart initialStart,
               int MIP_maxIncDegree, int CP_IncDegree, bool reducedCP, float minImp, bool useZoom,
               ISUDVariant isudVariant,
               int nbColumn, bool isTruncated, int maxLabel, int MaxCommittedLabel, bool pruneNodes, bool pruneArcs,
               bool discardSuboptimalPath, bool isDominanceReleased, bool isDropPickPossible,
               LabelingStrategy LabelingStrategy, SubproblemAlgorithm subAlgorithm, bool constPortion,
               bool vehiclePortion, bool dynamicPricing, bool partialPricing, bool routeRecycle, bool reoptimizeSP,
               int nbPick, SortPaths sortPath, SortColumns sortColumn, int bigM, int newRequestLimit,
               int solveTimeLimit, int populateTimeLimit, SolutionMode solutionMode, float MIPGap, int informTimeLimit,
               int pickupDeviationWindow, ReturnType returnPolicy, float maxWait,
               LabelingReOptimizeStrategy labelingReOptimizeStrategy, bool smoothDual, float wait_W1, float ride_W2,
               bool req_W3, bool ride_W4, bool relative_W5, bool normal_W6, double reducedCostThreshold);

    virtual ~Parameters();

    // Display function
    [[nodiscard]] std::string toString() const override;
    [[nodiscard]] std::string toStr() const;
};


//-----------------------------------------------------------------------------
//  Solver Option Struct
//-----------------------------------------------------------------------------
struct solverOption: public SolverBase {
    // Constructor and Destructor
    solverOption(bool isTruncated, int maxLabel, int MaxCommittedLabel, bool isDominanceReleased,
               bool pruneNodes, bool pruneArcs, bool discardSuboptimalPath, bool isDropPickPossible,
               LabelingStrategy labelingStrategy, LabelingReOptimizeStrategy labelingReOptimizeStrategy,
               bool reoptimizeSP, int nbPick, SortPaths pathSort, int newRequestLimit, float wait_W1,
               float ride_W2, bool req_W3, bool ride_W4, bool relative_W5, bool normal_W6);

    // Construct from Parameters
    explicit solverOption(const PParameters &MainParams);
    virtual ~solverOption();

    // Display function
    std::string toString() const override;

    // Functions to enable/disable labelling accelerations (heuristics)
    void disableHeuristics();
    void enableHeuristics(const PParameters &MainParams);

};

std::string boolToString(bool value);


#endif //PARAMETERS_H