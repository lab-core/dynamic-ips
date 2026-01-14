//
// Created by Elahe Amiri on 2025-11-02.
//

#ifndef CP_GUROBI_CPP_BASESOLVER_H
#define CP_GUROBI_CPP_BASESOLVER_H

#include "data/Instance.h"
#include "utilities/Tools.h"
#include "utilities/MyTools.h"
#include "utilities/InputPaths.h"
#include "CplexSolver/MIPSolver.h"
#include "CplexSolver/CPLEXSubProblem.h"
#include "GurobiSolver/MP_Gurobi.h"
#include "solvers/LabelingSubProblem.h"
#include "solvers/CG_Algorithm.h"
#include "solvers/ISUD_Algorithm.h"


//---------------------------------------------------------------------------------------------
//  BaseSolver class
//  This class serves as a base for different solver implementations.
//---------------------------------------------------------------------------------------------

class BaseSolver {
public:
    PSolverOption subProOptions_;                         // Subproblem solver options
    PRuntimeMetrics runtimeMetrics_;                      // Runtime metrics
    myTools::SharedVector<PLabel> labelsPool_;            // Pool of generated labels used for recycling
    PMasterAlgorithm MP_solver_;                          // Master Problem solver
    PMIPSolver MIPModel_;                                 // MIP Solver for solving the 3-index MIP formulation
    PGreedyModeler GreedyModel_;                          // Greedy modeler (fast insertion heuristic)


    // Shared state
    float elapsedTime_;                                   // elapsed time of the simulation  
    float avgEpochRuntime_;                               // average epoch runtime
    int epoch_;                                           // current epoch number
    float objValue_;                                      // objective value of the current epoch

    // Timers
    myTools::Timer *simulationTime_;                      // timer for total simulation time
    myTools::Timer *subProblemTime_;                      // timer for solving subproblems
    myTools::Timer *preprocessTime_;                      // time for preprocessing
    myTools::Timer *rebalancingTime_;                     // timer for determining the rebalancing start point
    myTools::Timer *rebalancingProcessTime_;              // time for rebalancing
    int rebalanceTime_;                                   // keep track of time since last rebalancing

    // Logging
    Tools::LogOutput* pLogRunTimesStream_;               // log for run times
    Tools::LogOutput* pLogEpochSubRuntimeStream_;        // log for subproblem run times at each epoch
    Tools::LogOutput* pLogEpochVehicleStream_;           // log for subproblem run times at each epoch

    // Subproblem tracking
    int nbOnePick_;                                       // number of SPs solved with single pick-up limit    
    int nbTwoPick_;                                       // number of SPs solved with two pick-up limit
    int nbThreePick_;                                     // number of SPs solved with three pick-up limit
    int nbRecycle_;                                       // number of SPs re-optimized   
    float heuristicEpochObj_;                             // objective value when CG stopped due to epoch time limit 

    // Constructor and Destructor
    BaseSolver(const PInstance & mainInst, const InputPaths &inputPaths);
    virtual ~BaseSolver();
    // Pure virtual function to perform the simulation
    virtual void doSimulation(PInstance & shared, InputPaths & input_paths, bool middle_save, float save_time) = 0;

    // function to determine which vehicles to select for solving subproblems
    static void selectVehiclesForSubproblem(const PInstance &EpochInst, int iter);

    // function to configure labeling subproblem for a given vehicle (set pickups limit, ...)
    void configureLabelingSubproblem(PLabelingSubPro &subProblem, PVehicle &vehicleObj, PInstance &EpochInst, int iter,
        vector2D<PRoute> &availableRoutes);

    // Template function to solve the subproblems
    template<typename SubProblemType>
    bool solve_SP(PInstance & EpochInst, PInstance & mainInst, int &iter, int &nbNegativeFound,
        vector2D<PRoute> &availableRoutes, float availableTime, int &nbRoutes, std::vector<std::unordered_set<std::string>> &duplicatesRoutes);

    // functions related to rebalancing policy based on Alonso mora Assignment
    void returnVehiclesAssign(const PInstance & EpochInst) const;
    void solveGurobiAssignment(const PInstance & EpochInst, std::vector<PVehicle> &idleVehicles) const;
    void solveCplexAssignment(const PInstance & EpochInst, std::vector<PVehicle> &idleVehicles) const;

    // function related to rebalancing policy based on returning to source
    void returnVehiclesSource(const PInstance & EpochInst) const;

    // function related to rebalancing policy based on returning to high demand zones
    void returnVehiclesZone(const PInstance & EpochInst) const;

    // function to reconstruct available routes from th elast epoch
    static void reconstructAvailableRoutes(const PInstance &mainInst, vector2D<PRoute> &availableRoutes);

    // function to build the epoch instance from the main instance
    void buildEpochInstance(PInstance &mainInst, PInstance &EpochInst, float elapsedTime, int &nbReceivedRequest);

    // function to log epoch information at the start of each epoch
    void logEpochInfo(int epoch, float elapsedTime, float simulationTime, float epochRuntime, InputPaths &inputPaths);

    // function to handle vehicle returns based on the return policy
    void handleVehicleReturn(PInstance &EpochInst);

    // function to update available routes by removing those including certain requests
    static void updateAvailableRoutes(boost::dynamic_bitset<> &removedRequests, vector2D<PRoute> &availableRoutes, PInstance &EpochInst);

    // function to create one-stop routes with enumeration
    static void CreateOneStopRoutes(const PVehicle &vehicle, std::vector<PRoute> &availableRoutes, const PInstance & pInst,
                                    const PInstance &EpochInst, int &nbNegative);

    // function to solve ad determine the dispatching plan for one epoch of the simulation
    void solveEpoch(PInstance & EpochInst, PInstance & mainInst, InputPaths &inputPaths);

    // function to print epoch runTime to file
    std::string saveRuntimes(const PInstance & EpochInst);
    std::string saveRuntimesGreedy(const PInstance & EpochInst);

    // function to generate final output string of the simulation
    std::string toString(const PInstance & mainInst) const;

    // function to create final output for greedy solution
    void createFinalOutputString(const PInstance &pInst) const;
};

//---------------------------------------------------------------------------------------------
//  RuntimeMetrics structure
//  This structure holds various runtime metrics for performance analysis.
//---------------------------------------------------------------------------------------------
struct RuntimeMetrics {
    // Basic epoch information
    float epochRuntime_;                        // total runtime of the epoch

    // Master problem times
    float masterEpochTime_;                     // time spent solving the master problem    
    float RPEpochTime_;                         // time spent solving the RP (Reduced Problem) 
    float CPEpochTime_;                         // time spent solving the CP (Complementary Problem)    
    float RPEpochBuildTime_;                    // time spent building the RP
    float CPEpochBuildTime_;                    // time spent building the CP 
    float isudMIPEpochTime_;                    // time spent solving ISUD MIP

    // Subproblem and preprocessing times
    float subproEpochTime_;                     // time spent solving the subproblems
    float GreedyTime_;                          // time spent in greedy heuristic

    // Algorithm metrics
    int nbDominated_;                           // number of labels removed via Domination Rules
    int nbGenerated_;                           // number of generated labels
    int nbPrunedArcs_;                          // number of labels detected as non-promising by pruned arcs
    int nbPrunedPath_;                          // number of labels detected as Unreachable by soft time window
    int nbEliminated_ ;                         // number of labels detected as Unreachable due to travel time
    int nbRecycledColumns_;                     // number of labels recycled from routes    
    int nbNegativeFound_;                       // number of negative reduced cost columns found
    int nbColumnsLess_50_;                      // number of vehicles with less than 50 columns generated
    int nbColumnsLess_100_;                     // number of vehicles with less than 100 columns generated  
    int nbColumnsLess_200_;                     // number of vehicles with less than 200 columns generated
    int nbRoutes_;                              // number of routes generated in the epoch 

    // Constructor
    RuntimeMetrics() : epochRuntime_(0.0f), masterEpochTime_(0), RPEpochTime_(0), CPEpochTime_(0), RPEpochBuildTime_(0),
                       CPEpochBuildTime_(0),isudMIPEpochTime_(0), subproEpochTime_(0.0f), GreedyTime_(0),nbDominated_(0),
                       nbGenerated_(0), nbPrunedArcs_(0), nbPrunedPath_(0),nbEliminated_(0),nbRecycledColumns_(0),
                       nbNegativeFound_(0),nbColumnsLess_50_(0),nbColumnsLess_100_(0),nbColumnsLess_200_(0),nbRoutes_(0){}

    // functions to reset and update metrics                  
    void resetSubproblemMetrics();
    void updateSubproblemMetrics(const PLabelingSubPro & subProblem );
    void updateSubproblemMetrics(const PCplexSubPro & subProblem );
};

#endif //CP_GUROBI_CPP_BASESOLVER_H