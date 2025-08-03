//
// Created by Ella on 2021-10-09.
//

#ifndef MASTERALGORITHM_H
#define MASTERALGORITHM_H
#include "data/Instance.h"
#include "solvers/GreedyModeler.h"
#include "solvers/LagrangianSolver.h"


//enum selectionMode { NR = 0, RP = 1, CP = 2};

//---------------------------------------------------------------------------------------------
//  ISUD algorithm to solve the master problem
//---------------------------------------------------------------------------------------------

class MasterAlgorithm {
public:
    // solvers
    PComplementPro CompPro_;
    PCP_Reduced CPGurobiPro_;
    std::unique_ptr<LagrangianSolver> lagSolver_;


    std::vector<std::bitset<LABEL_BIT_SIZE>> vehicleRequestsBits_;

    vector2D<PRoute> availableRoutes_;
    int nbRoutes_;
    int nbColumnsAdded_;
    int nbVehicles_;
    float availableTime_;                   // time left from availableTime_ to solve MP
    float timeLimit_;                       // total available time to solve the MP

    // Solution containers
    std::vector<PRoute> routeSolution_;
    std::vector<PRequest> zSolution_;

    std::vector<PRoute> InRouteSolution_;   // Initial solutions
    std::vector<PRequest> InZSolution_;

    SpMat Basis_;
    SpMat BasisInv_;
    std::vector<PRoute> addedRouteToBase_;
    Eigen::VectorXd costVector_;

    int nbCoveredTasks_;
    float maxReducedCost_;                  // max threshold for the reduced costs selection in CP
    float minReducedCost_;

    int RMPCounter_;                        // iteration of solving Master Problem in each epoch
    int SPIter_;                            // number of SP iterations
    int SPIters_;                           // total number of SP iterations

    int LPIter_;                            // number of LRMP iteration
    int MIPIter_;                           // number of Reduce Problem iteration

    int RPIter_;                            // number of RP iterations
    int CPIter_;                            // number of CP iterations
    int ZoomIter_;                          // number of Zoom iterations

    int CPSuccess_;                         // number of time CP succeed in finding integer
    int CPFails_;                           // number of time CP fails in finding integer


    int CGSuccess_;                         // number of time that CG where able to converge
    float objValue_;                        // objective value during MP iterations
    float upperbound_;
    float previousObj_;
    float lpObjValue_;                      // linear objective value
    float totalWaitTime_;                   // total waiting time without penalties
    float GreedyObjValue_;                  // objective value of Greedy method

    float MPEpochSolveTime_;                // save the total time used to solve MP models (RP, CG and MIP)
    float CPEpochSolveTime_;                // save the total time used to solve CP models
    float epochTime_;                       // time passed inside one epoch
    float iterTime_;                        // helps in calculating and saving epochTime_;
    float vehicleChange_;
    std::vector<std::pair<int,int>> adjacencyPairs_;
    std::vector<std::pair<int,int>> vehiclePairs_;

    myTools::Timer *masterTime_;
    myTools::Timer *RPTime_;
    myTools::Timer *CPTime_;

    myTools::Timer *MPBuildTime_;
    myTools::Timer *CPBuildTime_;
    myTools::Timer *ZOOMTime_;

    Tools::LogOutput* pLogMPResultsStream_;
    Tools::LogOutput* pLogIterReqDualStream_;
    Tools::LogOutput* pLogIterVehDualStream_;

    // Constructor and Destructor
    explicit MasterAlgorithm(const InputPaths &inputPaths);
    virtual ~MasterAlgorithm();


    // this function creates initial routes serving only one request and fills zSolution_ with available requests
    // Reduced problem is also solved to initialized dual costs
    void initializeVehicles(const PInstance &pInst);
    void setInitialDuals(PInstance &pInst, InputPaths &inputPaths, int epoch);
    void setLastDuals(PInstance &pInst);
    void createInitialSolution(PInstance &pInst, const PGreedyModeler &GreedyModel);
    void initialization(PInstance &pInst, const InputPaths &inputPaths, const PGreedyModeler &GreedyModel);

    //This function updates the incompatibility degree of availableRoutes
    void updateIncDegrees(PInstance &pInst);
    void calcAdjacenyPairs(PInstance &pInst);

    void assessReqCompatibility(PRoute &route, PInstance &pInst);
    void assessReqVehCompatibility(PRoute &route, PInstance &pInst);

    void calcCompatibilityM1(PRoute &route, PInstance &pInst);
    void calcCompatibilityM2(PRoute &route, PInstance &pInst);
    void calcCompatibilityM2Full(PRoute &route, PInstance &pInst);

    void updateScore1(PInstance &pInst);
    void updateScore(PInstance &pInst);


    // Display function
    std::string toString() const;
    std::string toStringTimersTotal() const;
    std::string toStringTimersAvg(int epoch) const;

    // function to evaluate available routes and find proper ones to be added to the models
    // this function updates the reduced cost for the routes in the pool
    void updateReducedCosts(const PInstance &pInst);
    void updateRoutesToAdd(SelectionMode selectMode, PInstance &pInst, std::vector<PRoute> &routesToAdd);
    void reFillRoutesToAdd(PInstance &pInst, std::vector<PRoute> &routesToAdd);

    void reFillRoutesToAddCP(PInstance &pInst, std::vector<PRoute> &routesToAdd);

    void updateRoutesToAddZoom(std::vector<PRoute> &routesToAdd) const;

    // function to save the reduced costs and incompatibility degree of the created routes
    void save_IncDegree_RDCost(const InputPaths &inputPaths, int epoch, int isudIter) const;

    std::string save_MPResults(int epoch, const std::string& model, int nbColumns, float reachTime, float subProTime,
                               float auxObj) const;
    void setCurrentRoutes(const PInstance &pInst) const;
    void setObjValue();
    void setAvailableTime();
    void setAvailableTime(const PInstance &pInst, float elapsedTime, int iteration);
    void updateEpochTimers(PRuntimeMetrics &runtimeMetrics);
    std::string runtimesToString(PRuntimeMetrics &runtimeMetrics);
    void createFinalOutputString(const PInstance &pInst, float subproblemTime, float greedyRuntime);

    // CP solvers
    void solveCP_CPLEX(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime);
    void solveCP_Gurobi(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime);


    void buildBasis(const PInstance &pInst);
    void pickRoutesRoundRobin1(int columnsNeeded);
    void pickRoutesRoundRobin(int columnsNeeded, const PInstance &pInst);
    void computeBasisInverse();
    void computePsudoInverse(const PInstance &pInst);
    void buildCostVector(const PInstance &pInst);
    bool validateBasisStructure(const PInstance &pInst);
    void buildBasis2(const PInstance &pInst);
    bool isDuplicatePatternForVehicle(const boost::dynamic_bitset<> &pattern,
                                      const std::vector<boost::dynamic_bitset<>> &usedPatterns);
    void exportBasisToCSV(const std::string &filename);
};


#endif //MASTERALGORITHM_H