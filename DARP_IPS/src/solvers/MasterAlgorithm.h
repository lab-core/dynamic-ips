//
// Created by Ella on 2021-10-09.
//

#ifndef MASTERALGORITHM_H
#define MASTERALGORITHM_H
#include "data/Instance.h"
#include "solvers/GreedyModeler.h"


//enum selectionMode { NR = 0, RP = 1, CP = 2};

//---------------------------------------------------------------------------------------------
//  ISUD algorithm to solve the master problem
//---------------------------------------------------------------------------------------------

class MasterAlgorithm {
public:
//    PReducedProblem ReducedPro_;
    PComplementPro CompPro_;
    PReducedProblem ReducedPro_;
    PMasterPro MasterPro_;
    PDualAuxSolver DualAuxSolver_;

    vector2D<PRoute> availableRoutes_;
    //   vector2D<int> subProResults_;
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

    int maxIncDegree_;

    int nbCoveredTasks_;
    float maxReducedCost_;                  // max threshold for the reduced costs selection in CP
    float minReducedCost_;

    int RMPCounter_;                        // iteration of solving Master Problem in each epoch
    int SPIter_;                            // number of SP iterations
    int SPIters_;                           // total number of SP iterations
    int RPIter_;                            // number of RP iterations
    int CPIter_;                            // number of CP iterations
    int LPIter_;                            // number of LRMP iteration
    int MIPIter_;                           // number of Reduce Problem iteration
    int ZoomIter_;                          // number of Zoom iterations
    int CPSuccess_;                         // number of time CP succeed in finding integer
    int CPFails_;                           // number of time CP fails in finding integer
    int CGSuccess_;                         // number of time that CG where able to converge
    float objValue_;                        // objective value during MP iterations
    float previousObj_;
    float lpObjValue_;             // linear objective value
    float totalWaitTime_;          // total waiting time without penalties
    float GreedyObjValue_;         // objective value of Greedy method
    float MPEpochSolveTime_;       // save the total time used to solve MP models (RP, CG and MIP)
    float CPEpochSolveTime_;       // save the total time used to solve CP models
    bool CPBuilt_;                  // check whether CP model is built or not (one time during each iteration CP is built)
    float epochTime_;               // time passed inside one epoch
    float iterTime_;                // helps in calculating and saving epochTime_;
    float vehicleChange_;
    std::vector<std::pair<int,int>> adjacencyPairs_;
    std::vector<std::pair<int,int>> vehiclePairs_;

    myTools::Timer *masterTime_;
    myTools::Timer *RPTime_;
    myTools::Timer *CPTime_;

    myTools::Timer *MPBuildTime_;
    myTools::Timer *CPBuildTime_;
    myTools::Timer *ZOOMTime_;

    Tools::LogOutput* pLogIsudResultsStream_;
    Tools::LogOutput* pLogIterReqDualStream_;
    Tools::LogOutput* pLogIterVehDualStream_;

    // Constructor and Destructor
    explicit MasterAlgorithm(const InputPaths &inputPaths);

    virtual ~MasterAlgorithm();

    // Getters and Setters
    void setObjValue();

    void setAvailableTime();

    // this function creates initial routes serving only one request and fills zSolution_ with available requests
    // Reduced problem is also solved to initialized dual costs
    void initializeVehicles(const PInstance &pInst);
    void createInitialSolution(PInstance &pInst, const PGreedyModeler &GreedyModel);
    void initialization(PInstance &pInst, const InputPaths &inputPaths, const PGreedyModeler &GreedyModel);

    // this function updates the incompatibility degree of availableRoutes
    static void calcIncompatibilityBit(const PRoute &route, const PInstance &pInst);
    void calcIncompatibilityM(const PRoute &route) const;
    void updateIncDegreesBit(const PInstance &pInst) const;
    void updateIncDegreesM(const PInstance &pInst);
    void updateScore1(const PInstance &pInst);
    void updateScore(const PInstance &pInst);

    // this function updates the reduced cost for the routes in the pool
    void updateReducedCosts(const PInstance &pInst);

    void solveISUD(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime);
    void solveISUD_improved(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime);
    void solveMP_CG(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime);
    void solveMP_CP(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime);

    void solveCP(PInstance &pInst, int epoch, InputPaths &inputPaths, float subProTime);

    void solveRLMP(PInstance &pInst, int epoch, const InputPaths &inputPaths, float subProTime);
    void solveRMP(const PInstance &pInst, int epoch, const InputPaths &inputPaths, float subProTime);
    void solveMP_MIP(PInstance &pInst, int epoch, const InputPaths &inputPaths, float subProTime);
    void solveRP(PInstance &pInst, const InputPaths &inputPaths, int epoch, float subProTime);

    void setCurrentRoutes(const PInstance &pInst) const;


    // These functions are used to solve the Master problem (CG, MP and RP)
    void solveRP_LPINT(PInstance &pInst, int compDegree, const InputPaths &inputPaths);
    void solveMP_LP(PInstance &pInst, const InputPaths &inputPaths, int epoch, float subProTime);
    void solveMP_INT(PInstance &pInst, const InputPaths &inputPaths);

    // Display function
    std::string toString() const;
    std::string toStringTimersTotal() const;
    std::string toStringTimersAvg(int epoch) const;

    // function to evaluate available routes and find proper ones to be added to the models
    void updateRoutesToAdd(selectionMode selectMode, const PInstance &pInst);
    void updateRoutesToAddZoom() const;

    // function to save the reduced costs and incompatibility degree of the created routes
    void save_IncDegree_RDCost(const InputPaths &inputPaths, int epoch, int isudIter) const;
    std::string save_MPResults(int epoch, const std::string& model, int nbColumns, float reachTime, float subProTime,
                               float auxObj) const;
    void setAvailableTime(const PInstance &pInst, float elapsedTime, int iteration);
};


#endif //MASTERALGORITHM_H