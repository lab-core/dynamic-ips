//
// Created by Ella on 1/28/2022.
//

#ifndef _LABELINGSUBPROBLEM_H
#define _LABELINGSUBPROBLEM_H

#include <set>
#include "data/Graph.h"
#include "utilities/MyTools.h"
#include <queue>

enum LabelStatus { ACTIVE = 0, DOMINATED = 1, INFEASIBLE = 2 };

class Label {
private:
    const unsigned int labelID_;      // request ID
public:
    static unsigned int labelCount_;  // Counter the number of requests
    const char* name_;
    int load_;                                      // consume capacity of the vehicle
    float passedTime_;                              // accumulated time of the path
    PVehicle *vehicle_;                             // the vehicle for which the route has created
    std::set<PNode> openNodes_;                     // list of open nodes that are picked up and not dropped off yet
    std::map<std::string, float> openReachTime_;    // time that vehicle is planned to reach each node
    std::set<PRequest> openRequests_;               // set of requests that have been started but not completed
    std::set<PRequest> completedRequests_;          // set of completed requests
    vector<PNode> pathNodes_;                       // list of nodes in the path of the vehicle
    std::set<std::string> onboards_;                // set of onboard nodes from previous epochs
    double reducedCost_;
    PNode *currentNode_;
    double totalDelay_;
    LabelStatus status_;
    int nbPickUp_;


    // Constructor and Destructor
    Label(PVehicle *vehicle, float reducedCost, PNode source);
    Label(const Label &label);
    virtual ~Label();
    // Getters and Setters
    const unsigned int getLabelId() const;

    bool operator() (const Label &rhs) const;
    void extend(PNode &outNode);
    // this function check the feasibility of the label before extension
    bool isExtendFeasible(PNode &outNode, int maxPickUp);
    bool isDominated(PLabel &otherLabel, SubSolveStatus status);
    // this function examine the label to be sure that it leads to a route with negative reduced cost
    bool isEliminated(int maxPickUp, PGraph &graph);
    PRoute labelToRoute();
    // Display function
    std::string toString() const;

};
struct CmpLabels {
    inline bool operator () (const PLabel &lhs, const PLabel &rhs) {
        return lhs->reducedCost_ > rhs->reducedCost_;
    }
};

 inline bool operator < (const PLabel &lhs, const PLabel &rhs) {return (lhs->reducedCost_ < rhs->reducedCost_); }
//-----------------------------------------------------------------------------
//  Labeling Sub problem class
//-----------------------------------------------------------------------------

class LabelingSubProblem {
public:
    PVehicle* Vehicle_;                         // the vehicle for which we are solving the sub problem
    PGraph subGraph_;                           // the graph of the feasible solution for the vehicle
    std::vector<PRequest> subRequests_;         // List of requests
    std::priority_queue<PLabel, vector<PLabel> , CmpLabels> workingLabels;  // list of active labels, ordered based on reduced cost
    int nbDominated_;                           // number of labels removed via Domination Rules
    int nbEliminated_;                          // number of labels removed via Elimination Rules
    int nbGenerated_;

    // Constructor and Destructor
    LabelingSubProblem(PVehicle &vehicle);

    virtual ~LabelingSubProblem();
    // calculation of penalties and initialization of the subgraph
    void initSubGraph(PInstance &pInst);
    // this function sort the list of nodes based of their dual values
    void sortNodes();
    // main function of the dynamic programming
    void solveDynamic(int maxPickUp);
    // function to convert solution to routes and save them in vehicle object
    void SolutionToRoutes(std::vector<PRoute> &availableRoutes, std::map<std::string , PRoute> &generatedRoutes);
    // Display function
    std::string toString() const;
};
typedef std::shared_ptr<LabelingSubProblem> PLabelingSubPro;
#endif //_LABELINGSUBPROBLEM_H
