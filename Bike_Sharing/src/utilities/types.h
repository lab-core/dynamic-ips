//
// Created by Elahe Amiri on 2024-03-08.
//

#ifndef DARP_IPS_TYPES_H
#define DARP_IPS_TYPES_H

#include <iostream>

// useful types
class Instance;
typedef std::shared_ptr<Instance> PInstance;
class Task;
typedef std::shared_ptr<Task> PTask;
class Stop;
typedef std::shared_ptr<Stop> PStop;
class Vehicle;
typedef std::shared_ptr<Vehicle> PVehicle;
class Graph;
typedef std::shared_ptr<Graph> PGraph;
class Node;
typedef std::shared_ptr<Node> PNode;
class Route;
typedef std::shared_ptr<Route> PRoute;
class RoutePlan;
typedef std::shared_ptr<Route> PRoutePlan;
class Label;
typedef std::shared_ptr<Label> PLabel;
struct Parameters;
typedef std::shared_ptr<Parameters> PParameters;
struct solverOption;
typedef std::shared_ptr<solverOption> PSolverOption;
class StopLabel;
typedef std::shared_ptr<StopLabel> PStopLabel;
class MasterModeler;
typedef std::shared_ptr<MasterModeler> PMasterModeler;
class LabelingSubProblem;
typedef std::shared_ptr<LabelingSubProblem> PLabelingSubPro;

// extern PTravelTime travelMat;

enum LabelingStrategy { PUSHING = 0, PULLING = 1};
enum MainAlgorithm {GREEDY = 0, MP_CG = 1, MP_MIP = 2};
enum NodeStatus { DEFINED = 0, PLANNED = 1, DONE = 2 , COMMITTED = 3};
enum SortVehicle { DUAL = 0, DEPART_TIME = 1, ROURE_SIZE = 2, BEST_REDUCE_COST = 3, SCORE = 4};
enum LabelStatus { ACTIVE = 0, DOMINATED = 1, INACTIVE = 2, OUTBOUND = 3, TERMINATED = 4};
enum selectionMode { NR = 0, RP = 1, CP = 2};
enum SortPaths {L_SCORE = 0, RD_COST = 1};
enum VarSign { POSITIVE, NEGATIVE };
enum SolutionStatus { NOT_SOLVED = 0, NEGATIVE_VALUE = 1, POSITIVE_VALUE = 2, FRACTIONAL = 3 , INFEASIBLE = 4};
enum TaskStatus {NO_ACTION = 0, COMPLETED = 1};
static const std::vector<std::string> reqStatusName = {
        "NO_ACTION", "ON_BOARD ", "COMPLETED" };

static const std::vector<std::string> SortPathsName = {
        "PATH_SCORE  ",
        "REDUCED_COST"};

static const std::vector<std::string> LabelingStrategyName = {
        "PUSHING",
        "PULLING" };

static const std::vector<std::string> subAlgorithmName = {
        "CPLEX        ",
        "LABEL_SETTING" };

static const std::vector<std::string> mainAlgorithmName = {
        "GREEDY",
        "MIP_CPLEX",
        "MP_CG",
        "MP_ISUD",
        "MP_MIP",
        "MP_CP"};

static const std::vector<std::string> warmStartName = {
        "GREEDY_START",
        "PRE_SOLUTION",
        "EMPTY_START "};

static const std::vector<std::string> solutionModeName = {
        "STATIC",
        "DYNAMIC",
        "ANYTIME"};

static const std::vector<std::string> InitialDualName = {
        "LAST_SOl",
        "PENALTY "};
static const std::vector<std::string> SubProSolveStartName = {
        "NOT_RESTRICTED     ",
        "TIME_RESTRICTED    ",
        "NUM_PICK_RESTRICTED"};

// Different node types and their names
enum NodeType { DEPART_NODE, SINK_NODE, TASK_STATION};
static const char *NodeTypeStr[] = {
        "DEPART ",
        "SINK   ",
        "STATION"
};

#endif //DARP_IPS_TYPES_H
