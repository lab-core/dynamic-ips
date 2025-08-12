//
// Created by Elahe Amiri on 2025-06-24.
//

#ifndef TYPES_H
#define TYPES_H

#include <memory>
#include <vector>
#include <array>
#include <ilcplex/ilocplex.h>

//-----------------------------------------------------------------------------
//  Constants and Configuration
//-----------------------------------------------------------------------------
namespace constants {
    constexpr int LARGE_CONSTANT = 9999999;
    constexpr int MAX_BIT_SIZE = 4000;
    constexpr int LABEL_BIT_SIZE = 4000;
    constexpr int MAX_ZONE = 350;
    constexpr int SENTENCE_SIZE = 50;
    constexpr int SERVICE_TIME = 30;
    constexpr int SECONDS_PER_HOUR = 3600;
    constexpr int SECONDS_PER_MINUTE = 60;
    constexpr float EPSILON = 0.1f;
}

//-----------------------------------------------------------------------------
//  Forward Declarations
//-----------------------------------------------------------------------------
class Instance;
class Request;
class Vehicle;
class Graph;
class Node;
class Route;
class Zone;
class ReducedProblem;
class ComplementPro;
class DualAuxSolver;
class Label;
class StopLabel;
class GreedyRoute;
class GreedyModeler;
class MIPMasterProblem;
class LabelingSubProblem;
class CPLEXSubProblem;
struct Parameters;
struct solverOption;
struct insertPosition;
struct ProgramConfig;
struct RuntimeMetrics;
class MP_Gurobi;
class RP_Gurobi;
class CP_Gurobi;
class CP_Reduced;
class GurobiModeler;
class MasterAlgorithm;
class CG_Algorithm;
class ISUD_Algorithm;
class MIPSolver;

//-----------------------------------------------------------------------------
//  Smart Pointer Type Aliases
//-----------------------------------------------------------------------------
using PInstance = std::shared_ptr<Instance>;
using PRequest = std::shared_ptr<Request>;
using PVehicle = std::shared_ptr<Vehicle>;
using PGraph = std::shared_ptr<Graph>;
using PNode = std::shared_ptr<Node>;
using PRoute = std::shared_ptr<Route>;
using PZone = std::shared_ptr<Zone>;
using PReducedProblem = std::shared_ptr<ReducedProblem>;
using PComplementPro = std::shared_ptr<ComplementPro>;
using PDualAuxSolver = std::shared_ptr<DualAuxSolver>;
using PLabel = std::shared_ptr<Label>;
using PStopLabel = std::shared_ptr<StopLabel>;
using PGreedyRoute = std::shared_ptr<GreedyRoute>;
using PGreedyModeler = std::shared_ptr<GreedyModeler>;
using PMasterPro = std::shared_ptr<MIPMasterProblem>;
using PLabelingSubPro = std::shared_ptr<LabelingSubProblem>;
using PCplexSubPro = std::shared_ptr<CPLEXSubProblem>;
using PParameters = std::shared_ptr<Parameters>;
using PSolverOption = std::shared_ptr<solverOption>;
using PInsertPosition = std::shared_ptr<insertPosition>;
using PConfig = std::unique_ptr<ProgramConfig>;
using PMP_Gurobi = std::shared_ptr<MP_Gurobi>;
using PRP_Gurobi = std::shared_ptr<RP_Gurobi>;
using PGurobiModeler = std::shared_ptr<GurobiModeler>;
using PCP_Gurobi = std::shared_ptr<CP_Gurobi>;
using PCP_Reduced = std::shared_ptr<CP_Reduced>;
using PMasterAlgorithm = std::shared_ptr<MasterAlgorithm>;
using PCG_Algorithm = std::unique_ptr<CG_Algorithm>;
using PISUD_Algorithm = std::unique_ptr<ISUD_Algorithm>;
using PMIPSolver = std::unique_ptr<MIPSolver>;
using PRuntimeMetrics = std::unique_ptr<RuntimeMetrics>;
//-----------------------------------------------------------------------------
//  Container Type Aliases
//-----------------------------------------------------------------------------
template<class T> using vector2D = std::vector<std::vector<T>>;
using IloNumVar2D = IloArray<IloNumVarArray>;
using IloNumVar3D = IloArray<IloNumVar2D>;
using IloNum2D = IloArray<IloNumArray>;

typedef Eigen::SparseMatrix<double> SpMat;
typedef Eigen::Triplet<double> Triplet;
//-----------------------------------------------------------------------------
//  Enumerations
//-----------------------------------------------------------------------------
enum LabelingStrategy : int {
    PUSHING = 0,
    PULLING = 1,
    RE_PULLING = 2
};

enum SubproblemAlgorithm : int {
    CPLEX_SUB = 0,
    LABEL_SETTING = 1
};

enum MainAlgorithm : int {
    GREEDY = 0,
    MIP_CPLEX = 1,
    RT_CG = 2,
    MP_ISUD = 3,
    MP_MIP = 4,
    MP_CP = 5,
    A_CG = 6
};

enum Approach : int {
    ISUD = 0,
    CG = 1,
    Greedy = 2
};

enum SolutionMode : int {
    STATIC = 0,
    DYNAMIC = 1,
    ANYTIME = 2
};

enum WarmStart : int {
    GREEDY_START = 0,
    PRE_SOLUTION = 1,
    EMPTY_ROUTES = 2
};

enum InitialDual : int {
    PENALTIES = 0,
    LAST_LP = 1,
    ADJUSTED = 2,
    INIT_CP = 3,
    BARRIER = 4,
    LAGRANGIAN = 5,
    INITIAL_LP = 6,
    GREEDY_D = 7
};

enum DualMethod : int {
    LMP = 0,
    AUX_D = 1,
    AUX_P = 2,
    AUX_BOX = 3,
    LP_CP = 4,
    INTERIOR = 5,
    LAGRANGE = 6
};

enum NodeStatus : int {
    DEFINED = 0,
    PLANNED = 1,
    DONE = 2,
    COMMITTED = 3
};

enum SortVehicle : int {
    DUAL = 0,
    DEPART_TIME = 1,
    ROUTE_SIZE = 2,
    BEST_REDUCE_COST = 3,
    SCORE = 4
};

enum LabelStatus : int {
    ACTIVE = 0,
    DOMINATED = 1,
    INACTIVE = 2,
    OUTBOUND = 3,
    TERMINATED = 4
};

enum SelectionMode : int {
    NR = 0,
    RP = 1,
    CP = 2
};

enum SortPaths : int {
    L_SCORE = 0,
    RD_COST = 1,
    LAMBDA = 2
};

enum SortColumns : int {
    NORMAL_RC = 0,
    RC = 1,
    LAMBDA_S = 2,
    COMP_C = 3,
    WAIT_S = 4
};

enum VarSign : int {
    POSITIVE = 0,
    NEGATIVE = 1
};

enum SolutionStatus : int {
    NOT_SOLVED = 0,
    NEGATIVE_VALUE = 1,
    POSITIVE_VALUE = 2,
    FRACTIONAL = 3,
    INFEASIBLE = 4
};

enum RequestStatus : int {
    NO_ACTION = 0,
    ON_BOARD = 1,
    COMPLETED = 2,
    REJECTED = 3
};

enum ReturnType : int {
    TO_SOURCE = 0,
    ZONE = 1,
    ASSIGN = 2
};

enum NodeType : int {
    SOURCE = 0,
    SINK = 1,
    PICKUP = 2,
    DROPOFF = 3
};

enum ModelSOLVER: int {
    CPLEX = 0,
    GUROBI = 1
};

//-----------------------------------------------------------------------------
//  String Mappings for Enums
//-----------------------------------------------------------------------------
namespace enum_strings {
    constexpr std::array<const char*, 3> labelingStrategyNames = {
        "PUSHING", "PULLING" , "RE_PULLING"
    };

    constexpr std::array<const char*, 2> subproblemAlgorithmNames = {
        "CPLEX        ", "LABEL_SETTING"
    };

    constexpr std::array<const char*, 7> mainAlgorithmNames = {
        "GREEDY", "MIP_CPLEX", "RT_CG", "MP_ISUD", "MP_MIP", "MP_CP", "A_CG"
    };

    constexpr std::array<const char*, 3> solutionModeNames = {
        "STATIC", "DYNAMIC", "ANYTIME"
    };

    constexpr std::array<const char*, 4> warmStartNames = {
        "GREEDY_START", "PRE_SOLUTION", "EMPTY_ROUTES ", "IP_SOLUTION "
    };

    constexpr std::array<const char*, 8> initialDualNames = {
        "PENALTIES", "LAST_LP", "ADJUSTED", "INIT_CP", "BARRIER", "LAGRANGE" , "INITIAL_LP", "GREEDY_D"
    };

    constexpr std::array<const char*, 7> dualMethodNames = {
        "LINEAR", "AUX_D", "AUX_P", "AUX_BOX", "LP_CP", "BARRIER", "LAGRANGE"
    };

    constexpr std::array<const char*, 4> nodeStatusNames = {
        "DEFINED", "PLANNED", "DONE", "COMMITTED"
    };

    constexpr std::array<const char*, 5> sortVehicleNames = {
        "DUAL", "DEPART_TIME", "ROUTE_SIZE", "BEST_REDUCE_COST", "SCORE"
    };

    constexpr std::array<const char*, 5> labelStatusNames = {
        "ACTIVE", "DOMINATED", "INACTIVE", "OUTBOUND", "TERMINATED"
    };

    constexpr std::array<const char*, 3> selectionModeNames = {
        "NR", "RP", "CP"
    };

    constexpr std::array<const char*, 3> sortPathsNames = {
        "PATH_SCORE  ", "REDUCED_COST", "LAMBDA_SCORE"
    };

    constexpr std::array<const char*, 5> sortColumnsNames = {
        "PATH_SCORE  ", "REDUCED_COST", "LAMBDA_SCORE", "COMP_SCORE ", "WAIT_SCORE "
    };

    constexpr std::array<const char*, 2> varSignNames = {
        "POSITIVE", "NEGATIVE"
    };

    constexpr std::array<const char*, 5> solutionStatusNames = {
        "NOT_SOLVED", "NEGATIVE_VALUE", "POSITIVE_VALUE", "FRACTIONAL", "INFEASIBLE"
    };

    constexpr std::array<const char*, 4> requestStatusNames = {
        "NO_ACTION", "ON_BOARD", "COMPLETED", "REJECTED"
    };

    constexpr std::array<const char*, 3> returnTypeNames = {
        "TO_SOURCE", "ZONE     ", "ASSIGN   "
    };

    constexpr std::array<const char*, 4> nodeTypeNames = {
        "SOURCE ", "SINK   ", "PICKUP ", "DROPOFF"
    };

    constexpr std::array<const char*, 2> modelSolverNames = {
        "CPLEX ", "GUROBI"
    };

    constexpr std::array<const char*, 3> approachNames = {
        "ISUD ", "CG  ", "Greedy"
    };

}

//-----------------------------------------------------------------------------
//  Enum Utility Functions (Completed)
//-----------------------------------------------------------------------------
namespace enum_utils {
    // Primary template declaration
    template<typename EnumType>
    const char* toString(EnumType value);

    // Specializations for toString
    template<>
    inline const char* toString<LabelingStrategy>(LabelingStrategy value) {
        auto index = static_cast<size_t>(value);
        return (index < enum_strings::labelingStrategyNames.size()) ?
               enum_strings::labelingStrategyNames[index] : "UNKNOWN";
    }

    template<>
    inline const char* toString<SubproblemAlgorithm>(SubproblemAlgorithm value) {
        auto index = static_cast<size_t>(value);
        return (index < enum_strings::subproblemAlgorithmNames.size()) ?
               enum_strings::subproblemAlgorithmNames[index] : "UNKNOWN";
    }

    template<>
    inline const char* toString<MainAlgorithm>(MainAlgorithm value) {
        auto index = static_cast<size_t>(value);
        return (index < enum_strings::mainAlgorithmNames.size()) ?
               enum_strings::mainAlgorithmNames[index] : "UNKNOWN";
    }

    template<>
    inline const char* toString<SolutionMode>(SolutionMode value) {
        auto index = static_cast<size_t>(value);
        return (index < enum_strings::solutionModeNames.size()) ?
               enum_strings::solutionModeNames[index] : "UNKNOWN";
    }

    template<>
    inline const char* toString<WarmStart>(WarmStart value) {
        auto index = static_cast<size_t>(value);
        return (index < enum_strings::warmStartNames.size()) ?
               enum_strings::warmStartNames[index] : "UNKNOWN";
    }

    template<>
    inline const char* toString<InitialDual>(InitialDual value) {
        auto index = static_cast<size_t>(value);
        return (index < enum_strings::initialDualNames.size()) ?
               enum_strings::initialDualNames[index] : "UNKNOWN";
    }

    template<>
    inline const char* toString<DualMethod>(DualMethod value) {
        auto index = static_cast<size_t>(value);
        return (index < enum_strings::dualMethodNames.size()) ?
               enum_strings::dualMethodNames[index] : "UNKNOWN";
    }

    template<>
    inline const char* toString<NodeStatus>(NodeStatus value) {
        auto index = static_cast<size_t>(value);
        return (index < enum_strings::nodeStatusNames.size()) ?
               enum_strings::nodeStatusNames[index] : "UNKNOWN";
    }

    template<>
    inline const char* toString<SortVehicle>(SortVehicle value) {
        auto index = static_cast<size_t>(value);
        return (index < enum_strings::sortVehicleNames.size()) ?
               enum_strings::sortVehicleNames[index] : "UNKNOWN";
    }

    template<>
    inline const char* toString<LabelStatus>(LabelStatus value) {
        auto index = static_cast<size_t>(value);
        return (index < enum_strings::labelStatusNames.size()) ?
               enum_strings::labelStatusNames[index] : "UNKNOWN";
    }

    template<>
    inline const char* toString<SelectionMode>(SelectionMode value) {
        auto index = static_cast<size_t>(value);
        return (index < enum_strings::selectionModeNames.size()) ?
               enum_strings::selectionModeNames[index] : "UNKNOWN";
    }

    template<>
    inline const char* toString<SortPaths>(SortPaths value) {
        auto index = static_cast<size_t>(value);
        return (index < enum_strings::sortPathsNames.size()) ?
               enum_strings::sortPathsNames[index] : "UNKNOWN";
    }

    template<>
    inline const char* toString<SortColumns>(SortColumns value) {
        auto index = static_cast<size_t>(value);
        return (index < enum_strings::sortColumnsNames.size()) ?
               enum_strings::sortColumnsNames[index] : "UNKNOWN";
    }

    template<>
    inline const char* toString<VarSign>(VarSign value) {
        auto index = static_cast<size_t>(value);
        return (index < enum_strings::varSignNames.size()) ?
               enum_strings::varSignNames[index] : "UNKNOWN";
    }

    template<>
    inline const char* toString<SolutionStatus>(SolutionStatus value) {
        auto index = static_cast<size_t>(value);
        return (index < enum_strings::solutionStatusNames.size()) ?
               enum_strings::solutionStatusNames[index] : "UNKNOWN";
    }

    template<>
    inline const char* toString<RequestStatus>(RequestStatus value) {
        auto index = static_cast<size_t>(value);
        return (index < enum_strings::requestStatusNames.size()) ?
               enum_strings::requestStatusNames[index] : "UNKNOWN";
    }

    template<>
    inline const char* toString<ReturnType>(ReturnType value) {
        auto index = static_cast<size_t>(value);
        return (index < enum_strings::returnTypeNames.size()) ?
               enum_strings::returnTypeNames[index] : "UNKNOWN";
    }

    template<>
    inline const char* toString<NodeType>(NodeType value) {
        auto index = static_cast<size_t>(value);
        return (index < enum_strings::nodeTypeNames.size()) ?
               enum_strings::nodeTypeNames[index] : "UNKNOWN";
    }

    template<>
    inline const char* toString<ModelSOLVER>(ModelSOLVER value) {
        auto index = static_cast<size_t>(value);
        return (index < enum_strings::modelSolverNames.size()) ?
               enum_strings::modelSolverNames[index] : "UNKNOWN";
    }

    template<>
    inline const char* toString<Approach>(Approach value) {
        auto index = static_cast<size_t>(value);
        return (index < enum_strings::approachNames.size()) ?
               enum_strings::approachNames[index] : "UNKNOWN";
    }

    // Stream operators for convenient output
    /*template<typename EnumType>
    std::ostream& operator<<(std::ostream& os, EnumType value) {
        return os << toString(value);
    }*/
}


#endif //TYPES_H
