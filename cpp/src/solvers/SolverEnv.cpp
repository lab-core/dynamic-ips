//
// SolverEnv.cpp
//
// Implementation of the process-wide solver-environment singletons. See
// SolverEnv.h for the rationale.
//

#include "SolverEnv.h"

namespace solverEnv {

#ifdef DARP_USE_GUROBI
GRBEnv& gurobi() {
    // Deferred-start env: configure parameters first, then call start() so the
    // license is acquired exactly once with the desired global settings.
    static GRBEnv env(true);
    static const bool initialized = []() {
        env.set(GRB_IntParam_UpdateMode,    1);
        env.set(GRB_IntParam_LogToConsole,  0);
        env.set(GRB_IntParam_OutputFlag,    0);
        env.start();
        return true;
    }();
    (void)initialized;
    return env;
}
#endif

#ifdef DARP_USE_CPLEX
IloEnv& cplex() {
    // Default-constructed IloEnv is active immediately. We deliberately do NOT
    // expose a way to end it: every IloModel / IloCplex / IloNumVar in the
    // program must be a child of this env, so calling .end() before process
    // exit would invalidate them all. Memory is released by the OS at exit.
    static IloEnv env;
    return env;
}
#endif

} // namespace solverEnv
