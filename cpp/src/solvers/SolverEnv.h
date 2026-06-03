//
// SolverEnv.h
//


#ifndef DARP_SOLVERS_SOLVERENV_H
#define DARP_SOLVERS_SOLVERENV_H

#ifdef DARP_USE_GUROBI
#include "gurobi_c++.h"
#endif

#ifdef DARP_USE_CPLEX
#include <ilcplex/ilocplex.h>
#endif

namespace solverEnv {

#ifdef DARP_USE_GUROBI
// Returns the process-wide GRBEnv. Started lazily on first call; thereafter
// every caller gets the same env. Default parameters mirror what BaseSolver
// and the modelers used to set on their per-instance envs.
GRBEnv& gurobi();
#endif

#ifdef DARP_USE_CPLEX
// Returns the process-wide IloEnv. Default-constructed (and therefore active)
// on first call. NEVER call .end() on this env: it must outlive every
// IloModel / IloCplex object created against it. The OS reclaims the memory
// at process exit.
IloEnv& cplex();
#endif

} // namespace solverEnv

#endif // DARP_SOLVERS_SOLVERENV_H
