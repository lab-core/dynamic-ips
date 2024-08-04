//
// Created by Ella on 2021-10-18.
//

#ifndef _MIPSOLVER_H
#define _MIPSOLVER_H

#include "data/Instance.h"
#include "utilities/InputPaths.h"

//-----------------------------------------------------------------------------
//  Contains MIP formulation to solve with Cplex
//  This is just for testing the results and comparison
//-----------------------------------------------------------------------------

void MIPSolver(PInstance& PInst, InputPaths &filePaths);

#endif //_MIPSOLVER_H
