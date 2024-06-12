//
// Created by Elahe Amiri on 2022-10-29.
//

#include "data/Instance.h"
#include "utilities/MyTools.h"
#include "solver/solver.h"

int main(int argc, char** argv) {
    std::string dataDir = "datasets/";
    InputPaths inputPaths(dataDir);

    // create output files for epoch results
    inputPaths.initializeInputs();

    // create solver
    std::shared_ptr<solver> instanceSolver = std::make_shared<solver>();
    instanceSolver->createInstanceFile(inputPaths.input_durationMatrix_, inputPaths.input_paramFile_,
                                   inputPaths.input_TaskData_, inputPaths.input_vehicleFile_);

    std::string results = instanceSolver->solveCG();
}
