//
// Created by Elahe Amiri on 2022-10-29.
//

#include "data/Instance.h"
#include "utilities/MyTools.h"
#include "utilities/ReadWrite.h"
#include "solver/solver.h"


using namespace std::chrono;

int main(int argc, char** argv) {
    std::ios_base::sync_with_stdio(false);
    std::string dataDir = "datasets/";
    std::string vehicleFile = "";
    std::string vehicleFolder = "";
    int nbLocations = 818;
    myTools::Timer *simulationTime_ = new myTools::Timer(); simulationTime_->init();

    std::string instFolder;                     // folder of instances
    std::cout << "Number of arguments = " << argc << std::endl;
    std::string instanceName = "Test1";
    simulationTime_->start();

    InputPaths inputPaths(dataDir);
    ReadWrite::readDurations(inputPaths.input_durationMatrix_, durationMatrix_, nbLocations);

    // create output files for epoch results
    inputPaths.initializeInputs(instanceName);
    PInstance mainInst = std::make_shared<Instance>(instanceName, nbLocations);
    ReadWrite::readParameters_json(inputPaths.input_paramFile_, mainInst);
    ReadWrite::readTasks(inputPaths.input_TaskData_, mainInst);
    ReadWrite::readVehicles(inputPaths.input_vehicleFile_, mainInst);
    inputPaths.initializeOutputs();

    // create solver
    std::shared_ptr<solver> instanceSolver = std::make_shared<solver>(mainInst, inputPaths);
    instanceSolver->solveCG_Epoch(mainInst, mainInst, inputPaths);
    std::cout << "Objective: " << instanceSolver->masterModel_->objValue_ << std::endl;

    simulationTime_->stop();

    std::cout << "simulation time: " << simulationTime_->dSinceInit().count();

}
