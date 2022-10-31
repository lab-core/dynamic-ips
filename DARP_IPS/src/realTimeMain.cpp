//
// Created by Elahe Amiri on 2022-10-29.
//

#include "data/Instance.h"
#include "utilities/Tools.h"
#include "utilities/MyTools.h"
#include "utilities/ReadWrite.h"
#include "solvers/solver.h"



using namespace std::chrono;
vector2D<float> durationMatrix_;
float saveTime = 3600;
bool middleSave = false;

int main(int argc, char** argv) {
    bool showLog = true;
    std::string dataDir = "datasets/";
    std::string instFolder = "capInstances";
    std::string instanceNamefile = "datasets/InstanceNames1.txt";
    std::vector<std::string> instNames;
    ReadWrite::readInstNames(instanceNamefile, instNames , 24);

    std::cout << "Number of arguments = " << argc << std::endl;

    for (int i = 0; i < 12; i++){
 //   for (int i = 1; i < argc; ++i) {
        std::string instanceName = instNames[i];
//        std::string instanceName = argv[i];
//        std::cout << "Instance : " << argv[i] << std::endl;

        // build the path of input files
        // create output files for epoch results
        InputPaths inputPaths(dataDir, instFolder, instanceName);

        // Read data files and initialize instance and parameters in output path
        std::cout << "# INITIALIZE OF THE MAIN INSTANCE" << std::endl;
    //    PInstance mainInst = ReadWrite::createMainInstance(inputPaths);
        PInstance mainInst = ReadWrite::readInstance(inputPaths.getInputInstanceData());
        mainInst->nbOnboards_ = 0;
        mainInst->nbVehicles_ = 70;
        ReadWrite::readParameters(inputPaths.getInputParamFile(), mainInst);
        ReadWrite::readDatafiles(inputPaths, mainInst);
        std::cout << mainInst->toString();
        ReadWrite::readDurations(inputPaths.getInputDurationData(), durationMatrix_, mainInst->nbLocations_);

        if (!showLog)
            freopen (inputPaths.getOutputSolutionLog().c_str(),"w",stdout);

        // create solver
        std::shared_ptr<solver> instanceSolver = std::make_shared<solver>(mainInst);

        if (mainInst->parameters_->solutionMode_ == DYNAMIC)
            instanceSolver->dynamicSolver(mainInst, inputPaths);
        else if (mainInst->parameters_->solutionMode_ == ANYTIME)
            instanceSolver->anyTimeSolver(mainInst, inputPaths);
        else
            instanceSolver->staticSolver(mainInst, inputPaths, "1", middleSave, saveTime);

        // testing the solution route
        for(auto  &vehicleObj : mainInst->vehicles_)
            vehicleObj->solutionRoute_->testRoute(vehicleObj, mainInst->parameters_->mainAlgorithm_ );

        std::cout << std::endl << std::endl;
        if (!showLog)
            fclose (stdout);

        // print final solution to txt file
        std::ofstream myFile;
        myFile.open (inputPaths.getOutputFinalLog());
        myFile << instanceSolver->toString(mainInst);
        myFile.close();

        mainInst->saveSolutionRoutes(inputPaths.getOutputFinalRoutes());
        mainInst->saveRequestsResults(inputPaths.getOutputFinalRequests());
        // save the final route solution
    }
}