//
// Created by Elahe Amiri on 2022-10-29.
//

#include "data/Instance.h"
#include "utilities/Tools.h"
#include "utilities/MyTools.h"
#include "utilities/ReadWrite.h"
#include "solvers/solver.h"



using namespace std::chrono;
float saveTime = 7200;
bool middleSave = false;

int main(int argc, char** argv) {

    std::ios_base::sync_with_stdio(false);
    std::string dataDir = "datasets/";
    std::string instFolder = "test";
//    std::string instanceNamefile = "datasets/InstanceNames.txt";
//    std::vector<std::string> instNames;
//    ReadWrite::readInstNames(instanceNamefile, instNames , 24);

    std::cout << "Number of arguments = " << argc << std::endl;
//    for (auto & fileName : instNames){
  //  for (int i = 1; i < argc; ++i) {
//        std::string instanceName = fileName;
        std::string instanceName = argv[1];
        std::cout << "Instance : " << argv[1]  << std::endl;

        // build the path of input files
        // create output files for epoch results
        InputPaths inputPaths(dataDir, instFolder, instanceName);

        // Read data files and initialize instance and parameters in output path
        std::cout << "# INITIALIZE OF THE MAIN INSTANCE" << std::endl;
        PInstance mainInst = ReadWrite::readInstance(inputPaths.getInputInstanceData());
        ReadWrite::readParameters(inputPaths.getInputParamFile(), mainInst);
        ReadWrite::readDatafiles(inputPaths, mainInst);
        std::cout << mainInst->toString();
        ReadWrite::readDurations(inputPaths.getInputDurationData(), durationMatrix_, mainInst->nbLocations_);

        // create solver
        std::shared_ptr<solver> instanceSolver = std::make_shared<solver>(mainInst, inputPaths);


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

        // print final solution to txt file
        Tools::LogOutput finalStream(inputPaths.getOutputFinalLog());
        finalStream << instanceSolver->toString(mainInst);
        finalStream.close();
        /*std::ofstream myFile;
        myFile.open (inputPaths.getOutputFinalLog());
        myFile << instanceSolver->toString(mainInst);
        myFile.close();*/

        Tools::LogOutput solutionRoutesStream(inputPaths.getOutputFinalRoutes());
        solutionRoutesStream << mainInst->saveSolutionRoutes();
        solutionRoutesStream.close();

        Tools::LogOutput requestResultsStream(inputPaths.getOutputFinalRequests());
        requestResultsStream << mainInst->saveRequestsResults();
        requestResultsStream.close();

 //       mainInst->saveSolutionRoutes(inputPaths.getOutputFinalRoutes());
 //       mainInst->saveRequestsResults(inputPaths.getOutputFinalRequests());
        // save the final route solution
//    }
}