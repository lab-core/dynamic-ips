//
// Created by Elahe Amiri on 2022-10-29.
//

#include "data/Instance.h"
#include "utilities/Tools.h"
#include "utilities/MyTools.h"
#include "utilities/ReadWrite.h"
#include "solvers/solver.h"



using namespace std::chrono;
float saveTime = 1800;
bool middleSave = false;
std::string instNum = "1";
int numVehicles = 1600;

int main(int argc, char** argv) {
    std::ios_base::sync_with_stdio(false);
    std::string dataDir = "datasets/";
    std::string vehicleFile = "vehicles_1600_4";
    std::string vehicleFolder = "sufficient_manhattan-vehicles";
    int nbLocations = 1718;

    std::vector<std::string> instNames;         // vector of instance file names
    std::string instFolder;                     // folder of instances
    std::cout << "Number of arguments = " << argc << std::endl;
    if (argc == 3){
        std::string instanceNames = "datasets/InstanceNames-120.txt";
        ReadWrite::readInstNames(instanceNames, instNames , 24);
        std::cout << "24 Instance read!! " << std::endl;
        instFolder = argv[1];
        std::string word = argv[2];
        vehicleFile = "vehicles_" + word + "_4";
        numVehicles = std::stoi(argv[2]);
    }
    else if (argc == 4){
        instFolder = argv[1];
        instNames.push_back(argv[2]);
        std::cout << "Instance : " << argv[1] << "/" << argv[2]  << std::endl;
        std::string word = argv[3];
        vehicleFile = "vehicles_" + word + "_4";
        numVehicles = std::stoi(argv[3]);
    }
    else
        myTools::throwError("There should be at least 2 arguments!");

    // build the path of input files
    // create output files for epoch results
    InputPaths inputPaths(dataDir, vehicleFile, vehicleFolder);
    ReadWrite::readDurations(inputPaths.getInputDurationData(), durationMatrix_, nbLocations);

    Tools::LogOutput finalInstanceStream("datasets/results.csv", true);
    for (auto & instanceName : instNames){
        // create output files for epoch results
        inputPaths.initializeInputs(instFolder, instanceName);

        // Read data files and initialize instance and parameters in output path
        std::cout << "# INITIALIZE OF THE MAIN INSTANCE" << std::endl;
        Request::requestCount_ = 0;
        PInstance mainInst = ReadWrite::readInstance(inputPaths.getInputInstanceData());
        mainInst->nbVehicles_ = numVehicles;
        ReadWrite::readParameters(inputPaths.getInputParamFile(), mainInst);
        ReadWrite::readDatafiles(inputPaths, mainInst);
        std::cout << mainInst->toString();

        // create solver
        std::shared_ptr<solver> instanceSolver = std::make_shared<solver>(mainInst, inputPaths);
        if (mainInst->parameters_->solutionMode_ == DYNAMIC){
            try {
                instanceSolver->dynamicSolver(mainInst, inputPaths, instNum, middleSave, saveTime);
            } catch (const std::exception &e) {
                std::cout << "DYNAMIC solving caught an exception=: "
                          << e.what() << std::endl;
            }
        }
        else if (mainInst->parameters_->solutionMode_ == ANYTIME){
            try {
                /*if (mainInst->parameters_->mainAlgorithm_ == GREEDY)
                    instanceSolver->anyTimeSolverEvent(mainInst, inputPaths);
                else*/
                    instanceSolver->anyTimeSolver(mainInst, inputPaths);
            } catch (const std::exception &e) {
                std::cout << "ANY_TIME solving caught an exception=: "
                          << e.what() << std::endl;
            }
        }

        else {
            try {
                instanceSolver->staticSolver(mainInst, inputPaths, "1", middleSave, saveTime);
            } catch (const std::exception &e) {
                std::cout << "STATIC solving caught an exception=: "
                          << e.what() << std::endl;
            }
        }
        // testing the solution route
        for(auto  &vehicleObj : mainInst->vehicles_)
            vehicleObj->solutionRoute_->testRoute(vehicleObj, mainInst->parameters_ );

        std::cout << std::endl << std::endl;

        // print final solution to txt file
        Tools::LogOutput finalStream(inputPaths.getOutputFinalLog());
        finalStream << instanceSolver->toString(mainInst);
        finalStream.close();

        // print final routes to csv
        Tools::LogOutput solutionRoutesStream(inputPaths.getOutputFinalRoutes());
        solutionRoutesStream << mainInst->saveSolutionRoutes();
        solutionRoutesStream.close();

        // print requests results to csv
        Tools::LogOutput requestResultsStream(inputPaths.getOutputFinalRequests());
        requestResultsStream << mainInst->saveRequestsResults();
        requestResultsStream.close();
        finalInstanceStream << mainInst->instRepStr_.str();
    }
    finalInstanceStream.close();
}