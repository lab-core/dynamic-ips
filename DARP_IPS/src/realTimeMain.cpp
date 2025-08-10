//
// Created by Elahe Amiri on 2022-10-29.
//

#include "data/Instance.h"
#include "utilities/Tools.h"
#include "utilities/MyTools.h"
#include "utilities/ReadWrite.h"
#include "solvers/Solver.h"
#include "utilities/ConfigParser.h"

using namespace std::chrono;

float saveTime = 3600;
bool middleSave = false;
int numEpochTests = 30;
bool solveEpoch = false;

int main(int argc, char** argv) {
    std::ios_base::sync_with_stdio(false);

    // Parse command line arguments
    PConfig config = std::make_unique<ProgramConfig>();
    if (!ConfigParser::parseArguments(argc, argv, config)) {
        return 1; // Exit with error code
    }

    // Print configuration for verification
    std::cout << "Starting program with the following configuration:\n";
    config->printConfig();
    std::cout << std::endl;

    // Set up paths and constants
    std::string dataDir = "datasets/";
    int numVehicles = config->numVehicles_;
    int nbLocations = 1718;

    // Prepare instance names
    std::vector<std::string> instNames;

    if (!config->instanceName_.empty()) {
        // Use specific instance
        instNames.push_back(config->instanceName_);
        std::cout << "Processing specific instance: " << config->instFolder_
                  << "/" << config->instanceName_ << std::endl;
    } else {
        // Read instances from file
        std::string instanceNames = "datasets/InstanceNames.txt";
        ReadWrite::readInstNames(instanceNames, instNames, 24, "_07-120m");
        std::cout << "24 instances read from file!" << std::endl;
    }

    // Build the path of input files
    InputPaths inputPaths(dataDir, config);
    ReadWrite::readDurations(inputPaths.getInputDurationData(), durationMatrix_, nbLocations);

    int max_i = 1, max_j = 1;

    if (config->scenario_ == "truncate") {
        max_i = 7;
        max_j = 3;
    }
    else if (config->scenario_ == "pruning")
        max_i = 3;
    else if (config->scenario_ == "dropPick")
        max_i = 2;
    else if (config->scenario_ == "initialDual")
        max_i = 3;

    for (auto & instanceName : instNames){
        for (int i = 0; i < max_i; ++i) {
            for (int j = 0; j < max_j; ++j) {
                std::this_thread::sleep_for(std::chrono::seconds(2));

                // Create output files for epoch results
                inputPaths.initializeInputs(config->instFolder_, instanceName);

                // Read data files and initialize instance and parameters
                std::cout << "# INITIALIZE OF THE MAIN INSTANCE" << std::endl;
                Request::requestCount_ = 0;
                PInstance mainInst = ReadWrite::readInstance(inputPaths.getInputInstanceData());
                if (mainInst->nbInitialOnboards_ == 0)
                    mainInst->nbVehicles_ = numVehicles;
                ReadWrite::readParametersJson(inputPaths.getInputParamFile(), mainInst, config->scenario_);
                mainInst->adjustParameters(config);

                // Configure parameters based on parameter file
                if (config->scenario_ == "truncate") {
                    mainInst->parameters_->MaxLabel_ = (i + 1) * 5;
                    mainInst->parameters_->sortPath_ = static_cast<SortPaths>(j);
                }
                else if (config->scenario_ == "pruning") {
                    mainInst->parameters_->pruneNodes_ = true;
                    if (i >= 1) mainInst->parameters_->pruneArcs_ = true;
                    if (i >= 2) mainInst->parameters_->discardSuboptimalPath_ = true;
                }
                else if (config->scenario_ == "dropPick") {
                    mainInst->parameters_->isDropPickPossible_ = (i == 1);
                }
                else if (config->scenario_ == "initialDual") {
                    if (i == 2)
                        mainInst->parameters_->initialDual_ =  static_cast<InitialDual>(7);
                    else
                        mainInst->parameters_->initialDual_ =  static_cast<InitialDual>(i);
                }

                ReadWrite::readDatafiles(inputPaths, mainInst, mainInst->parameters_->saveScratch_, config->scenario_);

                // Create solver and run appropriate algorithm
                std::unique_ptr<Solver> instanceSolver = std::make_unique<Solver>(mainInst, inputPaths);
                instanceSolver->solve(mainInst, inputPaths, middleSave, saveTime);

                // Test the solution route
                for (auto& vehicleObj : mainInst->vehicles_)
                    vehicleObj->solutionRoute_->testRoute(vehicleObj);

                if (!middleSave) {
                    if (mainInst->parameters_->timeWindow_ > 0) {
                        for (auto& requestObj : mainInst->requests_) {
                            if (requestObj->requestStatus_ != COMPLETED) {
                                requestObj->requestStatus_ = REJECTED;
                            }
                        }
                    }
                }
                // print final outputs
                Tools::LogOutput finalStream(inputPaths.getOutputFinalLog());
                finalStream << instanceSolver->toString(mainInst);
                finalStream.close();
                mainInst->writeFinalOutputs(inputPaths, config);
            }
        }
    }
    std::cout << "Program completed successfully." << std::endl;
    return 0;
}