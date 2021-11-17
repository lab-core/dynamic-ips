
#include "utilities/ReadWrite.h"
#include <iostream>
#include "Eigen/Dense"
#include "solvers/ISUDAlgorithm.h"
#include "solvers/SubProblem.h"
#include "solvers/CPLEXModeler.h"
#include "solvers/ReducedProblem.h"




using std::string;
using namespace std::chrono;

int main() {
    string dataDir = "datasets/";
    string instanceName = "20150713_16-01m";

    // build the path of input files
    InputPaths inputPaths(dataDir, instanceName);

    // Read data files and initialize instance
    std::cout << "# INITIALIZE THE INSTANCE" << std::endl;
    PInstance pInst = ReadWrite::readInstance(inputPaths.getInstanceData());
    ReadWrite::readTripRequests(inputPaths.getTripData(), pInst);
    std::cout << std::endl;
    std::cout << pInst->toString();

    std::shared_ptr<ISUDAlgorithm> isudObj = std::make_shared<ISUDAlgorithm>(pInst);
    isudObj->initialization(pInst);
    isudObj->ReducedPro_->buildModel(pInst, isudObj->zSolution_, isudObj->routeSolution_);
    isudObj->ReducedPro_->solveModel(pInst, isudObj->zSolution_, isudObj->routeSolution_, isudObj->generatedRoutes_);

    std::cout << "Solution Result:" << std::endl;
    for (int r = 0; r < isudObj->routeSolution_.size(); ++r) {
        std::cout << isudObj->routeSolution_[r]->toString();
    }

    for (int v = 0; v < pInst->nbVehicles_; ++v) {
        PSubProblem subProblem = std::make_shared<SubProblem>(pInst->vehicles_[v]);
        subProblem->subGraph_ = pInst->mainGraph_;
        subProblem->subRequests_ = pInst->requests_;
        subProblem->BuildModelCPLEX(isudObj->ReducedPro_->requestDuals_, isudObj->ReducedPro_->vehicleDuals_[v],
                                    isudObj->ReducedPro_->requestToOrder_);
        subProblem->SolveCPLEX();
        std::cout << subProblem->toString() << std::endl;
        subProblem->SolutionToRoutes(isudObj->availableRoutes_[pInst->vehicles_[v]->vehicleID_], isudObj->generatedRoutes_);
    }
    isudObj->reduceProImprove(pInst);
    std::cout << "Solution Result after RP improve:" << std::endl;
    for (int r = 0; r < isudObj->routeSolution_.size(); ++r) {
        std::cout << isudObj->routeSolution_[r]->toString();
    }
}
