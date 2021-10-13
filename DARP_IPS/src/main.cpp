
#include "utilities/ReadWrite.h"
#include "data/Graph.h"
#include <ilcplex/ilocplex.h>
#include <iostream>
#include <sstream>



using std::string;


int main() {
    string dataDir = "datasets/";
    string instanceName = "20150713_16-01m";

    // build the path of input files
    InputPaths inputPaths(dataDir, instanceName);

    // Read data files and initialize instance
    PInstance pInst = ReadWrite::readInstance(inputPaths.getInstanceData());
    ReadWrite::readTripRequests(inputPaths.getTripData(), pInst);

    // Create the Graph based on requests
    PGraph mainGraph = std::make_shared<Graph>();
    mainGraph->addNewRequests(pInst->requests_);
    
}
