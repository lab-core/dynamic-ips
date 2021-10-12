
#include "utilities/ReadWrite.h"
#include "data/Graph.h"



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

    float lat1 = 40.74027634;
    float long1 = -74.00778961;
    float lat2 = 40.7526741;
    float long2 = -73.96787262;
    std::cout << Tools::calcDistance(lat1, long1, lat2, long2)<< std::endl;
}
