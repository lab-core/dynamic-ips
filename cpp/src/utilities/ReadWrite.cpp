//
// Created by Ella on 2021-09-08.
//

#include "ReadWrite.h"
//extern PTravelTime travelMat;

//-----------------------------------------------------------------------------
//  ReadWrite class
//  functions to read input and write output
//-----------------------------------------------------------------------------

//************************************************************************
// Read the instance file and store content in an instance object
//************************************************************************

PInstance ReadWrite::readInstance(const std::string& strInstanceFile) {
    // open the file
    std::fstream file;
    std::cout << "Reading << " << strInstanceFile << " >>" << std::endl;
    file.open(strInstanceFile, std::fstream::in);
    if (!file.is_open())
    {
        std::cout << "While trying to read the file " << strInstanceFile << std::endl;
        std::cout << "The input file was not opened properly!" << std::endl;
        throw myTools::myException("The input file was not opened properly!", __LINE__);
    }

    // attributes to read the data file and initialize instance
    string title;
    std::string name;
    int nbVehicles = -1, nbRequests = -1, nbOnboards = 0, nbReceived = -1, nbLocations = -1;
    float simulationStart = -1;
    std::vector<PVehicle> vehicles;

    while (file.good()) {
        readUntilOneOfTwoChar(file, '=','\n', title);

        // read the name of instance
        if (strEndWith(title, "INSTANCE "))
            file >> name;

        // read simulation start time
        else if (strEndWith(title, "SIMULATION_START "))
            file >> simulationStart;

        // read number of vehicles
        else if (strEndWith(title, "NUM_VEHICLES "))
            file >> nbVehicles;

        // read the number of onboard requests
        else if (strEndWith(title, "NUM_ONBOARDS "))
            file >> nbOnboards;

        // read number of requests remained from previous epochs
        else if (strEndWith(title, "NUM_RECEIVED "))
            file >> nbReceived;

        // read the number of requests
        else if (strEndWith(title, "NUM_REQUESTS "))
            file >> nbRequests;

        // read number of stop locations
        else if (strEndWith(title, "NUM_LOCATIONS "))
            file >> nbLocations;

    }

    // main graph initialization with source and sink
    PGraph mainGraph = std::make_shared<Graph>();
    return std::make_shared<Instance>(name, simulationStart, nbVehicles, nbOnboards, nbReceived, vehicles, nbRequests,
                                      nbLocations, mainGraph);
}

//************************************************************************
// Read the vehicle file
//************************************************************************
void ReadWrite::readVehiclesData(const std::string& strTripsFile, const PInstance &pInstance) {
// open the file
    std::fstream file;
    std::cout << "Reading << " << strTripsFile << " >>" << std::endl;
    file.open(strTripsFile, std::fstream::in);
    if (!file.is_open())
    {
        std::cout << "While trying to read the file " << strTripsFile << std::endl;
        std::cout << "The input file was not opened properly!" << std::endl;

        throw myTools::myException("The input file was not opened properly!", __LINE__);
    }

    string title;
    std::vector<PVehicle> vehicles;
    int vehicleID = -1, capacity = -1, departID = -1, sinkID = -1, zoneID = -1;
    float departTime = -1, endTime = -1;

    while (file.good()) {
        readUntilOneOfTwoChar(file, '\n', '\r', title);
        if (strEndWith(title, "VEHICLES_INFO")) {
            for (int v = 0; v < pInstance->nbVehicles_; ++v) {
                file >> vehicleID;
                file >> capacity;
                file >> departTime;
                file >> endTime;
                file >> departID;
                file >> sinkID;
                file >> zoneID;
                if (departTime < pInstance->simulationStartTime_)
                    departTime = pInstance->simulationStartTime_;
                vehicleID = static_cast<int>(pInstance->vehicles_.size());
                pInstance->instGraph_->addNewNode(std::make_shared<Node>(departID, SOURCE, vehicleID, zoneID));
                pInstance->instGraph_->addNewNode(std::make_shared<Node>(sinkID, SINK, vehicleID, zoneID));
                pInstance->instGraph_->sourceNodes_.back()->reachTime_ = pInstance->simulationStartTime_;
                pInstance->instGraph_->sourceNodes_.back()->nodeDepartTime_ = pInstance->simulationStartTime_;
                pInstance->instGraph_->sourceNodes_.back()->nodeStatus_ = DONE;
                pInstance->vehicles_.emplace_back(std::make_shared<Vehicle>(vehicleID, capacity, departTime,
                                                                            endTime, pInstance->instGraph_->sourceNodes_.back(),
                                                                            pInstance->instGraph_->sinkNodes_.back()));
                pInstance->vehicles_.back()->startTime_ = pInstance->simulationStartTime_;
                pInstance->zones_[zoneID]->nbVehiclesRef_ ++;
            }
        }
    }
}

void ReadWrite::readVehiclesDataF(const std::string& strTripsFile, const PInstance &pInstance, vector2D<PNode> &routeNodes) {
// open the file
    std::fstream file;
    std::cout << "Reading << " << strTripsFile << " >>" << std::endl;
    file.open(strTripsFile, std::fstream::in);
    if (!file.is_open())
    {
        std::cout << "While trying to read the file " << strTripsFile << std::endl;
        std::cout << "The input file was not opened properly!" << std::endl;

        throw myTools::myException("The input file was not opened properly!", __LINE__);
    }

    string title;
    std::vector<PVehicle> vehicles;
    int vehicleID = -1, capacity = -1, departID = -1, sinkID = -1, departZoneID = -1, sinkZoneID = -1, routeSize = -1;
    float departTime = -1, endTime = -1, lDual = -1, iDual = -1;

    while (file.good()) {
        readUntilOneOfTwoChar(file, '\n', '\r', title);
        if (strEndWith(title, "VEHICLES_INFO")) {
            for (int v = 0; v < pInstance->nbVehicles_; ++v) {
                file >> vehicleID;
                file >> capacity;
                file >> departTime;
                file >> endTime;
                file >> departID;
                file >> sinkID;
                file >> departZoneID;
                file >> sinkZoneID;
                file >> routeSize;
                file >> lDual;
                file >> iDual;
                if (departTime < pInstance->simulationStartTime_)
                    departTime = pInstance->simulationStartTime_;
                pInstance->instGraph_->addNewNode(std::make_shared<Node>(departID, SOURCE, vehicleID, departZoneID));
                pInstance->instGraph_->addNewNode(std::make_shared<Node>(sinkID, SINK, vehicleID, sinkZoneID));
                pInstance->instGraph_->sourceNodes_.back()->reachTime_ = departTime;
                pInstance->instGraph_->sourceNodes_.back()->nodeDepartTime_ = departTime;
                pInstance->instGraph_->sourceNodes_.back()->nodeStatus_ = DONE;
                pInstance->vehicles_.emplace_back(std::make_shared<Vehicle>(vehicleID, capacity, departTime,
                                                                            endTime, pInstance->instGraph_->sourceNodes_.back(),
                                                                            pInstance->instGraph_->sinkNodes_.back()));
                pInstance->vehicles_.back()->dual_ = iDual;
                pInstance->vehicles_.back()->InitialDual_ = iDual;
                routeNodes[v].resize(routeSize);
            }
        }
    }
}

//************************************************************************
// Read the onboard file
//************************************************************************
void ReadWrite::readOnboardRequests(const std::string& strTripsFile, PInstance &pInstance, vector2D<PNode> &routeNodes) {
// open the file
    std::fstream file;
    std::cout << "Reading << " << strTripsFile << " >>" << std::endl;
    file.open(strTripsFile, std::fstream::in);
    if (!file.is_open())
    {
        std::cout << "While trying to read the file " << strTripsFile << std::endl;
        std::cout << "The input file was not opened properly!" << std::endl;

        throw myTools::myException("The input file was not opened properly!", __LINE__);
    }

    string title;
    // First pass: find "REQUESTS_INFO" and count the number of lines after it
    if (pInstance->nbOnboards_ == 0) {
        while (file.good()) {
            readUntilOneOfTwoChar(file, '\n', '\r', title);
            if (strEndWith(title, "REQUESTS_INFO")) {
                // Count the number of data lines after "REQUESTS_INFO"
                std::string line;
                while (std::getline(file, line)) {
                    // Skip empty lines
                    if (!line.empty() && line.find_first_not_of(" \t\r\n") != std::string::npos) {
                        pInstance->nbOnboards_++;
                    }
                }
                break;
            }
        }
    }
    pInstance->nbInitialOnboards_ = pInstance->nbOnboards_;

    // Close and reopen the file for the second pass
    file.close();
    file.open(strTripsFile, std::fstream::in);

    if (!file.is_open()) {
        std::cout << "While trying to reopen the file " << strTripsFile << std::endl;
        std::cout << "The input file was not opened properly!" << std::endl;
        throw myTools::myException("The input file was not opened properly!", __LINE__);
    }

    while (file.good()) {
        readUntilOneOfTwoChar(file, '\n', '\r', title);
        if (strEndWith(title, "REQUESTS_INFO")) {

            for (int r = 0; r < pInstance->nbOnboards_; ++r) {
                // attributes for reading the trip requests file
                int nbPassengers = -1, vehicleID = -1, pickZoneID = -1, dropZoneID = -1, position = -1;
                float pickUpID = -1, dropOffID = -1, earlyPick = -1, pickTime = -1, pickup_depart = -1;

                file >> nbPassengers;
                file >> pickUpID;
                file >> dropOffID;
                file >> earlyPick;
                file >> pickTime;
                file >> pickup_depart;
                file >> vehicleID;
                file >> pickZoneID;
                file >> dropZoneID;
                file >> position;

                pInstance->requests_.emplace_back(std::make_shared<Request>(pickUpID, dropOffID, earlyPick, earlyPick,
                                                                            nbPassengers, static_cast<float>(SERVICE_TIME),
                                                                            pickZoneID, dropZoneID));
                if (pInstance->parameters_->Req_W3_)
                    pInstance->requests_.back()->Req_W3_= static_cast<float>(pInstance->requests_.back()->nbPassengers_);
                else
                    pInstance->requests_.back()->Req_W3_ = 1.0;

                pInstance->requests_.back()->requestStatus_ = ON_BOARD;
                pInstance->requests_.back()->pickTime_ = pickTime;
                pInstance->requests_.back()->allocVehicleID_ = vehicleID;
                pInstance->requests_.back()->solVehicleID_ = vehicleID;

                pInstance->requests_.back()->setMinTravelTime(durationMatrix_[pickUpID][dropOffID]);
                pInstance->requests_.back()->setMaxTravelTime(pInstance->parameters_->alphaParam_, pInstance->parameters_->betaParam_);

                if (pInstance->parameters_->Relative_W5_ && pInstance->requests_.back()->minTravelTime_!= 0)
                    pInstance->requests_.back()->Relative_W5_= pInstance->requests_.back()->minTravelTime_;
                else
                    pInstance->requests_.back()->Relative_W5_ = 1.0;

                if (pInstance->parameters_->Ride_W4_)
                    pInstance->requests_.back()->Ride_W4_= pInstance->requests_.back()->minTravelTime_;
                else
                    pInstance->requests_.back()->Ride_W4_ = 0.0;

                pInstance->nameToRequest_[pInstance->requests_.back()->name_] = pInstance->requests_.back();
                std::string pickID = myTools::createNodeID(pInstance->requests_.back()->getRequestId(), PICKUP);
                std::string dropID = myTools::createNodeID(pInstance->requests_.back()->getRequestId(), DROPOFF);
                PNode pickNode = std::make_shared<Node>(pickID, pInstance->requests_.back(), PICKUP);
                PNode dropNode = std::make_shared<Node>(dropID, pInstance->requests_.back(), DROPOFF);
                pickNode->zoneID_ = pickZoneID;
                dropNode->zoneID_ = dropZoneID;
                pInstance->instGraph_->onboards_.push_back(dropNode);
                pInstance->instGraph_->addRequestToMainGraph(pickNode, dropNode);
                pInstance->vehicles_[vehicleID]->onboards_.push_back(dropID);
                pInstance->vehicles_[vehicleID]->numPassengers_+= pInstance->requests_.back()->nbPassengers_;
                pInstance->instGraph_->dropNodes_.back()->nodeStatus_ = PLANNED;
                pInstance->instGraph_->pickNodes_.back()->nodeStatus_ = DONE;
                pInstance->instGraph_->pickNodes_.back()->reachTime_ = pickTime;
                pInstance->instGraph_->pickNodes_.back()->nodeDepartTime_ = pickup_depart;
                routeNodes[vehicleID][position] = pInstance->instGraph_->dropNodes_.back();

            }
        }
    }
}

//************************************************************************
// Read the trip requests file
//************************************************************************

void ReadWrite::readTripRequests(const std::string& strTripsFile, PInstance &pInstance, int nbRequest) {
    // open the file
    std::fstream file;
    std::cout << "Reading << " << strTripsFile << " >>" << std::endl;
    file.open(strTripsFile, std::fstream::in);
    if (!file.is_open())
    {
        std::cout << "While trying to read the file " << strTripsFile << std::endl;
        std::cout << "The input file was not opened properly!" << std::endl;

        throw myTools::myException("The input file was not opened properly!", __LINE__);
    }

    string title;

    while (file.good()) {
        readUntilOneOfTwoChar(file, '\n', '\r', title);
        if (strEndWith(title, "REQUESTS_INFO")) {
            for (int r = 0; r < nbRequest; ++r) {
                // attributes for the reading trip requests file
                int nbPassengers = -1;
                int pickUpID = -1, dropOffID = -1, pickZoneID = -1, dropZoneID = -1;
                float earlyPick = -1, requestTime;

                file >> nbPassengers;
                file >> pickUpID;
                file >> dropOffID;
                file >> earlyPick;
                file >> pickZoneID;
                file >> dropZoneID;

                if (pInstance->parameters_->solutionMode_ == STATIC)
                    requestTime = 0;
                else
                    requestTime = earlyPick;
                pInstance->requests_.emplace_back(std::make_shared<Request>(pickUpID, dropOffID, requestTime,
                    earlyPick, nbPassengers, static_cast<float>(SERVICE_TIME), pickZoneID, dropZoneID));
                if (pInstance->parameters_->Req_W3_)
                    pInstance->requests_.back()->Req_W3_= static_cast<float>(pInstance->requests_.back()->nbPassengers_);
                else
                    pInstance->requests_.back()->Req_W3_ = 1.0;
                pInstance->requests_.back()->setMinTravelTime(durationMatrix_[pickUpID][dropOffID]);
                pInstance->requests_.back()->setMaxTravelTime(pInstance->parameters_->alphaParam_, pInstance->parameters_->betaParam_);

                if (pInstance->parameters_->Relative_W5_)
                    pInstance->requests_.back()->Relative_W5_= pInstance->requests_.back()->minTravelTime_;
                else
                    pInstance->requests_.back()->Relative_W5_ = 1.0;

                if (pInstance->parameters_->Ride_W4_)
                    pInstance->requests_.back()->Ride_W4_= pInstance->requests_.back()->minTravelTime_;
                else
                    pInstance->requests_.back()->Ride_W4_ = 0.0;
                pInstance->nameToRequest_.insert(std::pair<std::string , PRequest>(pInstance->requests_.back()->name_, pInstance->requests_.back()));
                std::string pickID = myTools::createNodeID(pInstance->requests_.back()->getRequestId(), PICKUP);
                std::string dropID = myTools::createNodeID(pInstance->requests_.back()->getRequestId(), DROPOFF);
                PNode pickNode = std::make_shared<Node>(pickID, pInstance->requests_.back(), PICKUP);
                PNode dropNode = std::make_shared<Node>(dropID, pInstance->requests_.back(), DROPOFF);
                pickNode->zoneID_ = pickZoneID;
                dropNode->zoneID_ = dropZoneID;
                pInstance->instGraph_->addRequestToMainGraph(pickNode,dropNode);

                if (pInstance->parameters_->timeWindow_ > 0)
                    pInstance->requests_.back()->penalty_ = pInstance->parameters_->timeWindow_;
                else
                    pInstance->requests_.back()->setPenalty(0, pInstance->parameters_,
                                                            pInstance->simulationStartTime_);
                pInstance->requests_.back()->latestPickup_ = pInstance->requests_.back()->earlyPick_ +
                                                             pInstance->requests_.back()->penalty_;
            }
        }
    }
}

void ReadWrite::readWaitRequests(const std::string& strTripsFile, PInstance &pInstance, int nbRequest, vector2D<PNode> &routeNodes) {
    // open the file
    std::fstream file;
    std::cout << "Reading << " << strTripsFile << " >>" << std::endl;
    file.open(strTripsFile, std::fstream::in);
    if (!file.is_open())
    {
        std::cout << "While trying to read the file " << strTripsFile << std::endl;
        std::cout << "The input file was not opened properly!" << std::endl;

        throw myTools::myException("The input file was not opened properly!", __LINE__);
    }

    string title;
    pInstance->nbWaiting_ = 0;

    while (file.good()) {
//        readUntilChar(file, '\n', title);
        readUntilOneOfTwoChar(file, '\n', '\r', title);
        if (strEndWith(title, "REQUESTS_INFO")) {
            for (int r = 0; r < nbRequest; ++r) {
                // attributes for reading the trip requests file
                int nbPassengers = -1;
                int pickUpID = -1, dropOffID = -1, pickZoneID = -1, vehicleID = -1, pickPosition = -1, dropPosition = -1;
                int dropZoneID = -1;
                float earlyPick = -1, lDual = -1, iDual = -1;

                file >> nbPassengers;
                file >> pickUpID;
                file >> dropOffID;
                file >> earlyPick;
                file >> pickZoneID;
                file >> dropZoneID;
                file >> lDual;
                file >> iDual;
                file >> vehicleID;
                file >> pickPosition;
                file >> dropPosition;

                pInstance->requests_.emplace_back(std::make_shared<Request>(pickUpID, dropOffID, earlyPick,
                    earlyPick, nbPassengers, static_cast<float>(SERVICE_TIME), pickZoneID, dropZoneID));
                if (pInstance->parameters_->Req_W3_)
                    pInstance->requests_.back()->Req_W3_= static_cast<float>(pInstance->requests_.back()->nbPassengers_);
                else
                    pInstance->requests_.back()->Req_W3_ = 1.0;
                pInstance->requests_.back()->setMinTravelTime(durationMatrix_[pickUpID][dropOffID]);
                pInstance->requests_.back()->setMaxTravelTime(pInstance->parameters_->alphaParam_, pInstance->parameters_->betaParam_);

                if (pInstance->parameters_->Relative_W5_)
                    pInstance->requests_.back()->Relative_W5_= pInstance->requests_.back()->minTravelTime_;
                else
                    pInstance->requests_.back()->Relative_W5_ = 1.0;

                if (pInstance->parameters_->Ride_W4_)
                    pInstance->requests_.back()->Ride_W4_= pInstance->requests_.back()->minTravelTime_;
                else
                    pInstance->requests_.back()->Ride_W4_ = 0.0;

                pInstance->nameToRequest_.insert(
                        std::pair<std::string, PRequest>(pInstance->requests_.back()->name_,
                                                         pInstance->requests_.back()));
                std::string pickID = myTools::createNodeID(pInstance->requests_.back()->getRequestId(), PICKUP);
                std::string dropID = myTools::createNodeID(pInstance->requests_.back()->getRequestId(), DROPOFF);
                PNode pickNode = std::make_shared<Node>(pickID, pInstance->requests_.back(), PICKUP);
                PNode dropNode = std::make_shared<Node>(dropID, pInstance->requests_.back(), DROPOFF);
                pickNode->zoneID_ = pickZoneID;
                dropNode->zoneID_ = dropZoneID;
                pInstance->instGraph_->addRequestToMainGraph(pickNode, dropNode);
                if (pInstance->parameters_->timeWindow_ > 0)
                    pInstance->requests_.back()->penalty_ = pInstance->parameters_->timeWindow_;
                else
                    pInstance->requests_.back()->setPenalty(0, pInstance->parameters_,
                                                            pInstance->simulationStartTime_);
                pInstance->requests_.back()->latestPickup_ = pInstance->requests_.back()->earlyPick_ +
                                                             pInstance->requests_.back()->penalty_;
                pInstance->requests_.back()->dual_ = iDual;
                pInstance->requests_.back()->InitialDual_ = iDual;
                pInstance->requests_.back()->lastDual_ = iDual;
                if (pickPosition != 0) {
                    routeNodes[vehicleID][pickPosition] = pInstance->instGraph_->pickNodes_.back();
                    routeNodes[vehicleID][dropPosition] = pInstance->instGraph_->dropNodes_.back();
                }
                pInstance->nbWaiting_++;

            }
        }
    }
}


//************************************************************************
// Read the duration data file
//************************************************************************

void ReadWrite::readDurations(const std::string& strDurFile, vector2D<float> &durationMat) {
// open the file
    std::fstream file;
    std::cout << "Reading << " << strDurFile << " >>" << std::endl;
    file.open(strDurFile, std::fstream::in);
    if (!file.is_open())
    {
        std::cout << "While trying to read the file " << strDurFile << std::endl;
        std::cout << "The input file was not opened properly!" << std::endl;

        throw myTools::myException("The input file was not opened properly!", __LINE__);
    }

    durationMat.clear();
    string title;
    int nbLocations = -1;

    readUntilOneOfTwoChar(file, '\n', '=', title);
    if (strEndWith(title, "nbLocations ")) {
        file >> nbLocations;
        durationMat.resize(nbLocations);
        for (int i = 0; i < nbLocations; ++i)
            durationMat[i].resize(nbLocations);
    }

    while (file.good()) {
        readUntilOneOfTwoChar(file, '\n', '\r', title);
        if (strEndWith(title, "DURATION_INFO")) {

            for (int l = 0; l < nbLocations*nbLocations; ++l) {
                // attributes for reading the trip requests file
                int startID = -1, endID = -1;
                float duration = -1;

                file >> startID;
                file >> endID;
                file >> duration;
                durationMat[startID][endID] = duration;
            }
        }
    }
}

//************************************************************************
// Read the parameters datafile
//************************************************************************

void ReadWrite::readParametersJson(const std::string& strParamFile, PInstance &pInstance, const std::string &scenarioName) {
    // open the JSON file
    std::ifstream file(strParamFile);
    std::cout << "Reading << " << strParamFile << " >>" << std::endl;

    if (!file.is_open()) {
        std::cout << "While trying to read the file " << strParamFile << std::endl;
        std::cout << "The input file was not opened properly!" << std::endl;
        throw myTools::myException("The input file was not opened properly!", __LINE__);
    }

    // Parse JSON
    json j;
    try {
        file >> j;
    } catch (const json::parse_error& e) {
        std::cout << "JSON parse error: " << e.what() << std::endl;
        throw myTools::myException("Failed to parse JSON file!", __LINE__);
    }

    // ==================== READ SCENARIO PARAMETERS ====================
    json scenarioParams;

    if (!scenarioName.empty()) {
        // Check if scenarios section exists
        if (j.find("scenarios") == j.end()) {
            std::cout << "Warning: No 'scenarios' section found in JSON file." << std::endl;
            throw myTools::myException("No scenarios section found!", __LINE__);
        }

        auto scenarios = j["scenarios"];
        if (scenarios.find(scenarioName) == scenarios.end()) {
            std::cout << "Error: Scenario '" << scenarioName << "' not found in scenarios section!" << std::endl;
            throw myTools::myException("Requested scenario not found!", __LINE__);
        }

        scenarioParams = scenarios[scenarioName];
        std::cout << "Loading scenario: " << scenarioName << std::endl;
    } else {
        std::cout << "No scenario specified, using default scenario parameters would be needed." << std::endl;
        throw myTools::myException("Scenario name is required!", __LINE__);
    }

    // Scenario Parameters (read from the selected scenario)
    float wait_W1 = scenarioParams.value("Wait_W1", 1.0f);
    float ride_W2 = scenarioParams.value("Ride_W2", 0.0f);
    bool req_W3 = scenarioParams.value("Req_W3", 0) != 0;
    bool ride_W4 = scenarioParams.value("Ride_W4", 0) != 0;
    bool relative_W5 = scenarioParams.value("Relative_W5", 0) != 0;
    bool normal_W6 = scenarioParams.value("Normal_W6", 0) != 0;
    bool vehicleReturn = scenarioParams.value("vehicleReturn", 0) != 0;
    int WaitForReturn = scenarioParams.value("WaitForReturn", LARGE_CONSTANT);
    float maxWait = scenarioParams.value("MaxWait", LARGE_CONSTANT);
    int returnType = scenarioParams.value("returnType", 1);
    float informTimeLimit = scenarioParams.value("informTimeLimit", LARGE_CONSTANT);
    float pickupDeviationWindow = scenarioParams.value("pickupDeviationWindow", LARGE_CONSTANT);
    int initialDual = scenarioParams.value("InitialDual", 1);
    int dualMethod = scenarioParams.value("DualMethod", 0);
    int initialStart = scenarioParams.value("warmStart", 1);
    int numIter = scenarioParams.value("NumIter", 1);
    int nbColumns = scenarioParams.value("NumColumn", 50);
    bool isTruncated = scenarioParams.value("isTruncated", 1) != 0;
    int maxLabel = scenarioParams.value("MaxLabel", 15);
    int maxCommittedLabel = scenarioParams.value("MaxCommittedLabel", 0);
    bool pruneNodes = scenarioParams.value("pruneNodes", 1) != 0;
    bool pruneArcs = scenarioParams.value("pruneArcs", 1) != 0;
    bool discardSuboptimalPath = scenarioParams.value("discardSuboptimalPath", 1) != 0;
    bool isPickDropPossible = scenarioParams.value("isDropPickPossible", 0) != 0;
    bool dynamicPricing = scenarioParams.value("Dynamic_Pricing", 0) != 0;
    bool partialPricing = scenarioParams.value("Partial_Pricing", 0) != 0;
    int nbPick = scenarioParams.value("nbPick", 4);
    int sortPath = scenarioParams.value("sortPath", 0);
    int sortColumn = scenarioParams.value("sortColumn", 1);
    bool routeRecycle = scenarioParams.value("Route_Recycle", 0) != 0;
    int newRequestLimit = scenarioParams.value("newRequestLimit", LARGE_CONSTANT);
    int strategy = scenarioParams.value("LabelingStrategy", 1);
    bool reoptimizeSP = scenarioParams.value("reoptimizeSP", 0) != 0;
    int reptimizeLabelstrategy = scenarioParams.value("LabelingReOptimizeStrategy", 2);
    bool smoothDual = scenarioParams.value("SmoothDual", 0) != 0;
    float alphaParam = scenarioParams.value("alphaParam", 1.5f);
    float betaParam = scenarioParams.value("betaParam", 240.0f);
    float deltaPram = scenarioParams.value("deltaPram", 420.0f);
    float epochLength = scenarioParams.value("epochLength", 30.0f);
    int penaltyL = scenarioParams.value("penaltyL", 30);
    float committedTime = scenarioParams.value("committedTime", 30.0f);
    int nbThreads = scenarioParams.value("nbThreads", 16);
    int mainAlgorithm = scenarioParams.value("mainAlgorithm", 0);
    int solutionMode = scenarioParams.value("solutionMode", 1);
    bool greedyReOptimize = scenarioParams.value("GreedyReOptimize", 0) != 0;
    float timeWindows = scenarioParams.value("timeWindows", 0.0f);
    int MIP_maxIncDegree = scenarioParams.value("MIP_maxIncDegree", 2);
    int CP_IncDegree = scenarioParams.value("CP_IncDegree", 10);
    bool reducedCP = scenarioParams.value("reducedCP", 0) != 0;
    float minImp = scenarioParams.value("minImp", 0.0025f);
    bool useZoom = scenarioParams.value("useZoom", 0) != 0;
    int isudVariant = scenarioParams.value("isudVariant", 1);
    bool isDominanceReleased = scenarioParams.value("isDominanceReleased", 0) != 0;
    int subAlgorithm = scenarioParams.value("subproblemAlgorithm", 1);
    bool constPortion = scenarioParams.value("constPortion", 0) != 0;
    bool vehiclePortion = scenarioParams.value("Vehicle_portion", 0) != 0;
    int bigM = scenarioParams.value("BigM", 27000);
    int solveTimeLimit = scenarioParams.value("solveTimeLimit", 300);
    int populateTimeLimit = scenarioParams.value("populateTimeLimit", 200);
    float mipGap = scenarioParams.value("MIPGap", 0.001f);
    double reducedCostThreshold = scenarioParams.value("reducedCostThreshold", 500.0);

    // ==================== VALIDATION ====================
    if (dynamicPricing && partialPricing) {
        std::cout << "It is not possible to activate dynamic and partial pricing simultaneously!" << std::endl;
        throw myTools::myException("Parameters are not valid!!", __LINE__);
    }

    // ==================== CREATE PARAMETERS OBJECT ====================
    pInstance->parameters_ = std::make_shared<Parameters>(
        alphaParam, betaParam, deltaPram, epochLength,
        penaltyL, committedTime, nbThreads,
        static_cast<InitialDual>(initialDual),
        static_cast<DualMethod>(dualMethod),
        static_cast<MainAlgorithm>(mainAlgorithm), numIter,
        greedyReOptimize, vehicleReturn, timeWindows,
        WaitForReturn,
        static_cast<WarmStart>(initialStart),
        MIP_maxIncDegree, CP_IncDegree, reducedCP, minImp,
        useZoom, static_cast<ISUDVariant>(isudVariant), nbColumns, isTruncated, maxLabel,maxCommittedLabel,
        pruneNodes, pruneArcs, discardSuboptimalPath,
        isDominanceReleased, isPickDropPossible,
        static_cast<LabelingStrategy>(strategy),
        static_cast<SubproblemAlgorithm>(subAlgorithm),
        constPortion, vehiclePortion, dynamicPricing, partialPricing,
        routeRecycle, reoptimizeSP, nbPick,
        static_cast<SortPaths>(sortPath),
        static_cast<SortColumns>(sortColumn),
        bigM, newRequestLimit, solveTimeLimit, populateTimeLimit,
        static_cast<SolutionMode>(solutionMode), mipGap,
        informTimeLimit, pickupDeviationWindow,
        static_cast<ReturnType>(returnType), maxWait,
        static_cast<LabelingReOptimizeStrategy>(reptimizeLabelstrategy),
        smoothDual, wait_W1, ride_W2, req_W3, ride_W4, relative_W5, normal_W6,
        reducedCostThreshold
    );

    std::cout << "Parameters loaded successfully with scenario: " << scenarioName << std::endl;
}
void ReadWrite::readZones(const string &strZoneFile, const PInstance &pInstance) {
// open the file
    std::fstream file;
    std::cout << "Reading << " << strZoneFile << " >>" << std::endl;
    file.open(strZoneFile, std::fstream::in);
    if (!file.is_open())
    {
        std::cout << "While trying to read the file " << strZoneFile << std::endl;
        std::cout << "The input file was not opened properly!" << std::endl;

        throw myTools::myException("The input file was not opened properly!", __LINE__);
    }

    string title;
    std::vector<int> excludeIDs = {192, 84, 129, 296, 50, 257, 142};

    while (file.good()) {
//        readUntilChar(file, '\n', title);
        readUntilOneOfTwoChar(file, '\n', '\r', title);
        if (strEndWith(title, "ZONE_INFO")) {
            pInstance->nbZones_ = 0;
            while (!file.eof()) {
                // attributes for reading the trip requests file
                int zoneID = -1, centerLocationID = -1;

                file >> zoneID;
                file >> centerLocationID;

                pInstance->nbZones_++;
                pInstance->zones_.insert(std::pair<int , PZone>(zoneID, std::make_shared<Zone>(zoneID, centerLocationID)));
                for (auto & item : excludeIDs) {
                    if (zoneID == item)
                        pInstance->zones_[zoneID]->highDemandZone_ = false;
                }
            }
        }
    }
    pInstance->sortZones();
}

// function that opens all input files and update main instance data
void ReadWrite::readDatafiles(InputPaths &inputPaths, PInstance &pInstance, const std::string& outputDir,
    const std::string& paramFile, int initialState) {
    readZones(inputPaths.getInputZones(), pInstance);
    vector2D<PNode> routeNodes;
    routeNodes.resize(pInstance->nbVehicles_);
    if (initialState == 2){
        readVehiclesDataF(inputPaths.getInputVehicleFile(), pInstance, routeNodes);
        if (pInstance->nbOnboards_ > 0)
            readOnboardRequests(inputPaths.getInputOnboardsFile(), pInstance, routeNodes);
        if (pInstance->nbWaiting_ > 0) {
            readWaitRequests(inputPaths.getInputWaitRequests(), pInstance, pInstance->nbWaiting_, routeNodes);
            //       if (!solveEpoch) {
            int num = 0;
            for (int v = 0; v < pInstance->nbVehicles_; ++v) {
                if (routeNodes[v].size() > 1) {
                    PRoute newRoute = std::make_shared<Route>(pInstance->vehicles_[v]->vehicleID_);
                    newRoute->addSource(pInstance->vehicles_[v]->departNode_, pInstance->vehicles_[v]->departTime_,
                                        pInstance->vehicles_[v]->numPassengers_);
                    for (int i = 1; i < routeNodes[v].size(); ++i) {
                        if (routeNodes[v][i] != nullptr) {
                            newRoute->addNode(routeNodes[v][i]);
                            if (routeNodes[v][i]->type_ == PICKUP) {
                                routeNodes[v][i]->related_Request_->solVehicleID_ = pInstance->vehicles_[v]->vehicleID_;
                            }
                        }
                        else
                            continue;
                        //                           newRoute->addSink(pInstance->vehicles_[v]->sinkNode_);
                    }
                    //                    newRoute->addSink(pInstance->vehicles_[v]->sinkNode_);
                    newRoute->calculateTripDelay(pInstance->parameters_->Wait_W1_, pInstance->parameters_->Ride_W2_);
                    pInstance->vehicles_[v]->setCurrentRoute(newRoute);
                }
            }
            //        }
        }
    }
    else if (initialState == 1) {
        readVehiclesDataF(inputPaths.getInputVehicleFileGeneral(), pInstance, routeNodes);
        readOnboardRequests(inputPaths.getInputOnboardsFileGeneral(), pInstance, routeNodes);
    }
    else
        readVehiclesData(inputPaths.getInputVehicleFileGeneral(), pInstance);

    if (initialState != 2) {
        readTripRequests(inputPaths.getInputTripData(), pInstance, pInstance->nbRequests_);
        pInstance->nbRequests_ += (pInstance->nbOnboards_ + pInstance->nbWaiting_);
    }
    else
    // if you want to solve just small tests disable reading Trips and use the following!
        pInstance->nbRequests_ = (pInstance->nbOnboards_ + pInstance->nbWaiting_);

    pInstance->nbNewRequests_ = pInstance->nbRequests_ - pInstance->nbOnboards_;

    inputPaths.initializeOutputs(eu::toString(pInstance->parameters_->mainAlgorithm_),
                                 eu::toString(pInstance->parameters_->solutionMode_),
                                 outputDir, pInstance->nbVehicles_, paramFile);

    // write the parameters in file
    std::ofstream myFile;
    myFile.open (inputPaths.getOutputParamFile());
    myFile << pInstance->parameters_->toString();
    myFile.close();

    Tools::LogOutput parametersStream(inputPaths.getOutputParamCsv(), true);
    parametersStream << "Instance,alpha,beta,delta,Wait_W1,Ride_W2,Req_W3,Ride_W4,Relative_W5,normal_W6,"
                        "epochLength,committedTime,informTimeLimit,pickupDeviationWindow,maxWait,nbThreads,"
                        "InitialDual,dualMethod,smoothDual,warmStart,mainAlgorithm,solutionMode,NumIter,"
                        "GreedyReOptimize,vehicleReturn,ReturnPolicy,MIP_maxIncDegree,CP_IncDegree,reducedCP,"
                        "useZoom,nbColumns,isTruncated,MaxLabel,MaxCommitLabel,isDominanceReleased,isDropPickPossible,"
                        "pruneNodes,pruneArcs,discardSuboptimalPath,LabelingStrategy,LabelingReOptimize,ReOptimizeStrategy,"
                        "Vehicle_portion,Dynamic_Pricing,Partial_Pricing,Route_Recycle,newRequestLimit,nbPick,"
                        "sortPath,sortColumn,MIPGap\n" << pInstance->name_ << ",";

    parametersStream << pInstance->parameters_->toStr();
    parametersStream.close();
    std::cout << pInstance->toString();
}

// Parsing functions
// Useful for reading a file stream until meeting the separating character
// (the characters that are read until the separating char are stored in ReadStr
bool ReadWrite::readUntilChar(std::fstream &pFile, char separator, std::string &pReadStr) {

    // clear any existing contents
    pReadStr.clear();
    // read up to the separator
    char c;
    while (pFile.get(c) && c != separator) {
        pReadStr.push_back(c);
        }

    // true if delimiter was found (stream still good), false on EOF/error
    return static_cast<bool>(pFile);
}

// Read a file stream until meeting one of the two separating characters
// (the characters that are read until the separating char are stored in ReadStr
bool ReadWrite::readUntilOneOfTwoChar(std::fstream &pFile, char separator1, char separator2, string &pReadStr) {
    // clear any previous contents
    pReadStr.clear();

    // read until one of the delimiters or EOF/error
    char c;
    while (pFile.get(c) && c != separator1 && c != separator2) {
        pReadStr.push_back(c);
    }

    // success if stream is still good (delimiter found, not EOF)
    return static_cast<bool>(pFile);
}
// check if a string (sentence) ends with a given substring (word)
bool ReadWrite::strEndWith(const std::string& sentence, const std::string& word) {
    int lenWord = static_cast<int>(word.length());
    int lenSentence = static_cast<int>(sentence.length());
    if (lenWord > lenSentence)
        return false;
    else {
        string endOfSentence = sentence.substr(lenSentence-lenWord, lenWord);
        return (!strcmp(word.c_str(), endOfSentence.c_str()));
    }
}

void ReadWrite::readInstNames(const string &strInstanceNameFile, vector<std::string> &fileNames, int nbInstances,
                              const std::string &index) {
// open the file
    std::fstream file;
    std::cout << "Reading << " << strInstanceNameFile << " >>" << std::endl;
    file.open(strInstanceNameFile, std::fstream::in);
    if (!file.is_open())
    {
        std::cout << "While trying to read the file " << strInstanceNameFile << std::endl;
        std::cout << "The input file was not opened properly!" << std::endl;

        throw myTools::myException("The input file was not opened properly!", __LINE__);
    }

    string title;
    std::string instName;

    // add this only when I want to use less vehicles
//    pInstance->nbVehicles_ = 45;
    while (file.good()) {
        for (int i = 0; i < nbInstances; ++i) {
            file >> instName;
            fileNames.push_back(instName + index);
        }
    }
}




