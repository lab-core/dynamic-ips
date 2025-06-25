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
    int nbVehicles = -1, nbRequests = -1, nbOnboards = -1, nbReceived = -1, nbLocations = -1;
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

    // add this only when I want to use less vehicles
//    pInstance->nbVehicles_ = 150;

    while (file.good()) {
//        readUntilChar(file, '\n', title);
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
                pInstance->instGraph_->sourceNodes_.back()->departTime_ = pInstance->simulationStartTime_;
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

    // add this only when I want to use less vehicles
//    pInstance->nbVehicles_ = 150;

    while (file.good()) {
//        readUntilChar(file, '\n', title);
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
                pInstance->instGraph_->sourceNodes_.back()->departTime_ = departTime;
                pInstance->instGraph_->sourceNodes_.back()->nodeStatus_ = DONE;
                pInstance->vehicles_.emplace_back(std::make_shared<Vehicle>(vehicleID, capacity, departTime,
                                                                            endTime, pInstance->instGraph_->sourceNodes_.back(),
                                                                            pInstance->instGraph_->sinkNodes_.back()));
/*                if (pInstance->parameters_->mainAlgorithm_ == RT_CG) {
                    pInstance->vehicles_.back()->dual_ = lDual;
                    pInstance->vehicles_.back()->InitialDual_ = lDual;
                }
                else {*/
                pInstance->vehicles_.back()->dual_ = iDual;
                pInstance->vehicles_.back()->InitialDual_ = iDual;
//                }
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

    while (file.good()) {
        //       readUntilChar(file, '\n', title);
        readUntilOneOfTwoChar(file, '\n', '\r', title);
        if (strEndWith(title, "REQUESTS_INFO")) {

            for (int r = 0; r < pInstance->nbOnboards_; ++r) {
                // attributes for reading the trip requests file
                int nbPassengers = -1, vehicleID = -1, pickZoneID = -1, dropZoneID = -1, position = -1;
                float pickUpID = -1, dropOffID = -1, earlyPick = -1, pickTime = -1, pickup_depart = -1;
 //               float deltaTime = -1;

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

                // the starting time of the instance is 16pm
                // deltaTime = static_cast<float>(nbPassengers * TimePerPassenger);
                pInstance->requests_.emplace_back(std::make_shared<Request>(pickUpID, dropOffID, earlyPick, earlyPick,
                                                                            nbPassengers, static_cast<float>(SERVICE_TIME),
                                                                            pickZoneID, dropZoneID));
                pInstance->requests_.back()->requestStatus_ = ON_BOARD;
                pInstance->requests_.back()->pickTime_ = pickTime;
                pInstance->requests_.back()->allocVehicleID_ = vehicleID;
                pInstance->requests_.back()->solVehicleID_ = vehicleID;

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
                pInstance->instGraph_->pickNodes_.back()->departTime_ = pickup_depart;
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
//        readUntilChar(file, '\n', title);
        readUntilOneOfTwoChar(file, '\n', '\r', title);
        if (strEndWith(title, "REQUESTS_INFO")) {
            for (int r = 0; r < nbRequest; ++r) {
                // attributes for the reading trip requests file
                int nbPassengers = -1;
                int pickUpID = -1, dropOffID = -1, pickZoneID = -1, dropZoneID = -1;
                float earlyPick = -1, requestTime;
 //               float deltaTime = -1;

                file >> nbPassengers;
                file >> pickUpID;
                file >> dropOffID;
                file >> earlyPick;
                file >> pickZoneID;
                file >> dropZoneID;

                // the starting time of the instance is 16pm
                // deltaTime = static_cast<float>(nbPassengers * TimePerPassenger);
                if (pInstance->parameters_->solutionMode_ == STATIC)
                    requestTime = 0;
                else
                    requestTime = earlyPick;
                pInstance->requests_.emplace_back(std::make_shared<Request>(pickUpID, dropOffID, requestTime,
                    earlyPick, nbPassengers, static_cast<float>(SERVICE_TIME), pickZoneID, dropZoneID));
                pInstance->nameToRequest_.insert(std::pair<std::string , PRequest>(pInstance->requests_.back()->name_, pInstance->requests_.back()));
                std::string pickID = myTools::createNodeID(pInstance->requests_.back()->getRequestId(), PICKUP);
                std::string dropID = myTools::createNodeID(pInstance->requests_.back()->getRequestId(), DROPOFF);
                PNode pickNode = std::make_shared<Node>(pickID, pInstance->requests_.back(), PICKUP);
                PNode dropNode = std::make_shared<Node>(dropID, pInstance->requests_.back(), DROPOFF);
                pickNode->zoneID_ = pickZoneID;
                dropNode->zoneID_ = dropZoneID;
                pInstance->instGraph_->addRequestToMainGraph(pickNode,dropNode);
                //        pInstance->instGraph_->addNewRequestToGraph(pInstance);

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
//    pInstance->instGraph_->addRequestsToGraph(pInstance);
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
 //               float deltaTime = -1;

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
                if (pickPosition != 0) {
                    routeNodes[vehicleID][pickPosition] = pInstance->instGraph_->pickNodes_.back();
                    routeNodes[vehicleID][dropPosition] = pInstance->instGraph_->dropNodes_.back();
                }
                pInstance->nbWaiting_++;

            }
        }
    }
//    pInstance->instGraph_->addRequestsToGraph(pInstance);
}


//************************************************************************
// Read the duration data file
//************************************************************************

void ReadWrite::readDurations(const std::string& strDurFile, vector2D<float> &durationMat, int nbLocations) {
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
    durationMat.resize(nbLocations);
    for (int i = 0; i < nbLocations; ++i)
        durationMat[i].resize(nbLocations);
    string title;

    while (file.good()) {
        //       readUntilChar(file, '\n', title);
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
void ReadWrite::readParameters(const std::string& strParamFile, PInstance &pInstance) {
// open the file
    std::fstream file;
    std::cout << "Reading << " << strParamFile << " >>" << std::endl;
    file.open(strParamFile, std::fstream::in);
    if (!file.is_open())
    {
        std::cout << "While trying to read the file " << strParamFile << std::endl;
        std::cout << "The input file was not opened properly!" << std::endl;

        throw myTools::myException("The input file was not opened properly!", __LINE__);
    }

    string title;

    float alphaParam = -1, betaParam = -1, deltaPram = -1, minImp = -1, committedTime = -1, epochLength = -1;
    float informTimeLimit = -1, pickupDeviationWindow = -1, timeWindows = -1, mipGap = -1, maxWait = -1;
    int penaltyL = -1, nbThreads = -1, bigM = -1, solveTimeLimit = -1, populateTimeLimit = -1;
    int strategy = -1, CP_IncDegree = -1, initialDual = -1, maxLabel = -1, WaitForReturn = -1, numVehicleSwitch = -1;
    bool isTruncated = false, pruneNodes = false, pruneArcs = false, constPortion = false;
    bool discardSuboptimalPath = false, isDominanceReleased = false, isPickDropPossible = false, useZoom = false;
    bool useMultiStage = false, vehiclePortion = false, usePick = false, greedyReOptimize = false;
    bool vehicleReturn = false, dynamicPricing = false, partialPricing = false, routeRecycle = false;
    int subAlgorithm = -1, mainAlgorithm = -1, initialStart = -1, MIP_maxIncDegree = -1;
    int solutionMode = -1, nbPick = -1, sortPath = -1, sortColumn = -1, nbColumns = -1, saveScratch = -1, numIter = -1;
    int returnType = -1;

    while (file.good()) {
        readUntilOneOfTwoChar(file, '=','\n', title);

        if (strEndWith(title, "alphaParam "))
            file >> alphaParam;

        else if (strEndWith(title, "betaParam "))
            file >> betaParam;

        else if (strEndWith(title, "deltaPram "))
            file >> deltaPram;

        else if (strEndWith(title, "epochLength "))
            file >> epochLength;

        else if (strEndWith(title, "penaltyL "))
            file >> penaltyL;

        else if (strEndWith(title, "committedTime "))
            file >> committedTime;

        else if (strEndWith(title, "nbThreads "))
            file >> nbThreads;

        else if (strEndWith(title, "InitialDual "))
            file >> initialDual;

        else if (strEndWith(title, "mainAlgorithm "))
            file >> mainAlgorithm;


        else if (strEndWith(title, "solutionMode "))
            file >> solutionMode;

        else if (strEndWith(title, "NumIter "))
            file >> numIter;

        else if (strEndWith(title, "GreedyReOptimize "))
            file >> greedyReOptimize;

        else if (strEndWith(title, "saveScratch "))
            file >> saveScratch;

        else if (strEndWith(title, "vehicleReturn "))
            file >> vehicleReturn;

        else if (strEndWith(title, "timeWindows "))
            file >> timeWindows;

        else if (strEndWith(title, "WaitForReturn "))
            file >> WaitForReturn;

        else if (strEndWith(title, "numVehicleSwitch "))
            file >> numVehicleSwitch;

        else if (strEndWith(title, "informTimeLimit "))
            file >> informTimeLimit;

        else if (strEndWith(title, "pickupDeviationWindow "))
            file >> pickupDeviationWindow;

        else if (strEndWith(title, "returnType "))
            file >> returnType;

        else if (strEndWith(title, "MaxWait "))
            file >> maxWait;

        else if (strEndWith(title, "warmStart "))
            file >> initialStart;

        else if (strEndWith(title, "MIP_maxIncDegree "))
            file >> MIP_maxIncDegree;

        else if (strEndWith(title, "CP_IncDegree "))
            file >> CP_IncDegree;

        else if (strEndWith(title, "useMultiStage "))
            file >> useMultiStage;

        else if (strEndWith(title, "minImp "))
            file >> minImp;

        else if (strEndWith(title, "useZoom "))
            file >> useZoom;

        else if (strEndWith(title, "NumColumn "))
            file >> nbColumns;

        else if (strEndWith(title, "isTruncated "))
            file >> isTruncated;

        else if (strEndWith(title, "MaxLabel "))
            file >> maxLabel;

        else if (strEndWith(title, "isDominanceReleased "))
            file >> isDominanceReleased;


        else if (strEndWith(title, "pruneNodes "))
            file >> pruneNodes;

        else if (strEndWith(title, "pruneArcs "))
            file >> pruneArcs;

        else if (strEndWith(title, "discardSuboptimalPath "))
            file >> discardSuboptimalPath;

        else if (strEndWith(title, "isDropPickPossible "))
            file >> isPickDropPossible;

        else if (strEndWith(title, "LabelingStrategy "))
            file >> strategy;

        else if (strEndWith(title, "subproblemAlgorithm "))
            file >> subAlgorithm;

        else if (strEndWith(title, "constPortion "))
            file >> constPortion;

        else if (strEndWith(title, "Vehicle_portion "))
            file >> vehiclePortion;

        else if (strEndWith(title, "Dynamic_Pricing "))
            file >> dynamicPricing;

        else if (strEndWith(title, "Partial_Pricing "))
            file >> partialPricing;

        else if (strEndWith(title, "Route_Recycle "))
            file >> routeRecycle;

        else if (strEndWith(title, "usePick "))
            file >> usePick;

        else if (strEndWith(title, "nbPick "))
            file >> nbPick;

        else if (strEndWith(title, "sortPath "))
            file >> sortPath;

        else if (strEndWith(title, "sortColumn "))
            file >> sortColumn;

        else if (strEndWith(title, "BigM "))
            file >> bigM;

        else if (strEndWith(title, "solveTimeLimit "))
            file >> solveTimeLimit;

        else if (strEndWith(title, "populateTimeLimit "))
            file >> populateTimeLimit;

        else if (strEndWith(title, "MIPGap "))
            file >> mipGap;
    }
    if (dynamicPricing && partialPricing){
        std::cout << "It is not possible to activate dynamic and partial pricing simultaneously!" << std::endl;
        throw myTools::myException("Parameters are not valid!!", __LINE__);
    }
    pInstance->parameters_ = std::make_shared<Parameters>(alphaParam, betaParam, deltaPram, epochLength,
                                                          penaltyL, committedTime, nbThreads,
                                                          static_cast<InitialDual>(initialDual),
                                                          static_cast<MainAlgorithm>(mainAlgorithm), numIter,
                                                          greedyReOptimize, vehicleReturn, timeWindows,
                                                          WaitForReturn, numVehicleSwitch,
                                                          static_cast<WarmStart>(initialStart),
                                                          MIP_maxIncDegree, CP_IncDegree, useMultiStage, minImp,
                                                          useZoom, nbColumns, isTruncated, maxLabel,
                                                          pruneNodes, pruneArcs, discardSuboptimalPath,
                                                          isDominanceReleased, isPickDropPossible,
                                                          static_cast<LabelingStrategy>(strategy),
                                                          static_cast<SubproblemAlgorithm>(subAlgorithm),
                                                          constPortion, vehiclePortion, dynamicPricing, partialPricing,
                                                          routeRecycle, usePick, nbPick,
                                                          static_cast<SortPaths>(sortPath),
                                                          static_cast<SortColumns>(sortColumn),
                                                          bigM, solveTimeLimit, populateTimeLimit,
                                                          static_cast<SolutionMode>(solutionMode), mipGap,
                                                          informTimeLimit, pickupDeviationWindow,
                                                          static_cast<ReturnType>(returnType), maxWait);
}

void ReadWrite::readParametersJsonFull(InputPaths &inputPaths, PInstance &pInstance) {
    // open the JSON file
    std::ifstream file(inputPaths.getInputParamFile());
    std::cout << "Reading << " << inputPaths.getInputParamFile() << " >>" << std::endl;

    if (!file.is_open()) {
        std::cout << "While trying to read the file " << inputPaths.getInputParamFile() << std::endl;
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


    // Model Parameters
    auto modelParams = j["modelParameters"];
    float alphaParam = modelParams.value("alphaParam", -1.0f);
    float betaParam = modelParams.value("betaParam", -1.0f);
    float deltaPram = modelParams.value("deltaPram", -1.0f);
    float epochLength = modelParams.value("epochLength", -1.0f);
    int penaltyL = modelParams.value("penaltyL", -1);
    float committedTime = modelParams.value("committedTime", -1.0f);
    int nbThreads = modelParams.value("nbThreads", -1);
    int mainAlgorithm = modelParams.value("mainAlgorithm", -1);
    int solutionMode = modelParams.value("solutionMode", -1);
    bool greedyReOptimize = modelParams.value("GreedyReOptimize", 0) != 0;
    bool vehicleReturn = modelParams.value("vehicleReturn", 0) != 0;
    float timeWindows = modelParams.value("timeWindows", -1.0f);
    int WaitForReturn = modelParams.value("WaitForReturn", -1);
    float maxWait = modelParams.value("MaxWait", -1.0f);
    int returnType = modelParams.value("returnType", -1);
    int numVehicleSwitch = modelParams.value("numVehicleSwitch", -1);
    float informTimeLimit = modelParams.value("informTimeLimit", -1.0f);
    float pickupDeviationWindow = modelParams.value("pickupDeviationWindow", -1.0f);

    // Parse MP_Parameters
    auto mpParams = j["MP_Parameters"];
    int initialDual = mpParams.value("InitialDual", -1);
    int initialStart = mpParams.value("warmStart", -1);
    int numIter = mpParams.value("NumIter", -1);
    int MIP_maxIncDegree = mpParams.value("MIP_maxIncDegree", -1);
    int CP_IncDegree = mpParams.value("CP_IncDegree", -1);
    bool useMultiStage = mpParams.value("useMultiStage", 0) != 0;
    float minImp = mpParams.value("minImp", -1.0f);
    bool useZoom = mpParams.value("useZoom", 0) != 0;
    int nbColumns = mpParams.value("NumColumn", -1);

    // Parse SP_Parameters
    auto spParams = j["SP_Parameters"];
    bool isTruncated = spParams.value("isTruncated", 0) != 0;
    int maxLabel = spParams.value("MaxLabel", -1);
    bool isDominanceReleased = spParams.value("isDominanceReleased", 0) != 0;
    bool pruneNodes = spParams.value("pruneNodes", 0) != 0;
    bool pruneArcs = spParams.value("pruneArcs", 0) != 0;
    bool discardSuboptimalPath = spParams.value("discardSuboptimalPath", 0) != 0;
    bool isPickDropPossible = spParams.value("isDropPickPossible", 0) != 0;
    int strategy = spParams.value("LabelingStrategy", -1);
    int subAlgorithm = spParams.value("subproblemAlgorithm", -1);
    bool constPortion = spParams.value("constPortion", 0) != 0;
    bool vehiclePortion = spParams.value("Vehicle_portion", 0) != 0;
    bool dynamicPricing = spParams.value("Dynamic_Pricing", 0) != 0;
    bool partialPricing = spParams.value("Partial_Pricing", 0) != 0;
    bool routeRecycle = spParams.value("Route_Recycle", 0) != 0;
    bool usePick = spParams.value("usePick", 0) != 0;
    int nbPick = spParams.value("nbPick", -1);
    int sortPath = spParams.value("sortPath", -1);
    int sortColumn = spParams.value("sortColumn", -1);

    // Parse cplexParameters
    auto cplexParams = j["cplexParameters"];
    int bigM = cplexParams.value("BigM", -1);
    int solveTimeLimit = cplexParams.value("solveTimeLimit", -1);
    int populateTimeLimit = cplexParams.value("populateTimeLimit", -1);
    float mipGap = cplexParams.value("MIPGap", -1.0f);

    // Validate parameters
    if (dynamicPricing && partialPricing) {
        std::cout << "It is not possible to activate dynamic and partial pricing simultaneously!" << std::endl;
        throw myTools::myException("Parameters are not valid!!", __LINE__);
    }

    // Create Parameters object
    pInstance->parameters_ = std::make_shared<Parameters>(
        alphaParam, betaParam, deltaPram, epochLength,
        penaltyL, committedTime, nbThreads,
        static_cast<InitialDual>(initialDual),
        static_cast<MainAlgorithm>(mainAlgorithm), numIter,
        greedyReOptimize, vehicleReturn, timeWindows,
        WaitForReturn, numVehicleSwitch,
        static_cast<WarmStart>(initialStart),
        MIP_maxIncDegree, CP_IncDegree, useMultiStage, minImp,
        useZoom, nbColumns, isTruncated, maxLabel,
        pruneNodes, pruneArcs, discardSuboptimalPath,
        isDominanceReleased, isPickDropPossible,
        static_cast<LabelingStrategy>(strategy),
        static_cast<SubproblemAlgorithm>(subAlgorithm),
        constPortion, vehiclePortion, dynamicPricing, partialPricing,
        routeRecycle, usePick, nbPick,
        static_cast<SortPaths>(sortPath),
        static_cast<SortColumns>(sortColumn),
        bigM, solveTimeLimit, populateTimeLimit,
        static_cast<SolutionMode>(solutionMode), mipGap,
        informTimeLimit, pickupDeviationWindow,
        static_cast<ReturnType>(returnType), maxWait
    );
}
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

    // ==================== READ DEFAULT PARAMETERS ====================
    auto defaultParams = j["defaultParameters"];

    // Default Parameters (stable parameters that rarely change)
    float alphaParam = defaultParams.value("alphaParam", -1.0f);
    float betaParam = defaultParams.value("betaParam", -1.0f);
    float deltaPram = defaultParams.value("deltaPram", -1.0f);
    float epochLength = defaultParams.value("epochLength", -1.0f);
    int penaltyL = defaultParams.value("penaltyL", -1);
    float committedTime = defaultParams.value("committedTime", -1.0f);
    int nbThreads = defaultParams.value("nbThreads", -1);
    int mainAlgorithm = defaultParams.value("mainAlgorithm", -1);
    int solutionMode = defaultParams.value("solutionMode", -1);
    bool greedyReOptimize = defaultParams.value("GreedyReOptimize", 0) != 0;
    float timeWindows = defaultParams.value("timeWindows", -1.0f);
    int MIP_maxIncDegree = defaultParams.value("MIP_maxIncDegree", -1);
    int CP_IncDegree = defaultParams.value("CP_IncDegree", -1);
    bool useMultiStage = defaultParams.value("useMultiStage", 0) != 0;
    float minImp = defaultParams.value("minImp", -1.0f);
    bool useZoom = defaultParams.value("useZoom", 0) != 0;
    bool isDominanceReleased = defaultParams.value("isDominanceReleased", 0) != 0;
    int strategy = defaultParams.value("LabelingStrategy", -1);
    int subAlgorithm = defaultParams.value("subproblemAlgorithm", -1);
    bool constPortion = defaultParams.value("constPortion", 0) != 0;
    bool vehiclePortion = defaultParams.value("Vehicle_portion", 0) != 0;
    bool routeRecycle = defaultParams.value("Route_Recycle", 0) != 0;
    bool usePick = defaultParams.value("usePick", 0) != 0;
    int bigM = defaultParams.value("BigM", -1);
    int solveTimeLimit = defaultParams.value("solveTimeLimit", -1);
    int populateTimeLimit = defaultParams.value("populateTimeLimit", -1);
    float mipGap = defaultParams.value("MIPGap", -1.0f);

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
    bool vehicleReturn = scenarioParams.value("vehicleReturn", 0) != 0;
    int WaitForReturn = scenarioParams.value("WaitForReturn", -1);
    float maxWait = scenarioParams.value("MaxWait", -1.0f);
    int returnType = scenarioParams.value("returnType", -1);
    int numVehicleSwitch = scenarioParams.value("numVehicleSwitch", -1);
    float informTimeLimit = scenarioParams.value("informTimeLimit", -1.0f);
    float pickupDeviationWindow = scenarioParams.value("pickupDeviationWindow", -1.0f);
    int initialDual = scenarioParams.value("InitialDual", -1);
    int initialStart = scenarioParams.value("warmStart", -1);
    int numIter = scenarioParams.value("NumIter", -1);
    int nbColumns = scenarioParams.value("NumColumn", -1);
    bool isTruncated = scenarioParams.value("isTruncated", 0) != 0;
    int maxLabel = scenarioParams.value("MaxLabel", -1);
    bool pruneNodes = scenarioParams.value("pruneNodes", 0) != 0;
    bool pruneArcs = scenarioParams.value("pruneArcs", 0) != 0;
    bool discardSuboptimalPath = scenarioParams.value("discardSuboptimalPath", 0) != 0;
    bool isPickDropPossible = scenarioParams.value("isDropPickPossible", 0) != 0;
    bool dynamicPricing = scenarioParams.value("Dynamic_Pricing", 0) != 0;
    bool partialPricing = scenarioParams.value("Partial_Pricing", 0) != 0;
    int nbPick = scenarioParams.value("nbPick", -1);
    int sortPath = scenarioParams.value("sortPath", -1);
    int sortColumn = scenarioParams.value("sortColumn", -1);

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
        static_cast<MainAlgorithm>(mainAlgorithm), numIter,
        greedyReOptimize, vehicleReturn, timeWindows,
        WaitForReturn, numVehicleSwitch,
        static_cast<WarmStart>(initialStart),
        MIP_maxIncDegree, CP_IncDegree, useMultiStage, minImp,
        useZoom, nbColumns, isTruncated, maxLabel,
        pruneNodes, pruneArcs, discardSuboptimalPath,
        isDominanceReleased, isPickDropPossible,
        static_cast<LabelingStrategy>(strategy),
        static_cast<SubproblemAlgorithm>(subAlgorithm),
        constPortion, vehiclePortion, dynamicPricing, partialPricing,
        routeRecycle, usePick, nbPick,
        static_cast<SortPaths>(sortPath),
        static_cast<SortColumns>(sortColumn),
        bigM, solveTimeLimit, populateTimeLimit,
        static_cast<SolutionMode>(solutionMode), mipGap,
        informTimeLimit, pickupDeviationWindow,
        static_cast<ReturnType>(returnType), maxWait
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
void ReadWrite::readDatafiles(InputPaths &inputPaths, PInstance &pInstance, int saveScratch, const std::string& paramFile) {
    readZones(inputPaths.getInputZones(), pInstance);
    vector2D<PNode> routeNodes;
    routeNodes.resize(pInstance->nbVehicles_);
    if (pInstance->nbOnboards_ > 0){
        readVehiclesDataF(inputPaths.getInputVehicleFile(), pInstance, routeNodes);
        readOnboardRequests(inputPaths.getInputOnboardsFile(), pInstance, routeNodes);
    }
    else
        readVehiclesData(inputPaths.getInputVehicleFileGeneral(), pInstance);
    if (pInstance->nbWaiting_ > 0) {
        readWaitRequests(inputPaths.getInputWaitRequests(), pInstance, pInstance->nbWaiting_, routeNodes);
 //       if (!solveEpoch) {
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
                    pInstance->vehicles_[v]->setCurrentRoute(newRoute);
                }
            }
//        }
    }

    if (!solveEpoch) {
        readTripRequests(inputPaths.getInputTripData(), pInstance, pInstance->nbRequests_);
        pInstance->nbRequests_ += (pInstance->nbOnboards_ + pInstance->nbWaiting_);
    }
    else{
        pInstance->nbRequests_ = (pInstance->nbOnboards_ + pInstance->nbWaiting_);
    }
    pInstance->nbNewRequests_ = pInstance->nbRequests_ - pInstance->nbOnboards_;

    inputPaths.initializeOutputs(eu::toString(pInstance->parameters_->mainAlgorithm_),
                                 eu::toString(pInstance->parameters_->solutionMode_),
                                 saveScratch, pInstance->nbVehicles_, paramFile);

    // write the parameters in file
    std::ofstream myFile;
    myFile.open (inputPaths.getOutputParamFile());
    myFile << pInstance->parameters_->toString();
    myFile.close();

    Tools::LogOutput parametersStream(inputPaths.getOutputParamCsv(), true);
    parametersStream << "Instance,alpha,beta,delta,epochLength,committedTime,informTimeLimit,pickupDeviationWindow,"
                        "maxWait,nbThreads,InitialDual,warmStart,mainAlgorithm,solutionMode,NumIter,"
                        "GreedyReOptimize,vehicleReturn,ReturnPolicy,MIP_maxIncDegree,"
                        "CP_IncDegree,useMultiStage,useZoom,nbColumns,isTruncated,MaxLabel,isDominanceReleased,"
                        "isDropPickPossible,pruneNodes,pruneArcs,discardSuboptimalPath,"
                        "LabelingStrategy,Vehicle_portion,Dynamic_Pricing,Partial_Pricing,Route_Recycle,nbPick,"
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




