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

    // attributes to read data file and initialize instance
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

        // read number of onboards
        else if (strEndWith(title, "NUM_ONBOARDS "))
            file >> nbOnboards;

        // read number of requests remained from previous epochs
        else if (strEndWith(title, "NUM_RECEIVED "))
            file >> nbReceived;

        // read number of requests
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
void ReadWrite::readVehiclesData(const std::string& strTripsFile, PInstance &pInstance) {
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
                pInstance->instGraph_->addNewNode(std::make_shared<Node>(departID, SOURCE, vehicleID, zoneID));
                pInstance->instGraph_->addNewNode(std::make_shared<Node>(sinkID, SINK, vehicleID, zoneID));
                pInstance->instGraph_->sourceNodes_.back()->reachTime_ = pInstance->simulationStartTime_;
                pInstance->instGraph_->sourceNodes_.back()->departTime_ = pInstance->simulationStartTime_;
                pInstance->instGraph_->sourceNodes_.back()->nodeStatus_ = DONE;
                pInstance->vehicles_.emplace_back(std::make_shared<Vehicle>(vehicleID, capacity, departTime,
                                                                            endTime, pInstance->instGraph_->sourceNodes_.back(),
                                                                            pInstance->instGraph_->sinkNodes_.back()));
                pInstance->vehicles_.back()->startTime_ = pInstance->simulationStartTime_;
            }
        }
    }
}

void ReadWrite::readVehiclesDataF(const std::string& strTripsFile, PInstance &pInstance, vector2D<PNode> &routeNodes) {
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
/*                if (pInstance->parameters_->mainAlgorithm_ == MP_CG) {
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
                // attributes for reading trip requests file
                int nbPassengers = -1, vehicleID = -1, pickZoneID = -1, dropZoneID = -1, position = -1;
 //               double pickUpLatitude = -1, pickUpLongitude = -1, dropOffLatitude = -1, dropOffLongitude = -1;
                float pickUpID = -1, dropOffID = -1, earlyPick = -1, pickTime = -1, pickup_depart = -1, deltaTime = -1;

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
 //               deltaTime = static_cast<float>(nbPassengers * TimePerPassenger);
                deltaTime = static_cast<float>(ServiceTime);
                pInstance->requests_.emplace_back(std::make_shared<Request>(pickUpID, dropOffID, earlyPick,
                                                                            nbPassengers, deltaTime, pickZoneID, dropZoneID));
                pInstance->requests_.back()->requestStatus_ = ON_BOARD;
                pInstance->requests_.back()->pickTime_ = pickTime;
                pInstance->requests_.back()->allocVehicleID_ = vehicleID;
                pInstance->requests_.back()->initialVehicleID_ = vehicleID;
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
                // attributes for reading trip requests file
                int nbPassengers = -1;
//                double pickUpLatitude = -1, pickUpLongitude = -1, dropOffLatitude = -1, dropOffLongitude = -1,
                int pickUpID = -1, dropOffID = -1, pickZoneID = -1, dropZoneID = -1;
                float earlyPick = -1, deltaTime = -1;

                file >> nbPassengers;
                file >> pickUpID;
                file >> dropOffID;
                file >> earlyPick;
                file >> pickZoneID;
                file >> dropZoneID;

                // the starting time of the instance is 16pm
        //        deltaTime = static_cast<float>(nbPassengers * TimePerPassenger);
                deltaTime = static_cast<float>(ServiceTime);
                pInstance->requests_.emplace_back(std::make_shared<Request>(pickUpID, dropOffID, earlyPick,
                                                                            nbPassengers, deltaTime, pickZoneID, dropZoneID));
                pInstance->nameToRequest_.insert(std::pair<std::string , PRequest>(pInstance->requests_.back()->name_, pInstance->requests_.back()));
                std::string pickID = myTools::createNodeID(pInstance->requests_.back()->getRequestId(), PICKUP);
                std::string dropID = myTools::createNodeID(pInstance->requests_.back()->getRequestId(), DROPOFF);
                PNode pickNode = std::make_shared<Node>(pickID, pInstance->requests_.back(), PICKUP);
                PNode dropNode = std::make_shared<Node>(dropID, pInstance->requests_.back(), DROPOFF);
                pickNode->zoneID_ = pickZoneID;
                dropNode->zoneID_ = dropZoneID;
                pInstance->instGraph_->addRequestToMainGraph(pickNode,dropNode);
        //        pInstance->instGraph_->addNewRequestToGraph(pInstance);
                pInstance->requests_.back()->setPenalty(0, pInstance->parameters_, pInstance->simulationStartTime_);
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

    while (file.good()) {
//        readUntilChar(file, '\n', title);
        readUntilOneOfTwoChar(file, '\n', '\r', title);
        if (strEndWith(title, "REQUESTS_INFO")) {
            for (int r = 0; r < nbRequest; ++r) {
                // attributes for reading trip requests file
                int nbPassengers = -1;
//                double pickUpLatitude = -1, pickUpLongitude = -1, dropOffLatitude = -1, dropOffLongitude = -1,
                int pickUpID = -1, dropOffID = -1, pickZoneID = -1, vehicleID = -1, pickPosition = -1, dropPosition = -1;
                int dropZoneID = -1;
                float earlyPick = -1, deltaTime = -1, lDual = -1, iDual = -1;

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

                // the starting time of the instance is 16pm
                //        deltaTime = static_cast<float>(nbPassengers * TimePerPassenger);
                deltaTime = static_cast<float>(ServiceTime);
                pInstance->requests_.emplace_back(std::make_shared<Request>(pickUpID, dropOffID, earlyPick,
                                                                            nbPassengers, deltaTime, pickZoneID, dropZoneID));
                pInstance->nameToRequest_.insert(std::pair<std::string , PRequest>(pInstance->requests_.back()->name_, pInstance->requests_.back()));
                std::string pickID = myTools::createNodeID(pInstance->requests_.back()->getRequestId(), PICKUP);
                std::string dropID = myTools::createNodeID(pInstance->requests_.back()->getRequestId(), DROPOFF);
                PNode pickNode = std::make_shared<Node>(pickID, pInstance->requests_.back(), PICKUP);
                PNode dropNode = std::make_shared<Node>(dropID, pInstance->requests_.back(), DROPOFF);
                pickNode->zoneID_ = pickZoneID;
                dropNode->zoneID_ = dropZoneID;
                pInstance->instGraph_->addRequestToMainGraph(pickNode,dropNode);
                //        pInstance->instGraph_->addNewRequestToGraph(pInstance);
                pInstance->requests_.back()->setPenalty(0, pInstance->parameters_, pInstance->simulationStartTime_);
                /*if (pInstance->parameters_->mainAlgorithm_ == MP_CG) {
                    pInstance->requests_.back()->dual_ = lDual;
                    pInstance->requests_.back()->InitialDual_ = lDual;
                }
                else{*/
                    pInstance->requests_.back()->dual_ = iDual;
                    pInstance->requests_.back()->InitialDual_ = iDual;
 //               }
                routeNodes[vehicleID][pickPosition] = pInstance->instGraph_->pickNodes_.back();
                routeNodes[vehicleID][dropPosition] = pInstance->instGraph_->dropNodes_.back();
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
                // attributes for reading trip requests file
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

    float alphaParam = -1, betaParam = -1, deltaPram = -1, minImp = -1, committedTime = -1, vehicle_portion = -1;
    int epochLength = -1, penaltyL = -1, nbThreads = -1, bigM = -1, solveTimeLimit = -1, populateTimeLimit = -1;
    int strategy = -1, CP_IncDegree = -1, initialDual = -1, maxLabel = -1;
    bool isTruncated = false, isSuccessorsLimited = false, isDominanceReleased = false, oneIter = false;
    bool isPickDropPossible = false, useZoom = false, useMultiStage = false, greedyPortion = false, usePick = false;
    bool greedyReOptimize = false, saveScratch = false;
    int subAlgorithm = -1, subproSolveStartState = -1 , mainAlgorithm = -1, initialStart = -1, MIP_maxIncDegree = -1;
    int solutionMode = -1, nbPick = -1, sortPath = -1;

    bool addOneRequestColumn = false;
    float mipGap = -1;

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

        else if (strEndWith(title, "addOneRequestColumn "))
            file >> addOneRequestColumn;

        else if (strEndWith(title, "solutionMode "))
            file >> solutionMode;

        else if (strEndWith(title, "OneIter "))
            file >> oneIter;

        else if (strEndWith(title, "GreedyReOptimize "))
            file >> greedyReOptimize;

        else if (strEndWith(title, "saveScratch "))
            file >> saveScratch;

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

        else if (strEndWith(title, "isTruncated "))
            file >> isTruncated;

        else if (strEndWith(title, "MaxLabel "))
            file >> maxLabel;

        else if (strEndWith(title, "isDominanceReleased "))
            file >> isDominanceReleased;

        else if (strEndWith(title, "isSuccessorsLimited "))
            file >> isSuccessorsLimited;

        else if (strEndWith(title, "isDropPickPossible "))
            file >> isPickDropPossible;

        else if (strEndWith(title, "SubproSolveStartState "))
            file >> subproSolveStartState;

        else if (strEndWith(title, "LabelingStrategy "))
            file >> strategy;

        else if (strEndWith(title, "subproblemAlgorithm "))
            file >> subAlgorithm;

        else if (strEndWith(title, "Vehicle_portion "))
            file >> vehicle_portion;

        else if (strEndWith(title, "Greedy_portion "))
            file >> greedyPortion;

        else if (strEndWith(title, "usePick "))
            file >> usePick;

        else if (strEndWith(title, "nbPick "))
            file >> nbPick;

        else if (strEndWith(title, "sortPath "))
            file >> sortPath;

        else if (strEndWith(title, "BigM "))
            file >> bigM;

        else if (strEndWith(title, "solveTimeLimit "))
            file >> solveTimeLimit;

        else if (strEndWith(title, "populateTimeLimit "))
            file >> populateTimeLimit;

        else if (strEndWith(title, "MIPGap "))
            file >> mipGap;
    }
    pInstance->parameters_ = std::make_shared<Parameters>(alphaParam, betaParam, deltaPram, epochLength,
                                                          penaltyL, committedTime, nbThreads,
                                                          static_cast<InitialDual>(initialDual),
                                                          static_cast<MainAlgorithm>(mainAlgorithm), oneIter,
                                                          greedyReOptimize, saveScratch,
                                                          static_cast<warmStart>(initialStart),
                                                          MIP_maxIncDegree, CP_IncDegree, useMultiStage, minImp,
                                                          useZoom, isTruncated, maxLabel, isSuccessorsLimited,
                                                          isDominanceReleased, isPickDropPossible,
                                                          static_cast<SubProSolveMode>(subproSolveStartState),
                                                          static_cast<LabelingStrategy>(strategy),
                                                          static_cast<subproblemAlgorithm>(subAlgorithm),
                                                          vehicle_portion, greedyPortion, usePick, nbPick,
                                                          static_cast<SortPaths>(sortPath),bigM,
                                                          solveTimeLimit, populateTimeLimit, addOneRequestColumn,
                                                          static_cast<SolutionMode>(solutionMode), mipGap);
}


// function that open all input files and update main instance data
void ReadWrite::readDatafiles(InputPaths &inputPaths, PInstance &pInstance, bool saveScratch) {
    vector2D<PNode> routeNodes;
    routeNodes.resize(pInstance->nbVehicles_);
    if (pInstance->nbOnboards_ > 0){
        ReadWrite::readVehiclesDataF(inputPaths.getInputVehicleFile(), pInstance, routeNodes);
        ReadWrite::readOnboardRequests(inputPaths.getInputOnboardsFile(), pInstance, routeNodes);
    }
    else
        ReadWrite::readVehiclesData(inputPaths.getInputVehicleFileGeneral(), pInstance);
    if (pInstance->nbWaiting_ > 0) {
        ReadWrite::readWaitRequests(inputPaths.getInputWaitRequests(), pInstance, pInstance->nbWaiting_, routeNodes);
        for (int v = 0; v < pInstance->nbVehicles_; ++v) {
            if (routeNodes[v].size()>1){
                PRoute newRoute = std::make_shared<Route>(pInstance->vehicles_[v]->vehicleID_);
                newRoute->addSource(pInstance->vehicles_[v]->departNode_, pInstance->vehicles_[v]->departTime_,
                                                                  pInstance->vehicles_[v]->numPassengers_);
                for (int i = 1; i < routeNodes[v].size(); ++i) {
                    if (routeNodes[v][i] != nullptr)
                        newRoute->addNode(routeNodes[v][i]);
                    else
                        newRoute->addSink(pInstance->vehicles_[v]->sinkNode_);
                }
                pInstance->vehicles_[v]->setCurrentRoute(newRoute);
            }
        }
    }
    ReadWrite::readTripRequests(inputPaths.getInputTripData(), pInstance, pInstance->nbRequests_);
    pInstance->nbRequests_ += (pInstance->nbOnboards_ + pInstance->nbWaiting_);
    pInstance->nbNewRequests_ += pInstance->nbWaiting_;

    inputPaths.initializeOutputs(mainAlgorithmName[pInstance->parameters_->mainAlgorithm_],
                                 solutionModeName[pInstance->parameters_->solutionMode_], saveScratch, pInstance->nbVehicles_);

    // write the parameters in file
    std::ofstream myFile;
    myFile.open (inputPaths.getOutputParamFile());
    myFile << pInstance->parameters_->toString();
    myFile.close();

    Tools::LogOutput parametersStream(inputPaths.getOutputParamCsv(), true);
    parametersStream << "Instance,alpha,beta,delta,epochLength,committedTime,nbThreads,InitialDual,warmStart,"
                        "mainAlgorithm,solutionMode,OneIter,GreedyReOptimize,MIP_maxIncDegree,CP_IncDegree,"
                        "useMultiStage,useZoom,isTruncated,MaxLabel,isDominanceReleased,isDropPickPossible,"
                        "LabelingStrategy,Greedy_portion,nbPick,sortPath,MIPGap\n" << pInstance->name_ << ",";

    parametersStream << pInstance->parameters_->toStr();
    parametersStream.close();
}

// Parsing functions
// Useful for reading a file stream until meeting the separating character
// (the characters that are read until the separating char are stored in ReadStr
bool ReadWrite::readUntilChar(std::fstream &pFile, char separator, std::string &pReadStr) {

    char cTmp = 'A';
    // empty ReadStr if it is not
    if (!pReadStr.empty())
        pReadStr.erase();

    // go through the file until the delimiter is met
    if (pFile.good()){
        cTmp = pFile.get();
    }
    while (cTmp != separator && pFile.good()){
        pReadStr.push_back(cTmp);
        cTmp = pFile.get();
    }
    if(!pFile.good())
        return false;
    return true;
}

// Read a file stream until meeting one of the two separating characters
// (the characters that are read until the separating char are stored in ReadStr
bool ReadWrite::readUntilOneOfTwoChar(std::fstream &pFile, char separator1, char separator2, string &pReadStr) {
    char cTmp = 'A';

    // empty ReadStr if it is not
    if (!pReadStr.empty())
        pReadStr.erase();

    // go through the file until the delimiter is met
    if (pFile.good()){
        cTmp = pFile.get();
    }
    while (cTmp != separator1 && cTmp != separator2 && pFile.good()){
        pReadStr.push_back(cTmp);
        cTmp = pFile.get();
    }
    if(!pFile.good())
        return false;
    return true;
}
// check if a string (sentence) ends with a given substring (word)
bool ReadWrite::strEndWith(const std::string& sentence, const std::string& word) {
    int lenWord = (int) word.length();
    int lenSentence = (int) sentence.length();
    if (lenWord > lenSentence)
        return false;
    else {
        string endOfSentence = sentence.substr(lenSentence-lenWord, lenWord);
        return (!strcmp(word.c_str(), endOfSentence.c_str()));
    }
}

void ReadWrite::readInstNames(const string &strInstanceNameFile, vector<std::string> &fileNames, int nbInstances,
                              std::string index) {
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


