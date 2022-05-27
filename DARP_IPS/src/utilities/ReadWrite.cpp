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

PInstance ReadWrite::readInstance(std::string strInstanceFile) {
    // open the file
    std::fstream file;
    std::cout << "Reading << " << strInstanceFile << " >>" << std::endl;
    file.open(strInstanceFile, std::fstream::in);
    if (!file.is_open())
    {
        std::cout << "While trying to read the file " << strInstanceFile << std::endl;
        std::cout << "The input file was not opened properly!" << std::endl;
        throw Tools::myException("The input file was not opened properly!", __LINE__);
    }

    // attributes to read data file and initialize instance
    string title;
    std::string name;
    int nbVehicles = -1, nbRequests = -1, nbOnboards = -1, nbReceived = -1;
    /*double sourceLatitude = -1, sourceLongitude = -1;
    double sinkLatitude = -1, sinkLongitude = -1;*/
    float simulationStart = -1;
    int sourceID = -1, sinkID = -1;
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

    }

    // main graph initialization with source and sink
    /*PGraph mainGraph = std::make_shared<Graph>(std::make_shared<Node>(sourceLatitude, sourceLongitude, sourceID, SOURCE),
                                               std::make_shared<Node>(sinkLatitude, sinkLongitude, sinkID, SINK));*/

    PGraph mainGraph = std::make_shared<Graph>();
    return std::make_shared<Instance>(name, simulationStart, nbVehicles, nbOnboards, nbReceived, vehicles, nbRequests,
                                      mainGraph);
}

//************************************************************************
// Read the vehicle file
//************************************************************************
void ReadWrite::readVehiclesData(std::string strTripsFile, PInstance &pInstance) {
// open the file
    std::fstream file;
    std::cout << "Reading << " << strTripsFile << " >>" << std::endl;
    file.open(strTripsFile, std::fstream::in);
    if (!file.is_open())
    {
        std::cout << "While trying to read the file " << strTripsFile << std::endl;
        std::cout << "The input file was not opened properly!" << std::endl;

        throw Tools::myException("The input file was not opened properly!", __LINE__);
    }

    string title;
    std::vector<PVehicle> vehicles;
    int vehicleID = -1, capacity = -1, departID = -1, sinkID = -1;
    float departTime = -1, endTime = -1;
//    double departLatitude = -1, departLongitude, sinkLatitude = -1, sinkLongitude = -1;


    while (file.good()) {
        readUntilChar(file, '\n', title);
        if (strEndWith(title, "VEHICLES_INFO")) {
            for (int v = 0; v < pInstance->nbVehicles_; ++v) {
                file >> vehicleID;
                file >> capacity;
                file >> departTime;
                file >> endTime;
                file >> departID;
                file >> sinkID;
                pInstance->instGraph_->addNewNode(std::make_shared<Node>(
                        departID, SOURCE, vehicleID));
                pInstance->instGraph_->addNewNode(std::make_shared<Node>(
                        sinkID, SINK, vehicleID));
                pInstance->vehicles_.emplace_back(std::make_shared<Vehicle>(vehicleID, capacity, departTime, endTime,
                                                                            Tools::createSourceID(vehicleID, SOURCE),
                                                                            Tools::createSourceID(vehicleID, SINK)));
                pInstance->vehicles_.back()->startTime_ = pInstance->simulationStartTime_;
            }
        }
    }
}

//************************************************************************
// Read the onboard file
//************************************************************************
void ReadWrite::readOnboardRequests(std::string strTripsFile, PInstance &pInstance) {
// open the file
    std::fstream file;
    std::cout << "Reading << " << strTripsFile << " >>" << std::endl;
    file.open(strTripsFile, std::fstream::in);
    if (!file.is_open())
    {
        std::cout << "While trying to read the file " << strTripsFile << std::endl;
        std::cout << "The input file was not opened properly!" << std::endl;

        throw Tools::myException("The input file was not opened properly!", __LINE__);
    }

    string title;

    while (file.good()) {
        readUntilChar(file, '\n', title);
        if (strEndWith(title, "REQUESTS_INFO")) {

            for (int r = 0; r < pInstance->nbOnboards_; ++r) {
                // attributes for reading trip requests file
                int nbPassengers = -1, vehicleID = -1;
 //               double pickUpLatitude = -1, pickUpLongitude = -1, dropOffLatitude = -1, dropOffLongitude = -1;
                float pickUpID = -1, dropOffID = -1, earlyPick = -1, pickTime = -1, deltaTime = -1;

                file >> nbPassengers;
                file >> pickUpID;
                file >> dropOffID;
                file >> earlyPick;
                file >> pickTime;

                file >> vehicleID;

                // the starting time of the instance is 16pm
                deltaTime = nbPassengers * TimePerPassenger;
                pInstance->requests_.emplace_back(std::make_shared<Request>( pickUpID, dropOffID, earlyPick,
                                                                             nbPassengers, deltaTime));
                pInstance->requests_.back()->requestStatus_ = ON_BOARD;
                pInstance->requests_.back()->pickTime_ = pickTime;
                pInstance->requests_.back()->vehicleID_ = vehicleID;

                pInstance->nameToRequest_[pInstance->requests_.back()->name_] = pInstance->requests_.back();
                pInstance->instGraph_->addRequestToGraph(pInstance->requests_.back());
        //        pInstance->instGraph_->addNewRequestToGraph(pInstance);
                std::string dropID = Tools::createNodeID(pInstance->requests_.back()->getRequestId(), DROPOFF);
                pInstance->vehicles_[vehicleID]->onboards_.push_back(dropID);
                pInstance->vehicles_[vehicleID]->numPassengers_+= pInstance->requests_.back()->nbPassengers_;
                pInstance->instGraph_->nodes_[dropID]->nodeStatus_ = PLANNED;
                (*pInstance->instGraph_->nodes_[dropID]->pairNode_)->nodeStatus_ = DONE;
                (*pInstance->instGraph_->nodes_[dropID]->pairNode_)->reachTime_ = pickTime;
            }
        }
    }
}

//************************************************************************
// Read the trip requests file
//************************************************************************

void ReadWrite::readTripRequests(std::string strTripsFile, PInstance &pInstance, int nbRequest) {
    // open the file
    std::fstream file;
    std::cout << "Reading << " << strTripsFile << " >>" << std::endl;
    file.open(strTripsFile, std::fstream::in);
    if (!file.is_open())
    {
        std::cout << "While trying to read the file " << strTripsFile << std::endl;
        std::cout << "The input file was not opened properly!" << std::endl;

        throw Tools::myException("The input file was not opened properly!", __LINE__);
    }

    string title;

    while (file.good()) {
        readUntilChar(file, '\n', title);
        if (strEndWith(title, "REQUESTS_INFO")) {

            for (int r = 0; r < nbRequest; ++r) {
                // attributes for reading trip requests file
                int nbPassengers = -1;
//                double pickUpLatitude = -1, pickUpLongitude = -1, dropOffLatitude = -1, dropOffLongitude = -1,
                int pickUpID = -1, dropOffID = -1;
                float earlyPick = -1, deltaTime = -1;

                file >> nbPassengers;
                file >> pickUpID;
                file >> dropOffID;
                file >> earlyPick;

                // the starting time of the instance is 16pm
//                earlyPick -= 57600;
                deltaTime = nbPassengers * TimePerPassenger;
                pInstance->requests_.emplace_back(std::make_shared<Request>( pickUpID, dropOffID, earlyPick,
                                                                             nbPassengers, deltaTime));
                pInstance->nameToRequest_[pInstance->requests_.back()->name_] = pInstance->requests_.back();
                pInstance->instGraph_->addRequestToGraph(pInstance->requests_.back());
        //        pInstance->instGraph_->addNewRequestToGraph(pInstance);
                pInstance->requests_.back()->setPenalty(0, pInstance->parameters_, pInstance->simulationStartTime_);
            }
        }
    }
//    pInstance->instGraph_->addRequestsToGraph(pInstance);
}


//************************************************************************
// Read the duration data file
//************************************************************************

void ReadWrite::readDurations(std::string strDurFile, vector2D<float> &durationMat, int nbLocations) {
// open the file
    std::fstream file;
    std::cout << "Reading << " << strDurFile << " >>" << std::endl;
    file.open(strDurFile, std::fstream::in);
    if (!file.is_open())
    {
        std::cout << "While trying to read the file " << strDurFile << std::endl;
        std::cout << "The input file was not opened properly!" << std::endl;

        throw Tools::myException("The input file was not opened properly!", __LINE__);
    }

    durationMat.clear();
    durationMat.resize(nbLocations);
    for (int i = 0; i < nbLocations; ++i)
        durationMat[i].resize(nbLocations);
    string title;

    while (file.good()) {
        readUntilChar(file, '\n', title);
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
void ReadWrite::readParameters(std::string strParamFile, PInstance &pInstance) {
// open the file
    std::fstream file;
    std::cout << "Reading << " << strParamFile << " >>" << std::endl;
    file.open(strParamFile, std::fstream::in);
    if (!file.is_open())
    {
        std::cout << "While trying to read the file " << strParamFile << std::endl;
        std::cout << "The input file was not opened properly!" << std::endl;

        throw Tools::myException("The input file was not opened properly!", __LINE__);
    }

    string title;

    float alphaParam = -1, betaParam = -1, deltaPram = -1;
    int epochLength = -1, bigM = -1, solveTimeLimit = -1, populateTimeLimit = -1, maxLabel = -1;
    bool isTruncated = 0, isSuccessorsLimited = 0, isDominanceReleased = 0,  sameDepot = 0;
    int subAlgorithm = 0, subproSolveStartState = 0 , mainAlgorithm = 0;
    int strategy = 0;

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


        else if (strEndWith(title, "sameDepot "))
            file >> sameDepot;

        else if (strEndWith(title, "mainAlgorithm "))
            file >> mainAlgorithm;

        else if (strEndWith(title, "isTruncated "))
            file >> isTruncated;

        else if (strEndWith(title, "MaxLabel "))
            file >> maxLabel;

        else if (strEndWith(title, "isDominanceReleased "))
            file >> isDominanceReleased;

        else if (strEndWith(title, "isSuccessorsLimited "))
            file >> isSuccessorsLimited;

        else if (strEndWith(title, "SubproSolveStartState "))
            file >> subproSolveStartState;

        else if (strEndWith(title, "LabelingStrategy "))
            file >> strategy;

        else if (strEndWith(title, "subproblemAlgorithm "))
            file >> subAlgorithm;

        else if (strEndWith(title, "BigM "))
            file >> bigM;

        else if (strEndWith(title, "solveTimeLimit "))
            file >> solveTimeLimit;

        else if (strEndWith(title, "populateTimeLimit "))
            file >> populateTimeLimit;
    }
    pInstance->parameters_ = std::make_shared<Parameters>(alphaParam, betaParam, deltaPram, epochLength, sameDepot,
                                                           static_cast<MainAlgorithm>(mainAlgorithm),
                                                          isTruncated, maxLabel, isSuccessorsLimited,
                                                          isDominanceReleased,
                                                          static_cast<SubProSolveStart>(subproSolveStartState),
                                                          static_cast<LabelingStrategy>(strategy),
                                                          static_cast<subproblemAlgorithm>(subAlgorithm),
                                                          bigM, solveTimeLimit, populateTimeLimit);
}

// function that open all input files and create the main instance
PInstance ReadWrite::createMainInstance(InputPaths &inputPaths) {
    PInstance mainInst = ReadWrite::readInstance(inputPaths.getInputInstanceData());
    ReadWrite::readVehiclesData(inputPaths.getInputVehicleFile(), mainInst);
    ReadWrite::readParameters(inputPaths.getInputParamFile(), mainInst);
    if (mainInst->nbOnboards_ > 0)
        ReadWrite::readOnboardRequests(inputPaths.getInputOnboardsFile(), mainInst);
    if (mainInst->nbWaiting_ > 0)
        ReadWrite::readTripRequests(inputPaths.getInputWaitRequests(), mainInst, mainInst->nbWaiting_);
    ReadWrite::readTripRequests(inputPaths.getInputTripData(), mainInst, mainInst->nbRequests_);
    mainInst->nbRequests_ += (mainInst->nbOnboards_ + mainInst->nbWaiting_);
    mainInst->nbNewRequests_ += mainInst->nbWaiting_;

    // write the parameters in file
    std::ofstream myFile;
    myFile.open (inputPaths.getOutputParamFile());
    myFile << mainInst->parameters_->toString();
    myFile.close();

    return mainInst;
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
bool ReadWrite::strEndWith(std::string sentence, std::string word) {
    int lenWord = word.length();
    int lenSentence = sentence.length();
    if (lenWord > lenSentence)
        return false;
    else {
        string endOfSentence = sentence.substr(lenSentence-lenWord, lenWord);
        return (!strcmp(word.c_str(), endOfSentence.c_str()));
    }
}











