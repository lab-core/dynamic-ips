//
// Created by Ella on 2021-09-08.
//

#include "ReadWrite.h"
extern PTravelTime travelMat;

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
    int nbVehicles = -1, nbRequests = -1;
    float sourceLatitude = -1, sourceLongitude = -1;
    float sinkLatitude = -1, sinkLongitude = -1;
    std::vector<PVehicle> vehicles;

    while (file.good()) {
        readUntilOneOfTwoChar(file, '=','\n', title);

        // read the name of instance
        if (strEndWith(title, "INSTANCE "))
            file >> name;

        // read number of vehicles
        else if (strEndWith(title, "NUM_VEHICLES "))
            file >> nbVehicles;

        // read number of requests
        else if (strEndWith(title, "NUM_REQUESTS "))
            file >> nbRequests;

        // read the source coordination
        else if (strEndWith(title, "SOURCE_LATITUDE "))
            file >> sourceLatitude;
        else if (strEndWith(title, "SOURCE_LONGITUDE "))
            file >> sourceLongitude;

        else if (strEndWith(title, "SINK_LATITUDE "))
            file >> sinkLatitude;
        else if (strEndWith(title, "SINK_LONGITUDE "))
            file >> sinkLongitude;

        // read vehicles specifications
        else if (strEndWith(title, "VEHICLES_INFO")) {
            for (int v = 0; v < nbVehicles; ++v) {
                int vehicleID, capacity, startTime, endTime;
                file >> vehicleID;
                file >> capacity;
                file >> startTime;
                file >> endTime;

                vehicles.emplace_back(std::make_shared<Vehicle>(vehicleID, capacity, startTime, endTime));
            }
        }
    }

    // main graph initialization with source and sink
    PGraph mainGraph = std::make_shared<Graph>(std::make_shared<Node>(sourceLatitude, sourceLongitude, SOURCE),
            std::make_shared<Node>(sinkLatitude, sinkLongitude, SINK));

    return std::make_shared<Instance>(name, nbVehicles, vehicles, nbRequests, mainGraph);
}

//************************************************************************
// Read the trip requests file
//************************************************************************

void ReadWrite::readTripRequests(std::string strTripsFile, PInstance pInstance) {
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

    std::vector<PRequest> requests;
    string title;

    while (file.good()) {
        readUntilChar(file, '\n', title);
        if (strEndWith(title, "REQUESTS_INFO")) {

            for (int r = 0; r < pInstance->nbRequests_; ++r) {
                // attributes for reading trip requests file
                int nbPassengers = -1;
                float pickUpLatitude = -1, pickUpLongitude = -1, dropOffLatitude = -1, dropOffLongitude = -1,
                        earlyPick = -1, minDist = -1, minTravelTime = -1, deltaTime = -1;

                file >> nbPassengers;
            //    file >> minDist;
                file >> pickUpLatitude;
                file >> pickUpLongitude;
                file >> dropOffLatitude;
                file >> dropOffLongitude;
                file >> earlyPick;

                // the starting time of the instance is 16pm
                earlyPick -= 57600;

                minDist = Tools::calcDistance(pickUpLatitude, pickUpLongitude, dropOffLatitude, dropOffLongitude);
                deltaTime = nbPassengers * TimePerPassenger;

//                minTravelTime = Tools::queryTravelTime(pickUpLatitude, pickUpLongitude, dropOffLatitude, dropOffLongitude);
                minTravelTime = 0;
                pInstance->requests_.emplace_back(std::make_shared<Request>(pickUpLatitude,
                                                                            pickUpLongitude, dropOffLatitude,
                                                                            dropOffLongitude,
                                                                            earlyPick, nbPassengers, deltaTime, minDist,
                                                                            minTravelTime));
                pInstance->nameToRequest_[pInstance->requests_.back()->name_] = pInstance->requests_.back();
            }
        }
    }

    // define an empty route for each vehicle and set it as the current route for the initialization
    pInstance->instGraph_->addNewRequests(pInstance->requests_);
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






