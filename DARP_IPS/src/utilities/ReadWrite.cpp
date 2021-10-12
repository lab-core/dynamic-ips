//
// Created by Ella on 2021-09-08.
//

#include "ReadWrite.h"


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

    return std::make_shared<Instance>(name, nbVehicles, vehicles, nbRequests);
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
                int requestID = -1, nbPassengers = -1;
                float pickUpLatitude = -1, pickUpLongitude = -1, dropOffLatitude = -1, dropOffLongitude = -1,
                        earlyPick = -1, minDist = -1, minTravelTime = -1, deltaTime = -1;

                requestID = r;
                file >> nbPassengers;
                file >> minDist;
                file >> pickUpLongitude;
                file >> pickUpLatitude;
                file >> dropOffLongitude;
                file >> dropOffLatitude;
                file >> earlyPick;

                /* ==================================needs modification=================================== */
                // I consider 10 seconds for each passenger to pickup or drop off
                // the constant 475 calculated by excel just to convert distance in mile to travel time in sec

                minDist = Tools::calcDistance(pickUpLatitude, pickUpLongitude, dropOffLatitude, dropOffLongitude);
                deltaTime = nbPassengers * TimePerPassenger;
                minTravelTime = minDist * 475;

                pInstance->requests_.emplace_back(std::make_shared<Request>(requestID, pickUpLatitude,
                                                                            pickUpLongitude, dropOffLatitude,
                                                                            dropOffLongitude,
                                                                            earlyPick, nbPassengers, deltaTime, minDist,
                                                                            minTravelTime));
            }
        }
    }
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






