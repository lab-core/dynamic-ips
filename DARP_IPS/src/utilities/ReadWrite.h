//
// Created by Ella on 2021-09-08.
//

#ifndef _READWRITE_H
#define _READWRITE_H


#include "data/Instance.h"
#include "utilities/InputPaths.h"


//-----------------------------------------------------------------------------
//  ReadWrite class
//  functions to read input and write output
//-----------------------------------------------------------------------------


class ReadWrite {
public:

    // Read the instance file and store content in an instance object
    static PInstance readInstance(const std::string& strInstanceFile);

    // Read the vehicle file
    static void readVehiclesData(const std::string& strTripsFile, PInstance &pInstance);
    static void readVehiclesDataF(const std::string& strTripsFile, PInstance &pInstance, vector2D<PNode> &routeNodes);

    // Read the onboard file
    static void readOnboardRequests(const std::string& strTripsFile, PInstance &pInstance, vector2D<PNode> &routeNodes);

    // Read the trip requests file
    static void readTripRequests(const std::string& strTripsFile, PInstance &pInstance, int nbRequest);
    static void readWaitRequests(const std::string& strTripsFile, PInstance &pInstance, int nbRequest, vector2D<PNode> &routeNodes);

    // Read duration data file
    static void readDurations(const std::string& strDurFile, vector2D<float> &durationMat, int nbLocations);

    // Read the parameters datafile
    static void readParameters(const std::string& strParamFile, PInstance &pInstance);
    static void readZones(const std::string& strZoneFile, PInstance &pInstance);

    // function that open all input files and update main instance data
    static void readDatafiles(InputPaths &inputPaths, PInstance &pInstance, int saveScratch, const std::string& paramFile);

    // Parsing functions
    // Read a file stream until meeting the separating character
    // (the characters that are read until the separating char are stored in pReadStr
    static bool readUntilChar(std::fstream &pFile, char separator, std::string &pReadStr);

    // Read a file stream until meeting one of the two separating characters
    // (the characters that are read until the separating char are stored in pReadStr
    static bool readUntilOneOfTwoChar(std::fstream &pFile, char separator1, char separator2, std::string &pReadStr);

    // check if a string (sentence) ends with a given substring (word)
    static bool strEndWith(const std::string& sentence, const std::string& word);

    // Read the parameters datafile
    static void readInstNames(const std::string& strInstanceNameFile, std::vector<std::string> &fileNames,
                              int nbInstances, std::string index);
};


#endif //_READWRITE_H
