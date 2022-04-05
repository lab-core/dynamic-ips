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
    static PInstance readInstance(std::string strInstanceFile);

    // Read the vehicle file
    static void readVehiclesData(std::string strTripsFile, PInstance &pInstance);

    // Read the onboard file
    static void readOnboardRequests(std::string strTripsFile, PInstance &pInstance);

    // Read the trip requests file
    static void readTripRequests(std::string strTripsFile, PInstance &pInstance, int nbRequest);

    // Read duration data file
    static void readDurations(std::string strDurFile, vector2D<float> &durationMat, int nbLocations);

    // Read the parameters datafile
    static void readParameters(std::string strParamFile, PInstance &pInstance);

    // function that open all input files and create the main instance
    static PInstance createMainInstance(InputPaths &inputPaths);

    // Parsing functions
    // Read a file stream until meeting the separating character
    // (the characters that are read until the separating char are stored in pReadStr
    static bool readUntilChar(std::fstream &pFile, char separator, std::string &pReadStr);

    // Read a file stream until meeting one of the two separating characters
    // (the characters that are read until the separating char are stored in pReadStr
    static bool readUntilOneOfTwoChar(std::fstream &pFile, char separator1, char separator2, std::string &pReadStr);

    // check if a string (sentence) ends with a given substring (word)
    static bool strEndWith(std::string sentence, std::string word);
};


#endif //DARP_IPS_READWRITE_H
