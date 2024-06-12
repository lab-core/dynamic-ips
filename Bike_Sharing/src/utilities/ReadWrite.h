//
// Created by Ella on 2021-09-08.
//

#ifndef _READWRITE_H
#define _READWRITE_H


#include "data/Instance.h"
#include "utilities/InputPaths.h"
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include <unordered_set>
#include <string>

//-----------------------------------------------------------------------------
//  ReadWrite class
//  functions to read input and write output
//-----------------------------------------------------------------------------
using namespace rapidjson;

class ReadWrite {
public:

    // Read duration data file
    static void readDurations_txt(const std::string& strDurFile, vector2D<float> &durationMat, int nbLocations);

    static void readTimeMatrix(const std::string& strDurFile, vector2D<float> &durationMat);

    // Read the parameters datafile
    static void readParameters1(const std::string& strParamFile, PInstance &pInstance);


    // Parsing functions
    // Read a file stream until meeting the separating character
    // (the characters that are read until the separating char are stored in pReadStr
    static bool readUntilChar(std::fstream &pFile, char separator, std::string &pReadStr);

    // Read a file stream until meeting one of the two separating characters
    // (the characters that are read until the separating char are stored in pReadStr
    static bool readUntilOneOfTwoChar(std::fstream &pFile, char separator1, char separator2, std::string &pReadStr);

    // check if a string (sentence) ends with a given substring (word)
    static bool strEndWith(const std::string& sentence, const std::string& word);

    static void readTasks(const std::string& strTaskFile, PInstance &pInstance);
    static void readVehicles(const std::string& strVehicleFile, PInstance &pInstance);
    static void readParameters(const std::string& strParamFile, PInstance &pInstance);
    static void readDurations(const std::string& strDurFile, vector2D<float> &durationMat);

    static void readDurations_py(const std::string& strDurFile, vector2D<float> &durationMat);
    static void readParameters_py(const std::string& jsonStr, PInstance &pInstance);
    static void readTasks_py(const std::string& jsonStr, PInstance &pInstance);
    static void readVehicles_py(const std::string& jsonStr, PInstance &pInstance);

    static PInstance createInstance(const std::string& jsonStrDuration, const std::string& jsonStrParam,
                                    const std::string& jsonStrTasks, const std::string& jsonStrVehicles);

    static PInstance createInstanceFile(const std::string& strDurFile, const std::string& strParamFile,
                                    const std::string& strTaskFile, const std::string& strVehicleFile);
};


#endif //_READWRITE_H
