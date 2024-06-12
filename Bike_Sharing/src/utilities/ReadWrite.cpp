//
// Created by Ella on 2021-09-08.
//

#include "ReadWrite.h"
//extern PTravelTime travelMat;

//-----------------------------------------------------------------------------
//  ReadWrite class
//  functions to read input and write output
//-----------------------------------------------------------------------------

void ReadWrite::readDurations_txt(const std::string& strDurFile, vector2D<float> &durationMat, int nbLocations) {
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

void ReadWrite::readDurations(const std::string& strDurFile, vector2D<float>& durationMat) {
    FILE* fp = fopen(strDurFile.c_str(), "r");
    if (!fp) {
        std::cerr << "Could not open file " << strDurFile << std::endl;
        throw std::runtime_error("File not opened properly");
    }

    char readBuffer[65536];
    rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));

    rapidjson::Document doc;
    doc.ParseStream(is);
    fclose(fp);

    if (!doc.IsObject()) {
        throw std::runtime_error("Invalid JSON format");
    }

    // Determine the number of locations
    int nbLocations = doc.MemberCount();

    durationMat.clear();
    durationMat.resize(nbLocations, std::vector<float>(nbLocations, 0.0f));

    for (auto& start : doc.GetObject()) {
        int startID = std::stoi(start.name.GetString());
        const auto& innerDict = start.value.GetObject();
        for (auto& end : innerDict) {
            int endID = std::stoi(end.name.GetString());
            float duration = static_cast<float>(end.value.GetDouble());
            durationMat[startID][endID] = duration;
        }
    }
}

void ReadWrite::readTimeMatrix(const std::string &strDurFile, vector2D<float> &durationMat) {
    // Read the JSON file
    // Attempt to open the JSON file
    std::ifstream ifs(strDurFile);
    if (!ifs) {
        std::cerr << "Failed to open file: " << strDurFile << std::endl;
        return;
    }

    std::string content((std::istreambuf_iterator<char>(ifs)),
                        std::istreambuf_iterator<char>());

    // Initialize variables to track parsing state
    bool isInQuotes = false, isEscaped = false;
    std::string key, value;
    int row = -1, column;
    bool isKey = true; // Toggle between parsing key and value

    for (char& ch : content) {
        if (ch == '\\') { // Handle escape character
            isEscaped = !isEscaped;
            continue;
        } else if (ch == '"' && !isEscaped) {
            isInQuotes = !isInQuotes;
            continue;
        } else if (isEscaped) {
            isEscaped = false;
        }

        if (isInQuotes || ch == ':' || ch == ',' || ch == '{' || ch == '}') {
            if (ch != ':' && ch != ',' && ch != '{' && ch != '}') {
                if (isKey) key += ch;
                else value += ch;
            } else if (ch == ':') {
                isKey = false; // Next characters belong to value
            } else if (ch == ',' || ch == '}') {
                if (!key.empty() && !value.empty()) {
                    column = std::stoi(key);
                    float val = std::stof(value);

                    // Ensure the matrix is large enough
                    if (row >= 0 && column >= 0) {
                        if (durationMat.size() <= row) durationMat.resize(row + 1);
                        if (durationMat[row].size() <= column) durationMat[row].resize(column + 1);
                        durationMat[row][column] = val;
                    }

                    key.clear();
                    value.clear();
                    isKey = true; // Next characters belong to key
                }

                if (ch == '}') {
                    row++; // Move to the next row for each closing brace
                }
            } else if (ch == '{' && !isKey) {
                // Handle nested object start
                continue;
            }
        }
    }

}


//************************************************************************
// Read the parameters datafile
//************************************************************************
void ReadWrite::readParameters1(const std::string& strParamFile, PInstance &pInstance) {
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

    float alphaParam = -1, betaParam = -1, deltaPram = -1, committedTime = -1, mipGap = -1;
    int epochLength = -1, penaltyL = -1, nbThreads = -1, nbColumns = -1, maxLabel = -1, nbStop = -1,solveTimeLimit = -1;
    bool isTruncated = false, isDominanceReleased = false, oneIter = false;
    bool isPickDropPossible = false, vehicle_portion = false;
    bool greedyReOptimize = false, vehicleReturn = false;
    int mainAlgorithm = -1, solutionMode = -1, labelingStrategy = -1, sortPath = -1;


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

        else if (strEndWith(title, "mainAlgorithm "))
            file >> mainAlgorithm;

        else if (strEndWith(title, "solutionMode "))
            file >> solutionMode;

        else if (strEndWith(title, "OneIter "))
            file >> oneIter;

        else if (strEndWith(title, "GreedyReOptimize "))
            file >> greedyReOptimize;


        else if (strEndWith(title, "vehicleReturn "))
            file >> vehicleReturn;


        else if (strEndWith(title, "NumColumn "))
            file >> nbColumns;

        else if (strEndWith(title, "isTruncated "))
            file >> isTruncated;

        else if (strEndWith(title, "MaxLabel "))
            file >> maxLabel;

        else if (strEndWith(title, "isDominanceReleased "))
            file >> isDominanceReleased;


        else if (strEndWith(title, "isDropPickPossible "))
            file >> isPickDropPossible;


        else if (strEndWith(title, "LabelingStrategy "))
            file >> labelingStrategy;

        else if (strEndWith(title, "Vehicle_portion "))
            file >> vehicle_portion;


        else if (strEndWith(title, "nbStop "))
            file >> nbStop;

        else if (strEndWith(title, "sortPath "))
            file >> sortPath;

        else if (strEndWith(title, "solveTimeLimit "))
            file >> solveTimeLimit;

        else if (strEndWith(title, "MIPGap "))
            file >> mipGap;
    }
    pInstance->parameters_ = std::make_shared<Parameters>(epochLength, nbThreads,
                                                          static_cast<MainAlgorithm>(mainAlgorithm), oneIter,
                                                          true, nbColumns, isTruncated, maxLabel,
                                                          isPickDropPossible, nbStop,
                                                          static_cast<SortPaths>(sortPath), mipGap);

}


void ReadWrite::readParameters(const std::string& strParamFile, PInstance &pInstance) {
    // Open the JSON file for reading
    std::ifstream file(strParamFile);
    if (!file.is_open()) {
        std::cerr << "Error opening JSON file: " << strParamFile << std::endl;
        throw myTools::myException("Error opening JSON file", __LINE__);
    }

    // Read JSON data into a string
    std::string jsonStr((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    // Parse the JSON data
    Document document;
    document.Parse(jsonStr.c_str());


    // Extract values from the JSON document
    int epochLength = document["epoch_length"].GetInt();
    int nbThreads = document["nb_threads"].GetInt();
    int mainAlgorithm = document["main_algorithm"].GetInt();
    int nbColumns = document["nb_column"].GetInt();
    bool isTruncated = document["is_truncated"].GetBool();
    int maxLabel = document["max_label"].GetInt();
    int nbStop = document["nb_stops"].GetInt();
    int sortPath = document["sort_path"].GetInt();
    float mipGap = document["mip_gap"].GetFloat();

    // Create Parameters object and set values
    pInstance->parameters_ = std::make_shared<Parameters>(epochLength, nbThreads,
                                                          static_cast<MainAlgorithm>(mainAlgorithm),
                                                          true, true, nbColumns, isTruncated, maxLabel,
                                                          false, nbStop, static_cast<SortPaths>(sortPath),
                                                          mipGap);
    pInstance->subProOptions_ = std::make_shared<solverOption>(pInstance->parameters_);
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

void ReadWrite::readTasks(const std::string& strTaskFile, PInstance &pInstance) {
    // Open the file
    std::fstream file;
    std::cout << "Reading << " << strTaskFile << " >>" << std::endl;
    file.open(strTaskFile, std::fstream::in);
    if (!file.is_open()) {
        std::cout << "While trying to read the file " << strTaskFile << std::endl;
        std::cout << "The input file was not opened properly!" << std::endl;
        throw myTools::myException("The input file was not opened properly!", __LINE__);
    }

    // Read the whole file into a std::string
    std::string jsonStr((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    // Parse the JSON data
    rapidjson::Document document;
    document.Parse(jsonStr.c_str());

    if (!document.IsObject()) {
        std::cerr << "JSON is not an object\n";
        throw std::runtime_error("JSON document is not an object.");
    }

    // Iterate over the JSON object
    for (auto it = document.MemberBegin(); it != document.MemberEnd(); ++it) {
        const rapidjson::Value& item = it->value;

        try {
            int station_id = item["station_id"].GetInt();
            std::string location_id = item["location_id"].GetString();
            int location_index = item["location_index"].GetInt();
            int nb_transfer = item["nb_transfer"].GetInt();  // Corrected from GetFloat to GetInt
            double relocate_bonus = item["relocate_bonus"].GetDouble();

            // Assuming Task constructor takes the right parameters in this order
            pInstance->tasks_.emplace_back(std::make_shared<Task>(station_id, location_id, location_index,
                                                                  nb_transfer, relocate_bonus));
            PNode taskNode = std::make_shared<Node>(location_index,pInstance->tasks_.back(),TASK_STATION);
            pInstance->instGraph_->addNewNode(taskNode);

        } catch (const std::exception& e) {
            std::cerr << "Exception while creating Task: " << e.what() << '\n';
        }

    }
    pInstance->nbTasks_ = pInstance->tasks_.size();
}

void ReadWrite::readVehicles(const std::string& strVehicleFile, PInstance &pInstance) {
    // Open the file
    std::fstream file;
    std::cout << "Reading << " << strVehicleFile << " >>" << std::endl;
    file.open(strVehicleFile, std::fstream::in);
    if (!file.is_open()) {
        std::cout << "While trying to read the file " << strVehicleFile << std::endl;
        std::cout << "The input file was not opened properly!" << std::endl;
        throw myTools::myException("The input file was not opened properly!", __LINE__);
    }

    // Read the whole file into a std::string
    std::string jsonStr((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    // Parse the JSON data
    rapidjson::Document document;
    document.Parse(jsonStr.c_str());

    if (!document.IsObject()) {
        std::cerr << "JSON is not an object\n";
        throw std::runtime_error("JSON document is not an object.");
    }

    // Iterate over the JSON object
    for (auto it = document.MemberBegin(); it != document.MemberEnd(); ++it) {
        const rapidjson::Value& item = it->value;

        try {
            int vehicleId = item["vehicle_id"].GetInt();
            std::string name = item["name"].GetString();
            float readyTime = item["ready_time"].GetFloat();
            float endTime = item["end_time"].GetFloat();
            int departIndex = item["depart_index"].GetInt();
            int capacity = item["capacity"].GetInt();
            int bikeLoad = item["bike_load"].GetInt();

            // Assuming Task constructor takes the right parameters in this order
            PNode departNode = std::make_shared<Node>(departIndex,DEPART_NODE);
            pInstance->vehicles_.emplace_back(std::make_shared<Vehicle>(vehicleId, name, readyTime, endTime, departIndex,
                                                                     capacity, bikeLoad, departNode));

            pInstance->instGraph_->addNewNode(departNode);

        } catch (const std::exception& e) {
            std::cerr << "Exception while creating Task: " << e.what() << '\n';
        }
    }
    PNode sinkNode = std::make_shared<Node>(661,SINK_NODE);
    pInstance->instGraph_->addNewNode(sinkNode);
    pInstance->nbVehicles_ = pInstance->vehicles_.size();
}

void ReadWrite::readDurations_py(const std::string& jsonStr, vector2D<float>& durationMat) {

    // Parse the Durations JSON data
    rapidjson::Document document;
    document.Parse(jsonStr.c_str());

    if (!document.IsObject()) {
        throw std::runtime_error("Invalid JSON format");
    }

    // Determine the number of locations
    int nbLocations = document.MemberCount();
    durationMat.resize(nbLocations, std::vector<float>(nbLocations, 0.0f));

    for (auto& start : document.GetObject()) {
        int startID = std::stoi(start.name.GetString());
        const auto& innerDict = start.value.GetObject();
        for (auto& end : innerDict) {
            int endID = std::stoi(end.name.GetString());
            float duration = static_cast<float>(end.value.GetDouble());
            durationMat[startID][endID] = duration;
        }
    }
}
void ReadWrite::readVehicles_py(const std::string& jsonStr, PInstance &pInstance) {
    // Parse the JSON data
    rapidjson::Document document;
    document.Parse(jsonStr.c_str());

    if (!document.IsObject()) {
        std::cerr << "JSON is not an object\n";
        throw std::runtime_error("JSON document is not an object.");
    }

    // Iterate over the JSON object
    for (auto it = document.MemberBegin(); it != document.MemberEnd(); ++it) {
        const rapidjson::Value& item = it->value;

        try {
            int vehicleId = item["vehicle_id"].GetInt();
            std::string name = item["name"].GetString();
            float readyTime = item["ready_time"].GetFloat();
            float endTime = item["end_time"].GetFloat();
            int departIndex = item["depart_index"].GetInt();
            int capacity = item["capacity"].GetInt();
            int bikeLoad = item["bike_load"].GetInt();

            // Assuming Task constructor takes the right parameters in this order
            PNode departNode = std::make_shared<Node>(departIndex, DEPART_NODE);
            pInstance->vehicles_.emplace_back(std::make_shared<Vehicle>(vehicleId, name, readyTime, endTime, departIndex,
                                                                        capacity, bikeLoad, departNode));

            pInstance->instGraph_->addNewNode(departNode);

        } catch (const std::exception& e) {
            std::cerr << "Exception while creating Task: " << e.what() << '\n';
        }
    }
    PNode sinkNode = std::make_shared<Node>(661, SINK_NODE);
    pInstance->instGraph_->addNewNode(sinkNode);
    pInstance->nbVehicles_ = pInstance->vehicles_.size();
}
void ReadWrite::readTasks_py(const std::string& jsonStr, PInstance &pInstance) {
    // Parse the JSON data
    rapidjson::Document document;
    document.Parse(jsonStr.c_str());

    if (!document.IsObject()) {
        std::cerr << "JSON is not an object\n";
        throw std::runtime_error("JSON document is not an object.");
    }

    // Iterate over the JSON object
    for (auto it = document.MemberBegin(); it != document.MemberEnd(); ++it) {
        const rapidjson::Value& item = it->value;

        try {
            int station_id = item["station_id"].GetInt();
            std::string location_id = item["location_id"].GetString();
            int location_index = item["location_index"].GetInt();
            int nb_transfer = item["nb_transfer"].GetInt();
            double relocate_bonus = item["relocate_bonus"].GetDouble();

            // Assuming Task constructor takes the right parameters in this order
            pInstance->tasks_.emplace_back(std::make_shared<Task>(station_id, location_id, location_index,
                                                                  nb_transfer, relocate_bonus));
            PNode taskNode = std::make_shared<Node>(location_index, pInstance->tasks_.back(), TASK_STATION);
            pInstance->instGraph_->addNewNode(taskNode);

        } catch (const std::exception& e) {
            std::cerr << "Exception while creating Task: " << e.what() << '\n';
        }
    }
    pInstance->nbTasks_ = pInstance->tasks_.size();
}
void ReadWrite::readParameters_py(const std::string& jsonStr, PInstance &pInstance) {
    // Parse the JSON data
    Document document;
    document.Parse(jsonStr.c_str());

    // Check for parsing errors
    if (document.HasParseError()) {
        std::cerr << "Error parsing JSON data" << std::endl;
        throw myTools::myException("Error parsing JSON data", __LINE__);
    }

    // Extract values from the JSON document
    int epochLength = document["epoch_length"].GetInt();
    int nbThreads = document["nb_threads"].GetInt();
    int mainAlgorithm = document["main_algorithm"].GetInt();
    int nbColumns = document["nb_column"].GetInt();
    bool isTruncated = document["is_truncated"].GetBool();
    int maxLabel = document["max_label"].GetInt();
    int nbStop = document["nb_stops"].GetInt();
    int sortPath = document["sort_path"].GetInt();
    float mipGap = document["mip_gap"].GetFloat();

    // Create Parameters object and set values
    pInstance->parameters_ = std::make_shared<Parameters>(epochLength, nbThreads,
                                                          static_cast<MainAlgorithm>(mainAlgorithm),
                                                          true, true, nbColumns, isTruncated, maxLabel,
                                                          false, nbStop, static_cast<SortPaths>(sortPath),
                                                          mipGap);
    pInstance->subProOptions_ = std::make_shared<solverOption>(pInstance->parameters_);
}

PInstance ReadWrite::createInstance(const std::string& jsonStrDuration, const std::string& jsonStrParam,
                          const std::string& jsonStrTasks, const std::string& jsonStrVehicles) {
    PInstance mainInst = std::make_shared<Instance>();

    readDurations_py(jsonStrDuration, mainInst->getDurationMatrix());
    readParameters_py(jsonStrParam, mainInst);
    readTasks_py(jsonStrTasks, mainInst);
    readVehicles_py(jsonStrVehicles, mainInst);

    return mainInst;
}

PInstance ReadWrite::createInstanceFile(const std::string& strDurFile, const std::string& strParamFile,
                                        const std::string& strTaskFile, const std::string& strVehicleFile) {
    PInstance mainInst = std::make_shared<Instance>();

    readDurations(strDurFile, mainInst->getDurationMatrix());
    readParameters(strParamFile, mainInst);
    readTasks(strTaskFile, mainInst);
    readVehicles(strVehicleFile, mainInst);

    return mainInst;
}






