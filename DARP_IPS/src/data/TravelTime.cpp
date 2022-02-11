//
// Created by Ella on 2021-12-15.
//

#include "TravelTime.h"

//-----------------------------------------------------------------------------
//  Node class
//  Define pickup or drop off nodes for each request
//-----------------------------------------------------------------------------
TravelTime::TravelTime(){}
TravelTime::~TravelTime() {}

void TravelTime::setNodeIdToInt(const std::map<std::string, int> &nodeIdToInt) {
    nodeIDToInt_ = nodeIdToInt;
}

/*void TravelTime::setDurationMat(PGraph &graph) {
    durationValues_.resize(graph->nbNodes_);
    for (int i = 0; i < graph->nbNodes_; ++i)
        durationValues_[i].resize(graph->nbNodes_);

    std::string startUrl = "http://206.12.92.28/ny/table/v1/driving/";
    for (int i = 0; i < graph->nbNodes_; ++i) {
        startUrl += std::to_string(graph->nodes_[graph->intToNodeID_[i]]->locLongitude_)
                    + "," + std::to_string(graph->nodes_[graph->intToNodeID_[i]]->locLatitude_);
        if (i != graph->nbNodes_ -1)
            startUrl += ";";
    }

    vector<std::string> source;
    int Index = 0;
    int splitter = 100;

    // it is possible to just query the data for 100 locations from OSRM with table service
    int rounds = (graph->nbNodes_ / splitter);

    for (int j = 0; j < rounds; ++j) {
        Index = splitter * j;
        source.push_back("");
        for (int i = Index; i < Index+splitter; ++i) {
            source[j] += std::to_string(i);
            if (i != Index + splitter-1)
                source[j] += ";";
        }
    }

    source.push_back("");
    for (int i = (splitter * rounds); i < graph->nbNodes_; ++i) {
        source.back() += std::to_string(i);
        if (i != graph->nbNodes_ - 1)
            source.back() += ";";
    }

    for (int s = 0; s < source.size(); ++s) {
        for (int d = 0; d < source.size(); ++d) {
            std::string url = startUrl + "?sources=" + source[s] + "&destinations=" + source[d] + "&generate_hints=false";
            std::string content = Tools::queryHTTPData(url);
            Json::Value jsonData;
            Json::CharReaderBuilder builder;
            JSONCPP_STRING errs;

            const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
            // parse the results and return duration time
            bool parsingSuccessful = reader->parse(content.c_str(), content.c_str() + content.size(), &jsonData,
                                                   &errs);
            if (parsingSuccessful) {
                // check the route code
                if (jsonData["code"] == "Ok") {
                    for (int i = 0; i < jsonData["durations"].size(); ++i) {
                        for (int j = 0; j < jsonData["durations"][i].size(); ++j) {
                            durationValues_[(s*splitter) + i][(d*splitter) + j] = jsonData["durations"][i][j].asDouble();
                        }
                    }
                }
                else if (jsonData["code"] == "TooBig")
                    std::cout << "Too many coordinates" << std::endl;
                else if (jsonData["code"] == "NoTable")
                    std::cout << "No route found." << std::endl;
                else if (jsonData["code"] == "NotImplemented")
                    std::cout << "This request is not supported." << std::endl;
            }
        }

    }
}*/

float TravelTime::queryTravelTime(PNode startNode, PNode endNode) {
    float travelTime = durationValues_[nodeIDToInt_[startNode->nodeID_]][nodeIDToInt_[endNode->nodeID_]];
    return travelTime;
}




