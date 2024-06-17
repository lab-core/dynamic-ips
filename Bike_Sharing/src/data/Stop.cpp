//
// Created by Elahe Amiri on 2024-03-04.
//

#include "Stop.h"

Stop::Stop(float arrivalTime, float departTime, int nbToTransfer, int bikeLoad, int locationIndex)
        : arrivalTime_(arrivalTime), departTime_(departTime), nbToTransfer_(nbToTransfer), bikeLoad_(bikeLoad),
          locationIndex_(locationIndex) {
}

std::string Stop::toString() const {
    std::stringstream repStr;
    repStr << "# STOP (Location Index: " << locationIndex_ << ")" << std::endl;
    repStr << "#\tArrival Time: " << arrivalTime_ << std::endl;
    repStr << "#\tDeparture Time: " << departTime_ << std::endl;
    repStr << "#\tNumber of Bikes to Transfer: " << nbToTransfer_ << std::endl;
    repStr << "#\tBike load in the truck: " << bikeLoad_ << std::endl;
    return repStr.str();
}

rapidjson::Value Stop::toJson() const {
    rapidjson::Document document;
    document.SetObject();
    rapidjson::Document::AllocatorType& allocator = document.GetAllocator();

    rapidjson::Value value;
    value.SetObject();

    // Convert locationIndex_ to rapidjson::Value
    value.AddMember("location_index_", rapidjson::Value().SetInt(locationIndex_), allocator);

    // Convert arrivalTime_ to rapidjson::Value
    value.AddMember("arrival_time", rapidjson::Value().SetFloat(arrivalTime_), allocator);

    // Convert departTime_ to rapidjson::Value
    value.AddMember("depart_time", rapidjson::Value().SetFloat(departTime_), allocator);

    // Convert nbToTransfer_ to rapidjson::Value
    value.AddMember("nb_transfer_", rapidjson::Value().SetInt(nbToTransfer_), allocator);

    // Convert bikeLoad_ to rapidjson::Value
    value.AddMember("bike_load_", rapidjson::Value().SetInt(bikeLoad_), allocator);

    return value;

    return value;
}