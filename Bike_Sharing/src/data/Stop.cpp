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

    value.AddMember("location_index_", locationIndex_, allocator);
    value.AddMember("arrival_time", arrivalTime_, allocator);
    value.AddMember("depart_time", departTime_, allocator);
    value.AddMember("nb_transfer_", nbToTransfer_, allocator);
    value.AddMember("bike_load_", bikeLoad_, allocator);

    return value;
}