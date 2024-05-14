//
// Created by Elahe Amiri on 2024-03-04.
//

#include "Stop.h"

Stop::Stop(float arrivalTime, float departTime, int nbToTransfer, int bikeLoad, int locationId)
        : arrivalTime_(arrivalTime), departTime_(departTime), nbToTransfer_(nbToTransfer), bikeLoad_(bikeLoad),
          locationID_(locationId) {
}

std::string Stop::toString() const {
    std::stringstream repStr;
    repStr << "# STOP (Location ID: " << locationID_ << ")" << std::endl;
    repStr << "#\tArrival Time: " << arrivalTime_ << std::endl;
    repStr << "#\tDeparture Time: " << departTime_ << std::endl;
    repStr << "#\tNumber of Bikes to Transfer: " << nbToTransfer_ << std::endl;
    repStr << "#\tBike load in the truck: " << bikeLoad_ << std::endl;
    return repStr.str();
}
