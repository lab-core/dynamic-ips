//
// Created by Elahe Amiri on 2024-03-04.
//

#ifndef DARP_IPS_STOP_H
#define DARP_IPS_STOP_H

#include "utilities/MyTools.h"
//-----------------------------------------------------------------------------
//  stop class
//  The ``Stop`` class mostly serves as a structure for storing basic
//        information about the stops during the vehicle routes
//  A stop is located somewhere along the network. Tasks arrive at the stop.
//-----------------------------------------------------------------------------

class Stop {
public:
    int locationIndex_;            // location of the stop point
    float arrivalTime_;         // time that vehicle is arrives at the stop
    float departTime_;          // time that vehicle departs from the stop
    int nbToTransfer_;          // number of bikes to pick up from the corresponding station
    int bikeLoad_;         // number of bikes picked from the corresponding station


    // Constructor and Destructor
    Stop(float arrivalTime, float departTime, int nbToTransfer, int bikeLoad, int locationIndex);

    // Display function
    std::string toString() const;

    // Conversion function to JSON
    rapidjson::Value toJson() const;
};


#endif //DARP_IPS_STOP_H
