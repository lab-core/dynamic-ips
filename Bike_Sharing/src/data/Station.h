//
// Created by Elahe Amiri on 2024-03-08.
//

#ifndef DARP_IPS_STATION_H
#define DARP_IPS_STATION_H

#include "utilities/MyTools.h"


class Station {
public:
    int stationID_;                         // station ID
    std::string shortName_;                 // short name of the station
    std::string name_;                      // station name
    int locationID_;                        // location id of the station
    int capacity_;                          // vehicle capacity
    int sector_;                            // sector where station is located
    PTask currentTask_;                     // current task in at the station

    // Constructor and Destructor
    Station(int stationId, const string &shortName, const string &name, int locationId, int capacity, int sector);
};


#endif //DARP_IPS_STATION_H
