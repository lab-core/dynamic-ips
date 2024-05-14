//
// Created by Elahe Amiri on 2024-03-08.
//

#include "Station.h"

Station::Station(int stationId, const string &shortName, const string &name, int locationId, int capacity, int sector)
        : stationID_(stationId), shortName_(shortName), name_(name), locationID_(locationId), capacity_(capacity),
          sector_(sector) {}
