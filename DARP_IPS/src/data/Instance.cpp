//
// Created by Ella on 2021-09-12.
//

#include "Instance.h"

//-----------------------------------------------------------------------------
//  Instance class
//  contains the instance data including vehicle info and requests
//-----------------------------------------------------------------------------

// Constructor and Destructor
Instance::Instance(std::string &name, int nbVehicles, std::vector<PVehicle> &vehicles, int nbRequests) :
                   name_(name), nbVehicles_(nbVehicles), vehicles_(vehicles), nbRequests_(nbRequests) {}

Instance::~Instance() {}
