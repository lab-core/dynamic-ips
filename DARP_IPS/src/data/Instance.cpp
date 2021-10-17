//
// Created by Ella on 2021-09-12.
//

#include "Instance.h"

//-----------------------------------------------------------------------------
//  Instance class
//  contains the instance data including vehicle info and requests
//-----------------------------------------------------------------------------

// Constructor and Destructor
Instance::Instance(std::string &name, int nbVehicles, std::vector<PVehicle> &vehicles, int nbRequests,
                   PGraph &mainGraph) : name_(name), nbVehicles_(nbVehicles), vehicles_(vehicles),
                   nbRequests_(nbRequests), mainGraph_(mainGraph) {
    std::cout << "Instance created!"<< std::endl;
}

Instance::~Instance() {}

// Display function
std::string Instance::toString() {
    std::stringstream repStr;
    repStr << std::endl;
    repStr << "***************************************************************************" << std::endl;
    repStr << "***************************    Instance Info    ***************************" << std::endl;
    repStr << "***************************************************************************" << std::endl;
    repStr << "# " << std::endl;
    repStr << "# INSTANCE_NAME       \t= " << name_ << std::endl;
    repStr << "# NUMBER_OF_VEHICLES  \t= " << nbVehicles_ <<std::endl;
    repStr << "# NUMBER_OF_REQUESTS  \t= " << nbRequests_ <<std::endl;
    repStr << "# " << std::endl;

    // display 3 requests information
    repStr << "# REQUESTS_INFO (3 records): ";
    repStr << "------------------------" << std::endl;
    for (int i = 0; i < 3; ++i) {
        repStr << requests_[i]->toString() << std::endl;
    }
    repStr << "# " << std::endl;
    repStr << "# VEHICLES_INFO (3 records): ";
    repStr << "------------------------" << std::endl;
    for (int i = 0; i < 3; ++i) {
        repStr << vehicles_[i]->toString() << std::endl;
    }

    return repStr.str();
}
