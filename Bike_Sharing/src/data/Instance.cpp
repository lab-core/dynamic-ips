//
// Created by Ella on 2021-09-12.
//

#include "Instance.h"
vector2D<float> durationMatrix_;
//-----------------------------------------------------------------------------
//  Instance class
//  contains the instance data including vehicle info and requests
//-----------------------------------------------------------------------------
// Constructor and Destructor
Instance::Instance(std::string &name, int nbVehicles, std::vector<PVehicle> &vehicles, int nbTasks,
                   int nbLocations) : name_(name), nbVehicles_(nbVehicles),
                   vehicles_(vehicles), nbTasks_(nbTasks), nbLocations_(nbLocations){
    nbTasks_ = nbTasks;
    std::cout << "Instance created!"<< std::endl;
    nbTasks_ = nbTasks;
    instGraph_ = std::make_shared<Graph>();
}

Instance::Instance(std::string &name, int nbLocations) : name_(name), nbLocations_(nbLocations){
    nbTasks_ = 0;
    nbVehicles_ = 0;
    instGraph_ = std::make_shared<Graph>();
}


Instance::Instance(const Instance &mainInst) : name_(mainInst.name_){
    nbVehicles_ = mainInst.nbVehicles_;
    vehicles_ = mainInst.vehicles_;
    parameters_ = mainInst.parameters_;
    nbTasks_ = 0;
    nbLocations_ = mainInst.nbLocations_;
    tasks_.reserve(mainInst.tasks_.size());
    nbTasks_ = 0;
}


void Instance::resetInstance() {
    nbTasks_ = 0;
    tasks_.clear();
    nbTasks_ = 0;
}


// Display function
std::string Instance::toString() {
    std::stringstream repStr;
    repStr << std::endl;
    repStr << "***************************************************************************" << std::endl;
    repStr << "**                             Instance Info                             **" << std::endl;
    repStr << "***************************************************************************" << std::endl;
    repStr << "# " << std::endl;
    repStr << "# INSTANCE_NAME       \t= " << name_ << std::endl;
    repStr << "# NUMBER_OF_VEHICLES  \t= " << nbVehicles_ <<std::endl;
    repStr << "# NUMBER_OF_REQUESTS  \t= " << nbTasks_ << std::endl;
    repStr << "# " << std::endl;

    repStr << "------------------------ PARAMETERS AND OPTIONS -------------------------" << std::endl;
    repStr << parameters_->toString();
    repStr << "# " << std::endl;
    return repStr.str();
}































