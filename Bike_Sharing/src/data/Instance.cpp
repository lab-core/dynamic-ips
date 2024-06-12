//
// Created by Ella on 2021-09-12.
//

#include "Instance.h"
//-----------------------------------------------------------------------------
//  Instance class
//  contains the instance data including vehicle info and requests
//-----------------------------------------------------------------------------
// Constructor and Destructor
Instance::Instance(std::string &name, int nbVehicles, std::vector<PVehicle> &vehicles, int nbTasks) : nbVehicles_(nbVehicles),
                   vehicles_(vehicles), nbTasks_(nbTasks){
    nbTasks_ = nbTasks;
    std::cout << "Instance created!"<< std::endl;
    nbTasks_ = nbTasks;
    instGraph_ = std::make_shared<Graph>();
}

Instance::Instance() {
    nbTasks_ = 0;
    nbVehicles_ = 0;
    instGraph_ = std::make_shared<Graph>();
}


Instance::Instance(const Instance &mainInst) {
    nbVehicles_ = mainInst.nbVehicles_;
    vehicles_ = mainInst.vehicles_;
    parameters_ = mainInst.parameters_;
    nbTasks_ = 0;
    tasks_.reserve(mainInst.tasks_.size());
    nbTasks_ = 0;
    durationMatrix_ = mainInst.durationMatrix_;
}



// Display function
std::string Instance::toString() {
    std::stringstream repStr;
    repStr << std::endl;
    repStr << "***************************************************************************" << std::endl;
    repStr << "**                             Instance Info                             **" << std::endl;
    repStr << "***************************************************************************" << std::endl;
    repStr << "# " << std::endl;
    repStr << "# NUMBER_OF_VEHICLES  \t= " << nbVehicles_ <<std::endl;
    repStr << "# NUMBER_OF_REQUESTS  \t= " << nbTasks_ << std::endl;
    repStr << "# " << std::endl;

    repStr << "------------------------ PARAMETERS AND OPTIONS -------------------------" << std::endl;
    repStr << parameters_->toString();
    repStr << "# " << std::endl;
    return repStr.str();
}

vector2D<float> &Instance::getDurationMatrix(){
    return durationMatrix_;
}

void Instance::setDurationMatrix(vector2D<float> &durationMatrix) {
    durationMatrix_ = durationMatrix;
}

rapidjson::Value Instance::createSolutionJson() const {

    // Create a document to use its allocator
    rapidjson::Document document;
    document.SetObject();
    rapidjson::Document::AllocatorType& allocator = document.GetAllocator();

    // Initialize json as an object
    rapidjson::Value json(rapidjson::kObjectType);

    for (const auto &vehicle : vehicles_) {
        if (vehicle == nullptr || vehicle->currentRoute_ == nullptr) {
            // Skip if vehicle or currentRoute_ is null
            continue;
        }

        // Convert vehicleID to a rapidjson::Value
        std::string vehicleIDStr = std::to_string(vehicle->vehicleID_);
        rapidjson::Value key(vehicleIDStr.c_str(), allocator);

        // Create the route value using the toJson method
        rapidjson::Value routeValue = vehicle->currentRoute_->toJson(allocator); // Assuming currentRoute_ is a pointer to Route

        // Add the key-value pair to the json object
        json.AddMember(key, routeValue, allocator);
    }

    return json;
}

const std::unordered_map<int, std::unordered_map<int, float>> &Instance::getDurationMatrixDict() const {
    return durationMatrixDict_;
}
































