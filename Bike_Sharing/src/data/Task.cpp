//
// Created by Ella on 2021-09-13.
//

#include "Task.h"


//---------------------------------------------------------------------------------------------
//  The ``Task`` class mostly serves as a structure for storing basic
//  information about the re-balancing tasks.
//---------------------------------------------------------------------------------------------

// Constructor and Destructor
Task::Task(int stationId, const string &locationId, int locationIndex, int nbTransfer, double relocateBonus)
        : stationID_(stationId), locationID_(locationId), locationIndex_(locationIndex), nbTransfer_(nbTransfer),
          relocateBonus_(relocateBonus) {
    dual_ = 0;
    assignedVehicleID_ = -1;
    performTime_ = LARGE_CONSTANT;
    reachTime_ = LARGE_CONSTANT;
    taskID_ = locationIndex;
}


// Getters and Setters
unsigned int Task::getTaskId() const {return taskID_;}

// Display function
std::string Task::toString() const {
    std::stringstream repStr;
    repStr << std::left;
    repStr << "# TASK ( " << taskID_ << " )" << std::endl;
    repStr << "-----------------------------------" << std::endl;

    // Task Details
    repStr << "#\t" << std::setw(24) << "Station ID: " << stationID_ << std::endl;
    repStr << "#\t" << std::setw(24) << "Location ID: " << locationID_ << std::endl;
    repStr << "#\t" << std::setw(24) << "Location Index: " << locationIndex_ << std::endl;
    repStr << "#\t" << std::setw(24) << "Number of Transfers: " << nbTransfer_ << std::endl;

    repStr << "#\t" << std::setw(24) << "Reach Time: " << reachTime_ << std::endl;
    repStr << "#\t" << std::setw(24) << "Perform Time: " << performTime_ << std::endl;
    repStr << "#\t" << std::setw(24) << "RelocateBonus: " << relocateBonus_ << std::endl;
    repStr << "#\t" << std::setw(24) << "Assigned Vehicle ID: " << assignedVehicleID_ << std::endl;

    repStr << "#" << std::endl;
    return repStr.str();
}


rapidjson::Value Task::toJson(rapidjson::Document::AllocatorType& allocator) const {
    rapidjson::Value value(rapidjson::kObjectType);

    value.AddMember("task_id", taskID_, allocator);
    value.AddMember("station_id", stationID_, allocator);
    value.AddMember("location_id", rapidjson::StringRef(locationID_.c_str()), allocator);
    value.AddMember("location_index", locationIndex_, allocator);
    value.AddMember("nb_transfer", nbTransfer_, allocator);
    value.AddMember("relocate_bonus", relocateBonus_, allocator);
    //   value.AddMember("assignedVehicleID_", assignedVehicleID_, allocator);

    /*// Print the JSON string to the console
    rapidjson::StringBuffer buffer;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
    value.Accept(writer);

    std::cout << buffer.GetString() << std::endl;*/

    return value;
}










