//
// Created by Ella on 2021-10-26.
//

#include "Route.h"


//---------------------------------------------------------------------------------------------
//  Route class
//  Route for each vehicle
//---------------------------------------------------------------------------------------------
unsigned int Route::routeCount_ = 0;

// Constructor and Destructor
Route::Route(int vehicleId) : routeID_(routeCount_++), vehicleID_(vehicleId) {
    totalBonus_ = 0.0;
    reducedCost_ = 0.0;
    totalDuration_ = 0.0;
    routeSize_ = 0;
    char* name2 = new char[255];
    strncpy(name2, std::to_string(routeID_).c_str(), 255);
    name_ = name2;
    mpAdded_ = false;
    score_ = 0;
}

Route::~Route(){
    delete[] name_;
}

// Getters and Setters
unsigned int Route::getRouteId() const {return routeID_;}

void Route::addSource(PNode &node, float departTime, int departLoad) {
    routeSize_ ++;
    if (node->type_ == DEPART_NODE) {
        routeStops_.emplace_back(std::make_shared<Stop>(departTime, departTime, 0,
                                                        departLoad, node->locationIndex_));
    }
}

void Route::addNode(PNode &node, vector2D<float>& durationMatrix) {
    routeSize_ ++;

    int load = routeStops_.back()->bikeLoad_+node->load_;
    float reachTime = routeStops_.back()->departTime_ + durationMatrix[routeStops_.back()->locationIndex_][node->locationIndex_];
    float departTime = reachTime + TimeToPark + abs(node->relatedTask_->nbTransfer_)*TimePerBike;
    routeStops_.emplace_back(std::make_shared<Stop>(reachTime, departTime, node->load_,
                                                    load, node->locationIndex_));
    assignedTasks_.push_back(node->relatedTask_);
    totalBonus_ += (node->relatedTask_->relocateBonus_);

}

void Route::addTask(PTask &task, vector2D<float>& durationMatrix) {
    routeSize_ ++;

    int load = routeStops_.back()->bikeLoad_+ task->nbTransfer_;
    float reachTime = routeStops_.back()->departTime_ + durationMatrix[routeStops_.back()->locationIndex_][task->locationIndex_];
    float departTime = reachTime + TimeToPark + abs(task->nbTransfer_)*TimePerBike;
    routeStops_.emplace_back(std::make_shared<Stop>(reachTime, departTime, task->nbTransfer_,
                                                    load, task->locationIndex_));
    assignedTasks_.push_back(task);
    totalBonus_ += (task->relocateBonus_);
}

rapidjson::Value Route::toJson(rapidjson::Document::AllocatorType& allocator) const {
    rapidjson::Value value(rapidjson::kObjectType);

    // Convert routeID_ to rapidjson::Value
    value.AddMember("route_id", rapidjson::Value().SetUint(routeID_), allocator);

    // Convert vehicleID_ to rapidjson::Value
    value.AddMember("vehicle_id", rapidjson::Value().SetInt(vehicleID_), allocator);

    // Convert totalBonus_ to rapidjson::Value
    value.AddMember("relocate_bonus", rapidjson::Value().SetFloat(totalBonus_), allocator);

    // Convert reducedCost_ to rapidjson::Value
    value.AddMember("reduced_cost", rapidjson::Value().SetDouble(reducedCost_), allocator);

    // Convert totalDuration_ to rapidjson::Value
    value.AddMember("duration", rapidjson::Value().SetDouble(totalDuration_), allocator);

    // Convert assignedTasks_ to rapidjson::Value array
    rapidjson::Value assignedTasksArray(rapidjson::kArrayType);
    for (const auto &task : assignedTasks_) {
        assignedTasksArray.PushBack(task->toJson(allocator), allocator);
    }
    value.AddMember("assigned_tasks", assignedTasksArray, allocator);

    /*// Print the JSON string to the console
    rapidjson::StringBuffer buffer;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
    value.Accept(writer);

    std::cout << buffer.GetString() << std::endl;*/

    return value;
}












