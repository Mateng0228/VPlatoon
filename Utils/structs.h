#ifndef BASELINE_STRUCTS_H
#define BASELINE_STRUCTS_H

#include <string>
#include <vector>

// Basic position units in Travel-Path
struct Position{
    int camera_id;
    double begin_time, end_time;
    Position(int camera_id, double begin_time, double end_time){
        this->camera_id = camera_id;
        this->begin_time = begin_time; this->end_time = end_time;
    }
};

// Travel-Path of each object
struct Path{
    int object_id;
    std::string object_name;
    std::vector<Position> positions;
    Path(int object_id, std::string object_name, std::vector<Position> positions){
        this->object_id = object_id;
        this->object_name = std::move(object_name);
        this->positions = std::move(positions);
    }
};

#endif //BASELINE_STRUCTS_H
