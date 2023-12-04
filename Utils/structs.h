#ifndef BASELINE_STRUCTS_H
#define BASELINE_STRUCTS_H

#include <string>
#include <utility>
#include <vector>

using namespace std;

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
    string object_name;
    vector<Position> positions;
    Path(int object_id, string object_name, vector<Position> positions){
        this->object_id = object_id;
        this->object_name = std::move(object_name);
        this->positions = std::move(positions);
    }
};
// meta-temporal-cluster extended path
struct TCS_Path : Path{
    vector<int> tcs_ids;
    vector<int> offsets;
    TCS_Path(int oid, string object_name): Path(oid, std::move(object_name), vector<Position>()){
    }
};

#endif //BASELINE_STRUCTS_H
