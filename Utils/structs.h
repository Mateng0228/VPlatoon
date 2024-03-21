#ifndef BASELINE_STRUCTS_H
#define BASELINE_STRUCTS_H

#include <string>
#include <vector>

using namespace std;
typedef long long ll;

// Basic position units in Travel-Path
struct Position{
    ll camera_id;
    double begin_time, end_time;
    Position(ll camera_id, double begin_time, double end_time){
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

class Hasher{
public:
    size_t operator()(const tuple<long long, int, int> &key) const {
        return fast_hash(key);
    }
private:
    // based on Thomas Wang algorithm
    static size_t fast_hash(const tuple<long long, int, int> &key) {
        long long a = get<0>(key);
        int b = get<1>(key);
        int c = get<2>(key);

        return static_cast<size_t>(a) * 2654435761U +
               static_cast<size_t>(b) * 2654435789U +
               static_cast<size_t>(c) * 2654435809U;
    }
};

#endif //BASELINE_STRUCTS_H
