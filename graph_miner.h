#ifndef BASELINE_GRAPH_MINER_H
#define BASELINE_GRAPH_MINER_H

#include "Utils/structs.h"

class GraphMiner{
    struct ObjectInfo{
        int object_id;
        int pid; // position in *travel path or cluster path*
        double begin_time, end_time;
        ObjectInfo(int oid, int pid, double begin_time, double end_time){
            this->object_id = oid;
            this->pid = pid;
            this->begin_time = begin_time; this->end_time = end_time;
        }
        bool operator<(const ObjectInfo &other) const {
            return object_id < other.object_id || (object_id == other.object_id && pid > other.pid);
        }
    };
    struct Cluster{
        int cluster_id;
        ll camera;
        int order; // current cluster's order in cluster graph
        bool skip = false;
        vector<ObjectInfo> members;

        Cluster(int cluster_id, ll camera_id):cluster_id(cluster_id), camera(camera_id){}
    };
    struct ExpandTuple;
    class Kosaraju;
    typedef vector<int> ClusterIds;
    typedef map<ObjectInfo, set<ObjectInfo>> ObjectMap;
    typedef vector<pair<double, double>> Intervals;

    int m, k, d;
    double eps;
    vector<Path> &travel_paths;
    vector<Cluster> clusters;
    vector<vector<pair<int, ClusterIds>>> cluster_paths;// pair(position_id, cluster_ids)

    void temporal_cluster(vector<ObjectInfo> &infos, vector<pair<int, int>> &cluster_ranges) const;
    void init_clusters();
    void compute_order();
    void compute_intervals(map<int, Intervals> &, map<int, Intervals>::iterator, pair<double, double> &, Intervals &);
    void expand(vector<int> &cluster_route, ObjectMap &obj_map);
    bool is_subpath(const vector<ll> &short_path, const vector<ll> &long_path);
    void deduplicate_time(map<vector<ll>, Intervals> &cameras_map);
    void deduplicate_twin(map<vector<ll>, Intervals> &super_map, map<vector<ll>, Intervals> &child_map);
public:
    map<vector<int>, map<vector<ll>, Intervals>> results;// <objs: <camera_path: time_intervals>>

    GraphMiner(vector<Path> &paths, int m, int k, int d, double eps);
    void run();
    void deduplicate();
    void print_results();
    void dump_results(string file_name);
};

#endif //BASELINE_GRAPH_MINER_H
