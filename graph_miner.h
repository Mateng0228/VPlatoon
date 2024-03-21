#ifndef BASELINE_GRAPH_MINER_H
#define BASELINE_GRAPH_MINER_H

#include "Utils/structs.h"

class GraphMiner{
    struct ObjectInfo{
        int object_id;
        int pid; // position in *travel path or simplified path*
        double begin_time, end_time;
        ObjectInfo(int oid, int pid, double begin_time, double end_time){
            this->object_id = oid;
            this->pid = pid;
            this->begin_time = begin_time; this->end_time = end_time;
        }
        bool operator<(const ObjectInfo &other) const {
            return object_id < other.object_id || (object_id == other.object_id && pid > other.pid);
        }
        bool operator==(const ObjectInfo &other) const {return object_id == other.object_id && pid == other.pid;}
        bool operator!=(const ObjectInfo &other) const {return !(*this == other);}
        bool operator<=(const ObjectInfo &other) const {return (*this < other) || (*this == other);}
    };
    struct Cluster{
        int cluster_id;
        tuple<ll, int, int> key; // <camera, min_order, max_order>
        vector<ObjectInfo> members;
        int order = -1; // current cluster's topological order in cluster graph
        bool skip = false;

        Cluster(int cluster_id, tuple<ll, int, int> &key):cluster_id(cluster_id),key(key){}
        ll get_camera(){return get<0>(key);}
    };
    typedef map<ObjectInfo, set<ObjectInfo>> ObjectMap;
    struct ConditionalCluster{
        ObjectMap members;
        tuple<ll, int, int> key;// <camera, min_order, max_order>
        int n_ends;// number of distinct end ObjectInfo
        ConditionalCluster(ObjectMap &mp, tuple<ll, int, int> &key, int n_ends):members(mp),key(key),n_ends(n_ends){}
    };
    class Kosaraju;
    typedef vector<int> ClusterIds;
    typedef vector<pair<double, double>> Intervals;

    int m, k, d;
    double eps;
    vector<Path> &travel_paths;
    vector<vector<pair<int, int>>> simplified_paths; //pair(pid, current position's order in corresponding camera)
    vector<Cluster> clusters;
    unordered_map<tuple<ll, int, int>, int, Hasher> key2cid;// map the cluster's key to its cluster_id

    void temporal_cluster(vector<ObjectInfo> &infos, vector<pair<int, int>> &cluster_ranges);
    void local_cluster(ll camera, vector<pair<ObjectInfo, ObjectInfo>> &infos, vector<ConditionalCluster> &cdt_clusters);
    void init_clusters(vector<vector<pair<int, ClusterIds>>> &cluster_paths);
    void compute_order();
    void compute_intervals(map<int, Intervals> &, map<int, Intervals>::iterator, pair<double, double> &, Intervals &);
    void expand(vector<ll> &camera_route, ObjectMap &obj_map);
    bool check_m(vector<ObjectInfo> &infos, int left, int right);
    bool check_m(vector<pair<ObjectInfo, ObjectInfo>> &infos, int left, int right);
public:
    map<vector<int>, map<vector<ll>, Intervals>> results;// <objs: <camera_path: time_intervals>>

    GraphMiner(vector<Path> &paths, int m, int k, int d, double eps);
    void run();
};

#endif //BASELINE_GRAPH_MINER_H
