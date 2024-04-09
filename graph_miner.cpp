#include <algorithm>
#include <map>
#include <set>
#include <stack>
#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include <fstream>
#include <random>
#include "graph_miner.h"

class GraphMiner::Kosaraju{
private:
    vector<vector<int>> &graph;
    int n_vertex;
    vector<bool> visited;
    stack<int> stk;

    void dfs_forward(int vid){
        visited[vid] = true;
        for(int to_vertex : graph[vid]){
            if(!visited[to_vertex]) dfs_forward(to_vertex);
        }
        stk.push(vid);
    }

    void dfs_reverse(vector<vector<int>> &graph_reverse, int vid, vector<int> &component){
        visited[vid] = true;
        component.push_back(vid);
        for(int to_vertex : graph_reverse[vid]){
            if(!visited[to_vertex]) dfs_reverse(graph_reverse, to_vertex, component);
        }
    }
public:
    explicit Kosaraju(vector<vector<int>> &graph): graph(graph){
        n_vertex = static_cast<int>(graph.size());
        visited = vector<bool>(n_vertex, false);
    }
    // return strongly connected components of topological order
    vector<vector<int>> get_scc(){
        //finding the sequence in the order of finishing time
        for(int vid = 0; vid < n_vertex; vid++){
            if(!visited[vid]) dfs_forward(vid);
        }
        //transposing $graph$
        vector<vector<int>> graph_reverse(n_vertex, vector<int>());
        for(int vid = 0; vid < n_vertex; vid++){
            for(int to_vertex : graph[vid]) graph_reverse[to_vertex].push_back(vid);
        }
        // dfs in $graph_reverse$ to get all scc
        vector<vector<int>> components;
        fill(visited.begin(), visited.end(), false);
        while(!stk.empty()){
            int vid = stk.top();
            stk.pop();
            if(!visited[vid]){
                components.emplace_back();
                vector<int> &component = components.back();
                dfs_reverse(graph_reverse, vid, component);
            }
        }
        return components;
    }
};


GraphMiner::GraphMiner(vector<Path> &paths, int m, int k, int d, double eps) : travel_paths(paths) {
    this->m = m;
    this->k = k;
    this->d = d;
    this->eps = eps;
}

void GraphMiner::temporal_cluster(vector<ObjectInfo> &infos, vector<pair<int, int>> &cluster_ranges){
    if(infos.empty()) return;
    // sort object's position (represented by ObjectInfo)
    sort(infos.begin(), infos.end(),[](const ObjectInfo& info1, const ObjectInfo& info2){
        if(info1.begin_time == info2.begin_time) return info1.end_time < info2.end_time;
        return info1.begin_time < info2.begin_time;
    });
    // cluster
    int left = 0, right = 0;
    double border = infos.front().begin_time + eps;
    while(right != infos.size() - 1){
        int next_idx = right + 1;
        double next_time = infos[next_idx].begin_time;
        if(next_time <= border) right++;
        else{
            if(check_m(infos, left, right)) cluster_ranges.emplace_back(left, right);
            left++;
            while(left != next_idx){
                double temp_border = infos[left].begin_time + eps;
                if(next_time <= temp_border) break;
                else left++;
            }
            border = infos[left].begin_time + eps;
            right++;
        }
    }
    if(check_m(infos, left, right)) cluster_ranges.emplace_back(left, right);
}

void GraphMiner::local_cluster(ll camera, vector<pair<ObjectInfo, ObjectInfo>> &infos, vector<ConditionalCluster> &cdt_clusters){
    if(infos.empty()) return;
    // sort current object's position
    sort(infos.begin(), infos.end(), [](const pair<ObjectInfo, ObjectInfo> &p1, const pair<ObjectInfo, ObjectInfo> &p2){
        const ObjectInfo &info1 = p1.second, &info2 = p2.second;
        if(info1.begin_time == info2.begin_time) return info1.end_time < info2.end_time;
        return info1.begin_time < info2.begin_time;
    });
    // cluster
    double border = infos.front().second.begin_time + eps;
    int left = 0, right = 0, n_distinct = 1;
    while(right != infos.size() - 1){
        int next_idx = right + 1;
        double next_time = infos[next_idx].second.begin_time;
        if(next_time <= border){
            if(infos[next_idx].second != infos[right].second) n_distinct += 1;
            right++;
        }
        else{
            if(check_m(infos, left, right)){
                // construct corresponding conditional cluster
                ObjectMap mp;
                for(int idx = left; idx <= right; ++idx){
                    ObjectInfo &begin_info = infos[idx].first, &end_info = infos[idx].second;
                    auto mp_itr = mp.find(begin_info);
                    if(mp_itr == mp.end()) mp[begin_info] = {end_info};
                    else mp_itr->second.insert(end_info);
                }
                ObjectInfo &min_info = infos[left].second, &max_info = infos[right].second;
                int min_order = simplified_paths[min_info.object_id][min_info.pid].second;
                int max_order = simplified_paths[max_info.object_id][max_info.pid].second;
                tuple<ll, int, int> key(camera, min_order, max_order);
                cdt_clusters.emplace_back(mp, key, n_distinct);
            }
            ++right; ++n_distinct;
            if(infos[left].second != infos[left + 1].second) --n_distinct;
            while(++left != next_idx){
                double temp_border = infos[left].second.begin_time + eps;
                if(next_time <= temp_border) break;
                if(infos[left].second != infos[left + 1].second) --n_distinct;
            }
            border = infos[left].second.begin_time + eps;
        }
    }
    if(check_m(infos, left, right)){
        // construct corresponding conditional cluster
        ObjectMap mp;
        for(int idx = left; idx <= right; ++idx){
            ObjectInfo &begin_info = infos[idx].first, &end_info = infos[idx].second;
            auto mp_itr = mp.find(begin_info);
            if(mp_itr == mp.end()) mp[begin_info] = {end_info};
            else mp_itr->second.insert(end_info);
        }
        ObjectInfo &min_info = infos[left].second, &max_info = infos[right].second;
        int min_order = simplified_paths[min_info.object_id][min_info.pid].second;
        int max_order = simplified_paths[max_info.object_id][max_info.pid].second;
        tuple<ll, int, int> key(camera, min_order, max_order);
        cdt_clusters.emplace_back(mp, key, n_distinct);
    }
}

void GraphMiner::init_clusters(vector<vector<pair<int, ClusterIds>>> &cluster_paths){
    // do temporal clustering for each camera
    map<ll, vector<ObjectInfo>> camera2objects;
    for(int oid = 0; oid < travel_paths.size(); oid++){
        Path &path = travel_paths[oid];
        for(int pid = 0; pid < path.positions.size(); pid++){
            Position &p = path.positions[pid];
            ll camera = p.camera_id;
            auto it = camera2objects.find(camera);
            if(it == camera2objects.end())
                camera2objects[camera] = vector<ObjectInfo>{ObjectInfo(oid, pid, p.begin_time, p.end_time)};
            else
                it->second.emplace_back(oid, pid, p.begin_time, p.end_time);
        }
    }
    // get complete $simplified_paths$ and local variable $cluster_paths$
    for(Path &path : travel_paths){
        simplified_paths.emplace_back(path.positions.size(), make_pair(-1, -1));
        cluster_paths.emplace_back(path.positions.size(), make_pair(-1, ClusterIds()));
    }
    int n_clusters = 0;
    for(auto &camera_entry : camera2objects){
        ll camera = camera_entry.first;
        vector<ObjectInfo> &infos = camera_entry.second;
        vector<pair<int, int>> cluster_ranges;
        temporal_cluster(infos, cluster_ranges);
        // fill up $simplified_paths$ and part of $cluster_paths$
        for(int info_order = 0; info_order < infos.size(); ++info_order){
            ObjectInfo &info = infos[info_order];

            pair<int, int> &p = simplified_paths[info.object_id][info.pid];
            p.first = info.pid;
            p.second = info_order;

            cluster_paths[info.object_id][info.pid].first = info.pid;
        }
        // fill up cluster-related data structure
        for(pair<int, int> &cluster_range : cluster_ranges){
            int min_order = cluster_range.first, max_order = cluster_range.second;
            tuple<ll, int, int> key(camera, min_order, max_order);
            clusters.emplace_back(n_clusters, key);
            key2cid[key] = n_clusters;

            for(int info_idx = min_order; info_idx <= max_order; ++info_idx){
                ObjectInfo &info = infos[info_idx];
                cluster_paths[info.object_id][info.pid].second.push_back(n_clusters);
            }
            n_clusters++;
        }
    }
    // simplify $simplified_paths$ and $cluster_paths$
    for(int path_idx = 0; path_idx < simplified_paths.size(); ++path_idx){
        vector<pair<int, int>> &sp_path = simplified_paths[path_idx];
        vector<pair<int, ClusterIds>> &cluster_path = cluster_paths[path_idx];

        int counts = 0;
        for(int pid = 0; pid < sp_path.size(); ++pid){
            ClusterIds &cids = cluster_path[pid].second;
            if(cids.empty()) continue;
            if(counts != pid){
                sp_path[counts] = sp_path[pid];
                cluster_path[counts] = cluster_path[pid];
            }
            ++counts;
        }

        sp_path.resize(counts);
        cluster_path.resize(counts);
    }
    // fill the parameter $member$ in each cluster
    for(int oid = 0; oid < cluster_paths.size(); ++oid){
        vector<Position> &travel_path = travel_paths[oid].positions;
        vector<pair<int, ClusterIds>> &cluster_path = cluster_paths[oid];
        for(int c_pid = 0; c_pid < cluster_path.size(); ++c_pid){
            int travel_pid = cluster_path[c_pid].first;
            Position &position = travel_path[travel_pid];
            for(int cluster_id : cluster_path[c_pid].second)
                clusters[cluster_id].members.emplace_back(oid, c_pid, position.begin_time, position.end_time);
        }
    }
}

void GraphMiner::compute_order(){
    // initialize cluster-related data structures
    vector<vector<pair<int, ClusterIds>>> cluster_paths;
    init_clusters(cluster_paths);
    // construct cluster graph according to cluster paths
    vector<set<int>> adj_sets(clusters.size(), set<int>());
    for(vector<pair<int, ClusterIds>> &cluster_path : cluster_paths){
        int length = static_cast<int>(cluster_path.size());
        for(int pid = 0; pid < length - 1; pid++){
            pair<int, ClusterIds> &crt_entry = cluster_path[pid], &nxt_entry = cluster_path[pid + 1];
            int interval = nxt_entry.first - crt_entry.first - 1;
            if(interval > d) continue;
            for(int source : crt_entry.second){
                set<int> &adj_set = adj_sets[source];
                for(int target : nxt_entry.second) adj_set.insert(target);
            }
        }
    }
    vector<vector<int>> graph;
    for(set<int> &adj_set : adj_sets){
        graph.emplace_back(adj_set.begin(), adj_set.end());
    }
    // compute the topological order of scc
    Kosaraju scc_finder(graph);
    vector<vector<int>> components = scc_finder.get_scc();
    for(int order = 0; order < components.size(); order++){
        vector<int> &cluster_ids = components[order];
        for(int cluster_id : cluster_ids){
            clusters[cluster_id].order = order;
        }
    }
}

void GraphMiner::compute_intervals(map<int, Intervals> &o2Is, map<int, Intervals>::iterator it, pair<double, double> &I, Intervals &Is) {
    if(it == o2Is.end()){
        Is.push_back(I);
        return;
    }
    if(it == o2Is.begin()){
        for(pair<double, double> &local_interval : it->second){
            I.first = local_interval.first;
            I.second = local_interval.second;
            compute_intervals(o2Is, next(it), I, Is);
        }
    }
    else{
        pair<double, double> stub_I(I);
        for(pair<double, double> &local_interval : it->second){
            I.first = min(I.first, local_interval.first);
            I.second = max(I.second, local_interval.second);
            compute_intervals(o2Is, next(it), I, Is);
            I.first = stub_I.first;
            I.second = stub_I.second;
        }
    }
}

void GraphMiner::expand(vector<ll> &camera_route, ObjectMap &obj_map){
    map<ll, vector<pair<ObjectInfo, ObjectInfo>>> camera2infos;
    for(auto &obj_entry : obj_map){
        const ObjectInfo &begin_info = obj_entry.first;
        set<ObjectInfo> &end_infos = obj_entry.second;

        int oid = begin_info.object_id;
        auto &travel_path = travel_paths[oid].positions;
        auto &sp_path = simplified_paths[oid];
        int border = static_cast<int>(sp_path.size()) - 1;
        for(const ObjectInfo &end_info : end_infos){
            int base_spid = end_info.pid, base_tpid = sp_path[base_spid].first;
            for(int nxt_spid = base_spid + 1; nxt_spid <= border; ++nxt_spid){
                int nxt_tpid = sp_path[nxt_spid].first;
                if(nxt_tpid - base_tpid - 1 > d) break;

                ll nxt_camera = travel_path[nxt_tpid].camera_id;
                ObjectInfo nxt_info(oid, nxt_spid, travel_path[nxt_tpid].begin_time, travel_path[nxt_tpid].end_time);
                auto itr = camera2infos.find(nxt_camera);
                if(itr == camera2infos.end()) camera2infos[nxt_camera] = {{begin_info, nxt_info}};
                else itr->second.emplace_back(begin_info, nxt_info);
            }
            border = base_spid;
        }
    }

    vector<ConditionalCluster> cdt_clusters;
    unordered_map<int, map<int, ClusterIds>> cluster_maps; // { oid : {tpid:cluster_ids} }
    unordered_map<int, vector<pair<int, ClusterIds>>> cluster_paths; // { oid : [(tpid,cluster_ids)] }
    unordered_map<int, vector<pair<int, int>>> cluster2positions; // { cluster_id : [(oid, pid in $cluster_paths$)] }
    int cluster_idx = 0;
    for(auto &camera_entry : camera2infos){
        ll camera = camera_entry.first;
        vector<pair<ObjectInfo, ObjectInfo>> &infos = camera_entry.second;
        if(infos.size() < m) continue;

        local_cluster(camera, infos, cdt_clusters);
        while(cluster_idx < cdt_clusters.size()){
            ObjectMap &member_map = cdt_clusters[cluster_idx].members;
            for(auto &member_entry : member_map){
                int oid = member_entry.first.object_id;
                auto &sp_path = simplified_paths[oid];
                map<int, ClusterIds> &cluster_map = cluster_maps[oid];
                for(const ObjectInfo &end_info : member_entry.second){
                    int tpid = sp_path[end_info.pid].first;
                    cluster_map[tpid].push_back(cluster_idx);
                }
            }
            ++cluster_idx;
        }
    }
    for(auto &map_entry : cluster_maps){
        int oid = map_entry.first;
        vector<pair<int, ClusterIds>> &cluster_path = cluster_paths[oid];
        for(auto &entry : map_entry.second){
            ClusterIds &cluster_ids = entry.second;
            sort(cluster_ids.begin(), cluster_ids.begin());
            auto it = unique(cluster_ids.begin(), cluster_ids.end());
            cluster_ids.erase(it, cluster_ids.end());

            cluster_path.emplace_back(entry.first, cluster_ids);
        }
    }
    for(auto &path_entry : cluster_paths){
        int oid = path_entry.first;
        vector<pair<int, ClusterIds>> &cluster_path = path_entry.second;
        for(int pid = 0; pid < cluster_path.size(); ++pid){
            ClusterIds &cluster_ids = cluster_path[pid].second;
            for(int cluster_id : cluster_ids) cluster2positions[cluster_id].emplace_back(oid, pid);
        }
    }

    vector<int> valid_clusters;
    for(int cid = 0; cid < cdt_clusters.size(); ++cid){
        unordered_set<int> common_clusters;
        vector<pair<int, int>> &positions = cluster2positions[cid];
        for(auto it = positions.begin(); it != positions.end(); ++it){
            vector<pair<int, ClusterIds>> &cluster_path = cluster_paths[it->first];
            int pid = it->second, tpid = cluster_path[pid].first;
            if(it == positions.begin()){
                for(int prev_pid = pid - 1; prev_pid >= 0 && tpid - cluster_path[prev_pid].first - 1 <= d; --prev_pid){
                    ClusterIds &prev_clusters = cluster_path[prev_pid].second;
                    for(int prev_cluster : prev_clusters)
                        if(prev_cluster != cid) common_clusters.insert(prev_cluster);
                }
            }
            else{
                unordered_set<int> intersection;
                for(int prev_pid = pid - 1; prev_pid >= 0 && tpid - cluster_path[prev_pid].first - 1 <= d; --prev_pid){
                    ClusterIds &prev_clusters = cluster_path[prev_pid].second;
                    for(int prev_cluster : prev_clusters){
                        if(prev_cluster != cid && common_clusters.find(prev_cluster) != common_clusters.end())
                            intersection.insert(prev_cluster);
                    }
                }
                common_clusters = intersection;
            }
            if(common_clusters.empty()) break;
        }
        if(common_clusters.empty()) valid_clusters.push_back(cid);
    }

//    vector<int> valid_clusters;
//    for(int cid = 0; cid < cdt_clusters.size(); ++cid) valid_clusters.push_back(cid);

    bool is_dominated = false; // whether the current pattern is dominated
    for(int cluster_id : valid_clusters){
        ConditionalCluster &cdt_cluster = cdt_clusters[cluster_id];
        auto &key = cdt_cluster.key;
        auto itr = key2cid.find(key);
        if(itr != key2cid.end()){
            int n_member = get<2>(key) - get<1>(key) + 1;
            if(cdt_cluster.n_ends == n_member) clusters[itr->second].skip = true;
        }

        ObjectMap &nxt_map = cdt_cluster.members;
        if(!is_dominated && nxt_map.size() == obj_map.size()){
            bool flag = true;
            auto nxt_it = nxt_map.begin(), crt_it = obj_map.begin();
            while(nxt_it != nxt_map.end()){
                if(*(nxt_it->second.begin()) <= *(crt_it->second.begin())){
                    flag = false;
                    break;
                }
                ++nxt_it;
                ++crt_it;
            }
            if(flag) is_dominated = true;
        }

        ll camera = get<0>(key);
        camera_route.push_back(camera);
        expand(camera_route, nxt_map);
        camera_route.pop_back();
    }

    if(!is_dominated && camera_route.size() >= k){
        map<int, vector<pair<double, double>>> oid2intervals;
        for(auto &obj_entry : obj_map){
            const ObjectInfo &begin_obj = obj_entry.first, &end_obj = *(obj_entry.second.begin());
            int oid = begin_obj.object_id;
            auto it = oid2intervals.find(oid);
            if(it == oid2intervals.end()) oid2intervals[oid] = {{begin_obj.begin_time, end_obj.begin_time}};
            else it->second.emplace_back(begin_obj.begin_time, end_obj.begin_time);
        }
        // get oids and time_intervals
        vector<int> oids;
        for(auto &entry : oid2intervals) oids.push_back(entry.first);
        Intervals intervals;
        pair<double, double> tmp_interval(-1, -1);
        compute_intervals(oid2intervals, oid2intervals.begin(), tmp_interval, intervals);
        // insert into results
        auto object_it = results.find(oids);
        if(object_it == results.end()) results[oids] = {{camera_route, intervals}};
        else{
            auto &route_map = object_it->second;
            auto route_it = route_map.find(camera_route);
            if(route_it == route_map.end()) route_map[camera_route] = intervals;
            else route_it->second.insert(route_it->second.end(), intervals.begin(), intervals.end());
        }
    }
}

bool GraphMiner::check_m(vector<ObjectInfo> &infos, int left, int right){
    if(right - left + 1 < m) return false;
    vector<int> oids;
    for(int idx = left; idx <= right; ++idx) oids.push_back(infos[idx].object_id);
    sort(oids.begin(), oids.end());
    int counts = 1, pre_val = oids[0];
    for(int idx = 1; idx < oids.size(); ++idx){
        if(pre_val != oids[idx]){
            counts += 1;
            pre_val = oids[idx];
        }
    }
    return counts >= m;
}

bool GraphMiner::check_m(vector<pair<ObjectInfo, ObjectInfo>> &infos, int left, int right){
    if(right - left + 1 < m) return false;
    vector<int> oids;
    for(int idx = left; idx <= right; ++idx) oids.push_back(infos[idx].first.object_id);
    sort(oids.begin(), oids.end());
    int counts = 1, pre_val = oids[0];
    for(int idx = 1; idx < oids.size(); ++idx){
        if(pre_val != oids[idx]){
            counts += 1;
            pre_val = oids[idx];
        }
    }
    return counts >= m;
}

void GraphMiner::run(){
    // get all the clusters and their topological order in graph
    compute_order();
    ClusterIds cluster_ids(clusters.size());
    for(int cid = 0; cid < cluster_ids.size(); cid++) cluster_ids[cid] = cid;
    sort(cluster_ids.begin(), cluster_ids.end(), [this](const int &cid1, const int &cid2){
        return clusters[cid1].order < clusters[cid2].order;
    });

//    std::random_device rd;
//    std::mt19937 g(rd());
//    std::shuffle(cluster_ids.begin(), cluster_ids.end(), g);

    // expand each cluster according to the order in $cluster_ids$
    for(int cluster_id : cluster_ids){
        Cluster &cluster = clusters[cluster_id];
        if(cluster.skip) continue;

        vector<ll> camera_route{cluster.get_camera()};
        ObjectMap obj_map;
        for(ObjectInfo &member : cluster.members) obj_map[member] = {member};
        expand(camera_route, obj_map);
    }
}