#include <algorithm>
#include <map>
#include <set>
#include <stack>
#include <iostream>
#include <unordered_set>
#include <unordered_map>
#include <fstream>
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

void GraphMiner::temporal_cluster(vector<ObjectInfo> &infos, vector<pair<int, int>> &cluster_ranges) const{
    if(infos.empty()) return;

    sort(infos.begin(), infos.end(),[](const ObjectInfo& info1, const ObjectInfo& info2){
        if(info1.begin_time == info2.begin_time) return info1.end_time < info2.end_time;
        return info1.begin_time < info2.begin_time;
    });

    int left = 0, right = 0;
    double border = infos.front().begin_time + eps;
    while(right != infos.size() - 1){
        int next_idx = right + 1;
        double next_time = infos[next_idx].begin_time;
        if(next_time <= border) right++;
        else{
            if(right - left + 1 >= m) cluster_ranges.emplace_back(left, right);
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
    if(right - left + 1 >= m) cluster_ranges.emplace_back(left, right);
}

void GraphMiner::init_clusters(){
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
    // construct cluster paths
    for(Path &path : travel_paths){
        cluster_paths.emplace_back(path.positions.size(), make_pair(-1, ClusterIds()));
    }

    int n_clusters = 0;
    for(auto &camera_entry : camera2objects){
        ll camera = camera_entry.first;
        vector<ObjectInfo> &infos = camera_entry.second;
        vector<pair<int, int>> cluster_ranges;
        temporal_cluster(infos, cluster_ranges);

        for(pair<int, int> &cluster_range : cluster_ranges){
            clusters.emplace_back(n_clusters, camera);
            for(int info_idx = cluster_range.first; info_idx <= cluster_range.second; info_idx++){
                ObjectInfo &info = infos[info_idx];
                cluster_paths[info.object_id][info.pid].second.push_back(n_clusters);
            }
            n_clusters++;
        }
    }

    for(vector<pair<int, ClusterIds>> &cluster_path : cluster_paths){
        int counts = 0;
        for(int pid = 0; pid < cluster_path.size(); pid++){
            ClusterIds &ids = cluster_path[pid].second;
            if(ids.empty()) continue;
            if(counts == pid) cluster_path[counts].first = pid;
            else cluster_path[counts] = make_pair(pid, ids);
            counts++;
        }
        cluster_path.resize(counts);
    }
    // fill the parameter $member$ in each cluster
    for(int oid = 0; oid < cluster_paths.size(); oid++){
        vector<Position> &travel_path = travel_paths[oid].positions;
        vector<pair<int, ClusterIds>> &cluster_path = cluster_paths[oid];
        for(int c_pid = 0; c_pid < cluster_path.size(); c_pid++){
            int travel_pid = cluster_path[c_pid].first;
            Position &position = travel_path[travel_pid];
            for(int cluster_id : cluster_path[c_pid].second){
                clusters[cluster_id].members.emplace_back(oid, c_pid, position.begin_time, position.end_time);
            }
        }
    }

}

void GraphMiner::compute_order() {
    // initialize cluster-related data structures
    init_clusters();
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

struct GraphMiner::ExpandTuple{
    set<int> crt_oids; // distinct object_ids in current cluster
    int num_maximals = 0; // number of ObjectInfos > all previous ObjectInfos from the same ObjectInfo in first cluster
    ObjectMap obj_map;
};
void GraphMiner::expand(vector<int> &cluster_route, ObjectMap &obj_map){
    map<int, ExpandTuple> expand_map;
    for(auto &obj_entry : obj_map){
        const ObjectInfo &begin_obj = obj_entry.first;
        set<ObjectInfo> &end_objs = obj_entry.second;

        int oid = begin_obj.object_id;
        auto &travel_path = travel_paths[oid].positions;
        auto &cluster_path = cluster_paths[oid];
        int border = static_cast<int>(cluster_path.size()) - 1;
        for(auto it = end_objs.begin(); it != end_objs.end(); ++it){
            int base_tpid = cluster_path[it->pid].first; // tpid means "position id in travel path"
            if(it == end_objs.begin()){
                for(int nxt_pid = it->pid + 1; nxt_pid <= border; ++nxt_pid){
                    int nxt_tpid = cluster_path[nxt_pid].first;
                    if(nxt_tpid - base_tpid - 1 > d) break;

                    ObjectInfo nxt_obj(oid, nxt_pid, travel_path[nxt_tpid].begin_time, travel_path[nxt_tpid].end_time);
                    for(int cluster_id : cluster_path[nxt_pid].second){
                        ExpandTuple *p_tuple = nullptr;
                        auto expand_it = expand_map.find(cluster_id);
                        if(expand_it == expand_map.end()) p_tuple = &(expand_map[cluster_id] = ExpandTuple());
                        else p_tuple = &(expand_it->second);

                        auto obj_it = p_tuple->obj_map.find(begin_obj);
                        if(obj_it == p_tuple->obj_map.end()){
                            p_tuple->crt_oids.insert(oid);
                            p_tuple->num_maximals += 1;
                            p_tuple->obj_map[begin_obj] = set<ObjectInfo>{nxt_obj};
                        }
                        else obj_it->second.insert(nxt_obj);
                    }
                }
                border = it->pid;
            }
            else{
                for(int nxt_pid = it->pid + 1; nxt_pid <= border; ++nxt_pid){
                    int nxt_tpid = cluster_path[nxt_pid].first;
                    if(nxt_tpid - base_tpid - 1 > d) break;

                    ObjectInfo nxt_obj(oid, nxt_pid, travel_path[nxt_tpid].begin_time, travel_path[nxt_tpid].end_time);
                    for(int cluster_id : cluster_path[nxt_pid].second){
                        ExpandTuple *p_tuple = nullptr;
                        auto expand_it = expand_map.find(cluster_id);
                        if(expand_it == expand_map.end()) p_tuple = &(expand_map[cluster_id] = ExpandTuple());
                        else p_tuple = &(expand_it->second);

                        auto obj_it = p_tuple->obj_map.find(begin_obj);
                        if(obj_it == p_tuple->obj_map.end()){
                            p_tuple->crt_oids.insert(oid);
                            p_tuple->obj_map[begin_obj] = set<ObjectInfo>{nxt_obj};
                        }
                        else obj_it->second.insert(nxt_obj);
                    }
                }
            }

        }
    }

    // traverse $expand_map$ in $cluster.order$ order to determine next clusters for expansion
    bool is_dominated = false; // whether the current pattern is dominated
    int num_begin_objs = static_cast<int>(obj_map.size());

    vector<pair<const int, ExpandTuple>*> entry_pointers;
    for(auto &entry : expand_map) entry_pointers.push_back(&entry);
    sort(entry_pointers.begin(), entry_pointers.end(),
         [this](pair<const int, ExpandTuple>* p1, pair<const int, ExpandTuple>* p2){
             return clusters[p1->first].order < clusters[p2->first].order;
    });
    for(auto entry_pointer : entry_pointers){
        int cluster_id = entry_pointer->first;
        ExpandTuple &tuple = entry_pointer->second;
        if(tuple.crt_oids.size() < m) continue;

        if(tuple.num_maximals == num_begin_objs) is_dominated = true;
        if(!clusters[cluster_id].skip){
            set<ObjectInfo> crt_members;
            for(auto &entry : tuple.obj_map){
                set<ObjectInfo> &partial_members = entry.second;
                crt_members.insert(partial_members.begin(), partial_members.end());
            }
            clusters[cluster_id].skip = (crt_members.size() == clusters[cluster_id].members.size());
        }
        cluster_route.push_back(cluster_id);
        expand(cluster_route, tuple.obj_map);
        cluster_route.pop_back();
    }
    // add to results
    if(!is_dominated && cluster_route.size() >= k){
        map<int, vector<pair<double, double>>> oid2intervals;
        for(auto &obj_entry : obj_map){
            const ObjectInfo &begin_obj = obj_entry.first, &end_obj = *(obj_entry.second.begin());
            int oid = begin_obj.object_id;
            auto it = oid2intervals.find(oid);
            if(it == oid2intervals.end()) oid2intervals[oid] = {{begin_obj.begin_time, end_obj.begin_time}};
            else it->second.emplace_back(begin_obj.begin_time, end_obj.begin_time);
        }
        // get oids, camera_path and time_intervals
        set<int> oid_set;
        for(auto &obj_entry : obj_map) oid_set.insert(obj_entry.first.object_id);
        vector<int> oids(oid_set.begin(), oid_set.end());

        vector<ll> camera_route(cluster_route.size());
        for(int idx = 0; idx < cluster_route.size(); ++idx)
            camera_route[idx] = clusters[cluster_route[idx]].camera;

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

void GraphMiner::run(){
    // get all the clusters and their topological order in graph
    compute_order();
    ClusterIds cluster_ids(clusters.size());
    for(int cid = 0; cid < cluster_ids.size(); cid++) cluster_ids[cid] = cid;
    sort(cluster_ids.begin(), cluster_ids.end(), [this](const int &cid1, const int &cid2){
        return clusters[cid1].order < clusters[cid2].order;
    });
    // expand each cluster according to the order in $cluster_ids$
    for(int cluster_id : cluster_ids){
        Cluster &cluster = clusters[cluster_id];
        if(cluster.skip) continue;

        vector<int> cluster_route{cluster_id};
        ObjectMap obj_map;
        for(ObjectInfo &member : cluster.members)
            obj_map.insert(make_pair(member, set<ObjectInfo>{member}));
        expand(cluster_route, obj_map);
    }
}

bool GraphMiner::is_subpath(const vector<ll> &short_path, const vector<ll> &long_path){
    if(short_path.size() > long_path.size()) return false;
    int s_ptr = 0, l_ptr = 0;
    while(s_ptr < short_path.size() && l_ptr < long_path.size()) {
        if(short_path[s_ptr] == long_path[l_ptr]) s_ptr++;
        l_ptr++;
    }
    return s_ptr == short_path.size();
}

void GraphMiner::deduplicate_twin(map<vector<ll>, Intervals> &super_map, map<vector<ll>, Intervals> &child_map){
    for(auto child_it = child_map.begin(); child_it != child_map.end();){
        const vector<ll> &path = child_it->first;
        vector<pair<double, double>> &intervals = child_it->second;
        for(auto &super_entry : super_map){
            const vector<ll> &super_path = super_entry.first;
            if(super_path.size() < path.size()) continue;

            if(is_subpath(path, super_path)){
                vector<pair<double, double>> &super_intervals = super_entry.second;
                for(auto it = intervals.begin(); it != intervals.end();){
                    bool erase_flag = false;
                    for(auto &super_interval : super_intervals){
                        if(super_interval.first > it->first) break;
                        if(super_interval.second >= it->second) {
                            erase_flag = true;
                            break;
                        }
                    }
                    if(erase_flag) it = intervals.erase(it);
                    else it++;
                }
                if(intervals.empty()) break;
            }
        }
        if(intervals.empty()) child_it = child_map.erase(child_it);
        else child_it++;
    }
}

void GraphMiner::deduplicate_time(map<vector<ll>, Intervals> &cameras_map){
    for(auto it = cameras_map.begin(); it != cameras_map.end();){
        vector<pair<double, double>> &time_intervals = it->second;
        if(time_intervals.empty()) it = cameras_map.erase(it);
        else{
            sort(time_intervals.begin(), time_intervals.end());
            vector<pair<double, double>> new_intervals;
            pair<double, double> pre_interval = time_intervals.front();
            for(int time_idx = 1; time_idx < time_intervals.size(); time_idx++){
                pair<double, double> &interval = time_intervals[time_idx];
                if(interval.first == pre_interval.first) pre_interval.second = interval.second;
                else{
                    if(interval.second > pre_interval.second){
                        new_intervals.push_back(pre_interval);
                        pre_interval = interval;
                    }
                }
            }
            new_intervals.push_back(pre_interval);
            it->second = new_intervals;
            it++;
        }
    }

    vector<vector<ll>> paths;
    for(auto &cameras_entry : cameras_map) paths.push_back(cameras_entry.first);
    vector<pair<pair<double, double>, int>> interval_infos;
    int entry_idx = 0;
    for(auto &cameras_entry : cameras_map){
        for(pair<double, double> &interval : cameras_entry.second)
            interval_infos.emplace_back(interval, entry_idx);
        entry_idx++;
    }
    sort(interval_infos.begin(), interval_infos.end(),
         [](const pair<pair<double, double>, int> &info1, const pair<pair<double, double>, int> &info2){
             return info1.first < info2.first;
         });

    vector<bool> contain_flags(interval_infos.size(), false);
    vector<vector<int>> child_pointers(interval_infos.size(), vector<int>());
    for(int info_idx = 0; info_idx < interval_infos.size(); info_idx++){
        if(contain_flags[info_idx]) continue;
        const pair<double, double> &interval = interval_infos[info_idx].first;
        for(int nxt_idx = info_idx + 1; nxt_idx < interval_infos.size(); nxt_idx++){
            const pair<double, double> &nxt_interval = interval_infos[nxt_idx].first;
            if(interval.first == nxt_interval.first){
                child_pointers[nxt_idx].push_back(info_idx);
                if(interval.second == nxt_interval.second) child_pointers[info_idx].push_back(nxt_idx);
            }
            else{
                if(interval.second < nxt_interval.first) break;
                if(interval.second >= nxt_interval.second) child_pointers[info_idx].push_back(nxt_idx);
            }
        }

        const vector<ll> &path = paths[interval_infos[info_idx].second];
        vector<int> &children = child_pointers[info_idx];
        for(int child_idx : children){
            if(contain_flags[child_idx]) continue;
            const vector<ll> &child_path = paths[interval_infos[child_idx].second];
            if(is_subpath(child_path, path)) contain_flags[child_idx] = true;
        }
    }

    cameras_map.clear();
    for(int info_idx = 0; info_idx < interval_infos.size(); info_idx++){
        if(contain_flags[info_idx]) continue;
        const vector<ll> &path = paths[interval_infos[info_idx].second];
        auto it = cameras_map.find(path);
        if(it == cameras_map.end())
            cameras_map[path] = vector<pair<double, double>>{interval_infos[info_idx].first};
        else
            it->second.push_back(interval_infos[info_idx].first);
    }
}

void GraphMiner::deduplicate(){
    // initialize necessary data structure
    vector<map<vector<int>, map<vector<ll>, Intervals>>::iterator> idx2it;
    int crt_idx = 0;
    map<int, vector<int>> length2idx;
    for(auto it = results.begin(); it != results.end(); it++){
        int length = int(it->first.size());
        auto length_it = length2idx.find(length);
        if(length_it == length2idx.end()) length2idx[length] = vector<int>{crt_idx};
        else length_it->second.push_back(crt_idx);

        idx2it.push_back(it);
        crt_idx++;
    }
    vector<int> idcs;
    vector<vector<int>> out_links;
    vector<int> starts;
    int item_idx = 0;
    for(auto &entry : length2idx){
        starts.push_back(item_idx);
        item_idx += int(entry.second.size());
        for(int idx : entry.second){
            idcs.push_back(idx);
            out_links.emplace_back();
        }
    }
    // get inversion list
    unordered_map<int, unordered_set<int>> inversion_lst;
    for(int i = 0; i < idcs.size(); i++){
        const vector<int> &object_ids = idx2it[idcs[i]]->first;
        for(int object_id : object_ids){
            auto it = inversion_lst.find(object_id);
            if(it == inversion_lst.end()) inversion_lst[object_id] = unordered_set<int>{i};
            else it->second.insert(i);
        }
    }
    for(int level = 0; level < starts.size(); level++){
        int start_pos = starts[level];
        int end_pos = (level == starts.size() - 1) ? idcs.size() - 1 : starts[level + 1] - 1;
        for(int i = start_pos; i <= end_pos; i++){
            const vector<int> &object_ids = idx2it[idcs[i]]->first;
            // choose object_id which belongs to the smallest number of object_set
            int min_number = INT_MAX; int min_object = -1;
            for(int object_id : object_ids){
                int crt_number = inversion_lst[object_id].size();
                if(crt_number < min_number){
                    min_number = crt_number;
                    min_object = object_id;
                }
            }
            if(min_object == -1) cerr << "very weird, from BIDE_Verifier::de_duplicate" << endl;

            unordered_set<int> intersection = inversion_lst[min_object];
            for(int object_id : object_ids){
                if(object_id == min_object) continue;
                unordered_set<int> &large_set = inversion_lst[object_id];
                for(auto it = intersection.begin(); it != intersection.end();){
                    if(large_set.find(*it) == large_set.end()) it = intersection.erase(it);
                    else it++;
                }
            }
            intersection.erase(i);
            for(int super_pos : intersection) out_links[i].push_back(super_pos);
            for(int object_id : object_ids) inversion_lst[object_id].erase(i);
        }
    }
    // continue to check according to $out_links$
    for(int pos = out_links.size() - 1; pos >= 0; pos--){
        deduplicate_time(idx2it[idcs[pos]]->second);
        for(int super_pos : out_links[pos]){
            deduplicate_twin(idx2it[idcs[super_pos]]->second, idx2it[idcs[pos]]->second);
        }
    }
    for(auto result_it = results.begin(); result_it != results.end();){
        if(result_it->second.empty()) result_it = results.erase(result_it);
        else result_it++;
    }
}

void GraphMiner::dump_results(string file_name){
    ofstream ofs;
    ofs.open(file_name, ios::out);
    ofs<<"Objects,Path"<<endl;
    for(auto &object_entry : results){
        auto &object_ids = object_entry.first;
        string s_obj;
        for(const int &object_id : object_ids){
            s_obj += travel_paths[object_id].object_name;
            if(&object_id != &object_ids.back()) s_obj += " ";
            else s_obj += ",";
        }
        for(auto &path_entry : object_entry.second){
            string s_camera;
            for(const ll &camera : path_entry.first){
                s_camera += to_string(camera);
                if(&camera != &path_entry.first.back()) s_camera += " ";
                else s_camera += "\n";
            }
            ofs<<s_obj<<s_camera;
        }
    }
    ofs.close();
}