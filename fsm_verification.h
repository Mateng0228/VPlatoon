#ifndef BASELINE_FSM_VERIFICATION_H
#define BASELINE_FSM_VERIFICATION_H

#include <algorithm>
#include <set>
#include <unordered_map>
#include <climits>
#include "fsm.h"

class BIDE_Verifier{
public:
    int m, k, d;
    double eps;
    vector<Path> &SDB;
    map<vector<int>, map<vector<int>, vector<pair<double, double>>>> results;// <object_ids: <camera_path: time_intervals>>

    BIDE_Verifier(vector<Path> &SDB, int m, int k, int d, double eps):SDB(SDB){
        this->m = m; this->k = k; this->d = d;
        this->eps = eps;
        p_camera_path = nullptr;
        p_appearances = nullptr;
        p_cluster_lst = nullptr;
        p_inversions_lst = nullptr;
    }

    void verify(vector<pair<vector<int>, vector<Appearance>>> &sequences){
        if(!results.empty()) results.clear();

        for(auto &seq_entry : sequences){
            const vector<int> &camera_path = seq_entry.first;
            if(camera_path.size() < k) continue;

            vector<Appearance> &appearances = seq_entry.second;
            vector<vector<pair<int, int>>> clusters_lst; // clusters
            vector<vector<pair<int, pair<int, int>>>> inversions_lst; // position and clusters of ap in each camera
            // assign values to private pointers
            p_camera_path = &camera_path;
            p_appearances = &appearances;
            p_cluster_lst = &clusters_lst;
            p_inversions_lst = &inversions_lst;
            // cluster appearances in each camera
            for(int camera_idx = 0; camera_idx < camera_path.size(); camera_idx++){
                vector<tuple<int, int, int>> positions;
                for(int ap_id = 0; ap_id < appearances.size(); ap_id++)
                    positions.emplace_back(ap_id, appearances[ap_id].sid, appearances[ap_id].pids[camera_idx]);
                clusters_lst.emplace_back();
                vector<pair<int, int>> &clusters = clusters_lst.back();
                inversions_lst.emplace_back();
                vector<pair<int, pair<int, int>>> &inversions = inversions_lst.back();

                temporal_cluster(positions, clusters, inversions);
            }
            // get all clusters in first camera
            vector<int> ap_ids;
            for(int ap_id = 0; ap_id < appearances.size(); ap_id++) ap_ids.push_back(ap_id);
            sort(ap_ids.begin(), ap_ids.end(), [&](const int &i1, const int &i2){
                Appearance &ap1 = appearances[i1], &ap2 = appearances[i2];
                return SDB[ap1.sid].positions[ap1.pids.front()].begin_time < SDB[ap2.sid].positions[ap2.pids.front()].begin_time;
            });
            // begin dfs
            vector<pair<int, int>> &clusters = clusters_lst.front();
            for(pair<int, int> &p : clusters){
                vector<int> cluster(ap_ids.begin() + p.first, ap_ids.begin() + p.second + 1);
                if(!check_numbers(cluster, appearances)) continue;

                vector<int> camera_ids{0};
                dfs_mine(cluster, camera_ids);
            }
        }
    }

    void deduplicate(){
        // initialize necessary data structure
        vector<map<vector<int>, map<vector<int>, vector<pair<double, double>>>>::iterator> idx2it;
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

private:
    const vector<int> *p_camera_path;
    vector<Appearance> *p_appearances;
    vector<vector<pair<int, int>>> *p_cluster_lst;
    vector<vector<pair<int, pair<int, int>>>> *p_inversions_lst;

    void dfs_mine(vector<int> &ap_ids, vector<int> &camera_ids){
        int last_cid = camera_ids.back();
        if(last_cid == int(p_camera_path->size()) - 1 && camera_ids.size() >= k){
            add_result(ap_ids, camera_ids);
            return;
        }

        bool insert_flag = true;
        int nxt_cid = last_cid + 1;
        vector<pair<int, pair<int, int>>> &nxt_inversions = p_inversions_lst->at(nxt_cid);
        map<int, vector<int>> cluster2aps;
        for(int ap_id : ap_ids){
            pair<int, int> &cluster_range = nxt_inversions[ap_id].second;
            if(cluster_range.first == -1) continue;
            for(int cluster_id = cluster_range.first; cluster_id <= cluster_range.second; cluster_id++){
                auto itr = cluster2aps.find(cluster_id);
                if(itr == cluster2aps.end())
                    cluster2aps.insert(make_pair(cluster_id, vector<int>{ap_id}));
                else
                    itr->second.push_back(ap_id);
            }
        }

        for(auto &entry : cluster2aps){
            if(entry.second.size() < m) continue;

            vector<int> &new_ap_ids = entry.second;
            vector<int> new_camera_ids = camera_ids;
            new_camera_ids.push_back(nxt_cid);

            if(new_ap_ids.size() == ap_ids.size()) insert_flag = false;
            dfs_mine(new_ap_ids, new_camera_ids);
        }

        if(insert_flag && camera_ids.size() >= k) add_result(ap_ids, camera_ids);
    }

    void add_result(vector<int> &ap_ids, vector<int> &camera_ids){
        bool need_collect = false;
        map<int, vector<int>> oid2aids;
        for(int ap_id : ap_ids){
            int o_id = p_appearances->at(ap_id).sid;
            auto it = oid2aids.find(o_id);
            if(it == oid2aids.end()) oid2aids[o_id] = vector<int>{ap_id};
            else{
                need_collect = true;
                it->second.push_back(ap_id);
            }
        }
        if(oid2aids.size() < m) return;
        vector<vector<int>> aids_lst;
        if(need_collect){
            vector<int> aids;
            collect_aids(oid2aids, oid2aids.begin(), aids, aids_lst);
        }
        else aids_lst.emplace_back(ap_ids.begin(), ap_ids.end());

        vector<pair<double, double>> time_intervals;
        for(vector<int> &aids : aids_lst){
            double start_time = INT_MAX, end_time = -1;
            for(int ap_id : aids){
                Appearance &appearance = p_appearances->at(ap_id);
                double crt_time = SDB[appearance.sid].positions[appearance.pids[camera_ids.front()]].begin_time;
                start_time = min(start_time, crt_time);
            }
            for(int ap_id : aids){
                Appearance &appearance = p_appearances->at(ap_id);
                double crt_time = SDB[appearance.sid].positions[appearance.pids[camera_ids.back()]].begin_time;
                end_time = max(end_time, crt_time);
            }
            time_intervals.emplace_back(start_time, end_time);
        }

        // insert into results
        vector<int> object_ids;
        for(auto &entry : oid2aids) object_ids.push_back(entry.first);
        vector<int> cameras;
        for(int camera_id : camera_ids) cameras.push_back(p_camera_path->at(camera_id));
        auto object_it = results.find(object_ids);
        if(object_it == results.end())
            results[object_ids] = map<vector<int>, vector<pair<double, double>>>{{cameras, time_intervals}};
        else{
            auto &path_map = object_it->second;
            auto path_it = path_map.find(cameras);
            if(path_it == path_map.end())
                path_map[cameras] = vector<pair<double, double>>{time_intervals};
            else{
                path_it->second.insert(path_it->second.end(), time_intervals.begin(), time_intervals.end());
            }
        }
    }

    void collect_aids(map<int, vector<int>> &oid2aids, map<int, vector<int>>::iterator it, vector<int> &aids, vector<vector<int>> &answer) {
        if(it == oid2aids.end()){
            answer.push_back(aids);
            return;
        }
        for(int ap_id : it->second){
            aids.push_back(ap_id);
            collect_aids(oid2aids, next(it), aids, answer);
            aids.pop_back();
        }
    }

    /**
     * @param positions -> (appearance_id, sid, position)
     * @param clusters -> (begin_idx in $pids$, end_idx in $pids$)
     * @param inversions -> (current ap_id's detailed position in SDB[Path], inversion item of $clusters$)
     **/
    void temporal_cluster(vector<tuple<int, int, int>> &positions, vector<pair<int, int>> &clusters, vector<pair<int, pair<int, int>>> &inversions){
        // sort pids
        sort(positions.begin(), positions.end(), [&](const tuple<int, int, int> &p1, const tuple<int, int, int> &p2){
            return SDB[get<1>(p1)].positions[get<2>(p1)].begin_time < SDB[get<1>(p2)].positions[get<2>(p2)].begin_time;
        });
        // fill in related list
        for(int idx = 0; idx < positions.size(); idx++) inversions.emplace_back(-1, make_pair(-1, -1));
        vector<double> time_lst;
        for(auto &t : positions){
            time_lst.push_back(SDB[get<1>(t)].positions[get<2>(t)].begin_time);
            inversions[get<0>(t)].first = get<2>(t);
        }
        // cluster
        int begin_idx = 0, end_idx = 0;
        double border = time_lst.front() + eps;
        while(end_idx < time_lst.size() - 1){
            int next_idx = end_idx + 1;
            double next_start = time_lst[next_idx];
            if(next_start <= border) end_idx++;
            else{
                if(end_idx - begin_idx + 1 >= m) clusters.emplace_back(begin_idx, end_idx);// inclusive
                while(begin_idx != next_idx){
                    if(next_start - time_lst[begin_idx] <= eps) break;
                    else begin_idx++;
                }
                border = time_lst[begin_idx] + eps;
                end_idx++;
            }
        }
        if(end_idx - begin_idx + 1 >= m) clusters.emplace_back(begin_idx, end_idx);
        if(clusters.empty()) return;
        // fill in inversions
        int pre_idx = clusters.front().first;
        for(int cluster_idx = 0; cluster_idx < clusters.size(); cluster_idx++){
            int curr_idx = clusters[cluster_idx].second;
            for(int idx = pre_idx; idx <= curr_idx; idx++) inversions[get<0>(positions[idx])].second.first = cluster_idx;
            if(cluster_idx + 1 < clusters.size() && clusters[cluster_idx + 1].first > curr_idx) pre_idx = clusters[cluster_idx + 1].first;
            else pre_idx = curr_idx + 1;
        }
        pre_idx = clusters.back().second;
        for(int cluster_idx = clusters.size() - 1; cluster_idx >= 0; cluster_idx--){
            int curr_idx = clusters[cluster_idx].first;
            for(int idx = curr_idx; idx <= pre_idx; idx++) inversions[get<0>(positions[idx])].second.second = cluster_idx;
            if(cluster_idx - 1 >= 0 && clusters[cluster_idx - 1].second < curr_idx) pre_idx = clusters[cluster_idx - 1].second;
            else pre_idx = curr_idx - 1;
        }
    }

    // check if object numbers satisfy $m$
    bool check_numbers(vector<int> &ap_ids, vector<Appearance> &appearances){
        if(ap_ids.empty()) return false;

        vector<int> sids;
        for(int ap_id : ap_ids) sids.push_back(appearances[ap_id].sid);
        sort(sids.begin(), sids.end());
        int counts = 1, pre_val = sids[0];
        for(int idx = 1; idx < sids.size(); idx++){
            if(pre_val != sids[idx]){
                counts += 1;
                pre_val = sids[idx];
            }
        }
        return counts >= m;
    }

    // deduplication sub-function based on brute force
    void deduplicate_bf(map<vector<int>, vector<pair<double, double>>> &cameras_map){
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

        vector<map<vector<int>, vector<pair<double, double>>>::iterator> idx2it;
        vector<int> idcs;
        int crt_idx = 0;
        for(auto it = cameras_map.begin(); it != cameras_map.end(); it++){
            idx2it.push_back(it);
            idcs.push_back(crt_idx++);
        }
        sort(idcs.begin(), idcs.end(), [&idx2it](const int &idx1, const int &idx2){
            return idx2it[idx1]->first.size() < idx2it[idx2]->first.size();
        });
        for(int pos = idcs.size() - 1; pos >= 0; pos--){
            auto &path = idx2it[idcs[pos]]->first;
            auto &intervals = idx2it[idcs[pos]]->second;
            for(int super_pos = pos + 1; super_pos < idcs.size(); super_pos++){
                auto &super_path = idx2it[idcs[super_pos]]->first;
                if(super_path.size() == path.size()) continue;

                if(is_subpath(path, super_path)){
                    auto &super_intervals = idx2it[idcs[super_pos]]->second;
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
        }

        for(auto it = cameras_map.begin(); it != cameras_map.end();){
            if(it->second.empty()) it = cameras_map.erase(it);
            else it++;
        }
    }

    // improved deduplication based on time intervals
    void deduplicate_time(map<vector<int>, vector<pair<double, double>>> &cameras_map){
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

        vector<vector<int>> paths;
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

            const vector<int> &path = paths[interval_infos[info_idx].second];
            vector<int> &children = child_pointers[info_idx];
            for(int child_idx : children){
                if(contain_flags[child_idx]) continue;
                const vector<int> &child_path = paths[interval_infos[child_idx].second];
                if(is_subpath(child_path, path)) contain_flags[child_idx] = true;
            }
        }

        cameras_map.clear();
        for(int info_idx = 0; info_idx < interval_infos.size(); info_idx++){
            if(contain_flags[info_idx]) continue;
            const vector<int> &path = paths[interval_infos[info_idx].second];
            auto it = cameras_map.find(path);
            if(it == cameras_map.end())
                cameras_map[path] = vector<pair<double, double>>{interval_infos[info_idx].first};
            else
                it->second.push_back(interval_infos[info_idx].first);
        }
    }

    void deduplicate_twin(map<vector<int>, vector<pair<double, double>>> &super_map, map<vector<int>, vector<pair<double, double>>> &child_map){
        for(auto child_it = child_map.begin(); child_it != child_map.end();){
            const vector<int> &path = child_it->first;
            vector<pair<double, double>> &intervals = child_it->second;
            for(auto &super_entry : super_map){
                const vector<int> &super_path = super_entry.first;
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

    bool is_subpath(const vector<int> &small_path, const vector<int> &large_path) {
        if(small_path.size() > large_path.size()) return false;
        int s_ptr = 0, l_ptr = 0;
        while(s_ptr < small_path.size() && l_ptr < large_path.size()) {
            if(small_path[s_ptr] == large_path[l_ptr]) s_ptr++;
            l_ptr++;
        }
        return s_ptr == small_path.size();
    }

};

#endif //BASELINE_FSM_VERIFICATION_H