#ifndef EXPERIMENT_DEDUPLICATION_H
#define EXPERIMENT_DEDUPLICATION_H

#include <map>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <climits>
#include <iostream>
#include <algorithm>
#include "Utils/structs.h"

class Deduplicator{
    map<vector<int>, map<vector<ll>, vector<pair<double, double>>>> &results;

    // deduplication sub-function based on time intervals called by $deduplicate()$
    void sub_deduplicate(map<vector<ll>, vector<pair<double, double>>> &cameras_map);
    // another deduplication sub-function called by $deduplicate()$
    void sub_deduplicate_twin(map<vector<ll>, vector<pair<double, double>>> &super_map, map<vector<ll>, vector<pair<double, double>>> &child_map);
    bool is_subpath(const vector<ll> &small_path, const vector<ll> &large_path);
public:
    explicit Deduplicator(map<vector<int>, map<vector<ll>, vector<pair<double, double>>>> &results): results(results){}
    // main deduplication function
    void deduplicate();
};

void Deduplicator::deduplicate(){
    // initialize necessary data structure
    vector<map<vector<int>, map<vector<ll>, vector<pair<double, double>>>>::iterator> idx2it;
    int crt_idx = 0;
    map<int, vector<int>> length2idx;
    for(auto it = results.begin(); it != results.end(); it++){
        int length = static_cast<int>(it->first.size());
        length2idx[length].push_back(crt_idx);

        idx2it.push_back(it);
        crt_idx++;
    }
    vector<int> idcs;
    vector<vector<int>> out_links;
    vector<int> starts;
    int item_idx = 0;
    for(auto &entry : length2idx){
        starts.push_back(item_idx);
        item_idx += static_cast<int>(entry.second.size());
        for(int idx : entry.second){
            idcs.push_back(idx);
            out_links.emplace_back();
        }
    }
    // get inversion list
    unordered_map<int, unordered_set<int>> inversion_lst;
    for(int i = 0; i < idcs.size(); i++){
        const vector<int> &object_ids = idx2it[idcs[i]]->first;
        for(int object_id : object_ids) inversion_lst[object_id].insert(i);
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
        sub_deduplicate(idx2it[idcs[pos]]->second);
        for(int super_pos : out_links[pos]){
            sub_deduplicate_twin(idx2it[idcs[super_pos]]->second, idx2it[idcs[pos]]->second);
        }
    }
    for(auto result_it = results.begin(); result_it != results.end();){
        if(result_it->second.empty()) result_it = results.erase(result_it);
        else result_it++;
    }
}

void Deduplicator::sub_deduplicate(map<vector<ll>, vector<pair<double, double>>> &cameras_map){
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
    map<pair<double, double>, vector<int>> interval_map;
    for(auto &cameras_entry : cameras_map) paths.push_back(cameras_entry.first);
    int path_id = 0;
    for(auto &cameras_entry : cameras_map){
        for(pair<double, double> &interval : cameras_entry.second)
            interval_map[interval].push_back(path_id);
        ++path_id;
    }
    for(auto &interval_entry : interval_map){
        vector<int> &path_ids = interval_entry.second;
        sort(path_ids.begin(), path_ids.end(), [&paths](const int &id1, const int &id2){
            return paths[id1].size() > paths[id2].size();
        });

        vector<int> new_ids;
        vector<bool> skip_flags(path_ids.size(), false);
        for(int i = 0; i < path_ids.size(); ++i){
            if(skip_flags[i]) continue;
            int pid = path_ids[i];
            new_ids.push_back(pid);

            vector<ll> &long_path = paths[pid];
            for(int j = i + 1; j < path_ids.size(); ++j){
                if(skip_flags[j]) continue;
                vector<ll> &short_path = paths[path_ids[j]];
                if(long_path.size() == short_path.size()) continue;
                if(is_subpath(short_path, long_path)) skip_flags[j] = true;
            }
        }
        path_ids = new_ids;
    }

    vector<pair<pair<double, double>, int>> interval_entries;
    for(auto &interval_entry : interval_map){
        const pair<double, double> &interval = interval_entry.first;
        for(int pid : interval_entry.second)
            interval_entries.emplace_back(interval, pid);
    }
    vector<bool> contain_flags(interval_entries.size(), false);
    vector<vector<int>> child_pointers(interval_entries.size(), vector<int>());
    for(int idx = 0; idx < interval_entries.size(); ++idx){
        if(contain_flags[idx]) continue;
        const pair<double, double> &interval = interval_entries[idx].first;
        for(int nxt_idx = idx + 1; nxt_idx < interval_entries.size(); ++nxt_idx){
            const pair<double, double> &nxt_interval = interval_entries[nxt_idx].first;
            if(interval.first == nxt_interval.first){
                if(interval.second != nxt_interval.second)
                    child_pointers[nxt_idx].push_back(idx);
            }
            else{
                if(interval.second < nxt_interval.first) break;
                if(interval.second >= nxt_interval.second) child_pointers[idx].push_back(nxt_idx);
            }
        }

        const vector<ll> &path = paths[interval_entries[idx].second];
        vector<int> &children = child_pointers[idx];
        for(int child_idx : children){
            if(contain_flags[child_idx]) continue;
            const vector<ll> &child_path = paths[interval_entries[child_idx].second];
            if(is_subpath(child_path, path)) contain_flags[child_idx] = true;
        }
    }

    cameras_map.clear();
    for(int info_idx = 0; info_idx < interval_entries.size(); ++info_idx){
        if(contain_flags[info_idx]) continue;
        const vector<ll> &path = paths[interval_entries[info_idx].second];
        cameras_map[path].push_back(interval_entries[info_idx].first);
    }
}

void Deduplicator::sub_deduplicate_twin(map<vector<ll>, vector<pair<double, double>>> &super_map, map<vector<ll>, vector<pair<double, double>>> &child_map){
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

bool Deduplicator::is_subpath(const vector<ll> &small_path, const vector<ll> &large_path){
    if(small_path.size() > large_path.size()) return false;
    int s_ptr = 0, l_ptr = 0;
    while(s_ptr < small_path.size() && l_ptr < large_path.size()) {
        if(small_path[s_ptr] == large_path[l_ptr]) s_ptr++;
        l_ptr++;
    }
    return s_ptr == small_path.size();
}

#endif //EXPERIMENT_DEDUPLICATION_H
