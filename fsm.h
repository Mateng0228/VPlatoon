#ifndef BASELINE_FSM_H
#define BASELINE_FSM_H

#include <vector>
#include <set>
#include <map>
#include <unordered_set>
#include <iostream>
#include <algorithm>
#include "Utils/structs.h"

using namespace std;

struct Appearance{
    int sid; int begin_id; int end_id;
    vector<int> pids;
    Appearance(int id, int begin, int end){
        this->sid = id;
        this->begin_id = begin;
        this->end_id = end;
    }
};

class Gap_BIDE{
private:
    vector<Path> &SDB;
    int threshold;
    int max_gap;
    map<vector<ll>, vector<Appearance>> sequential_patterns;

    void pattern_growth(vector<ll> &sequence, pair<int, vector<Appearance>> &appearances){
        map<ll, pair<int, vector<Appearance>>> forward_items;
        forward_check(appearances, forward_items);
        bool has_forward_extension = false;
        for(auto &entry : forward_items){
            if(entry.second.second.size() == appearances.second.size()){
                has_forward_extension = true;
                break;
            }
        }
        // output current pattern if necessary
        if(!has_forward_extension) sequential_patterns[sequence] = appearances.second;
        // pattern growth
        for(auto &entry : forward_items){
            if(entry.second.first >= threshold){
                vector<ll> attached_sequence(sequence);
                attached_sequence.push_back(entry.first);
                pattern_growth(attached_sequence, entry.second);
            }
        }
    }

    void forward_check(pair<int, vector<Appearance>> &appearances, map<ll, pair<int, vector<Appearance>>> &item_map){
        Path *crt_path = nullptr;
        unordered_set<ll> cameras;
        for(const Appearance &appearance : appearances.second){
            if(crt_path == nullptr) crt_path = &SDB[appearance.sid];
            else if(crt_path != &SDB[appearance.sid]){
                crt_path = &SDB[appearance.sid];
                for(ll camera_id : cameras) item_map.find(camera_id)->second.first += 1;
                cameras.clear();
            }
            int path_length = static_cast<int>(crt_path->positions.size());
            for(int pos_id = appearance.end_id + 1; pos_id <= min(appearance.end_id + 1 + max_gap, path_length - 1); ++pos_id){
                ll camera_id = crt_path->positions[pos_id].camera_id;
                cameras.insert(camera_id);
                Appearance new_appearance(crt_path->object_id, appearance.begin_id, pos_id);
                new_appearance.pids = appearance.pids;
                new_appearance.pids.push_back(pos_id);

                auto itr = item_map.find(camera_id);
                if(itr == item_map.end()){
                    item_map.insert(make_pair(
                            camera_id,
                            make_pair(0, vector<Appearance>{new_appearance})
                    ));
                }
                else itr->second.second.push_back(new_appearance);
            }
        }
        for(ll camera_id : cameras) item_map.find(camera_id)->second.first += 1;
    }

public:
    Gap_BIDE(vector<Path> &SDB, int threshold, int max_gap): SDB(SDB){
        this->threshold = threshold;
        this->max_gap = max_gap;
    }

    map<vector<ll>, vector<Appearance>>& frequent_sequential_mining(){
        if(!sequential_patterns.empty()) sequential_patterns.clear();

        map<ll, pair<int, vector<Appearance>>> stub_map;
        for(Path &path : SDB){
            unordered_set<ll> cameras;
            for(int pos_id = 0; pos_id < path.positions.size(); ++pos_id){
                ll camera_id = path.positions[pos_id].camera_id;
                cameras.insert(camera_id);
                Appearance new_appearance(path.object_id, pos_id, pos_id);
                new_appearance.pids.push_back(pos_id);

                auto itr = stub_map.find(camera_id);
                if(itr == stub_map.end()){
                    stub_map.insert(make_pair(
                            camera_id,
                            make_pair(0, vector<Appearance>{new_appearance})
                    ));
                }
                else itr->second.second.push_back(new_appearance);
            }
            for(ll camera_id : cameras) stub_map.find(camera_id)->second.first += 1;
        }
        for(auto &entry : stub_map){
            if(entry.second.first >= threshold){
                vector<ll> sequence{entry.first};
                pattern_growth(sequence, entry.second);
            }
        }

        return sequential_patterns;
    }
};

class TCS_BIDE{
private:
    struct object_info{
        int object_id;
        int pid;
        double begin_time, end_time;
        object_info(int oid, int pid, double begin_time, double end_time){
            this->object_id = oid; this->pid = pid;
            this->begin_time = begin_time; this->end_time = end_time;
        }
    };
    vector<Path> &SDB;
    int threshold;
    int k;
    int max_gap;
    double eps;
    vector<pair<vector<ll>, vector<Appearance>>> sequential_patterns;

    void init_tcs(){
        map<ll, vector<object_info>> camera2objects;
        for(int oid = 0; oid < SDB.size(); oid++){
            Path &path = SDB[oid];
            for(int pid = 0; pid < path.positions.size(); pid++){
                Position &p = path.positions[pid];
                ll camera = p.camera_id;
                auto it = camera2objects.find(camera);
                if(it == camera2objects.end())
                    camera2objects[camera] = {object_info(oid, pid, p.begin_time, p.end_time)};
                else
                    it->second.emplace_back(oid, pid, p.begin_time, p.end_time);
            }
        }

        vector<vector<int>> tcs_matrix; // record which meta-cluster each position in SDB belonging to
        for(Path &path : SDB) tcs_matrix.emplace_back(path.positions.size(), -1);
        // fill the $tcs_matrix$
        for(auto &camera_entry : camera2objects){
            ll camera = camera_entry.first;
            vector<object_info> &infos = camera_entry.second;
            vector<pair<int, int>> clusters, meta_clusters;
            temporal_cluster(infos, clusters, meta_clusters);

            for(int meta_idx = 0; meta_idx < meta_clusters.size(); meta_idx++){
                int left_cid = meta_clusters[meta_idx].first, right_cid = meta_clusters[meta_idx].second;
                int begin_idx = clusters[left_cid].first, end_idx = clusters[right_cid].second;
                for(int idx = begin_idx; idx <= end_idx; idx++){
                    object_info &info = infos[idx];
                    tcs_matrix[info.object_id][info.pid] = meta_idx;
                }
            }
        }
        // obtain tcs version SDB according to tcs matrix
        for(int oid = 0; oid < tcs_matrix.size(); oid++){
            Path &ref_path = SDB[oid];
            tcs_paths.emplace_back(oid, ref_path.object_name);
            TCS_Path &tcs_path = tcs_paths.back();

            vector<int> &mask = tcs_matrix[oid];
            vector<int> valid_pids;
            for(int pid = 0; pid < mask.size(); pid++){
                if(mask[pid] != -1) valid_pids.push_back(pid);
            }

            pair<int, int> idx_range(-1, -1);
            for(int idx = 0; idx < static_cast<int>(valid_pids.size()) - 1; idx++){
                if(idx_range.first == -1) idx_range = make_pair(idx, idx);
                else idx_range.second = idx;

                int gap = valid_pids[idx + 1] - valid_pids[idx] - 1;
                if(gap > max_gap){
                    if(idx_range.second - idx_range.first + 1 >= k){
                        for(int crt_idx = idx_range.first; crt_idx <= idx_range.second; crt_idx++){
                            int pid = valid_pids[crt_idx];
                            tcs_path.positions.push_back(ref_path.positions[pid]);
                            tcs_path.tcs_ids.push_back(mask[pid]);
                            tcs_path.offsets.push_back(pid);
                        }
                    }
                    idx_range = make_pair(-1, -1);
                }
            }
            // deal with last element of $valid_pids$
            if(idx_range.first == -1){
                if(1 >= k){
                    int pid = valid_pids.back();
                    tcs_path.positions.push_back(ref_path.positions[pid]);
                    tcs_path.tcs_ids.push_back(mask[pid]);
                    tcs_path.offsets.push_back(pid);
                }
            }
            else{
                idx_range.second += 1;
                if(idx_range.second - idx_range.first + 1 >= k){
                    for(int crt_idx = idx_range.first; crt_idx <= idx_range.second; crt_idx++){
                        int pid = valid_pids[crt_idx];
                        tcs_path.positions.push_back(ref_path.positions[pid]);
                        tcs_path.tcs_ids.push_back(mask[pid]);
                        tcs_path.offsets.push_back(pid);
                    }
                }
            }

        }

    }

    void temporal_cluster(vector<object_info> &infos, vector<pair<int, int>> &clusters, vector<pair<int, int>> &meta_clusters){
        if(infos.empty()) return;

        sort(infos.begin(), infos.end(),[](const object_info& info1, const object_info& info2){
            if(info1.begin_time == info2.begin_time) return info1.end_time < info2.end_time;
            return info1.begin_time < info2.begin_time;
        });

        int meta_left = 0, meta_right = 0;
        int left = 0, right = 0;
        int pre_right = -1;
        double border = infos.front().begin_time + eps;
        while(right != infos.size() - 1){
            int next_idx = right + 1;
            double next_time = infos[next_idx].begin_time;
            if(next_time <= border) right++;
            else{
                if(right - left + 1 >= threshold){
                    if(pre_right != -1){
                        if(pre_right < left){
                            meta_clusters.emplace_back(meta_left, meta_right);
                            meta_left = meta_right + 1;
                            meta_right = meta_left;
                        }
                        else meta_right++;
                    }

                    clusters.emplace_back(left, right);
                    pre_right = right;
                }
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
        if(right - left + 1 >= threshold){
            if(pre_right != -1){
                if(pre_right < left){
                    meta_clusters.emplace_back(meta_left, meta_right);
                    meta_clusters.emplace_back(meta_right + 1, meta_right + 1);
                }
                else meta_clusters.emplace_back(meta_left, meta_right + 1);
            }
            else meta_clusters.emplace_back(meta_left, meta_right);

            clusters.emplace_back(left, right);
        }
        else if(pre_right != -1) meta_clusters.emplace_back(meta_left, meta_right);
    }

    void pattern_growth(vector<pair<ll, int>> &mark_seq, pair<int, vector<Appearance>> &appearances){
        map<pair<ll, int>, pair<int, vector<Appearance>>> forward_items;
        forward_check(appearances, forward_items);
        bool has_forward_extension = false;
        for(auto &entry : forward_items){
            if(entry.second.second.size() == appearances.second.size()){
                has_forward_extension = true;
                break;
            }
        }
        // output current pattern if necessary
        if(!has_forward_extension && mark_seq.size() >= k){
            vector<ll> sequence;
            for(auto &mark : mark_seq) sequence.push_back(mark.first);
            sequential_patterns.emplace_back(sequence, appearances.second);
        }
        // pattern growth
        for(auto &entry : forward_items){
            if(entry.second.first >= threshold){
                vector<pair<ll, int>> attached_seq(mark_seq);
                attached_seq.push_back(entry.first);
                pattern_growth(attached_seq, entry.second);
            }
        }
    }

    void forward_check(pair<int, vector<Appearance>> &appearances, map<pair<ll, int>, pair<int, vector<Appearance>>> &item_map){
        TCS_Path *crt_path = nullptr;
        set<pair<ll, int>> marks;
        for(const Appearance &appearance : appearances.second){
            if(crt_path == nullptr) crt_path = &tcs_paths[appearance.sid];
            else if(crt_path != &tcs_paths[appearance.sid]){
                crt_path = &tcs_paths[appearance.sid];
                for(auto &mark : marks) item_map.find(mark)->second.first += 1;
                marks.clear();
            }
            int path_length = static_cast<int>(crt_path->positions.size());
            int base_pid = crt_path->offsets[appearance.end_id];
            for(int idx = appearance.end_id + 1; idx <= min(appearance.end_id + 1 + max_gap, path_length - 1); idx++){
                int crt_pid = crt_path->offsets[idx];
                if(crt_pid - base_pid - 1 > max_gap) break;

                ll camera_id = crt_path->positions[idx].camera_id;
                int meta_id = crt_path->tcs_ids[idx];
                pair<ll, int> mark(camera_id, meta_id);
                marks.insert(mark);
                Appearance new_appearance(crt_path->object_id, appearance.begin_id, idx);
                new_appearance.pids = appearance.pids;
                new_appearance.pids.push_back(crt_pid);

                auto itr = item_map.find(mark);
                if(itr == item_map.end()){
                    item_map.insert(make_pair(
                            mark,
                            make_pair(0, vector<Appearance>{new_appearance})
                    ));
                }
                else itr->second.second.push_back(new_appearance);
            }
        }
        for(auto &mark : marks) item_map.find(mark)->second.first += 1;
    }
public:
    vector<TCS_Path> tcs_paths;

    vector<pair<vector<ll>, vector<Appearance>>>& frequent_sequential_mining(){
        if(!sequential_patterns.empty()) sequential_patterns.clear();

        map<pair<ll, int>, pair<int, vector<Appearance>>> stub_map; //<(camera,tcs_id), (frequent, appearances)>
        for(TCS_Path &path : tcs_paths){
            set<pair<ll, int>> marks;
            for(int idx = 0; idx < path.positions.size(); idx++){
                ll camera_id = path.positions[idx].camera_id;
                int meta_id = path.tcs_ids[idx];
                int pos_id = path.offsets[idx];

                pair<ll, int> mark(camera_id, meta_id);
                marks.insert(mark);
                Appearance new_appearance(path.object_id, idx, idx);
                new_appearance.pids.push_back(pos_id);

                auto itr = stub_map.find(mark);
                if(itr == stub_map.end()){
                    stub_map.insert(make_pair(
                            mark,
                            make_pair(0, vector<Appearance>{new_appearance})
                    ));
                }
                else itr->second.second.push_back(new_appearance);
            }
            for(auto &mark : marks) stub_map.find(mark)->second.first += 1;
        }
        for(auto &entry : stub_map){
            if(entry.second.first >= threshold){
                vector<pair<ll, int>> mark_seq{entry.first};
                pattern_growth(mark_seq, entry.second);
            }
        }

        return sequential_patterns;
    }

    TCS_BIDE(vector<Path> &SDB, int m, int k, int d, double eps): SDB(SDB){
        this->threshold = m;
        this->k = k;
        this->max_gap = d;
        this->eps = eps;
        init_tcs();
    }
};

#endif //BASELINE_FSM_H
