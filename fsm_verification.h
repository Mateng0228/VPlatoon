#ifndef BASELINE_FSM_VERIFICATION_H
#define BASELINE_FSM_VERIFICATION_H

#include <algorithm>
#include <set>
#include <unordered_map>
#include <climits>
//#include "fsm.h"

class BIDE_Verifier{
public:
    int m, k, d;
    double eps;
    vector<Path> &SDB;
    map<vector<int>, map<vector<ll>, vector<pair<double, double>>>> results;

    BIDE_Verifier(vector<Path> &SDB, int m, int k, int d, double eps):SDB(SDB){
        this->m = m; this->k = k; this->d = d;
        this->eps = eps;
        p_camera_path = nullptr;
        p_appearances = nullptr;
        p_cluster_lst = nullptr;
        p_inversions_lst = nullptr;
    }

    map<vector<int>, map<vector<ll>, vector<pair<double, double>>>>& verify(vector<pair<vector<ll>, vector<Appearance>>> &sequences){
        if(!results.empty()) results.clear();

        for(auto &seq_entry : sequences){
            const vector<ll> &camera_path = seq_entry.first;
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
            for(int camera_idx = 0; camera_idx < camera_path.size(); ++camera_idx){
                vector<tuple<int, int, int>> positions;
                for(int ap_id = 0; ap_id < appearances.size(); ++ap_id)
                    positions.emplace_back(ap_id, appearances[ap_id].sid, appearances[ap_id].pids[camera_idx]);
                clusters_lst.emplace_back();
                vector<pair<int, int>> &clusters = clusters_lst.back();
                inversions_lst.emplace_back();
                vector<pair<int, pair<int, int>>> &inversions = inversions_lst.back();

                temporal_cluster(positions, clusters, inversions);
            }
            // get all clusters in first camera
            vector<int> ap_ids;
            for(int ap_id = 0; ap_id < appearances.size(); ++ap_id) ap_ids.push_back(ap_id);
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

        return results;
    }

private:
    const vector<ll> *p_camera_path;
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

        // insert into ptr_results
        vector<int> object_ids;
        for(auto &entry : oid2aids) object_ids.push_back(entry.first);
        vector<ll> cameras;
        for(int camera_id : camera_ids) cameras.push_back(p_camera_path->at(camera_id));
        auto object_it = results.find(object_ids);
        if(object_it == results.end())
            results[object_ids] = {{cameras, time_intervals}};
        else{
            auto &path_map = object_it->second;
            auto path_it = path_map.find(cameras);
            if(path_it == path_map.end())
                path_map[cameras] = time_intervals;
            else
                path_it->second.insert(path_it->second.end(), time_intervals.begin(), time_intervals.end());
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
        for(int idx = 0; idx < positions.size(); ++idx) inversions.emplace_back(-1, make_pair(-1, -1));
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

};

#endif //BASELINE_FSM_VERIFICATION_H