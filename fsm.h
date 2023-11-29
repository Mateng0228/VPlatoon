#ifndef BASELINE_FSM_H
#define BASELINE_FSM_H

#include <vector>
#include <map>
#include <unordered_set>
#include <iostream>
#include "Utils/structs.h"

using namespace std;

struct Appearance{
    int sid; int begin_pid; int end_pid;
    vector<int> pids;
    Appearance(int id, int begin, int end): sid(id), begin_pid(begin), end_pid(end){}
};

class Gap_BIDE{
private:
    vector<Path> &SDB;
    int threshold;
    int max_gap;
    map<vector<int>, vector<Appearance>> sequential_patterns;

    void pattern_growth(vector<int> &sequence, pair<int, vector<Appearance>> &appearances);
    void forward_check(pair<int, vector<Appearance>> &appearances, map<int, pair<int, vector<Appearance>>> &item_map);
    void backward_check(pair<int, vector<Appearance>> &appearances, map<int, pair<int, vector<Appearance>>> &item_map);
public:

    Gap_BIDE(vector<Path> &SDB, int threshold, int max_gap): SDB(SDB){
        this->threshold = threshold;
        this->max_gap = max_gap;
    }

    map<vector<int>, vector<Appearance>>& frequent_sequential_mining(){
        if(!sequential_patterns.empty()) sequential_patterns.clear();

        map<int, pair<int, vector<Appearance>>> stub_map;
        for(Path &path : SDB){
            unordered_set<int> cameras;
            for(int pos_id = 0; pos_id < path.positions.size(); pos_id++){
                int camera_id = path.positions[pos_id].camera_id;
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
                else{
                    itr->second.second.push_back(new_appearance);
                }
            }
            for(int camera_id : cameras) stub_map.find(camera_id)->second.first += 1;
        }
        for(auto &entry : stub_map){
            if(entry.second.first >= threshold){
                vector<int> sequence{entry.first};
                pattern_growth(sequence, entry.second);
            }
        }

        return sequential_patterns;
    }
};

void Gap_BIDE::pattern_growth(vector<int> &sequence, pair<int, vector<Appearance>> &appearances) {
    map<int, pair<int, vector<Appearance>>> backward_items;
    backward_check(appearances, backward_items);
    bool need_prune = false, has_backward_extension = false;
    for(auto &entry : backward_items){
        if(entry.second.second.size() == appearances.second.size()){
            need_prune = true;
            break;
        }
        if(entry.second.first == appearances.first) has_backward_extension = true;
    }
    if(need_prune) return;

    map<int, pair<int, vector<Appearance>>> forward_items;
    forward_check(appearances, forward_items);
    bool has_forward_extension = false;
    for(auto &entry : forward_items){
        if(entry.second.first == appearances.first){
            has_forward_extension = true;
            break;
        }
    }

    // output current pattern if necessary
    if(!has_backward_extension && !has_forward_extension){
        sequential_patterns.insert(make_pair(sequence, appearances.second));

//        cout<<"<";
//        for(int camera_id : sequence) cout<<camera_id<<",";
//        cout<<">:"<<"(";
//        for(Appearance &ap : appearances.second){
//            cout<<"[#"<<ap.sid<<"#";
//            for(int pos : ap.pids) cout<<pos<<",";
//            cout<<"] ";
//        }
//        cout<<")"<<endl;
    }

    // pattern growth
    for(auto &entry : forward_items){
        if(entry.second.first >= threshold){
            vector<int> attached_sequence(sequence);
            attached_sequence.push_back(entry.first);
            pattern_growth(attached_sequence, entry.second);
        }
    }
}

void Gap_BIDE::forward_check(pair<int, vector<Appearance>> &appearances, map<int, pair<int, vector<Appearance>>> &item_map){
    Path *crt_path = nullptr;
    unordered_set<int> cameras;
    for(const Appearance &appearance : appearances.second){
        if(crt_path == nullptr) crt_path = &SDB[appearance.sid];
        else if(crt_path != &SDB[appearance.sid]){
            crt_path = &SDB[appearance.sid];
            for(int camera_id : cameras) item_map.find(camera_id)->second.first += 1;
            cameras.clear();
        }
        int path_length = crt_path->positions.size();
        for(int pos_id = appearance.end_pid + 1; pos_id <= min(appearance.end_pid + 1 + max_gap, path_length - 1); pos_id++){
            int camera_id = crt_path->positions[pos_id].camera_id;
            cameras.insert(camera_id);
            Appearance new_appearance(crt_path->object_id, appearance.begin_pid, pos_id);
            new_appearance.pids = appearance.pids;
            new_appearance.pids.push_back(pos_id);

            auto itr = item_map.find(camera_id);
            if(itr == item_map.end()){
                item_map.insert(make_pair(
                        camera_id,
                        make_pair(0, vector<Appearance>{new_appearance})
                ));
            }
            else{
                itr->second.second.push_back(new_appearance);
            }
        }
    }
    for(int camera_id : cameras) item_map.find(camera_id)->second.first += 1;
}

void Gap_BIDE::backward_check(pair<int, vector<Appearance>> &appearances, map<int, pair<int, vector<Appearance>>> &item_map) {
    Path *crt_path = nullptr;
    unordered_set<int> cameras;
    for(Appearance &appearance : appearances.second){
        if(crt_path == nullptr) crt_path = &SDB[appearance.sid];
        else if(crt_path != &SDB[appearance.sid]){
            crt_path = &SDB[appearance.sid];
            for(int camera_id : cameras) item_map.find(camera_id)->second.first += 1;
            cameras.clear();
        }
        for(int pos_id = appearance.begin_pid - 1; pos_id >= max(appearance.begin_pid - 1 - max_gap, 0); pos_id--){
            int camera_id = crt_path->positions[pos_id].camera_id;
            cameras.insert(camera_id);
            auto itr = item_map.find(camera_id);
            if(itr == item_map.end()){
                item_map.insert(make_pair(
                        camera_id,
                        make_pair(0, vector<Appearance>{Appearance(crt_path->object_id, pos_id, appearance.end_pid)})
                ));
            }
            else{
                itr->second.second.emplace_back(crt_path->object_id, pos_id, appearance.end_pid);
            }
        }
    }
    for(int camera_id : cameras) item_map.find(camera_id)->second.first += 1;
}


#endif //BASELINE_FSM_H
