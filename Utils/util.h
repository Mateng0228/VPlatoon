#ifndef BASELINE_UTIL_H
#define BASELINE_UTIL_H

#include <dirent.h>
#include <fstream>
#include <set>
#include <map>
#include "structs.h"

using namespace std;

// get travel path of each object from given folder
void get_paths(string path, vector<Path>& paths) {
    vector<string> file_names;
    DIR *pDir;
    struct dirent* ptr;
    if(!(pDir = opendir(path.c_str()))) return;
    while((ptr = readdir(pDir)) != nullptr) {
        if(strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
            file_names.emplace_back(ptr->d_name);
    }
    closedir(pDir);

    for(int idx = 0;idx < file_names.size();idx++){
        string &file_name = file_names[idx];
        string file_path = path + '/' + file_names[idx];
        string object_name = file_name.substr(0, file_name.find('.'));
        vector<Position> positions;

        ifstream fin;
        fin.open(file_path, ios::in);
        string buff;
        while(getline(fin, buff)){
            int cut1 = -1, cut2 = -1;
            for(int i = 0; i<buff.size(); i++){
                if(buff[i] == ','){
                    if(cut1 == -1) cut1 = i;
                    else if(cut2 == -1) cut2 = i;
                }
            }
            string part1 = buff.substr(0, cut1);
            string part2 = buff.substr(cut1 + 1, cut2 - cut1 - 1);
            string part3 = buff.substr(cut2 + 1, buff.size() - cut2 - 1);
            ll camera_id = stoll(part1);
            double begin_time = stod(part2), end_time = stod(part3);
            positions.emplace_back(camera_id, begin_time, end_time);
        }
        fin.close();

        paths.emplace_back(idx, object_name, positions);
    }
}

void check(string path1, string path2){
    map<set<int>, set<vector<int>>> convoy1;
    ifstream fin1;
    fin1.open(path1, ios::in);
    string buffer;
    getline(fin1, buffer);
    while(getline(fin1, buffer)){
        set<int> objects;
        vector<int> sequence;
        int pos = buffer.find(',');
        string object_str = buffer.substr(0, pos);
        string sequence_str = buffer.substr(pos + 1, buffer.length() - 1 - pos);
        object_str += " ";
        int left = 0, right = object_str.find(' ', 0);
        while(right != std::string::npos){
            objects.insert(stoi(object_str.substr(left, right - left)));
            left = right + 1;
            right = object_str.find(' ', left);
        }
        sequence_str += " ";
        left = 0, right = sequence_str.find(' ', 0);
        while(right != std::string::npos){
            sequence.push_back(stoi(sequence_str.substr(left, right - left)));
            left = right + 1;
            right = sequence_str.find(' ', left);
        }
        if(convoy1.find(objects) == convoy1.end()) convoy1[objects] = set<vector<int>>();
        convoy1[objects].insert(sequence);
    }
    fin1.close();

    map<set<int>, set<vector<int>>> convoy2;
    ifstream fin2;
    fin2.open(path2, ios::in);
    getline(fin2, buffer);
    while(getline(fin2, buffer)){
        set<int> objects;
        vector<int> sequence;
        int pos = buffer.find(',');
        string object_str = buffer.substr(0, pos);
        string sequence_str = buffer.substr(pos + 1, buffer.length() - 1 - pos);
        object_str += " ";
        int left = 0, right = object_str.find(' ', 0);
        while(right != std::string::npos){
            objects.insert(stoi(object_str.substr(left, right - left)));
            left = right + 1;
            right = object_str.find(' ', left);
        }
        sequence_str += " ";
        left = 0, right = sequence_str.find(' ', 0);
        while(right != std::string::npos){
            sequence.push_back(stoi(sequence_str.substr(left, right - left)));
            left = right + 1;
            right = sequence_str.find(' ', left);
        }
        if(convoy2.find(objects) == convoy2.end()) convoy2[objects] = set<vector<int>>();
        convoy2[objects].insert(sequence);
    }
    fin2.close();

    map<set<int>, set<vector<int>>> diff_map1, diff_map2;
    for(auto &p : convoy1){
        const set<int>& objects1 = p.first;
        if(convoy2.find(objects1) == convoy2.end()) diff_map1[objects1] = p.second;
        else{
            set<vector<int>>& sequence_list1 = p.second;
            set<vector<int>>& sequence_list2 = convoy2[objects1];
            for(const vector<int>& sequence1 : sequence_list1){
                if(sequence_list2.find(sequence1) == sequence_list2.end()){
                    if(diff_map1.find(objects1) == diff_map1.end()) diff_map1[objects1] = set<vector<int>>();
                    diff_map1[objects1].insert(sequence1);
                }
                else sequence_list2.erase(sequence1);
            }
            if(!sequence_list2.empty()) diff_map2[objects1] = sequence_list2;
        }
        convoy2.erase(objects1);
    }
    diff_map2.insert(convoy2.begin(), convoy2.end());

    cout<<"convoy1 differences:--------------------"<<endl;
    int counts = 0;
    for(auto &p : diff_map1){
        cout<<"{";
        for(int object : p.first) cout<<object<<",";
        cout<<"} : [";
        for(const vector<int>& sequence : p.second){
            counts++;
            for(int item : sequence) cout<<item<<"-";
            cout<<", ";
        }
        cout<<"]"<<endl;
    }
    cout<<counts<<endl;
    counts = 0;
    cout<<"convoy2 differences:--------------------"<<endl;
    for(auto &p : diff_map2){
        cout<<"{";
        for(int object : p.first) cout<<object<<",";
        cout<<"} : [";
        for(const vector<int>& sequence : p.second){
            counts++;
            for(int item : sequence) cout<<item<<"-";
            cout<<", ";
        }
        cout<<"]"<<endl;
    }
    cout<<counts<<endl;
}

#endif //BASELINE_UTIL_H
