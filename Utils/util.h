#ifndef BASELINE_UTIL_H
#define BASELINE_UTIL_H

#include <string>
#include <vector>
#include <dirent.h>
#include <cstring>
#include <map>
#include <iostream>
#include <fstream>
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
            int camera_id = stoi(part1);
            double begin_time = stod(part2), end_time = stod(part3);
            positions.emplace_back(camera_id, begin_time, end_time);
        }
        fin.close();

        paths.emplace_back(idx, object_name, positions);
    }
}

#endif //BASELINE_UTIL_H
