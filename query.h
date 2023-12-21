#ifndef BASELINE_QUERY_H
#define BASELINE_QUERY_H

#include "Utils/util.h"
#include "Utils/structs.h"
#include "fsm.h"
#include "fsm_verification.h"

using namespace std;

class Query{
private:
    vector<Path> travel_paths;

    void print_results(map<vector<int>, map<vector<int>, vector<pair<double, double>>>> &results);
    void dump_results(map<vector<int>, map<vector<int>, vector<pair<double, double>>>> &results);
public:
    string dataset;
    int m, k, d;
    double eps;

    Query(string dataset, int m, int k, int d, double eps){
        this->dataset = std::move(dataset);
        this->m = m; this->k = k; this->d = d;
        this->eps = eps;

        get_paths("datasets/" + this->dataset, travel_paths);
    }
    void mine();
};

void Query::mine() {
    cout<<"("<<m<<","<<k<<","<<d<<","<<eps<<") ";
    clock_t begin_time = clock();

//    Gap_BIDE fsm(travel_paths, m, d);
//    map<vector<int>, vector<Appearance>> &seq_map = fsm.frequent_sequential_mining();
//    vector<pair<vector<int>, vector<Appearance>>> sequences;
//    for(auto &entry : seq_map) sequences.emplace_back(entry.first, entry.second);
    TCS_BIDE fsm(travel_paths, m, k, d, eps);
    vector<pair<vector<int>, vector<Appearance>>> &sequences = fsm.frequent_sequential_mining();
    cout<<"fsm: "<<static_cast<double>(clock() - begin_time) / CLOCKS_PER_SEC<<"s("<<sequences.size()<<"), ";

    clock_t record_time = clock();
    BIDE_Verifier verifier(travel_paths, m, k, d, eps);
    verifier.verify(sequences);
    int numbers = 0;
    for(auto &object_entry : verifier.results){
        for(auto &path_entry : object_entry.second){
            numbers += static_cast<int>(path_entry.second.size());
        }
    }
    cout<<"verify: "<<static_cast<double>(clock() - record_time) / CLOCKS_PER_SEC<<"s("<<numbers<<"), ";

    record_time = clock();
    verifier.deduplicate();
    cout<<"deduplicate: "<<static_cast<double>(clock() - record_time) / CLOCKS_PER_SEC<<", ";

    map<vector<int>, map<vector<int>, vector<pair<double, double>>>> &results = verifier.results;
    numbers = 0;
    for(auto &object_entry : results){
        for(auto &path_entry : object_entry.second){
            numbers += static_cast<int>(path_entry.second.size());
        }
    }
    cout<<"numbers: "<<numbers<<",";
    cout<<"total time: "<<static_cast<double>(clock() - begin_time) / CLOCKS_PER_SEC<<endl;

}

void Query::print_results(map<vector<int>, map<vector<int>, vector<pair<double, double>>>> &results){
    for(auto &object_entry : results){
        auto &object_ids = object_entry.first;
        cout<<"{";
        for(const int &object_id : object_ids){
            cout<<travel_paths[object_id].object_name;
            if(&object_id != &object_ids.back()) cout<<",";
        }
        cout<<"}:[";
        for(auto &path_entry : object_entry.second){
            for(const int &camera : path_entry.first){
                cout<<camera;
                if(&camera != &path_entry.first.back()) cout<<"-";
            }
            cout<<"(";
            for(auto &p : path_entry.second){
                cout<<"<"<<p.first<<":"<<p.second<<">,";
            }
            cout<<"), ";
        }
        cout<<"]"<<endl;
    }
}

void Query::dump_results(map<vector<int>, map<vector<int>, vector<pair<double, double>>>> &results){
    ofstream ofs;
    ofs.open("test.txt", ios::out);
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
            for(const int &camera : path_entry.first){
                s_camera += to_string(camera);
                if(&camera != &path_entry.first.back()) s_camera += " ";
                else s_camera += "\n";
            }
            ofs<<s_obj<<s_camera;
        }
    }
    ofs.close();
}

#endif //BASELINE_QUERY_H
