#ifndef BASELINE_QUERY_H
#define BASELINE_QUERY_H

#include "Utils/util.h"
#include "Utils/structs.h"
#include "fsm.h"
#include "fsm_verification.h"
#include "graph_miner.h"
#include "deduplication.h"

using namespace std;

class Query{
private:
    vector<Path> travel_paths;

    int get_results_number();
    void print_results();
    void dump_results(string file_name);
public:
    string dataset;
    int m, k, d;
    double eps;
    map<vector<int>, map<vector<ll>, vector<pair<double, double>>>> *ptr_results;// <object_ids: <camera_path: time_intervals>>

    Query(string dataset, int m, int k, int d, double eps){
        this->dataset = std::move(dataset);
        this->m = m; this->k = k; this->d = d;
        this->eps = eps;

        get_paths("datasets/" + this->dataset, travel_paths);
    }
    void mine_baseline();
    void mine_improved();
};

void Query::mine_baseline() {
    cout<<"Baseline("<<m<<","<<k<<","<<d<<","<<eps<<"): ";
    clock_t begin_time = clock();

//    Gap_BIDE fsm(travel_paths, m, d);
//    map<vector<ll>, vector<Appearance>> &seq_map = fsm.frequent_sequential_mining();
//    vector<pair<vector<ll>, vector<Appearance>>> sequences;
//    for(auto &entry : seq_map) sequences.emplace_back(entry.first, entry.second);
    TCS_BIDE fsm(travel_paths, m, k, d, eps);
    vector<pair<vector<ll>, vector<Appearance>>> &sequences = fsm.frequent_sequential_mining();
    cout<<"fsm: "<<static_cast<double>(clock() - begin_time) / CLOCKS_PER_SEC<<"s("<<sequences.size()<<"), ";

    clock_t record_time = clock();
    BIDE_Verifier verifier(travel_paths, m, k, d, eps);
    ptr_results = &verifier.verify(sequences);
    cout<<"verify: "<<static_cast<double>(clock() - record_time) / CLOCKS_PER_SEC<<"s("<<get_results_number()<<"), ";

    record_time = clock();
    Deduplicator deduplicator(*ptr_results);
    deduplicator.deduplicate();
    cout<<"deduplicate: "<<static_cast<double>(clock() - record_time) / CLOCKS_PER_SEC<<"s("<<get_results_number()<<"), ";
    cout<<"total time: "<<static_cast<double>(clock() - begin_time) / CLOCKS_PER_SEC<<endl;

//    print_results();
//    dump_results("results/test0.txt");
}

void Query::mine_improved(){
    cout<<"Improved("<<m<<","<<k<<","<<d<<","<<eps<<"): ";
    clock_t begin_time = clock();

    GraphMiner miner(travel_paths, m, k, d, eps);
    ptr_results = &miner.results;
    miner.run();
    cout<<"mine: "<<static_cast<double>(clock() - begin_time) / CLOCKS_PER_SEC<<"s("<<get_results_number()<<"), ";

    clock_t record_time = clock();
    Deduplicator deduplicator(*ptr_results);
    deduplicator.deduplicate();
    cout<<"deduplicate: "<<static_cast<double>(clock() - record_time) / CLOCKS_PER_SEC<<"s("<<get_results_number()<<"), ";
    cout<<"total time: "<<static_cast<double>(clock() - begin_time) / CLOCKS_PER_SEC<<endl;

//    print_results();
//    dump_results("results/test1.txt");
}

int Query::get_results_number(){
    int numbers = 0;
    for(auto &object_entry : *ptr_results){
        for(auto &path_entry : object_entry.second){
            numbers += static_cast<int>(path_entry.second.size());
        }
    }
    return numbers;
}

void Query::print_results(){
    for(auto &object_entry : *ptr_results){
        auto &object_ids = object_entry.first;
        cout<<"{";
        for(const int &object_id : object_ids){
            cout<<travel_paths[object_id].object_name;
            if(&object_id != &object_ids.back()) cout<<",";
        }
        cout<<"}:[";
        for(auto path_it = object_entry.second.begin(); path_it != object_entry.second.end(); ++path_it){
            for(const ll &camera : path_it->first){
                cout<<camera;
                if(&camera != &path_it->first.back()) cout<<"-";
            }
            cout<<"(";
            for(auto &p : path_it->second){
                cout<<"<"<<p.first<<":"<<p.second<<">";
                if(&p != &path_it->second.back()) cout<<",";
            }
            cout<<")";
            if(next(path_it) != object_entry.second.end()) cout<<", ";
        }
        cout<<"]"<<endl;
    }
}

void Query::dump_results(string file_name){
    ofstream ofs;
    ofs.open(file_name, ios::out);
    ofs<<"Objects,Path"<<endl;
    for(auto &object_entry : *ptr_results){
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

#endif //BASELINE_QUERY_H
