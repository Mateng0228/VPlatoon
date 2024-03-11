#include <iostream>
#include "query.h"
#include "graph_miner.h"

using namespace std;

int main(int argc, char *argv[]){
    string dataset = string(argv[1]);
    int m = stoi(argv[2]);
    int k = stoi(argv[3]);
    int d = stoi(argv[4]);
    double eps = stod(argv[5]);
//    Query query(dataset, m, k, d, eps);
//    query.mine_baseline();

    clock_t begin_time = clock();
    vector<Path> SDB;
    get_paths("datasets/" + dataset, SDB);
    clock_t record_time = clock();
    GraphMiner miner(SDB, m, k, d, eps);
    map<vector<int>, map<vector<ll>, vector<pair<double, double>>>> &results = miner.results;
    miner.run();
    int numbers = 0;
    for(auto &object_entry : results){
        for(auto &path_entry : object_entry.second){
            numbers += static_cast<int>(path_entry.second.size());
        }
    }
    cout<<"main-part: "<<static_cast<double>(clock() - record_time) / CLOCKS_PER_SEC<<"s("<<numbers<<"), ";

    record_time = clock();
    numbers = 0;
    miner.deduplicate();
    for(auto &object_entry : results){
        for(auto &path_entry : object_entry.second){
            numbers += static_cast<int>(path_entry.second.size());
        }
    }
    cout<<"deduplicate: "<<static_cast<double>(clock() - record_time) / CLOCKS_PER_SEC<<"s("<<numbers<<"), ";
    cout<<"total time: "<<static_cast<double>(clock() - begin_time) / CLOCKS_PER_SEC<<endl;
//    miner.print_results();
//    miner.dump_results("results/test1.txt");

//    check("results/test0.txt", "results/test1.txt");

    return 0;
}