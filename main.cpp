#include <iostream>
#include "query.h"

using namespace std;

int main(int argc, char *argv[]){
    string dataset = string(argv[1]);
    int m = stoi(argv[2]);
    int k = stoi(argv[3]);
    int d = stoi(argv[4]);
    double eps = stod(argv[5]);
    Query query(dataset, m, k, d, eps);
    query.mine();

    return 0;
}