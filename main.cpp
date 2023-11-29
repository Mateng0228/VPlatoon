#include <iostream>
#include "query.h"

using namespace std;

int main(int argc, char *argv[]){
    Query query(string(argv[1]), stoi(argv[2]), stoi(argv[3]), stoi(argv[4]), stod(argv[5]));
    query.mine();

    return 0;
}