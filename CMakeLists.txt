cmake_minimum_required(VERSION 3.22)
project(Baseline)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(Baseline main.cpp Utils/util.h Utils/structs.h query.h fsm.h fsm_verification.h Test.h graph_miner.h graph_miner.cpp)
