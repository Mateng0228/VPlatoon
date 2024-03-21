cmake_minimum_required(VERSION 3.22)
project(Experiment)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(Experiment main.cpp Utils/util.h Utils/structs.h query.h fsm.h fsm_verification.h graph_miner.h graph_miner.cpp deduplication.h)
