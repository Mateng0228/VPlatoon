<h2 align="center">Mining Platoon Patterns from Traffic Videos</h2>

This project provides the source code for our paper  "Mining Platoon Patterns from Traffic Videos".

## Requirements
- **Operating System:** Linux
- **C++ Standard Requirement:** requires a minimum of C++11 for compatibility with features like constexpr and variadic templates. 
- **Compiler Requirements:** GCC 11.4 or later (for support of GCC-specific extensions)
- **Build Tools Requirement:**
  - **CMake:** Version 3.22 or later is required for configuring the build process.
  - **Make:** A compatible version 4.3 of the 'make' tool is required to build the project.

## Installation
Before starting, please ensure that you have installed the necessary C++ runtime library and met all the requirements outlined in the previous section.

1. **Clone the repository to your local machine:**
```shell
git clone https://github.com/Mateng0228/VPlatoon.git
```
2. **Change to the directory of this project:**
```shell
cd VPlatoon
```
3. **Create a 'build' directory and change to it:**
```shell
mkdir build
cd build
```
4. **Compile current project using CMake and Make:**
```shell
cmake ..
make
```

## Usage
You can now find and quickly run the final executable file in the corresponding "build" directory following the instructions below:
```shell
YourExecutableFile "dataset-name" "m" "k" "d" "epsilon"
```
- Note that you must switch to the root directory before running the executable file (due to relative path addressing in the code). If the executable file is located in the corresponding "build" directory, it means you should run the program using the format of "build/file-name".
- Please ensure that the "datasets" directory contains the "dataset-name" directory corresponding to your dataset. You can download all datasets used in our paper from the link provided in "datasets/Download_Source.txt".
- The meanings of the parameters "m", "k", "d", and "eps" are consistent with those in the paper.
- For more detailed and customized execution, please refer to the "main.cpp" source file and modify it as needed.

## Supplementary Artifacts
We also provide supplementary codes and artifacts related to the scalability and effectiveness analysis in our paper.
1. [CityFlowV2](https://www.aicitychallenge.org/2022-data-and-evaluation/): The original video dataset of our recovered trjectory dataset "CityFlow".
2. [TCS-tree](https://github.com/Mateng0228/Co-movement-Pattern-Mining-from-Videos): The state-of-the-art VConvoy (previous video-based co-movement pattern) mining algorithm.
3. Multi-camera object tracking algorithms from AI City Challenge [2020](https://github.com/KevinQian97/ELECTRICITY-MTMC) and [2022](https://github.com/Yejin0111/AICITY2022-Track1-MTMC): These are used for trajectory recovery.
4. [TMerge](https://ieeexplore.ieee.org/abstract/document/10184538/): The track merging algorithm used to refine recovered trajectories in the effectiveness experiment.
5. [motmetrics](https://github.com/cheind/py-motmetrics): A library for addressing IDF-related issues.


## Contact
If you encounter any problems, please contact the code maintainer at [mt0228@zju.edu.cn](mailto:mt0228@zju.edu.cn). Please note that simply raising "Issues" in github may not always be the most effective way to get our attention.

## License
The source code is released under the MIT License. See the [LICENSE](./LICENSE) file for details.  
The associated technical report is available on arXiv under a perpetual, non-exclusive license: [arXiv:2412.20177](https://arxiv.org/abs/2412.20177).
