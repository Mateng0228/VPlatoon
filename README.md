<h2 align="center">Mining Platoon Patterns from Traffic Videos</h2>

This project provides the source code for our paper  "Mining Platoon Patterns from Traffic Videos".

## Requirements
- **Operating System:** Linux
- **C++ Standard Requirement:** requires a minimum of C++11 for compatibility with features like constexpr and variadic templates. 
- **Compiler Requirements:** GCC 11.4 or later (for support of GCC-specific extensions)
- **Build Tools Requirement:**
  - **CMake:** Version 3.22 or later is required for configuring the build process.
  - **Make:** A compatible version 4.3 of the 'make' tool is required to build the project.

## Installation & Usage
> Before starting, please ensure that you have installed and met all the requirements mentioned in the previous section.

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
5. **Run the executable file:**
```shell
Experiment "dataset-name" "m" "k" "d" "epsilon"
``` 
