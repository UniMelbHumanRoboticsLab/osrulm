# Installation

## OpenSIM
The project can be used with OpenSIM 4.xx (tested with 4.5) which can be found at https://simtk.org/frs/?group_id=91. 


## Visual Studio
The project uses Visual Studio to develop and compile. It has been tested with Visual Studio 2017? and 2019??. 
As of 2024, VS 2017 can be downloaded from here [https://download.visualstudio.microsoft.com/download/pr/8729ca3d-c3b2-4b32-b6fb-a7ea468a4af2/4448a86b1ae7d5b90bdc9c51e3f18b8f6ab0d3176560aa23b03f102380e02746/vs_Community.exe](https://download.visualstudio.microsoft.com/download/pr/8729ca3d-c3b2-4b32-b6fb-a7ea468a4af2/4448a86b1ae7d5b90bdc9c51e3f18b8f6ab0d3176560aa23b03f102380e02746/vs_Community.exe).


## CMake
The project relies on cmake (>3.2) to generate a valid Visual Studio project which can then be compiled.

## How to build

In main folder:
```
mkdir build
cd build
cmake  -DCMAKE_GENERATOR_PLATFORM=x64 ..
```
(specify an x64 build to match OpenSIM version)

This generates a VS solution OSimTrial in build folder. Open the solution, set OSimTrial as Startup solution and build.


## Upper-limb model

This requires the MoBL-ARMS Dynamic Upper Limb model from https://simtk.org/projects/upexdyn.




Go back to [README](./README.md).