# rovi-project

## to run part 1
cmake in SamplePlugin/build 
make in build

run ./planning           (to run motion planning and vision.)

The reachability analysis can be found in the src/ folder in the root directory. The algorithm was implemented as a class in the following files: reachabilityAnalyzer.hpp, reachabilityAnalyzer.cpp and reachabilityAnalysis.cpp (contains the main loop).
The planning part of the code is in SamplePlugin/src/planning.cpp
M2 can be found in src/M2 simulated depth sensor.py
M3 vision code is in SamplePlugin/src/stereo.cpp

