# Highway driving project

In this project, the ego-vehicle will drive the highway with speed-limit. To complete the track faster, the ego-vehicle attempts to change lanes if the front vehicle is slow. To do that, this project composed of two steps: behavior planning, trajectory planning. 

 First, behavior planner checks the right or left lane is fine.

 Second, if the lane is fine, trajectory planner will generate the spline based trajectory.

This project is one of the [udacity self-driving car course projects](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013).  

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.
5. Move to simulator directory: `../simulator`.
6. Run simulator: `./sim.x86_64`.


## Simulator
To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```
More details in this link
[releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  




## Dependencies

* Linux 16.04 LTS

* cmake >= 3.5

* make >= 4.1
  
  * Make is installed by default on most Linux distros
  
* gcc/g++ >= 5.4
  
  * gcc / g++ is installed by default on most Linux distros
  
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Result

![](video/LaneChange_9.gif)
