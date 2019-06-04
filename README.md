# advanced-robotics

## Phase 2
### How to run this code
- Just run `python motionPlanning.py` and follow the instructions.

A sample screenshot can be found in `result.JPG`.
Five sample figures can also be found in this folder.

### How it works
- We decompose the space based on the obstacle inputs.
We consider the drone as a point of the center of mass.
The C_obs is augmented to all directions by half of the body size of the drone.
- For path finding problem, we first use the roadmap algorithm to find the nodes,
and then pass to A* algorithm.
The output of A* algorithm should be a feasible path,
which is shown as green line in figure 1.
- Then we generate the trajectory by 0-order polynomial planning (evenly spaced).
- Finally pass the trajectory into the PD controller we designed last time.
We give the timing together for (4) and (5), since the step (4) does not need extra time.

## Phase 1
### How to run this code
1. Please copy this folder to your ROS workspace (e.g., catkin_ws/src/this_folder) and `catkin_make` your workspace.
2. Run `rospack profile` just in case ROS cannot find the path of this package.
3. Run `roslaunch advanced-robotics run.launch` and follow the instructions on your terminal.

### Software Framework
This is a ROS package that contains three ROS nodes: `main.py`, `model2D.py` and `model3D.py`. After launching, `main.py` will request from user the problem dimensionality and the appropriate inputs, and then publish this information to either `model2D.py` or `model3D.py` depending on user inputs. Then one of the model scripts will run the simulation and show the results.

### Design Idea
For 2D case, we use the same linearized model as the one proposed in class. For 3D case, we use the 12 dimensional nonlinear model. PD controllers are used in both cases. All the PD parameters are tuned by ourselves. More details can be found in the report and please find it in this folder. The report contains personal information and hence has not been uploaded on Github.
