# advanced-robotics
### How to run this code
1. Please copy this folder to your ROS workspace (e.g., catkin_ws/src/this_folder) and `catkin_make` your workspace.
2. Run `rospack profile` just in case ROS cannot find the path of this package.
3. Run `roslaunch advanced-robotics run.launch` and follow the instructions on your terminal.

### Software Framework
This is a ROS package that contains three ROS nodes: `main.py`, `model2D.py` and `model3D.py`. After launching, `main.py` will request from user the problem dimensionality and the appropriate inputs, and then publish this information to either `model2D.py` or `model3D.py` depending on user inputs. Then one of the model scripts will run the simulation and show the results.

### Design Idea
For 2D case, we use the same linearized model as the one proposed in class. For 3D case, we use the 12 dimensional nonlinear model. PD controllers are used in both cases. All the PD parameters are tuned by ourselves. More details can be found in the report and please find it in this folder. The report contains personal information and hence has not been uploaded on Github.
