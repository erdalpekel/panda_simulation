# panda_simulation

This package was built for ROS melodic running under Ubuntu 18.04. Run the following command to make sure that all additional packages are installed:

```
sudo apt-get install ros-melodic-moveit-ros-move-group ros-melodic-controller-manager* ros-melodic-moveit* ros-melodic-effort-controllers ros-melodic-joint-trajectory-controller ros-melodic-gazebo-ros* ros-melodic-rviz*
```
It is also important that you build the *libfranka* library from source and pass its directory to *catkin_make*  when building this ROS package as described in [this tutorial](https://frankaemika.github.io/docs/installation.html#building-from-source).

Currently it includes a controller parameter config file and a launch file to launch the [Gazebo](http://gazebosim.org) simulation environment and the Panda robot from FRANKA EMIKA in it with the necessary controllers.

Build the catkin workspace and run the simulation:
```
catkin_make -j4 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/path/to/libfranka/build
source devel/setup.bash
roslaunch panda_simulation simulation.launch
```



You can see the full explanation in my [blog post](https://erdalpekel.de/?p=55).
