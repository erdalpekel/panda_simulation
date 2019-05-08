# panda_simulation

This package was written for ROS melodic running under Ubuntu 18.04. Run the following commands to make sure that all additional packages are installed:

```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/erdalpekel/panda_simulation.git
git clone https://github.com/erdalpekel/panda_moveit_config.git
git clone --branch simulation https://github.com/erdalpekel/franka_ros.git
cd ..
sudo apt-get install libboost-filesystem-dev
rosdep install --from-paths src --ignore-src -y --skip-keys libfranka --skip-keys moveit_perception
cd ..
```
It is also important that you build the *libfranka* library from source and pass its directory to *catkin_make*  when building this ROS package as described in [this tutorial](https://frankaemika.github.io/docs/installation.html#building-from-source).

Currently it includes a controller parameter config file and a launch file to launch the [Gazebo](http://gazebosim.org) simulation environment and the Panda robot from FRANKA EMIKA in it with the necessary controllers.

Build the catkin workspace and run the simulation:
```
catkin_make -j4 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/path/to/libfranka/build
source devel/setup.bash
roslaunch panda_simulation simulation.launch
```

Depending on your operating systems language you might need to export the numeric type so that rviz can read the floating point numbers in the robot model correctly:

```
export LC_NUMERIC="en_US.UTF-8"
```
Otherwise, the robot will appear in rviz in a collapsed state.


You can see the full explanation in my [blog post](https://erdalpekel.de/?p=55).

## Extension: _MoveIt!_ constraint-aware planning

This repository was extended with a ROS node that communicates with the _MoveIt!_ Planning Scene API. It makes sure that the motion planning pipeline avoids collision objects in the environment specified by the user in a separate directory (`~/.panda_simulation`) as _JSON_ files. You can read more about it in the accompanying [blog post](https://erdalpekel.de/?p=123).

## Extension: Publishing a box at Panda's hand in _Gazebo_

This repository was extended with a node that publishes a simple box object in the _Gazebo_ simulation at the hand of the robot. The position of this box will get updated as soon as the robot moves. You can read more about it in the accompanying [blog post](https://erdalpekel.de/?p=178).