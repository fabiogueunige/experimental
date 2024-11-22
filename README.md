# Installation 

For installation you need to have this package on the pc:

```bash
git clone https://github.com/fabiogueunige/experimental.git
git clone https://github.com/CarmineD8/ros2_aruco.git
```

``` bash
apt install ros-foxy-gazebo-ros-pkgs
apt install ros-foxy-joint-state-publisher ros-foxy-joint-state-publisher-gui
apt install ros-foxy-xacro
```

after for launch it run on terminal 
``` bash
ros2 launch robot_urdf gazebo_circle.launch.py
```

## for update the packages needed

``` bash
sudo apt-get update
sudo apt-get upgrade
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys E88979FB9B30ACF2
sudo apt-get update
sudo apt-get install ros-foxy-control*
sudo apt-get install ros-foxy-ros-control*
sudo apt-get install ros-foxy-gazebo*
```


