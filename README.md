## Prequisites needed

``` bash
sudo apt-get update
sudo apt-get upgrade
```
If return some errors run this command, else skip and run the next
```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys E88979FB9B30ACF2
sudo apt-get update
```

```bash
sudo apt-get install ros-foxy-control*
sudo apt-get install ros-foxy-ros-control*
sudo apt-get install ros-foxy-gazebo*
```

``` bash
apt install ros-foxy-gazebo-ros-pkgs
apt install ros-foxy-joint-state-publisher ros-foxy-joint-state-publisher-gui
apt install ros-foxy-xacro
```

# Installation 

Inside the ros 2 workspace clone this two package

```bash
git clone https://github.com/fabiogueunige/experimental.git
git clone https://github.com/CarmineD8/ros2_aruco.git
```

after for launch it run on terminal 
``` bash
ros2 launch robot_urdf gazebo_circle.launch.py
```




