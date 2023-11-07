# ros_decawave
Positioning System based on Decawave's DWM1001 Ultra Wide Band transceivers.
For more informations see [docs](https://github.com/verlab/ros_decawave/tree/master/docs/DWM1001_DWM1001-DEV_MDEK1001_Sources_and_Docs_v8).

### Download and Compilation
Download this repo into the ros workspace and compile:
```
git clone https://github.com/wust-dcr/decawave_ros src/
rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
colcon build
```

### Using
Run the launch:
```
ros2 launch decawave_driver decawave.launch.py
```
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/ieaP79FDLC0/0.jpg)](https://www.youtube.com/watch?v=ieaP79FDLC0)

### Topics
```
$ rostopic list
/pose # tag position
/rosout
/rosout_agg
/status # anchors status
/syscommand
/tf
/tf_static
/trajectory # tag path
```
