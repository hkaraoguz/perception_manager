Perception Manager Package
==========================

Requirements
-----------
* Kinect like RGB-D sensor
* pcl
* OpenCV
* [ROS CPP Utils package](https://github.com/hkaraoguz/ros_cpp_utils.git)


This package acts as a middleware between the lower level perception sources and the higher-level control/decision software. It can be used to filter out the noisy data from the low-level perception sources. Depending on the sensor modality, it can output the object locations and sizes w.r.t the robot coordinate frame. 

Running it with Yumi Pedestal Setup
-----------------
For using the node on the Yumi pedestal setup with the color segmentation modality, the following command is used:
```
roslaunch perception_manager yumi_pedestal.launch
```
### Query Objects service
The objects perceived within the workspace can be queried using **perception_manager/query_objects** service.
