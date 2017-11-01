Perception Manager Package
==========================

Requirements
-----------
* Kinect like RGB-D sensor
* [CPP Utils package](git@gits-15.sys.kth.se:FACT/cpp_utils.git)


This package acts as a middleware between the lower level perception sources and the higher-level control/decision software. It can be used to filter out the noisy data from the low-level perception sources. For vision based sources it can use the depth information from the camera to calculate the metric locations.

Static Yumi Setup
-----------------
For using the node at the static Yumi setup with the color segmentation modality, the following command is used:
```
roslaunch perception_manager yumi_fact.launch
```
### Query Objects service
The segments perceived within the workspace can be queried using **perception_manager/query_objects** service.
