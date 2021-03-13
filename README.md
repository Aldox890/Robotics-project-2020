# Robotics-project
## Description
University project developed using ROS for the course Robotics during accademic year 2019/2020. It requires to read positional data from a rosbag between a car and an obstacle and evaluate the risk of collision.

## ARCHIVE:
### src/
- distance.cpp : service to calculate the distance.
- sub.cpp : node that subscribes to the bag topic and converts from lla into ENU.
- pub.cpp : node that subscribes to the odomCar and odomObs topics; and publishes the custom message to the distanceTopic.

### srv/
- distance.srv : interface of the distance service.

### msg/
- CustomMessage.msg : custom message used to publish final distnance and status_flag info.

### launch/
- launcher.launch: launch file.

## PARAMETER:
The final status flag value depends on min && max params:
```
<param name="min" value="1" type="double" />
<param name="max" value="5" type="double" />
```
Safe > max >= Unsafe >= min > Crash

The enu zero depends on these params:

```
<param name="latitude" value="45" type="double" />
<param name="longitude" value="9" type="double" />
<param name="altitude" value="224" type="double" />
```
## TF TREE:
```
/world
|
|__/car
|
|__/obstacle
```
## CUSTOM MESSAGE STRUCTURE:
CustomMessage.msg:
```
float64 distance
string status_flag
```
## HOW TO START USING THE NODES:
After running roscore: type $roslaunch pub_sub launcher.launch
the launcher will run:
- 2 instances of node subscriber (one for each GPS source)
- the publisher node
- the distance service
## OTHER INFO
The final message is published on topic "distanceTopic".
