# unit04_unity
[![unit04_test](.image/unit04_test.gif)](https://www.youtube.com/watch?v=C1V_L85p0-I)  
Unit04 model on Unity.
It communicate sensor info and drive command via ROS using ROS#.  
The following sensors are added.

- Velodyne Pack (VLP16)
- IMU
- RGB Camera

# How to use
## 1. Launch ROS packages
#### 1-1 rosbridge
Launch the `rosbridge` with following command.
```bash
$ roslaunch rosbridge_server rosbridge_websocket.launch address:=localhost
```
#### 1-2 velodyne_pointcloud
Launch the `velodyne_pointcloud` package with following launch file.  
Please create launch file by copy and paste following script.
```xml
<launch>
  <arg name="calibration" default="$(find velodyne_pointcloud)/params/VLP16db.yaml" />
  <arg name="manager" default="velodyne_pointcloud" />
  <arg name="max_range" default="100.0" />
  <arg name="min_range" default="0.9" />

  <node pkg="velodyne_pointcloud" type="cloud_node" name="$(arg manager)">
    <param name="model" value="VLP16"/>
    <param name="calibration" value="$(arg calibration)"/>
    <param name="max_range" value="$(arg max_range)"/>
    <param name="min_range" value="$(arg min_range)"/>
    <!-- <param name="view_direction" value="0"/> -->
    <!-- <param name="view_width" value="360"/> -->
  </node>
</launch>
```

        
## 2. Clone and run on Unity
#### 2-1
Just to clone this repo with `git clone` command.
#### 2-2
Then, open the project file with UnityHub.
#### 2-3
Finally, RUN the scene file named `unit04_test`.

# LICENSE
Copyright [2020] Ryodo Tanaka groadpg@gmail.com

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.

## Dependencies
- [RosSharp](https://github.com/siemens/ros-sharp) (Apache2.0 License)
