# unit04_unity
[![unit04_test](.image/unit04_test.gif)](https://www.youtube.com/watch?v=C1V_L85p0-I)  
Unit04 model on Unity.
It communicate sensor info and drive command via ROS using ROS#.  
The following sensors are added.

- Velodyne Pack (VLP16)
- IMU
- RGB Camera

# How to use
## 1. Launch rosbridge
    Launch the `rosbridge` with following command.
    
```bash
$ roslaunch rosbridge_server rosbridge_websocket.launch address:=localhost
```
        
## 2. Clone and run on Unity
#### 2-1
    Just to clone this repo with `git clone` command.
#### 2-2
    Then, open the project file with UnityHub.
#### 2-3
    Finally, RUN the scene file named `unit04_test`.
    
