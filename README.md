# Introducion

This repository includes code from the National Technology Olympiad. 
In this track, our task was to build a system for delivering goods to a potential city. 
Equipment involved in the delivery: a small car, quadrocopter, robotic arm.

Code for simulation available in [this](https://github.com/vadim-rm/NagibCarSimulator) repository.

Out team consists 4 members:
  * [Vadim Mazhitov](https://github.com/vadim-rm) (Quadcopter's part, Simulation Creation, Docker Setup)
  * [Said Azizov](https://github.com/proton-bit) (Car's self-driving part) 
  * [Kikimov Daniil](https://github.com/katsushooter) (Simulation Creation)
  * [Dmitry Kutsenko](https://github.com/kdimon15) (Robotic Arm)
  
# Running the ROS2 part
```bash
docker build -t nagib2 -f Dockerfile .
docker run -it -p 10000:10000 nagib2 /bin/bash

./setup.sh

ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
ros2 run car main
```
