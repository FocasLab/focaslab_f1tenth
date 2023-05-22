# Autonomous Racing using Symbolic Motion Planner (SMP)

Autonomous_Racing is framework which is first tested out in the F1Tenth Car. It is based on Symbolic controller based approach, to tackle navigation through the obstacle course while providing safety guarantees. It can be used to perform Autonomous Navigation (SMP).

# Table of Contents
**Credits**

**Setup**
* [Installation](#Installation)
* [Example](#Example)

# Installation
Installation instructions for Linux.

**Prerequisites**
1. If not already done so, install [ROS](http://wiki.ros.org/ROS/Installation) (Desktop-Full is recommended).
   

**Installation**
1. Move to your catkin workspace:
    ```shell script
    cd ~/catkin_ws/src
    ```

2. Download repo using a SSH key or via HTTPS:
    ```shell
    git clone https://github.com/FocasLab/focaslab_f1tenth.git
    ```

3. Install system dependencies:
    ```shell script
    sudo apt-get install python-wstool python-catkin-tools
    ```

4. Install these packages for the F1Tenth Simulation
    ```shell script
    sudo apt-get install ros-noetic-tf2-geometry-msgs 
    sudo apt-get install ros-noetic-ackermann-msgs 
    sudo apt-get install ros-noetic-joy 
    sudo apt-get install ros-noetic-map-server
    ```

5. Compile and source:
    ```shell 
    catkin_make
    source ../devel/setup.bash
    ```

Recommended Specs ( equivalent or higher )
* Processor - Intel i7 
* RAM - 16GB 
* Graphics Card - 4GB
* OS - Ubuntu 20.04 with ROS Noetic
