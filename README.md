# ROS 2 ↔ micro-ROS Bidirectional Integer Exchange

This repo contains two ROS 2 nodes (`talker` and `listener`) and a launch file for bridging integer messages (`std_msgs/msg/Int32`) between a PC (ROS 2) and an embedded microcontroller (micro-ROS over UDP6).

The micro-ROS firmware (client side) is maintained in a separate repository.  [EmDonato/pubsub_doubleNode_wifi_uRos](https://github.com/EmDonato/pubsub_doubleNode_wifi_uRos.git)

---

## Table of Contents

1. [Package Layout](#package-layout)  
2. [Dependencies](#dependencies)  
3. [Building](#building)  
4. [micro-ROS Agent Setup](#micro-ros-agent-setup)  
5. [launch](#launch)  
6. [Modifying for Your Environment](#modifying-for-your-environment)  
7. [How It Works](#how-it-works)  
8. [Troubleshooting](#troubleshooting)  

---

## Package Layout

pubsub_template_uros/  
├── CMakeLists.txt  
├── package.xml  
├── launch/  
│ └── launch_pubSub.py   
├── src/  
│ ├── talker.cpp  
│ └── listener.cpp  
└── README.md  


- **talker.cpp**  
  - Publishes on `/micro_ros_sub_topic` the last integer read from a named pipe.  
  - Creates a named pipe (`/tmp/input_pipe`) and spawns a terminal for user input.  
  - Sends each entered integer at 2 Hz (500 ms timer).  

- **listener.cpp**  
  - Subscribes to `micro_ros_pub_topic` with Best Effort QoS.  
  - Logs each received `Int32` from the micro-ROS side.  

- **uros_bridge.launch.py**  
  - Starts the micro-ROS Agent (UDP6).  
  - Launches the `talker` and `listener` nodes under ROS 2.

---

## Dependencies

Install system and ROS 2 dependencies before building:

```bash
# On Ubuntu + ROS 2 Humble/Foxy/Galactic
sudo apt update
sudo apt install \
  ros-<distro>-rclcpp \
  ros-<distro>-std-msgs \
  ros-<distro>-micro-ros-agent \
  gnome-terminal              # (for talker input window)

Replace <distro> with your ROS 2 distribution name.
```
## Building

1. Clone or copy this package into a ROS 2 workspace:  
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/yourusername/pubsub_template_uros.git
    ```
2. Install any missing dependencies via rosdep:  
    ```bash
    cd ~/ros2_ws
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    ```
3. Build and source:  
      ```bash
      colcon build  
      source install/local_setup.bash  
      ```
## micro-ROS Agent Setup

The micro-ROS Agent bridges between ROS 2 and microcontrollers over UDP, Serial, etc. In our launch file we use UDP6 on port 8888. Make sure your embedded firmware runs a micro-ROS client targeting the same transport and port.
Launch File

## launch
```py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
      # Start micro-ROS Agent (UDP6, port 8888)
      ExecuteProcess(
        cmd=[
          'bash', '-c',
          'source ~/Desktop/agent/install/local_setup.bash && '
          'ros2 run micro_ros_agent micro_ros_agent udp6 --port 8888'
        ],
        output='screen',
      ),

      # ROS 2 Talker (reads pipe, publishes Int32)
      Node(
        package='pubsub_template_uros',
        executable='talker',
        name='ros2_talker',
        output='screen',
      ),

      # ROS 2 Listener (subscribes Int32 from micro-ROS)
      Node(
        package='pubsub_template_uros',
        executable='listener',
        name='ros2_listener',
        output='screen',
      ),
    ])
```
## Running the bridge
```bash
# From your ROS 2 workspace:
source install/local_setup.bash
ros2 launch pubsub_template_uros uros_bridge.launch.py
```
This opens a terminal window for integer input. Enter values and watch both ROS 2 log output and micro-ROS client logs.

## Modifying for Your Environment
  1. Agent Location
        - Edit the cmd in ExecuteProcess to source the correct path to your micro-ROS Agent.

  2. Transport & Port
  
        - Change udp6 --port 8888 to match your client’s configuration (e.g. udp4, serial --dev /dev/ttyUSB0).
  
  3. Topic Names
  
        - In talker.cpp and listener.cpp, adjust /micro_ros_sub_topic and micro_ros_pub_topic to suit your naming conventions.
      
  4. QoS Profiles
  
        - Listener uses Best Effort. If your network is reliable, consider reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE).

## How It Works

  1. talker.cpp
  
        - On startup, spawns a bash terminal that continually writes user-entered lines to /tmp/input_pipe.
        
        - A background thread (read_loop) opens that pipe, reads each line, parses it to int, and updates an atomic last_value_.
        
        - A timer callback fires every 500 ms, publishes the latest integer on /micro_ros_sub_topic.
  
  2. listener.cpp
        
        - Subscribes to micro_ros_pub_topic with QoS “Keep Last 1, Best Effort”.
        
        - On each message, logs the received data field.
        
  3. Launch
  
        - The micro-ROS Agent relays messages between ROS 2 and the embedded client.
        
        - talker → /micro_ros_sub_topic → Agent → microcontroller
        
        - microcontroller → Agent → micro_ros_pub_topic → listener

## Troubleshooting
    
- Pipe errors: Ensure /tmp is writable and no stale pipe exists (rm -f /tmp/input_pipe).

- Agent fails to start: Verify you sourced the correct workspace and that micro_ros_agent is installed.

- QoS mismatches: If messages drop, adjust reliability and history settings.

- Permissions: Running on Linux may require chmod +x on your executables and proper user permissions for /tmp.

