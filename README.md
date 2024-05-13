![](https://github.com/open-rmf/charger_fleet/workflows/build/badge.svg)

# Charger Fleet

## Contents

- **[About](#About)**
- **[Installation Instructions](#installation-instructions)**
  - [Prerequisites](#prerequisites)
  - [Message Generation](#message-generation)
  - [Client in ROS 1](#client-in-ros-1)
  - [Client and Server in ROS 2](#client-and-server-in-ros-2)
- **[Plans](#plans)**

</br>
</br>

## About

Welcome to `charger_fleet`, an open-source charger fleet management system. 
Sometimes it is called the "Fun Charger Fleet For Friends" (F5).

**Note**, this repository is under active development. Things will be quite unstable
for a while. Please open an issue ticket on this repo if you have problems.
Cheers.

</br>
</br>

## Installation Instructions

### Prerequisites

* [Ubuntu 20.04 LTS](https://releases.ubuntu.com/20.04/)
* [ROS1 - Noetic](https://wiki.ros.org/noetic)
* [ROS2 - Galactic](https://docs.ros.org/en/galactic/index.html)

Install all non-ROS prerequisite packages,

```bash
sudo apt update && sudo apt install \
  git wget qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools \
  python3-rosdep \
  python3-vcstool \
  python3-colcon-common-extensions \
  # maven default-jdk   # Uncomment to install dependencies for message generation
```

</br>

### Message Generation

Message generation via `FleetMessages.idl` is done using `dds_idlc` from `CycloneDDS`. For convenience, the generated mesasges and files has been done offline and committed into the code base. They can be found [here](./charger_fleet/src/messages/FleetMessages.idl).

```bash
./dds_idlc -allstructs FleetMessages.idl
```

</br>

### Client in ROS 1

Start a new ROS 1 workspace, and pull in the necessary repositories,

```bash
mkdir -p ~/cf_ros1_ws/src
cd ~/cf_ros1_ws/src
git clone https://github.com/tannhat22/charger_fleet.git -b main
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.7.x
```

Install all the dependencies through `rosdep`,

```bash
cd ~/cf_ros1_ws
rosdep install --from-paths src --ignore-src --rosdistro noetic -yr
```

Source ROS 1 and build,

```bash
cd ~/cf_ros1_ws
source /opt/ros/noetic/setup.bash
colcon build
```

</br>

### Client and Server in ROS 2

Start a new ROS 2 workspace, and pull in the necessary repositories,

```bash
mkdir -p ~/cf_ros2_ws/src
cd ~/cf_ros2_ws/src
git clone https://github.com/tannhat22/charger_fleet -b main
git clone https://github.com/open-rmf/rmf_internal_msgs -b main
```

Install all the dependencies through `rosdep`,

```bash
cd ~/cf_ros2_ws
rosdep install --from-paths src --ignore-src --rosdistro galactic -yr
```

Source ROS 2 and build, 

```bash
cd ~/cf_ros2_ws
source /opt/ros/galactic/setup.bash
colcon build

# Optionally use the command below to only build the relevant packages,
# colcon build --packages-up-to \
#     charger_fleet cf_examples_ros2 charger_fleet_server_ros2 charger_fleet_client_ros2
```

</br>
</br>

## Examples

### Barebones Example

This example emulates a running ROS 1 robot,

```bash
source ~/cf_ros1_ws/install/setup.bash
roslaunch cf_examples_ros1 fake_client.launch
```

This example emulates a running ROS 2 robot,

```bash
source ~/cf_ros2_ws/install/setup.bash
ros2 launch cf_examples_ros2 fake_client.launch.xml
```

The clients will then start subscribing to all the necessary topics, and start publishing robot states over DDS to the server. Start the server using

```bash
source ~/cf_ros2_ws/install/setup.bash
ros2 launch cf_examples_ros2 fake_server.launch.xml

# Verify that the server registers the fake clients
# [INFO] [1636706176.184055177] [fake_server_node]: registered a new robot: [fake_ros1_robot]
# [INFO] [1636706176.184055177] [fake_server_node]: registered a new robot: [fake_ros2_robot]
```

ROS 2 messages over the `/fleet_states` topic can also be used to verify that the clients are registered,

```bash
source ~/cf_ros2_ws/install/setup.bash
ros2 topic echo /fleet_states
``