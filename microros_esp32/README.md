# ESP32C3-microROS-motor
A microROS package to control motors

Tested on humble.

USB serial connection was used.

Edit your parameters in config.h

## 1. Hardware:
- Motor driver - TB6612FNG
- ESP32C3
- Two brushed DC motor with encoder

# 2. microROS setup
```
mkdir microros_ws
cd microros_ws
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
```
```
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build    # build micro-ROS tools
source install/setup.bash   # source the workspace
```
```
ros2 run micro_ros_setup create_agent_ws.sh # create the micro-ros Agent
ros2 run micro_ros_setup build_agent.sh # build the agent packages
source install/setup.bash
```
## 2.1 Set USB port permission
```
sudo chmod a+rw /dev/ttyACM0    # give permission to usb port /ttyACM0
```
```
sudo usermod -aG dialout $USER  # add yourself into dialout group
```
```
su - $USER  # start a new shell session
```
Restart your linux system.

## 2.2 Run microROS
```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```
After setting up the microROS agent, run:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
## 3. Topics
Subscriber: /cmd_vel

Publisher: /right_wheel_speed  /left_wheel_speed  (rad/s)

