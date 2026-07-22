# Jetson ROS 2 Workspace

## Install dependencies (once)

```bash
sudo apt install ros-<distro>-micro-ros-agent   # bridges Pico serial -> ROS 2 topics
pip install dynamixel-sdk
```

Replace `<distro>` with your installed ROS 2 distro (e.g. `humble`, `jazzy`).

## Install udev rules (once)

```bash
sudo cp src/robot_bringup/config/99-robot-usb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Unplug/replug both the Pico and U2D2, then confirm:

```bash
ls -l /dev/ttyPICO /dev/ttyU2D2
```

If these don't show up, run `udevadm info -a -n /dev/ttyACM0` (or wherever it landed)
and fix the idVendor/idProduct values in the rules file to match your actual hardware —
the values in the file are typical defaults, not guaranteed for your specific units.

## Build

```bash
cd jetson_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Run everything

```bash
ros2 launch robot_bringup robot_bringup.launch.py
```

## Sanity checks

```bash
ros2 topic list                      # should show /imu/data, /wheel_encoders,
                                      # /wheel_cmd, /servo_cmd, /servo_state
ros2 topic echo /wheel_encoders      # should update as wheels turn
ros2 topic echo /servo_state         # should update as servos move

# Manually drive one wheel to sanity-check direction/PWM before trusting kinematics:
ros2 topic pub /wheel_cmd std_msgs/msg/Float32MultiArray "{data: [0.3, 0.0, 0.0, 0.0]}"

# Manually sweep a servo:
ros2 topic pub /servo_cmd std_msgs/msg/Float64MultiArray "{data: [0.3, 0.0, 0.0, 0.0]}"
```
