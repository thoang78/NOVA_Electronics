# Robot Software Stack 

```
Jetson Orin Nano  --USB(serial)-->  Pico 2  --> Cytron x2 (4 drive motors + 4 encoders), IMU, fans
Jetson Orin Nano  --USB-C(U2D2)-->  4x Dynamixel servos (daisy chained) — one steering servo per wheel
```

## Two things get flashed/run, in this order

### 1. Pico 2 — flash once, then it just runs standalone
Lives in `pico_firmware/`. This is a micro-ROS node written in C using the Pico SDK.
It appears to ROS 2 as a normal node once connected — no custom parsing code needed on
the Jetson side.

### 2. Jetson — a ROS 2 workspace, not flashed, just launched
Lives in `jetson_ros2_ws/`. Contains:
- `micro_ros_agent` (bridges the Pico's USB serial into real ROS 2 topics)
- `dynamixel_node.py` (talks to the 4 servos over the U2D2 using dynamixel_sdk)
- `wheel_kinematics_node.py` (turns a `/cmd_vel` Twist into per-wheel speed + steer angle)
- udev rules so the Pico and U2D2 always show up at the same `/dev/...` path
- a launch file that starts everything together

## Build order (do NOT skip steps — debug each layer in isolation first)

1. **Pico alone, on the bench.** Build/flash the firmware, confirm it enumerates as a
   micro-ROS node when you run the agent from a laptop. Confirm encoder counts change
   when you spin a motor shaft by hand, and that sending a wheel_cmd spins the right motor
   in the right direction.
2. **U2D2 + one servo.** Confirm `dynamixel_node.py` can ping and move a single servo
   before wiring the daisy chain.
3. **Jetson end-to-end.** Plug both into the Jetson, install udev rules, run the launch
   file, confirm `ros2 topic list` shows everything and `ros2 topic echo /wheel_encoders`
   / `/imu/data` are updating.
4. **Only then** wire up `wheel_kinematics_node.py` to a teleop or planner.

See the README inside each subfolder for exact commands.
