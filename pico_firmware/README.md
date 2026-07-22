# Pico 2 micro-ROS Firmware

## What this does
- Publishes `sensor_msgs/msg/Imu` on `imu/data` (IMU reading — **stub**, see TODO below)
- Publishes `std_msgs/msg/Int32MultiArray` on `wheel_encoders` — 4 raw encoder counts,
  order: [front_left, front_right, back_left, back_right]
- Subscribes `std_msgs/msg/Float32MultiArray` on `wheel_cmd` — 4 motor duty cycles,
  range -1.0 to 1.0, same wheel order as above
- Subscribes `std_msgs/msg/Float32MultiArray` on `fan_cmd` — 2 values, 0.0-1.0 duty cycle

## BEFORE YOU BUILD: fill in your real GPIO numbers

Pin numbers on your diagram (Pin16, Pin17, ...) are **physical/board pin numbers**, not
GPIO numbers. Look up the Pico 2 pinout diagram and convert each one, then edit the
`PIN_*` block at the top of `src/main.c`. I left them as placeholders — the code will
compile but won't drive the right pins until you do this.

## Get the dependencies

```bash
cd pico_firmware
git clone -b jazzy https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git libmicroros_src
# Follow that repo's instructions to build libmicroros for the Pico 2 (rp2350) target
# and drop the resulting libmicroros/ folder (with .a + include/) next to this README.

git clone https://github.com/raspberrypi/pico-sdk.git
export PICO_SDK_PATH=$(pwd)/pico-sdk
cd pico-sdk && git submodule update --init && cd ..
```

## Build

```bash
mkdir build && cd build
cmake -DPICO_BOARD=pico2 ..
make -j4
```

This produces `pico_firmware.uf2`.

## Flash

Hold BOOTSEL, plug the Pico 2 into your laptop/dev machine, then:

```bash
cp pico_firmware.uf2 /media/$USER/RP2350/
```

(or drag-and-drop in a file manager — it mounts as a USB drive in bootloader mode)

## Test standalone (before plugging into the Jetson)

On a laptop with ROS 2 + micro-ROS agent installed:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

Then in another terminal: `ros2 topic list` should show `/imu/data` and
`/wheel_encoders`. `ros2 topic echo /wheel_encoders` should change when you spin a
wheel by hand.

## TODO — IMU driver

The BNO085 uses the SH2 protocol, which is nontrivial to bring up on bare Pico SDK
(Adafruit's library targets Arduino, not Pico SDK). `imu_read_stub()` in `main.c`
currently returns zeros so the topic exists and the rest of the pipeline can be tested.
Two practical paths to a real reading:
1. Port CEVA's official SH2 driver (https://github.com/ceva-dsp/sh2) to the Pico SDK's
   I2C HAL — this is the "proper" route but is real firmware work.
2. If you don't strictly need the Pico to own the IMU, wire the BNO085 to the Jetson's
   own I2C bus instead and read it there with Adafruit's CircuitPython/Blinka library —
   far less firmware work, at the cost of one more wire back to the Jetson.
