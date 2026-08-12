# Pico 2 micro-ROS Firmware

## What this does
- Publishes `sensor_msgs/msg/Imu` on `imu/data` (IMU reading — **stub**, see TODO below)
- Publishes `std_msgs/msg/Int32MultiArray` on `wheel_encoders` — 4 raw encoder counts,
  order: [front_left, front_right, back_left, back_right]
- Subscribes `std_msgs/msg/Float32MultiArray` on `wheel_cmd` — 4 motor duty cycles,
  range -1.0 to 1.0, same wheel order as above
- Subscribes `std_msgs/msg/Float32MultiArray` on `fan_cmd` — 2 values, 0.0-1.0 duty cycle

**BEFORE YOU BUILD: fill in your real GPIO numbers**

Pin numbers on your diagram (Pin16, Pin17, ...) are **physical/board pin numbers**, not
GPIO numbers. Look up the Pico 2 pinout diagram and convert each one, then edit the
`PIN_*` block at the top of `src/main.c`. I left them as placeholders — the code will
compile but won't drive the right pins until you do this.

## Get the dependencies


```bash
mkdir pico_firmware #If the directory is not created yet
cd pico_firmware

#Install tools needed to build PICO SDK and micro-ros firmware
sudo apt install cmake g++ gcc-arm-none-eabi doxygen libnewlib-arm-none-eabi git python3


#Clone Raspberry Pi Pico SDK repo
git clone https://github.com/raspberrypi/pico-sdk.git
export PICO_SDK_PATH=$(pwd)/pico-sdk
cd pico-sdk && git submodule update --init && cd ..

#Clone the following repo for RP2350 speciifc micro-ros firmware 
git clone --recurse-submodules https://github.com/samyarsadat/Micro-ROS-RP2350.git

#Note: Since this repo uses a docker in its workflow, you will need to install a docker and integrated it into WSL. For Ubuntu 24.04 you can use the following in your WSL.
sudo apt update
sudo apt install docker.io
sudo usermod -aG docker $USER
#Close and reopen WSL terminal
sudo service docker start
docker --version



#Open up the docker through powershell using the following commands:
cd ~/pico_firmware/Micro-ROS-RP2350/pico_fw_ws
code .

#This will open the docker container in VScode, where you will automatically get a pop-up to Rebuild the Open Container, press Rebuild.

#Note: If this does not pop-up make sure to you have the Dev Container extension installed on VScode.

#Troubleshooting Point:
# If the docker's container fails to rebuild continuously (Repo's README calls for second attempt at container rebuild as first fail is expected.) then edit this file:



~/pico_firmware/Micro-ROS-RP2350/pico_fw_ws/.devcontainer/devcontainer.json 

#Edit by omitting the following lines in above file, under the runArgs array:
"--gpus", "all",
"-v", "/tmp/.X11-unix:/tmp/.X11-unix:rw",
"--env=DISPLAY",


## Build the micro-ROS library 
bash ~/pico_ws/libmicroros/build_uros.sh -f


ls -la ~/pico_ws/libmicroros/firmware/build/include
find ~/pico_ws/libmicroros/firmware/build -iname "libmicroros.a"

## Build

```bash
cd ~/pico_ws/build
cmake ..
make -j$(nproc)
```

This produces `pico_firmware.uf2`.

## Flash

Hold BOOTSEL, plug the Pico 2 into your laptop/dev machine, then:

```bash
cp pico_firmware.uf2 /media/$USER/RP2350/
```
Or drag-and-drop in a file manager — it mounts as a USB drive in bootloader mode

Once the uf2 is imported into the Pico's bootloader, the USB drive should unmount itself automatically but the Pico itself should still be recognized as a usb device. To check use the command in powershell: 

            uspid list

and make sure that the Pico's serial connection is attached, using this:

            usbipd attach --wsl --busid 2-5

You will also need bind the device, so that the pico will be shared with the WSL work environment. This is done by:

            usbipd bind --busid 2-5

Note: An powershell with administrive privileges will be needed for binding.

For when the micro_ros_agent is running, a good check to do is make sure that the serial connection between the Pico and WSL is recognized, which in this case is /dev/ttyACM0. 
This check can be done by running the following in WSL:
      
         ls -l /dev/ttyACM*

What should return is:
         /dev/ttyACM0

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
