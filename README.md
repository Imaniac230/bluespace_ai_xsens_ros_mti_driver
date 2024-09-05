# Xsens MTi driver for ROS 2.0

## Changes over the original bluespace-ai version
This branch was tested to compile and work in ROS 2.0 Humble. Compatibility with all supported devices should still remain unchanged. However, this version was only tested using an `MTi-680G` device, so your experience may vary. CMake minimum version was bumped to `3.5`.

### Merged PRs from the original repo:
1. https://github.com/bluespace-ai/bluespace_ai_xsens_ros_mti_driver/pull/17
2. https://github.com/bluespace-ai/bluespace_ai_xsens_ros_mti_driver/pull/11
3. https://github.com/bluespace-ai/bluespace_ai_xsens_ros_mti_driver/pull/18
4. https://github.com/bluespace-ai/bluespace_ai_xsens_ros_mti_driver/pull/6

### Additional enhancements:
1. Fixed problem with device ID check not being correctly utilized if scanning was enabled.
2. More robust baudrate reconfiguration and reconnection when using the `config_baudrate` parameter.
   > To make a successful initial connection to the device, always use the currently active baudrate in `baudrate`, or enable scanning with `scan_for_devices`.
3. Enhanced the device output configuration to also be able to specify the data type, data format, and data output frequency using `DATA_TYPE|DATA_FORMAT|FREQUENCY` flags in the `output_configuration` parameter. For example, to configure temperature output at 400 Hz using the `Float` format add: `Temperature|Float|400`.
   > The driver will detect if you're trying to use an unsupported frequency for a given data type.

   > The driver will not detect if you're using an unsupported format for a given data type. If you don't see your configured data output, try setting the format to the default `Float`.
4. Added support for additional device configuration:
   1. Added an `option_flags` parameter to configure the device option flags. For example, to enable the device orientation smoother algorithm add: `EnableOrientationSmoother`.
      > The driver will detect if you're trying to use an option flag unsupported by the device.
   2. Added parameters and services `reset_heading` and `reset_inclination` to enable orientation resets on startup and during runtime.
   3. Added a `gnss_platform` parameter to set the platform type on a supported device.
   4. Added a `location_id` parameter.
   5. Added an `apply_current_utc_time` parameter to enable setting the current host machine local time to the MTi device on startup.
   6. Added a `gnss_lever_arm` parameter to enable configuring the sensor-relative GNSS antenna position on supported devices.
   7. Added an `initial_position_lla` parameter to enable configuring the initial LLA sensor position if not available from GNSS.
   8. Added an `rtcm_baudrate` parameter to enable configuring the baudrate for the RTCM input port.
5. Fixed publisher destruction errors on driver exit - refactored to unique pointers with proper destructor semantics.
6. Properly filling in the used gnss service (`NavSatStatus`) in the `NavSatFix` ros message, based on the available satellite information status.

### Issues:
There appears to be an issue, which may also be related to the [ROS timestamps note](#notes) below. A given ROS publisher output frequency correlates with the number of configured device output data types. For example, setting only temperature device output at 400 Hz:
```
output_configuration:
    - "Temperature|Float|400"
```
will result in a lower actual publishing frequency:
```bash
$ ros2 topic hz /temperature
```
```
average rate: 72.561
	min: 0.000s max: 0.103s std dev: 0.03457s window: 74
average rate: 71.552
	min: 0.000s max: 0.104s std dev: 0.03478s window: 146
average rate: 72.631
	min: 0.000s max: 0.104s std dev: 0.03438s window: 225
```
However, by adding additional output data types, the frequency increases. If at least another additional six 400 Hz device output data types are configured:
```
output_configuration:
    - "Temperature|Float|400"
    - "Quaternion|Float|400"
    - "Acceleration|Float|400"
    - "FreeAcceleration|Float|400"
    - "AltitudeEllipsoid|Float|400"
    - "LatLon|Float|400"
    - "VelocityXYZ|Float|400"
```
the temperature publishing frequency will be at the desired 400 Hz as well:
```bash
$ ros2 topic hz /temperature
```
```
average rate: 397.848
	min: 0.000s max: 0.015s std dev: 0.00493s window: 399
average rate: 399.071
	min: 0.000s max: 0.015s std dev: 0.00493s window: 800
average rate: 399.230
	min: 0.000s max: 0.015s std dev: 0.00493s window: 1201
```
So the solution would be to enable most of the available device data output types, and then select the desired outputs by enabling the individual publishers. 

## The original readme from bluespace-ai
**The text was reformatted for better markdown presentation, however, actual contents are unchanged.**
### Documentation:
You can find the full documentation in `<your MT SDK directory>/doc/xsensdeviceapi/doc/html/index.html` under "ROS MTi driver" section. The SDK can be downloaded from https://www.xsens.com/software-downloads. Please note, this is a 3rd Party driver built from MTSDK2021.2 with no official support. Check the compatibility section for the compatible devices. For official support on Xsens MTi products, please refer to Xsens knowledge base: https://base.xsens.com

### Prerequisites:
* ROS 2.0 Galactic/Foxy
* C/C++ Compiler: GCC 5.4.0 or MSVC 14.0
* C++14

### Building:
* Copy `bluespace_ai_xsens_mti_driver` folder into your ROS 2.0 workspace `src` folder.
        Make sure the permissions are set to `o+rw` on your files and directories.
* Build Xsens MTi driver package:
    ```bash
    $ colcon build
    ```
* Source workspace:
    ```bash
    $ source install/setup.bash
    ```
> Note: Building of 'xspublic' from the ament workspace has been automated in the CMake script. To build it manually, run the following from the ROS2.0 workspace root:
        `$ pushd src/bluespace_ai_xsens_ros_mti_driver/lib/xspublic && make && popd`

Changes in this release compared to the Xsens ROS 1.0 driver open source:
* Added ROS 2.0 support: The ROS 1.0 wrapper node was modified to work with ROS 2.0.
* Migration to declared configuration parameters: ROS 1.0 driver supported undeclared parameters. Since ROS 2.0 guidelines discourage undeclared parameters, there are some minor modifications in the `xsens_mti_node.yaml` to support all the configuration capabilities from ROS1.0. For more details check the config file.
  * A boolean parameter `scan_for_devices` was added to enable switching between port scanning and connecting to a specific port for Xsens devices.
  * A boolean parameter `enable_logging` was added for logging. When enabled, the name of the file needs to be specified in the log_file parameter.
* Magnetometer topic switched to `sensor_msgs/MagneticField` : The message type of the magnetometer measurement topic (`/imu/mag`) was switched to `sensor_msgs/MagneticField` from previous `geometry_msgs/Vector3Stamped`. This was a TODO item left in the original source.

### Running:
* Configure your MTi device to output desired data (e.g. for display example - orientation output)
* Launch the Xsens MTi driver from your ament workspace:
    ```bash
    $ ros2 launch bluespace_ai_xsens_mti_driver xsens_mti_node.launch.py
    ```

* After the device has been detected, you can communicate with it from another process / terminal window. For example:
    ```bash
    $ ros2 topic echo /filter/quaternion
    ```
* This will result in a continuous stream of data output:
    ```
    ---
    header: 
      seq: 1386351
      stamp: 
        secs: 1545223809
        nsecs: 197252179
      frame_id: "imu_link"
    quaternion: 
      x: 0.00276306713931
      y: 0.00036825647112
      z: -0.89693570137
      w: -0.442152231932
    ---
    ```

* There is also an example that shows a 3D visualization of the device (orientation data should be enabled in the device):
    ```bash
    $ ros2 launch bluespace_ai_xsens_mti_driver display.launch.py
    ```

### Compatibility:
* The driver is compatible with following Xsens IMU product lines:
  * MTi 1-series
  * MTi 10-series
  * MTi 100-series
  * MTi 600-series
* The driver has been tested on amd64 as well as ARM 64-bit (aarch64) architectures. 


### Notes:
* ROS timestamps:
  * The data messages from devices are time stamped on arrival in the ROS driver. When collecting data at higher rates, eg 100 Hz, the times between reads can differ from the configured output rate in the device. This is caused by possible buffering in the USB/FTDI driver. For instance:
    ```
    10 us, 10 us, 10 us, 45 ms, 10 us, 10 us, 10 us, 39 ms, 10 us, etc.
    ```
    instead of 
    ```
    10 ms, 10 ms, 10 ms, 10 ms, 10 ms, 10 ms, 10 ms, 10 ms, 10 ms, etc.
    ```
    > Work-around: for accurate and stable time stamp information, users can make use of the sensor reported time stamp (`/imu/time_ref`) instead.

### Troubleshooting
1. The Mti1 (Motion Tracker Development Board) is not recognized.
   * Support for the Development Board is present in recent kernels. (Since June 12, 2015). If your kernel does not support the Board, you can add this manually:
     ```bash
     $ sudo /sbin/modprobe ftdi_sio
     $ echo 2639 0300 | sudo tee /sys/bus/usb-serial/drivers/ftdi_sio/new_id
     ```
2. The device is recognized, but I cannot ever access the device.
   * Make sure you are in the correct group (often `dialout` or `uucp`) in order to access the device. You can test this with:
     ```bash
     $ ls -l /dev/ttyUSB0
     ```
     ```
     crw-rw---- 1 root dialout 188, 0 May  6 16:21 /dev/ttyUSB0
     ```
     ```bash
     $ groups
     ```
     ```
     dialout audio video usb users plugdev
     ```
   * If you aren't in the correct group, you can fix this in two ways.
     1. Add yourself to the correct group. You can add yourself to it by using your distributions user management tool, or call:
        ```bash
        $ sudo usermod -G dialout -a $USER
        ```
        > Be sure to replace `dialout` with the actual group name if it is different.

        > After adding yourself to the group, either relogin to your user, or call:
        > ```bash
        > $ newgrp dialout
        > ```
        > to add the current terminal session to the group.
     2. Use udev rules. Alternatively, put the following rule into `/etc/udev/rules.d/99-custom.rules`:
        ```
        SUBSYSTEM=="tty", ATTRS{idVendor}=="2639", ACTION=="add", GROUP="$GROUP", MODE="0660"
        ```
        > Change `$GROUP` into your desired group (e.g. `adm`, `plugdev`, or `usb`).
3. The device is inaccessible for a while after plugging it in.
   * When having problems with the device being busy the first 20 seconds after plugin, purge the modemmanager application.

### Acknowledgement:
Thank you to Steven Gies and his engineering team at Xsens Technologies for testing this driver against their complete MTi IMU product portfolio and reviewing the driver source.