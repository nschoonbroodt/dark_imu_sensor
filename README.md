# IMU Sensor Simulation

This repository contains 2 nodes, one generating fake trajectory data, and one "converting" these data to IMU reading. The main code of interest is in ```imu_sensor/src/imu_sensor.[hc]pp```.

## Build and launch
In an environement setup for ROS 2 (rolling), run the following to build and launch:

```
colcon build --packages-select imu_sensor
ros2 launch launch/launch_simulation.yaml 
```

## Parameters
The useful parameters to the imu_sensor node are:

- ```imu_message_period```: control the period of message sending by the IMU sensor. In ms (missing the check to only allow some values, also in Hz may be easier to understand)
- ```message_destination```: 0 to send imu_data to ros topic, 1 to send it to a physical hardware bus (currently will write messages on the console, as no driver have been developped yet)
- ```imu_id```: a uint8 identifying this specific unit
- ```inject_status_failure```: if set to true, the IMU status will display IMU_FAIL
- ```inject_random_data```: if true, data will be modified randomly (not really implement currently. When no failure, all data are 0, when this is true, one data is non 0)
