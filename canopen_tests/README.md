# CANopen Tests

Enable launch tests with --cmake-args -DCANOPEN_ENABLED.
They can only be run on devices that have vcan0 enabled.


# Important Notes
The Cogen feature of ROS2_CANOpen is not currently working. The following instructions are a work-around in order to get a working .bin file into the ROS_CANOpen

 - copy the bus.yml file and cia402_slave.eds file into the cia_402 directory of the lely project
 - run the dcfgen -r bus.yml command
 - copy the master.dcf file and motor.bin file to ros2_cows corresponding install file
 - change the file path of within master.dcf to the correct directory if necessary

 cd /home/fionnomuiri/canTutorials/ros2_cows/src/ros2_canopen/canopen_tests/config/cia402
 dcfgen -r bus.yml
 scp master.dcf motor_1.bin user@172.16.0.200:~/ros2_cows/install/canopen_tests/share/canopen_tests/config/cia402
 scp -r canopen_tests user@172.16.0.200:~/ros2_cows/src/ros2_canopen
