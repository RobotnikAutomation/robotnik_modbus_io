#elevator_controller_interface

##Dependencies

###Libraries

* Modbus library
``` 
$ sudo apt-get install libmodbus-dev
``` 

###ROS dependencies

* robotnik_msgs
``` 
$ sudo apt-get install ros-indigo-robotnik-msgs
``` 

##Start-up

This component is intended to set and read digital I/O from a modbus server.

To test the component:

* Check that IP of the modbus I/O is correct (by default 192.168.1.185)

* Run the node

``` 
$ roslaunch elevator_controller_interface ave_maria.launch
```

* View digital and analog input status

```
$ rostopic echo /elevator_controller_interface/input_output

digital_inputs: [False, True, False, True, True, False, False, False]
digital_outputs: [False, False, False, False, False, False, False, False]
analog_inputs: []
analog_outputs: []

```

* Set digital outputs

```
# enables the output 3
$ rosservice call /elevator_controller_interface/write_digital_output 3 true
# disables the output 3
$ rosservice call /elevator_controller_interface/write_digital_output 3 true
# disables all the outputs
$ rosservice call /elevator_controller_interface/write_digital_output 0 false
```

