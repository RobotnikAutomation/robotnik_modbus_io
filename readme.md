# robotnik_modbus_io

## Dependencies

### Libraries

* Modbus library
``` 
$ sudo apt-get install libmodbus-dev
``` 
or download it from here (Last version tested 3.1.4):

http://libmodbus.org/download 

### CMakeList

If you compile this package and you have any issue related with modbus headers, check where the modbus library was installed, by default CMakeList will look for it here: 

```
include_directories(
	include
	/usr/include/modbus
	${catkin_INCLUDE_DIRS}
)
```


### ROS dependencies

* robotnik_msgs
``` 
$ sudo apt-get install ros-indigo-robotnik-msgs
``` 

## Start-up

This component is intended to set and read digital I/O from a modbus server.

To test the component:

* Check that IP of the modbus I/O is correct (by default 192.168.1.195)

* Run the node

``` 
$ roslaunch robotnik_modbus_io robotnik_modbus_io.launch
```

* View digital and analog input status

```
$ rostopic echo /robotnik_modbus_io/input_output

digital_inputs: [False, True, False, True, True, False, False, False]
digital_outputs: [False, False, False, False, False, False, False, False]
analog_inputs: []
analog_outputs: []

```

* Set digital outputs

```
# enables the output 3
$ rosservice call /robotnik_modbus_io/write_digital_output 3 true
# disables the output 3
$ rosservice call /robotnik_modbus_io/write_digital_output 3 false
# disables all the outputs
$ rosservice call /robotnik_modbus_io/write_digital_output 0 false
```

