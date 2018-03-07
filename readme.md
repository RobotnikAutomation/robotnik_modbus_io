# robotnik_modbus_io

## Dependencies

### Libraries

* [Modbus library v3.1.4](http://libmodbus.org/2016/libmodbus-v3-1-4/)

``` 
$ sudo apt-get install libmodbus-dev
``` 

### ROS dependencies

* [robotnik_msgs](https://github.com/RobotnikAutomation/robotnik_msgs)

## Start-up

This component is intended to set and read digital I/O from a modbus server.

To test the component:

* Check that IP of the modbus is correct (by default 192.168.0.195)

* Run the node

``` 
$ roslaunch robotnik_modbus_io robotnik_modbus_io.launch
```

* View input status

```
$ rostopic echo /robotnik_modbus_io/inputs

inputs: [False, True, False, True, True, False, False, False, ...]
```

* Set outputs

```
# enables the bit 2 in the register number 0
$ rosservice call /robotnik_modbus_io/write_in_register "n_register: 0 bit: 2 value: 1"

# disenables the bit 2 in the register number 0
$ rosservice call /robotnik_modbus_io/write_in_register "n_register: 0 bit: 2 value: 0"
```

