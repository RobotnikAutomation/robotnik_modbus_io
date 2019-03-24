/*
 * Software License Agreement (BSD License)
 *
 *  modbus_interface_node
 *  Copyright (c) 2012, Robotnik Automation, SLL
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <cassert>
#include <cerrno>
#include <cstdio>
#include <csignal>

#include <iostream>
#include <typeinfo>

#include <boost/format.hpp>

#include <ros/time.h>
#include <self_test/self_test.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>

#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <robotnik_msgs/Register.h>
#include <robotnik_msgs/Registers.h>
#include <robotnik_msgs/inputs_outputs.h>
#include <robotnik_msgs/set_digital_output.h>
#include <robotnik_msgs/set_modbus_register.h>
#include <robotnik_msgs/get_modbus_register.h>
#include <robotnik_msgs/State.h>

#include <modbus.h>

#include <pthread.h>

#define MODBUS_DESIRED_FREQ 1.0

#define SLAVE_NUMBER 1
#define MODBUS_DEFAULT_DIGITAL_OUTPUTS 80
#define MODBUS_DEFAULT_DIGITAL_INPUTS 80
#define MODBUS_DEFAULT_ANALOG_OUTPUTS 2
#define MODBUS_DEFAULT_MIN_DIGITAL_OUTPUTS 4  // Min. number of digital outputs (factory default)
#define MODBUS_DEFAULT_MIN_DIGITAL_INPUTS 8   // Min. number of digital inputs (factory default)
#define MODBUS_DEFAULT_MIN_ANALOG_INPUTS 0    // Min. number of analog inputs (factory default)
#define MODBUS_DEFAULT_MIN_REGISTERS 0        // Min. number of registers to read

#define MODBUS_DEFAULT_ANALOG_INPUT_DIVISOR 30000.0
#define MODBUS_DEFAULT_ANALOG_INPUT_MULTIPLIER 10.0
#define MODBUS_MAX_COMM_ERRORS 10     // Max number of erros in communication to consider an error
#define MODBUS_ERROR_RECOVERY_TIME 5  // Tries the recovery every X seconds

bool MODBUS_DEFAULT_BIG_ENDIAN =
    false;  // defines endianness of the modbus device. false = little endian (PC), true = big endian

using namespace std;

class modbusNode
{
public:
  // Robotnik_msgs object
  robotnik_msgs::inputs_outputs reading_;
  robotnik_msgs::Registers registers_;

  // tcp/ip data
  string ip_address_;
  int port_;
  // components state
  int state, previous_state;

  // ROS objects
  self_test::TestRunner self_test_;
  diagnostic_updater::Updater diagnostic_;

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  ros::Publisher modbus_io_data_pub_;
  ros::Publisher modbus_io_registers_pub_;
  ros::Publisher state_pub_;
  ros::ServiceServer modbus_io_write_digital_srv_;
  ros::ServiceServer modbus_io_write_digital_input_srv_;

  ros::ServiceServer set_modbus_register_srv_;
  ros::ServiceServer set_modbus_registers_srv_;
  ros::ServiceServer get_modbus_register_srv_;

  bool running_;
  // Config params
  int digital_inputs_;
  int digital_outputs_;
  int analog_inputs_;
  int digital_inputs_addr_;
  int digital_outputs_addr_;
  int read_registers_addr_;
  int number_of_registers_to_read_;
  bool big_endian_;
  // Flag to read full registers automatically
  bool read_modbus_registers_enabled_;

  // Error counters and flags
  int error_count_;
  int slow_count_;
  std::string was_slow_;
  std::string error_status_;

  double desired_freq_;
  diagnostic_updater::FrequencyStatus freq_diag_;

  // Modbus member variables
  modbus_t* mb_;
  uint16_t tab_reg_[32];
  uint16_t* dout_;    // used to read digital outputs.
  uint16_t dout384_;  // store digital output registers to activate each one separatedly (not use)
  uint16_t dout385_;  // store digital output registers to activate each one separatedly (not use)

  int registers_for_io_;
  //! saves the analog inputs address
  vector<int> analog_inputs_addr_;
  //! variable divisor to apply to the analog input register
  double analog_register_divisor_;
  //! variable multiplier to apply to the analog input register
  double analog_register_multiplier_;

  float max_delay_;
  //! num of erros in the modbus communication
  int modbus_errors_;
  //! Saves the time of the error
  ros::Time communication_error_time;

  int number_of_outputs_;

  pthread_mutex_t lock_;
  // Constructor
  modbusNode(ros::NodeHandle h)
    : self_test_()
    , diagnostic_()
    , node_handle_(h)
    , private_node_handle_("~")
    , error_count_(0)
    , slow_count_(0)
    , desired_freq_(20)
    , freq_diag_(diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.05))
  {
    running_ = false;
    // READ PARAMS
    private_node_handle_.param("ip_address", ip_address_, string("192.168.0.20"));
    private_node_handle_.param("port", port_, 502);
    private_node_handle_.param("digital_outputs", digital_outputs_, MODBUS_DEFAULT_DIGITAL_OUTPUTS);
    private_node_handle_.param("digital_inputs", digital_inputs_, MODBUS_DEFAULT_DIGITAL_INPUTS);
    private_node_handle_.param("analog_inputs", analog_inputs_, MODBUS_DEFAULT_MIN_ANALOG_INPUTS);
    private_node_handle_.param("number_of_registers_to_read", number_of_registers_to_read_,
                               MODBUS_DEFAULT_MIN_REGISTERS);
    private_node_handle_.param("analog_register_divisor", analog_register_divisor_,
                               MODBUS_DEFAULT_ANALOG_INPUT_DIVISOR);
    private_node_handle_.param("analog_register_multiplier", analog_register_multiplier_,
                               MODBUS_DEFAULT_ANALOG_INPUT_MULTIPLIER);
    private_node_handle_.param("desired_freq", desired_freq_, 10.0);

    private_node_handle_.param("digital_inputs_addr", digital_inputs_addr_, 0);
    private_node_handle_.param("digital_outputs_addr", digital_outputs_addr_, 10);  // new used
    private_node_handle_.param("read_registers_addr", read_registers_addr_, 10);

    private_node_handle_.param<bool>("big_endian", big_endian_, MODBUS_DEFAULT_BIG_ENDIAN);
    private_node_handle_.param<bool>("read_modbus_registers_enabled", read_modbus_registers_enabled_, false);
    // Checks the min num of digital outputs
    /*if(digital_outputs_ < MODBUS_DEFAULT_MIN_DIGITAL_OUTPUTS){
      digital_outputs_ = MODBUS_DEFAULT_MIN_DIGITAL_OUTPUTS;
      ROS_INFO("modbus_io: Setting num of digital outputs to the minimum value = %d",
    MODBUS_DEFAULT_MIN_DIGITAL_OUTPUTS);
      }
    // Checks the min num of digital inputs
    if(digital_inputs_ < MODBUS_DEFAULT_MIN_DIGITAL_INPUTS){
    digital_inputs_ = MODBUS_DEFAULT_MIN_DIGITAL_INPUTS;
    ROS_INFO("modbus_io: Setting num of digital inputs to the minimum value = %d", MODBUS_DEFAULT_MIN_DIGITAL_INPUTS);
    }
     */
    XmlRpc::XmlRpcValue list;
    private_node_handle_.getParam("analog_inputs_addr", list);

    // Checks that the read param type is correct
    if (list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("modbus_io: Wrong read type (%d) for analog_inputs_addr param", list.getType());
    }
    else
    {
      // Saves the array into class vector
      for (int32_t i = 0; i < list.size(); ++i)
      {
        analog_inputs_addr_.push_back(static_cast<int>(list[i]));
      }
    }

    if ((int)analog_inputs_addr_.size() < analog_inputs_)
    {
      ROS_WARN("modbus_io: the number of analog inputs (%d) is different to the number of analog inputs addresses "
               "(%d). Revise the config files.",
               analog_inputs_, (int)analog_inputs_addr_.size());
      // resize the number of analog inputs
      analog_inputs_ = (int)analog_inputs_addr_.size();
    }

    ROS_INFO("modbus_io: Settings -> DO = %d (register %d), DI = %d (register %d), AI = %d", digital_outputs_,
             digital_outputs_addr_, digital_inputs_, digital_inputs_addr_, analog_inputs_);

    modbus_io_data_pub_ = private_node_handle_.advertise<robotnik_msgs::inputs_outputs>("input_output", 100);
    modbus_io_registers_pub_ = private_node_handle_.advertise<robotnik_msgs::Registers>("registers", 100);
    state_pub_ = private_node_handle_.advertise<robotnik_msgs::State>("state", 1);

    modbus_io_write_digital_srv_ =
        private_node_handle_.advertiseService("write_digital_output", &modbusNode::write_digital_output_srv, this);
    set_modbus_register_srv_ =
        private_node_handle_.advertiseService("set_modbus_register", &modbusNode::set_modbus_register_cb, this);
    set_modbus_registers_srv_ =
        private_node_handle_.advertiseService("set_modbus_registers", &modbusNode::set_modbus_registers_cb, this);
    get_modbus_register_srv_ =
        private_node_handle_.advertiseService("get_modbus_register", &modbusNode::get_modbus_register_cb, this);

    self_test_.add("Connect Test", this, &modbusNode::ConnectTest);

    diagnostic_.add(freq_diag_);

    diagnostic_.add("Device Status", this, &modbusNode::deviceStatus);

    // Initializes the outputs/inputs vector. Setup
    reading_.digital_inputs.resize(digital_inputs_);
    reading_.digital_outputs.resize(digital_outputs_);
    reading_.analog_inputs.resize(analog_inputs_);
    registers_.registers.resize(number_of_registers_to_read_);
    max_delay_ = 1.0 / MODBUS_DESIRED_FREQ;

    registers_for_io_ = 20;
    number_of_outputs_ = 5;
    dout_ = new uint16_t[number_of_outputs_];

    // Initializes to zero the output data vector
    for (int i = 0; i < number_of_outputs_; i++)
    {
      dout_[i] = 0;
    }

    previous_state = state = robotnik_msgs::State::INIT_STATE;
    modbus_errors_ = 0;
  }

  // Destructor
  ~modbusNode()
  {
    stop();
  }

  int start()
  {
    stop();

    if (pthread_mutex_init(&lock_, NULL) != 0)
    {
      ROS_ERROR("modbus_io::start: could not initiate mutex");
      return -1;
    }

    if (connectModbus() != 0)
      return -1;

    ROS_INFO("modbus_io::start: connected to MODBUS IO BOARD at %s on port %d", ip_address_.c_str(), port_);
    freq_diag_.clear();

    running_ = true;

    switchToState(robotnik_msgs::State::READY_STATE);

    return (0);
  }

  int stop()
  {
    if (running_)
    {
      ROS_INFO("modbus_io::stop: Closing modbus connection");
      disconnectModbus();
      running_ = false;
      pthread_mutex_destroy(&lock_);
    }
    ROS_INFO("modbus_io::stop STOP");
    return (0);
  }

  int connectModbus()
  {
    ROS_INFO_THROTTLE(10, "modbus_io::connectModbus: connecting to %s:%d", ip_address_.c_str(), port_);
    mb_ = modbus_new_tcp(ip_address_.c_str(), port_);
    if (mb_ == NULL)
    {
      dealWithModbusError();
      return -1;
    }
    if (modbus_connect(mb_) == -1)
    {
      dealWithModbusError();
      ROS_ERROR_THROTTLE(10, "modbus_io::connectModbus: connection Error to %s:%d!", ip_address_.c_str(), port_);
      return -1;
    }
    ROS_INFO_THROTTLE(10, "modbus_io::connectModbus: connected to %s:%d!", ip_address_.c_str(), port_);

    // Set the slave
    int iret = modbus_set_slave(mb_, SLAVE_NUMBER);
    if (iret == -1)
    {
      dealWithModbusError();
      ROS_ERROR_THROTTLE(10, "modbus_io::setSlave: Invalid slave ID");
      return -1;
    }

    return 0;
  }

  int disconnectModbus()
  {
    modbus_close(mb_);
    modbus_free(mb_);

    ROS_INFO_THROTTLE(10, "modbus_io::disconnectModbus: disconnected from %s:%d!", ip_address_.c_str(), port_);
    return 0;
  }

  int read_and_publish()
  {
    static double prevtime = 0;

    double starttime = ros::Time::now().toSec();
    if (prevtime && prevtime - starttime > max_delay_)
    {
      ROS_WARN("modbus_io::read_and_publish: Full loop took %f ms. Nominal is %f ms.", 1000 * (prevtime - starttime),
               1000 * max_delay_);
      was_slow_ = "Full modbus_io loop was slow.";
      slow_count_++;
    }

    if (read_modbus_registers_enabled_)
    {
      getIntData(registers_);
    }
    else
    {
      getData(reading_);
    }

    double endtime = ros::Time::now().toSec();
    if (endtime - starttime > max_delay_)
    {
      ROS_WARN_THROTTLE(10, "modbus_io::read_and_publish: Gathering data took %f ms. Nominal is %f ms.",
               1000 * (endtime - starttime), 1000 * max_delay_);
      was_slow_ = "Full modbus_interface loop was slow.";
      slow_count_++;
    }
    prevtime = starttime;
    starttime = ros::Time::now().toSec();
    modbus_io_data_pub_.publish(reading_);
    modbus_io_registers_pub_.publish(registers_);

    endtime = ros::Time::now().toSec();
    if (endtime - starttime > max_delay_)
    {
      ROS_WARN_THROTTLE(10, "modbus_io::read_and_publish: Publishing took %f ms. Nominal is %f ms.", 1000 * (endtime - starttime),
               1000 * max_delay_);
      was_slow_ = "Full modbus_io loop was slow.";
      slow_count_++;
    }

    freq_diag_.tick();

    return 0;
  }

  bool spin()
  {
    ros::Time t_now;

    ros::Rate r(desired_freq_);
    while (!ros::isShuttingDown())  // Using ros::isShuttingDown to avoid restarting the node during a shutdown.
    {
      if (start() == 0)
      {
        while (node_handle_.ok())
        {
          switch (state)
          {
            case robotnik_msgs::State::READY_STATE:
              read_and_publish();

              if (modbus_errors_ > MODBUS_MAX_COMM_ERRORS)
                switchToState(robotnik_msgs::State::FAILURE_STATE);

              break;

            case robotnik_msgs::State::FAILURE_STATE:
              t_now = ros::Time::now();

              if ((t_now - communication_error_time).toSec() > MODBUS_ERROR_RECOVERY_TIME)
              {
                ROS_INFO("modbus_io::spin: trying to recover");
                disconnectModbus();
                sleep(1);
                if (connectModbus() == 0)
                {
                  ROS_INFO("modbus_io::spin: reconnected to modbus!");
                  switchToState(robotnik_msgs::State::READY_STATE);
                }
                communication_error_time = t_now;
              }

              break;
          }

          self_test_.checkTest();
          diagnostic_.update();

          // publish component state
          robotnik_msgs::State msg;
          msg.state = state;
          msg.desired_freq = desired_freq_;
          msg.state_description = getStateString(state);
          state_pub_.publish(msg);

          ros::spinOnce();
          r.sleep();
        }
      }
      else
      {
        // No need for diagnostic here since a broadcast occurs in start
        // when there is an error.
        sleep(1);
        self_test_.checkTest();
        ros::spinOnce();
      }
    }

    ROS_INFO("modbus_io::spin - calling stop !");
    stop();
    return true;
  }

  void ConnectTest(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    // connection test
    // TBC
    status.summary(0, "Connected successfully.");
  }

  int16_t switchEndianness(int16_t reg)
  {
    if (big_endian_)
      return htobe16(reg);  // host endianness to big endian
    else
      return htole16(reg);  // host endianness to little endian
  }

  void getData(robotnik_msgs::inputs_outputs& data)
  {
    // Adress	 Description
    // 0000 	 1 to 8 inputs module IL ETH
    // 0001 	 1 to 4 outuputs module IL ETH

    int16_t x;
    int iret;

    // Read digital 16 bit inputs registers. Each bit is an input
    iret = modbus_read_registers(mb_, digital_inputs_addr_, registers_for_io_, tab_reg_);
    if (iret != registers_for_io_)
    {
      dealWithModbusError();
      return;
    }
    for (int j = 0; j < registers_for_io_; j++)
    {
      x = switchEndianness(tab_reg_[j]);
      for (int i = 0; i < 16; i++)
      {
        data.digital_inputs[i + 16 * j] = x & 1;
        x >>= 1;
      }
    }
    /*
    iret = modbus_read_registers(mb_, digital_outputs_addr_, 5, tab_reg_);
    if (iret != 5)
    {
      dealWithModbusError();
      return;
    }
*/
    for (int j = 0; j < number_of_outputs_; j++)
    {
      x = switchEndianness(dout_[j]);
      for (int i = 0; i < 16; i++)
      {
        data.digital_outputs[i + 16 * j] = x & 1;
        x >>= 1;
      }
    }
  }

  void getIntData(robotnik_msgs::Registers& registers)
  {
    int16_t x;
    int iret;

    // Read digital 16 bit inputs registers. Each bit is an input
    iret = modbus_read_registers(mb_, read_registers_addr_, number_of_registers_to_read_, tab_reg_);
    if (iret != number_of_registers_to_read_)
    {
      dealWithModbusError();
      return;
    }
    for (int j = 0; j < number_of_registers_to_read_; j++)
    {
      x = switchEndianness(tab_reg_[j]);
      robotnik_msgs::Register reg;
      reg.id = read_registers_addr_ + j;
      reg.value = x;
      registers_.registers.push_back(reg);
    }
  }

  void deviceStatus(diagnostic_updater::DiagnosticStatusWrapper& status)
  {
    if (!running_)
      status.summary(2, "modbus_io is stopped");
    else if (!was_slow_.empty())
    {
      status.summary(1, "Excessive delay");
      was_slow_.clear();
    }
    else
      status.summary(0, "modbus_io is running");

    status.add("Error count", error_count_);
    status.add("Excessive delay", slow_count_);
  }

  void dealWithModbusError()
  {
    ROS_WARN("modbus_io::error: %s (errorno: %d)", modbus_strerror(errno), errno);
    modbus_errors_++;
  }

  //------------------------------------------------------------------
  // SERVICE WRITE DIGITAL OUTPUT
  // req.output (int8)
  // req.value (bool)
  //------------------------------------------------------------------
  // req.ret (bool)
  //------------------------------------------------------------------

  bool write_digital_output_srv(robotnik_msgs::set_digital_output::Request& req,
                                robotnik_msgs::set_digital_output::Response& res)
  {
    // pthread_mutex_lock(&lock_);

    int iret = -1;
    uint16_t register_value, shift_bit;  // register value, bit
    int out = req.output;
    // ROS_INFO("modbus_io::write_digital_output_srv: out %d to %d", out, req.value);
    if (out <= 0)
    {
      if (req.value)
      {
        register_value = 0xFFFF;
        ROS_DEBUG("modbus_io::write_digital_output_srv: ALL OUTPUTS ENABLED (out = %d)", out);
      }
      else
      {
        register_value = 0x0000;
        ROS_DEBUG("modbus_io::write_digital_output_srv: ALL OUTPUTS DISABLED (out = %d)", out);
      }
      register_value = switchEndianness(register_value);
      iret = modbus_write_registers(mb_, digital_outputs_addr_, number_of_outputs_, dout_);
      if (iret != number_of_outputs_)
      {
        dealWithModbusError();
        res.ret = false;
      }
    }
    else
    {
      req.output -= 1;
      if (req.output > this->digital_outputs_ - 1)
      {
        res.ret = false;
        ROS_ERROR_THROTTLE(10, "modbus_io::write_digital_output_srv: OUTPUT NUMBER %d OUT OF RANGE [1 -> %d]", req.output + 1,
                  this->digital_outputs_);
        pthread_mutex_unlock(&lock_);
        res.ret = false;
      }
      else
      {
        int base_address = req.output / 16;
        int output_number_in_register = req.output % 16;
        shift_bit = (uint16_t)1 << output_number_in_register;  // shifts req.output number to the left
        if (req.value)
        {
          register_value = dout_[base_address] | shift_bit;
        }
        else
        {
          register_value = dout_[base_address] & ~shift_bit;
        }
        ROS_DEBUG("modbus_io::write_digital_output_srv service request: OUTPUT=%d, VALUE=%d", (int)req.output + 1,
                  (int)req.value);

        register_value = switchEndianness(register_value);
        dout_[base_address] = register_value;
        iret = modbus_write_registers(mb_, digital_outputs_addr_, number_of_outputs_, dout_);
        // ROS_INFO("modbus_io::write_digital_output_srv service request: OUTPUT=%d, VALUE=%d, address = %d",
        // (int)req.output + 1,
        //        (int)req.value,digital_outputs_addr_);
        if (iret != number_of_outputs_)
        {
          dealWithModbusError();
          res.ret = false;
        }
      }
    }
    if (iret < 0)
    {
      res.ret = false;
    }
    else
    {
      res.ret = true;
    }
    // pthread_mutex_unlock(&lock_);
    return true;
  }

  // Writes registers by using function modbus_write_register
  bool set_modbus_register_cb(robotnik_msgs::set_modbus_register::Request& req,
                              robotnik_msgs::set_modbus_register::Response& res)
  {
    res.ret = false;

    int iret = modbus_write_register(mb_, req.address, (uint16_t)req.value);
    if (iret != number_of_outputs_)
    {
      dealWithModbusError();
      res.ret = false;
    }
    res.ret = true;

    return true;
  }

  // Writes registers by using function modbus_write_registers based on initial digital_outputs_addr_
  bool set_modbus_registers_cb(robotnik_msgs::set_modbus_register::Request& req,
                               robotnik_msgs::set_modbus_register::Response& res)
  {
    int reg = req.address - digital_outputs_addr_;
    int dout_length = number_of_outputs_;  // sizeof(*dout_);
    // ROS_WARN("modbus_io::set_modbus_register_cb: reg %d to %d", req.address, req.value);
    res.ret = false;

    if (reg > 0 && reg < dout_length)
    {
      dout_[reg] = switchEndianness((uint16_t)req.value);
      int iret = modbus_write_registers(mb_, digital_outputs_addr_, number_of_outputs_, dout_);
      // ROS_INFO("modbus_io::set_modbus_registers_cb: reg = %d, address = %d, value = %x", reg,
      // digital_outputs_addr_+reg, dout_[reg] );
      if (iret != number_of_outputs_)
      {
        dealWithModbusError();
        res.ret = false;
      }
      else
        res.ret = true;
    }
    else
    {
      ROS_ERROR_THROTTLE(10, "modbus_io::set_modbus_registers_cb: register out of range: reg = %d, length allowed = %d", reg,
                dout_length);
    }
    return true;
  }

  bool get_modbus_register_cb(robotnik_msgs::get_modbus_register::Request& req,
                              robotnik_msgs::get_modbus_register::Response& res)
  {
    int iret = modbus_read_registers(mb_, req.address, 1, &res.value);
    if (iret != 1)
    {
      dealWithModbusError();
      res.ret = false;
      return true;
    }
    res.ret = true;
    return true;
  }

  void switchToState(int new_state)
  {
    if (new_state == state)
      return;

    // saves the previous state
    previous_state = state;
    ROS_INFO("modbus_io::SwitchToState: %s -> %s", getStateString(state), getStateString(new_state));
    state = new_state;

    switch (state)
    {
      case robotnik_msgs::State::READY_STATE:
        // reseting the errors
        modbus_errors_ = 0;

        break;
      case robotnik_msgs::State::FAILURE_STATE:
        // Inits timer for recovery
        communication_error_time = ros::Time::now();

        break;
    }
  }

  /*!	\fn char *getStateString(int state)
   *	\brief Gets the state as a string
   */
  char* getStateString(int state)
  {
    switch (state)
    {
      case robotnik_msgs::State::INIT_STATE:
        return (char*)"INIT";
        break;
      case robotnik_msgs::State::STANDBY_STATE:
        return (char*)"STANDBY";
        break;
      case robotnik_msgs::State::READY_STATE:
        return (char*)"READY";
        break;
      case robotnik_msgs::State::EMERGENCY_STATE:
        return (char*)"EMERGENCY";
        break;
      case robotnik_msgs::State::FAILURE_STATE:
        return (char*)"FAILURE";
        break;
      case robotnik_msgs::State::SHUTDOWN_STATE:
        return (char*)"SHUTDOWN";
        break;
      default:
        return (char*)"UNKNOWN";
        break;
    }
  }
};

// TODO: SIGNINT HANDLER??

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robotnik_modbus_io");

  ros::NodeHandle nh;

  modbusNode mn(nh);
  mn.spin();

  return (0);
}
