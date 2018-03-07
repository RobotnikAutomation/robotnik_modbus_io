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
#include <robotnik_msgs/inputs_outputs.h>
#include <robotnik_msgs/set_digital_output.h>
#include <robotnik_msgs/set_modbus_register.h>
#include <robotnik_msgs/get_modbus_register.h>
#include <robotnik_msgs/State.h>
#include <robotnik_modbus_io/write_modbus.h>
#include <robotnik_modbus_io/modbus_inputs.h>

#include <modbus.h>

#include <pthread.h>

#define MODBUS_DESIRED_FREQ 1.0

#define SLAVE_NUMBER 1
#define MODBUS_DEFAULT_DIGITAL_OUTPUTS 80
#define MODBUS_DEFAULT_DIGITAL_INPUTS 40
#define MODBUS_DEFAULT_ANALOG_OUTPUTS 2
#define MODBUS_DEFAULT_MIN_DIGITAL_OUTPUTS 4  // Min. number of digital outputs (factory default)
#define MODBUS_DEFAULT_MIN_DIGITAL_INPUTS 8   // Min. number of digital inputs (factory default)
#define MODBUS_DEFAULT_MIN_ANALOG_INPUTS 0    // Min. number of analog inputs (factory default)

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
  robotnik_modbus_io::modbus_inputs reading_;

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
  ros::Publisher state_pub_;
  ros::ServiceServer modbus_io_write_digital_srv_;
  ros::ServiceServer modbus_io_write_digital_input_srv_;
  ros::ServiceServer modbus_write_srv_;

  ros::ServiceServer set_modbus_register_srv_;
  ros::ServiceServer get_modbus_register_srv_;

  bool running_;
  // Config params
  int writing_addr_;
  int reading_addr_;
  int num_read_reg_;
  bool big_endian_;

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
  uint16_t dout384_;  // store digital output registers to activate each one separatedly (not use)
  uint16_t dout385_;  // store digital output registers to activate each one separatedly (not use)
  uint16_t data_output_[5]; // store the registers to be sent

  float max_delay_;
  //! num of erros in the modbus communication
  int modbus_errors_;
  //! Saves the time of the error
  ros::Time communication_error_time;

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
    private_node_handle_.param("desired_freq", desired_freq_, 10.0);

    private_node_handle_.param("writing_addr", writing_addr_, 0);
    private_node_handle_.param("reading_addr", reading_addr_, 10);

    private_node_handle_.param<bool>("big_endian", big_endian_, MODBUS_DEFAULT_BIG_ENDIAN);
    

    modbus_io_data_pub_ = private_node_handle_.advertise<robotnik_modbus_io::modbus_inputs>("inputs", 100);
    state_pub_ = private_node_handle_.advertise<robotnik_msgs::State>("state", 1);

    modbus_write_srv_ =
        private_node_handle_.advertiseService("write_in_register", &modbusNode::write_multiple_register_srv, this);

    set_modbus_register_srv_ =
        private_node_handle_.advertiseService("set_modbus_register", &modbusNode::set_modbus_register_cb, this);
    get_modbus_register_srv_ =
        private_node_handle_.advertiseService("get_modbus_register", &modbusNode::get_modbus_register_cb, this);

    self_test_.add("Connect Test", this, &modbusNode::ConnectTest);

    diagnostic_.add(freq_diag_);

    diagnostic_.add("Device Status", this, &modbusNode::deviceStatus);

    // Initializes the inputs vector
    reading_.inputs.resize(400);
    num_read_reg_ = 20;

    max_delay_ = 1.0 / MODBUS_DESIRED_FREQ;

    // Initializes the output data vector
    for(int i=0; i<5; i++){
      data_output_[i] = 0;
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
    ROS_INFO("modbus_io::connectModbus: connecting to %s:%d", ip_address_.c_str(), port_);
    mb_ = modbus_new_tcp(ip_address_.c_str(), port_);
    if (mb_ == NULL)
    {
      dealWithModbusError();
      return -1;
    }
    if (modbus_connect(mb_) == -1)
    {
      dealWithModbusError();
      ROS_ERROR("modbus_io::connectModbus: connection Error!");
      return -1;
    }
    ROS_INFO("modbus_io::connectModbus: connected to %s:%d!", ip_address_.c_str(), port_);

    // Set the slave
    int iret = modbus_set_slave(mb_, SLAVE_NUMBER);
    if(iret == -1){
      dealWithModbusError();
      ROS_ERROR("modbus_io::setSlave: Invalid slave ID");
      return -1;
    }
   //modbus_set_response_timeout(mb_, 0, 200000);

    return 0;
  }

  int disconnectModbus()
  {
    modbus_close(mb_);
    modbus_free(mb_);

    ROS_INFO("modbus_io::disconnectModbus: disconnected from %s:%d!", ip_address_.c_str(), port_);
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

    getData(reading_);

    double endtime = ros::Time::now().toSec();
    if (endtime - starttime > max_delay_)
    {
      ROS_WARN("modbus_io::read_and_publish: Gathering data took %f ms. Nominal is %f ms.",
               1000 * (endtime - starttime), 1000 * max_delay_);
      was_slow_ = "Full modbus_interface loop was slow.";
      slow_count_++;
    }
    prevtime = starttime;
    starttime = ros::Time::now().toSec();
    modbus_io_data_pub_.publish(reading_);

    endtime = ros::Time::now().toSec();
    if (endtime - starttime > max_delay_)
    {
      ROS_WARN("modbus_io::read_and_publish: Publishing took %f ms. Nominal is %f ms.", 1000 * (endtime - starttime),
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

  void getData(robotnik_modbus_io::modbus_inputs& data)
  {

    int16_t x;
    int iret;

    // Read digital 16 bit inputs registers. Each bit is an input
    iret = modbus_read_registers(mb_, reading_addr_, num_read_reg_, tab_reg_);
    // return the number of read registers if successful. Otherwise error
    if (iret != num_read_reg_){
      dealWithModbusError();
      return;
    }

     for (int j = 0; j < num_read_reg_; j++){
      x = switchEndianness(tab_reg_[j]);
      for (int i = 0; i < 16; i++)
      {
        data.inputs[i + 16 * j] = x & 1;
        x >>= 1;
      }
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

  // SERVICES

  //------------------------------------------------------------------
  // SERVICE WRITE MULTIPLE REGISTERS
  // req.register (int8)
  // req.bit (int8)
  // req.value (int8)
  //------------------------------------------------------------------
  // req.ret (bool)
  //------------------------------------------------------------------

  bool write_multiple_register_srv(robotnik_modbus_io::write_modbus::Request& req,
                                robotnik_modbus_io::write_modbus::Response& res){
    pthread_mutex_lock(&lock_);

    res.ret = true;
    // There are only 5 registers
    if(req.n_register > 4){
      ROS_WARN("The register must be an integer between 0-4");
      return false;
    }

    // Enable an especific bit
    if(req.value == 1){
      data_output_[req.n_register] = data_output_[req.n_register] | (uint16_t) pow(2, req.bit);
    // Disable an especific bit
    }else{
      data_output_[req.n_register] = data_output_[req.n_register] & ~(uint16_t) pow(2, req.bit);
    }

    int val = modbus_write_registers(mb_, writing_addr_, 5, data_output_);
    // return the number of written registers if successful
    if (val != 5){
      dealWithModbusError();
      res.ret = false;
    }
/*
    for(int i = 0; i < 5; i++){
      ROS_INFO("register %d: value: %d", i, data_output_[i]);
    }
*/
    pthread_mutex_unlock(&lock_);
    return res.ret;

  }

  bool set_modbus_register_cb(robotnik_msgs::set_modbus_register::Request& req,
                              robotnik_msgs::set_modbus_register::Response& res)
  {
    int iret = modbus_write_register(mb_, req.address, req.value);
    if (iret != 1)
    {
      dealWithModbusError();
      res.ret = false;
      return true;
    }
    res.ret = true;
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
