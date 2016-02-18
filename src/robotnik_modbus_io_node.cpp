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

#include <assert.h>
#include <boost/format.hpp>
#include <iostream>
#include "modbus.h"
#include <stdio.h>
#include <typeinfo>

#include <signal.h>
#include "ros/time.h"
#include "self_test/self_test.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"

#include "std_srvs/Empty.h"
#include "std_msgs/Bool.h"
#include <robotnik_msgs/inputs_outputs.h>
#include <robotnik_msgs/set_digital_output.h>


#define MODBUS_DESIRED_FREQ	10.0

int MODBUS_DEFAULT_DIGITAL_OUTPUTS = 8;
int MODBUS_DEFAULT_DIGITAL_INPUTS  = 8;

//int MODBUS_DEFAULT_MIN_DIGITAL_OUTPUTS = 8;	// Min. number of digital outputs (factory default)
int MODBUS_DEFAULT_MIN_DIGITAL_OUTPUTS  = 4;
int MODBUS_DEFAULT_MIN_DIGITAL_INPUTS	= 8;	// Min. number of digital inputs (factory default)


using namespace std;

class modbusNode
{
	
public:
	//Robotnik_msgs object
	robotnik_msgs::inputs_outputs reading_;
	
	//tcp/ip data
	string ip_address_;
	int port_;

	//ROS objects
	self_test::TestRunner self_test_;
	diagnostic_updater::Updater diagnostic_;

	ros::NodeHandle node_handle_;
	ros::NodeHandle private_node_handle_;
	ros::Publisher elevator_modbus_io_data_pub_;
	ros::ServiceServer elevator_modbus_io_write_digital_srv_;
	ros::ServiceServer elevator_modbus_io_write_digital_input_srv_;

	bool running;
	// Config params
	int digital_inputs_;
	int digital_outputs_;
	int digital_inputs_addr_;
	int digital_outputs_addr_;
	//int digital_outputs_addr1_;
    //int digital_outputs_addr2_;

	// Error counters and flags
	int error_count_;
	int slow_count_;
	std::string was_slow_;
	std::string error_status_;

	double desired_freq_;
	diagnostic_updater::FrequencyStatus freq_diag_;

	// Modbus member variables
	modbus_t *mb_;
	uint16_t tab_reg_[32];
	uint16_t din_;  // used to read and save digital inputs
	uint16_t dout_;  // used to read digital outputs
    uint16_t dout384_; // store digital output registers to activate each one separatedly (not use)
    uint16_t dout385_; // store digital output registers to activate each one separatedly (not use)

	float max_delay;
	
	//Constructor
	modbusNode(ros::NodeHandle h): 
	self_test_(), diagnostic_(), node_handle_(h), private_node_handle_("~"), error_count_(0), slow_count_(0), desired_freq_(20),
	freq_diag_(diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.05))
	{
		running = false;
		ros::NodeHandle modbus_node_handle(node_handle_, "modbus_io");
		// READ PARAMS
		private_node_handle_.param("ip_address", ip_address_, string("127.0.0.1"));
		private_node_handle_.param("port", port_, 502);
		private_node_handle_.param("digital_outputs", digital_outputs_, MODBUS_DEFAULT_DIGITAL_OUTPUTS);
		private_node_handle_.param("digital_inputs", digital_inputs_, 	MODBUS_DEFAULT_DIGITAL_INPUTS);
		private_node_handle_.param("digital_inputs_addr", digital_inputs_addr_, 0);
		//private_node_handle_.param("digital_outputs_addr1", digital_outputs_addr1_, 384);
		//private_node_handle_.param("digital_outputs_addr2", digital_outputs_addr2_, 385); //not used
		private_node_handle_.param("digital_outputs_adrr", digital_outputs_addr_, 100); //new used

		// Checks the min num of digital outputs
		/*if(digital_outputs_ < MODBUS_DEFAULT_MIN_DIGITAL_OUTPUTS){
			digital_outputs_ = MODBUS_DEFAULT_MIN_DIGITAL_OUTPUTS;
			ROS_INFO("elevator_elevator_modbus_io: Setting num of digital outputs to the minimum value = %d", MODBUS_DEFAULT_MIN_DIGITAL_OUTPUTS);
		}
		// Checks the min num of digital inputs
		if(digital_inputs_ < MODBUS_DEFAULT_MIN_DIGITAL_INPUTS){
			digital_inputs_ = MODBUS_DEFAULT_MIN_DIGITAL_INPUTS;
			ROS_INFO("elevator_elevator_modbus_io: Setting num of digital inputs to the minimum value = %d", MODBUS_DEFAULT_MIN_DIGITAL_INPUTS);
		}
		*/

		//ROS_INFO("elevator_modbus_io: Settings -> DO = %d (register %d %d), DI = %d (register %d)",
		//digital_outputs_, digital_outputs_addr1_, digital_outputs_addr2_, digital_inputs_, digital_inputs_addr_ );
		
		ROS_INFO("modbus_io: Settings -> DO = %d (register %d), DI = %d (register %d)",
		digital_outputs_, digital_outputs_addr_, digital_inputs_, digital_inputs_addr_ );

		ROS_INFO("modbus_io: %d", digital_outputs_addr_);
 
		elevator_modbus_io_data_pub_ = modbus_node_handle.advertise<robotnik_msgs::inputs_outputs>("input_output", 100);

		elevator_modbus_io_write_digital_srv_ = modbus_node_handle.advertiseService("write_digital_output", &modbusNode::write_digital_output, this);
		elevator_modbus_io_write_digital_input_srv_ = modbus_node_handle.advertiseService("write_digital_input", &modbusNode::write_digital_input, this);

		self_test_.add("Connect Test", this, &modbusNode::ConnectTest);

		diagnostic_.add( freq_diag_ );

		diagnostic_.add( "Device Status", this, &modbusNode::deviceStatus );

		// Initializes the outputs/inputs vector. Setup
		reading_.digital_inputs.resize(digital_inputs_);
		reading_.digital_outputs.resize(digital_outputs_);
		max_delay = 1.0 / MODBUS_DESIRED_FREQ;

		din_= 0;
		dout_= 0;
		//dout384_ = 0;
        //dout385_ = 0;
    }

	//Destructor
	~modbusNode()
	{
		stop();
	}

	int start()
	{
		stop();

		mb_=modbus_new_tcp(ip_address_.c_str(),port_);
		ROS_INFO("start: connecting to %s:%d", ip_address_.c_str(), port_);
		if (modbus_connect(mb_)== -1){
			ROS_ERROR ("modbus_io::start - Connection Error!");
			return -1;
		}

		ROS_INFO("Connected to MODBUS IO BOARD at %s on port %d", ip_address_.c_str(), port_ );
		freq_diag_.clear();

		running = true;

		return(0);
	}

	int stop()
	{
		if(running)
		{
			modbus_close(mb_);
			modbus_free(mb_);
			running = false;
			ROS_INFO("Closing modbus connection");
		}
		ROS_INFO("STOP");
		return(0);
	}

	int read_and_publish()
	{
		static double prevtime = 0;

		double starttime = ros::Time::now().toSec();
		if (prevtime && prevtime - starttime > 0.05)
		{
			//ROS_WARN("Full elevator_modbus_io loop took %f ms. Nominal is 10ms.", 1000 * (prevtime - starttime));
			was_slow_ = "Full modbus_io loop was slow.";
			slow_count_++;
		}

		getData(reading_);

		double endtime = ros::Time::now().toSec();
		if (endtime - starttime > max_delay)
		{
			//ROS_WARN("Gathering data took %f ms. Nominal is 10ms.", 1000 * (endtime - starttime));
			was_slow_ = "Full elevator_modbus_imterface loop was slow.";
			slow_count_++;
		}
		prevtime = starttime;
		starttime = ros::Time::now().toSec();
		elevator_modbus_io_data_pub_.publish(reading_);

		endtime = ros::Time::now().toSec();
		if (endtime - starttime > max_delay)
		{
			ROS_WARN("Publishing took %f ms. Nominal is 10 ms.", 1000 * (endtime - starttime));
			was_slow_ = "Full modbus_io loop was slow.";
			slow_count_++;
		}

		freq_diag_.tick();
		return(0);
	}

	bool spin()
	{
		ros::Rate r(MODBUS_DESIRED_FREQ);
		while (!ros::isShuttingDown()) // Using ros::isShuttingDown to avoid restarting the node during a shutdown.
		{

			if (start() == 0)
			{
				while(node_handle_.ok()) {
					if(read_and_publish() < 0)
						break;
					self_test_.checkTest();
					diagnostic_.update();
					ros::spinOnce();
					r.sleep();
				}
			} else {
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

	void getData(robotnik_msgs::inputs_outputs& data)
	{
		// Adress	 Description
		// 0000 	 1 to 8 inputs module IL ETH
		// 0001 	 1 to 4 outuputs module IL ETH
		
		int16_t x;
		// Read digital 16 bit inputs registers. Each bit is an input
		modbus_read_registers(mb_, digital_inputs_addr_, 1, tab_reg_);
		//ROS_INFO("elevator_modbus_io::getData - Read %d ",tab_reg_[0]);
		x = tab_reg_[0];
		//din_ = tab_reg_[0];
		din_ = x;
		for (int i=0; i<8; i++) {
			data.digital_inputs[i] = x&1;
			x>>=1;
		}
		/*for (int i=0; i<8; i++) {
			data.digital_inputs[i] = din_&1;
			din_>>=1;
		}*/
		
		//ROS_INFO("modbus_io::write_getData: VALUE=%d", (int)tab_reg_[0]);

		// Read digital outputs
		// modbus_read_registers(mb_, 1, 1, tab_reg_);
		//modbus_read_registers(mb_, digital_outputs_addr1_, 1, tab_reg_); //384
		modbus_read_registers(mb_, digital_outputs_addr_, 1, tab_reg_);
		x = tab_reg_[0];
		dout_ = x;
		//dout_ = tab_reg_[0];
		//dout384_ = dout_;
		for (int i=0; i<8; i++) {
			data.digital_outputs[i] = x&1;
			x>>=1;
		}
		
		/*for (int i=0; i<8; i++) {
			data.digital_outputs[i] = dout_&1;
			dout_>>=1;
		}*/
		
		/*
		modbus_read_registers(mb_, digital_outputs_addr2_, 1, tab_reg_); //385
		dout_ = x = tab_reg_[0];
		for (int i=4; i<8; i++) {
			data.digital_outputs[i] = x&1;
			x>>=1;
		}
		dout385_ = dout_;
		*/

		//ROS_INFO("getData: %x", dout_);

	}

	void deviceStatus(diagnostic_updater::DiagnosticStatusWrapper &status)
	{
		if (!running)
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
	
	
	//------------------------------------------------------------------
	//SERVICE WRITE DIGITAL OUTPUT
	//req.output (int8)
	//req.value (bool)
	//------------------------------------------------------------------
	//req.ret (bool)
	//------------------------------------------------------------------

	bool write_digital_output(
		robotnik_msgs::set_digital_output::Request &req, 
		robotnik_msgs::set_digital_output::Response &res){
			
		int iret;
		uint16_t register_value, shift_bit;	//register value, bit
		int out = req.output;
		
		if(out <= 0){
			if (req.value){
				register_value = 0xFF;
				ROS_INFO("modbus_io::write_digital_output: ALL OUTPUTS ENABLED (out = %d)", out);
			}else{
				register_value = 0x00;
				ROS_INFO("modbus_io::write_digital_output: ALL OUTPUTS DISABLED (out = %d)", out);
			}
			iret=modbus_write_register(mb_, digital_outputs_addr_, register_value);
		}else{
			req.output -= 1;
			if(req.output > this->digital_outputs_-1){
				res.ret = false;
				ROS_ERROR("modbus_io::write_digital_output: OUTPUT NUMBER %d OUT OF RANGE [1 -> %d]", req.output+1, this->digital_outputs_);
				return false;
			}else{
				shift_bit = (uint16_t) 1<<req.output; //shifts req.output number to the left
				if (req.value){
					register_value = dout_ | shift_bit;
				}else{
					register_value = dout_ & ~shift_bit;
				}
				iret=modbus_write_register(mb_, digital_outputs_addr_, register_value);
				ROS_INFO("modbus_io::write_digital_output service request: OUTPUT=%d, VALUE=%d", (int)req.output+1, (int)req.value);
			}
		}
		if (iret < 0) {
			res.ret = false;
		}else{
			res.ret = true;
		}
		return res.ret;
	}
			
		/*
			
			
		if (/*(req.output <= 0) || req.output > this->digital_outputs_) { */
		/*	res.ret = false;
			ROS_ERROR("modbus_io::write_digital_output: Error on the output number %d. Out of range [1 -> %d]", req.output, this->digital_outputs_);
			return false;
		}
		req.output -= 1;	*/ // Internally the device uses outputs from [0 to max_outputs - 1]
		
		
		// For negative output number, disables all the output no matter the output value
		/*
		if(out <= 0){
			register_value = 0;
			iret=modbus_write_register(mb_, digital_outputs_addr1_, register_value);
			iret=modbus_write_register(mb_, digital_outputs_addr2_, register_value);
			ROS_INFO("modbus_io::write_digital_output: Disabling all outputs (out = %d)", out);
		}else{
			if (req.output < MODBUS_DEFAULT_MIN_DIGITAL_OUTPUTS) {// 384
				shift_bit = (uint16_t) 1<<req.output;
				if (req.value){
					register_value = dout384_ | shift_bit;
				}else{
					register_value = dout384_ & ~shift_bit;
				}//ROS_INFO("WORD 384=%x", register_value);
				iret=modbus_write_register(mb_, digital_outputs_addr1_, register_value);
			}
			else {
				shift_bit = (uint16_t) 1<<(req.output-4);  //
				if (req.value){
					register_value = dout385_ | shift_bit;
				}else{
					register_value = dout385_ & ~shift_bit;
				}//ROS_INFO("WORD 385 =%x", register_value);
				iret=modbus_write_register(mb_, digital_outputs_addr2_, register_value);
			}*/
		/*
		ROS_INFO("modbus_io::write_digital_output service request: OUTPUT=%d, VALUE=%d", (int)req.output+1, (int)req.value );
		}
		if (iret < 0) {
			res.ret = false;
		}else{
			res.ret = true;
		}
		return res.ret; 
	}*/
	
	// Used for testing
	//------------------------------------------------------------------
	//SERVICE WRITE DIGITAL INPUT
	//req.output (int8)
	//req.value (bool)
	//------------------------------------------------------------------
	//req.ret (bool)
	//------------------------------------------------------------------
	
	bool write_digital_input(
		robotnik_msgs::set_digital_output::Request &req,
		robotnik_msgs::set_digital_output::Response &res
	){
		
		int iret;
		uint16_t register_value, shift_bit;	//register value, bit
		int in = req.output;
		
		if(in <= 0){
			if (req.value){
				register_value = 0xFF;
				ROS_INFO("modbus_io::write_digital_input: ALL INPUTS ENABLED (in = %d)", in);
			}else{
				register_value = 0x00;
				ROS_INFO("modbus_io::write_digital_input: ALL INPUTS DISABLED (in = %d)", in);
			}
			iret=modbus_write_register(mb_, digital_inputs_addr_, register_value);
		}else{
			req.output -= 1;
			if(req.output > this->digital_inputs_-1){
				res.ret = false;
				ROS_ERROR("modbus_io::write_digital_input: INPUT NUMBER %d OUT OF RANGE [1 -> %d]", req.output+1, this->digital_inputs_);
				return false;
			}else{
				shift_bit = (uint16_t) 1<<req.output; //shifts req.output number to the left
				if (req.value){
					register_value = din_ | shift_bit;
				}else{
					register_value = din_ & ~shift_bit;
				}
				iret=modbus_write_register(mb_, digital_inputs_addr_, register_value);
				ROS_INFO("modbus_io::write_digital_input service request: INPUT=%d, VALUE=%d", (int)req.output+1, (int)req.value);
			}
		}
		if (iret < 0) {
			res.ret = false;
		}else{
			res.ret = true;
		}
		return res.ret;

		/*if ((req.output > this->digital_inputs_)) {
			res.ret = false;
			ROS_ERROR("modbus_io::write_digital_input: Error on the output number %d. Out of range [1 -> %d]", req.output, this->digital_inputs_);
			return false;
		}
		req.output -= 1;	// Internally the device uses outputs from [0 to max_outputs - 1]
		int iret;
		uint16_t register_value, shift_bit;
		
		if (req.output < MODBUS_DEFAULT_MIN_DIGITAL_INPUTS) {
			
			// IF output is zero, set all the inputs to the desired value
			if(req.output < 0){
				if (req.value)
					register_value = 0xFF;
				else
					register_value = 0;
			}else{
				shift_bit = (uint16_t) 1<<req.output;  //
				if (req.value){
					register_value = din_ | shift_bit;
				}else{
					register_value = din_ & ~shift_bit;
				}
			}
			
			iret=modbus_write_register(mb_, digital_inputs_addr_, register_value);
		}
		
		ROS_INFO("modbus_io::write_digital_input: service request: output=%d, value=%d, ret=%d", (int)req.output+1, (int)req.value, iret );
		if (iret < 0) {
			res.ret = false;
		}else{
			res.ret = true;
		}
		return res.ret;*/
	}



};
//SIGNINT HANDLER??
int main(int argc, char** argv)
{
  ros::init(argc, argv, "modbus_io");

  ros::NodeHandle nh;

  modbusNode mn(nh);
  mn.spin();

  return(0);
}
