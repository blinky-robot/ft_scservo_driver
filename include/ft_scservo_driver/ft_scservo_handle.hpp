/*
 * Copyright (c) 2015, Scott K Logan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _scservo_handle_hpp
#define _scservo_handle_hpp

#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <ros/ros.h>

#include "ft_scservo_driver/ft_scservo_driver.hpp"

namespace ft_scservo_driver
{
	class SCServoHandle
	{
	public:
		enum OPERATION_MODE
		{
			OPERATION_MODE_POSITION = 0,
			OPERATION_MODE_VELOCITY,
		};

		SCServoHandle(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv);
		~SCServoHandle();

		void registerPositionHandles(hardware_interface::PositionActuatorInterface &api);
		void registerStateHandles(hardware_interface::ActuatorStateInterface &asi);
		void registerVelocityHandles(hardware_interface::VelocityActuatorInterface &avi);

		void read();
		void write();

	private:
		const ros::NodeHandle nh;
		const ros::NodeHandle nh_priv;

		ft_scservo_driver::SCServoBus scs;
		int servo_count;
		std::vector<int> servos;
		std::string this_name;

		std::vector<enum OPERATION_MODE> actuator_mode;
		std::vector<bool> actuator_engaged;

		std::vector<double> actuator_cmd_pos;
		std::vector<double> actuator_cmd_vel;
		std::vector<double> actuator_eff;
		std::vector<double> actuator_pos;
		std::vector<double> actuator_vel;

		std::vector<hardware_interface::ActuatorStateHandle> actuator_handle;
		std::vector<hardware_interface::ActuatorHandle> actuator_handle_cmd_pos;
		std::vector<hardware_interface::ActuatorHandle> actuator_handle_cmd_vel;
	};
}

#endif /* _scservo_handle_hpp */
