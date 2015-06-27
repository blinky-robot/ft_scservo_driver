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

#include "ft_scservo_driver/ft_scservo_handle.hpp"

namespace ft_scservo_driver
{
	SCServoHandle::SCServoHandle(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv)
		: nh(nh),
		  nh_priv(nh_priv),
		  scs(nh, nh_priv),
		  servo_count(0),
		  this_name("scservo_driver")
	{
		std::stringstream strstr;
		std::string actuator_name;
		std::string op_mode;

		scs.start();

		nh_priv.param("servos", servos, servos);

		servo_count = servos.size();

		if (servo_count > 0)
		{
			for (std::vector<int>::iterator it = servos.begin(); it != servos.end(); it++)
			{
				strstr << "servo_" << *it;
				if (!nh_priv.getParam(strstr.str() + "/actuator_name", actuator_name))
				{
					actuator_name = strstr.str();
					ROS_WARN_NAMED(this_name, "Actuator name for servo at id '%d' is not set - defaulting to '%s'", *it, actuator_name.c_str());
				}
				if (!nh_priv.getParam(strstr.str() + "/operation_mode", op_mode))
				{
					actuator_mode.push_back(OPERATION_MODE_POSITION);
				}
				else
				{
					if (op_mode == "profile_velocity" || op_mode == "vel" || op_mode == "velocity")
					{
						actuator_mode.push_back(OPERATION_MODE_VELOCITY);
					}
					else
					{
						if (op_mode != "profile_position" && op_mode != "pos" && op_mode != "position")
						{
							ROS_WARN_NAMED(this_name, "Invalid operation_mode value '%s' - defaulting to position control", op_mode.c_str());
						}

						actuator_mode.push_back(OPERATION_MODE_POSITION);
					}
				}

				actuator_engaged.push_back(false);

				actuator_cmd_pos.push_back(std::numeric_limits<double>::quiet_NaN());
				actuator_cmd_vel.push_back(std::numeric_limits<double>::quiet_NaN());
				actuator_eff.push_back(std::numeric_limits<double>::quiet_NaN());
				actuator_pos.push_back(std::numeric_limits<double>::quiet_NaN());
				actuator_vel.push_back(std::numeric_limits<double>::quiet_NaN());

				actuator_handle.push_back(hardware_interface::ActuatorStateHandle(actuator_name, &actuator_pos.back(), &actuator_vel.back(), &actuator_eff.back()));
				actuator_handle_cmd_pos.push_back(hardware_interface::ActuatorHandle(actuator_handle.back(), &actuator_cmd_pos.back()));
				actuator_handle_cmd_vel.push_back(hardware_interface::ActuatorHandle(actuator_handle.back(), &actuator_cmd_vel.back()));
			}
		}
	}

	SCServoHandle::~SCServoHandle()
	{
		scs.stop();
	}

	void SCServoHandle::registerPositionHandles(hardware_interface::PositionActuatorInterface &api)
	{
		int i;

		for (i = 0; i < servo_count; i++)
		{
			if (actuator_mode[i] == OPERATION_MODE_POSITION)
			{
				api.registerHandle(actuator_handle_cmd_pos[i]);
			}
		}
	}

	void SCServoHandle::registerStateHandles(hardware_interface::ActuatorStateInterface &asi)
	{
		int i;

		for (i = 0; i < servo_count; i++)
		{
			asi.registerHandle(actuator_handle[i]);
		}
	}

	void SCServoHandle::registerVelocityHandles(hardware_interface::VelocityActuatorInterface &avi)
	{
		int i;

		for (i = 0; i < servo_count; i++)
		{
			if (actuator_mode[i] == OPERATION_MODE_VELOCITY)
			{
				avi.registerHandle(actuator_handle_cmd_vel[i]);
			}
		}
	}

	void SCServoHandle::read()
	{
		int i;

		for (i = 0; i < servo_count; i++)
		{
			try
			{
				scs.getStatus(servos[i], actuator_vel[i], actuator_pos[i], actuator_eff[i]);
			}
			catch(ft_scservo_driver::Exception &e)
			{
				ROS_WARN_NAMED(this_name, "Failed to read state of servo at id '%d': %s", servos[i], e.what());
			}
		}
	}

	void SCServoHandle::write()
	{
		int i;

		for (i = 0; i < servo_count; i++)
		{
			try
			{
				switch(actuator_mode[i])
				{
				default:
				case OPERATION_MODE_POSITION:
					if (isnan(actuator_cmd_pos[i]))
					{
						if (actuator_engaged[i])
						{
							scs.setEnableTorque(servos[i], false);
							actuator_engaged[i] = false;
						}
					}
					else
					{
						if (!actuator_engaged[i])
						{
							scs.setEnableTorque(servos[i], true);
							actuator_engaged[i] = true;
						}

						scs.setPosition(servos[i], actuator_cmd_pos[i]);
					}
					break;
				case OPERATION_MODE_VELOCITY:
					scs.setVelocity(servos[i], actuator_cmd_vel[i]);
					break;
				}
			}
			catch(ft_scservo_driver::Exception &e)
			{
				ROS_WARN_NAMED(this_name, "Failed to command servo at id '%d': %s", servos[i], e.what());
			}
		}
	}
}
