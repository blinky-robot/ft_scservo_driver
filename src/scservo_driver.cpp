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

#include "ft_scservo_driver/scservo_driver.hpp"

namespace ft_scservo_driver
{
	/**
	 * Public Data
	 */
	const int SCServoBus::default_baud = (int)SC_BAUD_1M;
	const std::string SCServoBus::default_port = "/dev/ttyACM0";
	const int SCServoBus::default_timeout = SC_TIMEOUT_DEFAULT;

	/**
	 * Public Functions
	 */
	Exception::Exception(const enum SC_ERROR sc_error)
		: ros::Exception(sc_strerror(sc_error)),
		  sc_error(sc_error)
	{
	}

	SCServoBus::SCServoBus(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv)
		: diag_timer(nh.createTimer(ros::Duration(1.0), &SCServoBus::diagTimerCallback, this)),
		  nh(nh),
		  nh_priv(nh_priv),
		  scd(-1),
		  this_name("scservo_driver")
	{
		nh_priv.param("baud", baud, SCServoBus::default_baud);
		nh_priv.param("port", port, SCServoBus::default_port);
		nh_priv.param("servos", servo_list, servo_list);
		nh_priv.param("timeout", timeout, SCServoBus::default_timeout);
	}

	SCServoBus::~SCServoBus()
	{
		if (scd >= 0)
		{
			sc_close(scd);
		}
	}

	void SCServoBus::close()
	{
		boost::recursive_mutex::scoped_lock lock(io_mutex);

		if (scd >= 0)
		{
			sc_close(scd);
			scd = -1;
			ROS_INFO_NAMED(this_name, "Disconnected from bus");
		}
	}

	void SCServoBus::getStatus(int id, double &velocity, double &position, double &effort)
	{
		boost::recursive_mutex::scoped_lock lock(io_mutex);

		if (!SCServoBus::stat())
		{
			SCServoBus::start();

			if (!SCServoBus::stat())
			{
				throw Exception(SC_ERROR_NOT_CONNECTED);
			}
		}

		if (servos.find(id) == servos.end())
		{
			throw Exception(SC_ERROR_INVALID_PARAM);
		}

		servos[id]->getStatus(velocity, position, effort);
	}

	void SCServoBus::open()
	{
		boost::recursive_mutex::scoped_lock lock(io_mutex);

		if (SCServoBus::stat())
		{
			ROS_DEBUG_NAMED(this_name, "Tried to open the port, but it is already open");
			return;
		}

		scd = sc_open(port.c_str(), (enum SC_BAUD)baud, (uint8_t)timeout);
		if (scd < 0)
		{
			ROS_WARN_THROTTLE_NAMED(1, this_name, "Attempt to open device '%s' failed: %s", port.c_str(), sc_strerror(scd));
			return;
		}

		ROS_INFO_NAMED(this_name, "Connected to bus at '%s'", port.c_str());
	}

	void SCServoBus::setEnableTorque(int id, bool enable)
	{
		boost::recursive_mutex::scoped_lock lock(io_mutex);

		if (!SCServoBus::stat())
		{
			SCServoBus::start();

			if (!SCServoBus::stat())
			{
				throw Exception(SC_ERROR_NOT_CONNECTED);
			}
		}

		if (servos.find(id) == servos.end())
		{
			throw Exception(SC_ERROR_INVALID_PARAM);
		}

		servos[id]->setEnableTorque(enable);
	}

	void SCServoBus::setPosition(int id, double position)
	{
		boost::recursive_mutex::scoped_lock lock(io_mutex);

		if (!SCServoBus::stat())
		{
			SCServoBus::start();

			if (!SCServoBus::stat())
			{
				throw Exception(SC_ERROR_NOT_CONNECTED);
			}
		}

		if (servos.find(id) == servos.end())
		{
			throw Exception(SC_ERROR_INVALID_PARAM);
		}

		servos[id]->setPosition(position);
	}

	bool SCServoBus::stat()
	{
		boost::recursive_mutex::scoped_lock lock(io_mutex);

		return (scd < 0) ? false : true;
	}

	void SCServoBus::start()
	{
		boost::recursive_mutex::scoped_lock lock(io_mutex);

		if (!SCServoBus::stat())
		{
			SCServoBus::open();
		}

		diag_timer.start();

		if (SCServoBus::stat())
		{
			ROS_DEBUG_NAMED(this_name, "Bringing up %u servos", (int)servo_list.size());

			for (std::vector<int>::iterator it = servo_list.begin(); it != servo_list.end(); it++)
			{
				if (*it < 0 || *it > SC_MAX_ID)
				{
					ROS_ERROR_NAMED(this_name, "Invalid servo ID '%d'", *it);
				}
				else if (servos.find(*it) == servos.end())
				{
					try
					{
						servos[(uint8_t)*it] = new SCServoBus::Servo(this, nh_priv, (uint8_t)*it);
					}
					catch (Exception &e)
					{
						if (servos.find(*it) != servos.end())
						{
							delete servos[(uint8_t)*it];
							servos.erase((uint8_t)*it);
						}

						ROS_ERROR_NAMED(this_name, "Communication failure: %s", e.what());

						SCServoBus::close();

						break;
					}
				}
			}
		}
	}

	void SCServoBus::stop()
	{
		boost::recursive_mutex::scoped_lock lock(io_mutex);

		diag_timer.stop();

		for (std::map<int, SCServoBus::Servo *>::iterator it = servos.begin(); it != servos.end(); it++)
		{
			delete it->second;
			servos.erase(it);
		}
	}

	/**
	 * Private Functions
	 */
	void SCServoBus::diagTimerCallback(const ros::TimerEvent &event)
	{
		ROS_DEBUG_NAMED(this_name, "Diagnostic callback");

		if (!SCServoBus::stat())
		{
			SCServoBus::start();
		}

		for (std::map<int, SCServoBus::Servo *>::iterator it = servos.begin(); it != servos.end() && SCServoBus::stat(); it++)
		{
			boost::recursive_mutex::scoped_lock lock(io_mutex);

			try
			{
				it->second->diagUpdate();
			}
			catch(Exception &e)
			{
				if (e.sc_error != SC_ERROR_CHECKSUM_FAILURE)
				{
					ROS_ERROR_NAMED(this_name, "Communication failure: %s", e.what());

					SCServoBus::close();
				}
			}
		}

		diag_timer.setPeriod(ros::Duration(1.0));
	}

	SCServoBus::Servo::Servo(class SCServoBus *parent, const ros::NodeHandle &nh_priv, const uint8_t id)
		: id(id),
		  nh_priv(ros::NodeHandle(nh_priv, SCServoBus::Servo::getNodeHandleName())),
		  diag(parent->nh, SCServoBus::Servo::nh_priv),
		  dyn_re_cb(boost::bind(&SCServoBus::Servo::dynReCallback, this, _1, _2)),
		  dyn_re_srv(NULL),
		  parent(parent),
		  rad_offset(-1.890499397),
		  rad_per_tick(-0.003695991),
		  relax_service(SCServoBus::Servo::nh_priv.advertiseService("relax", &SCServoBus::Servo::relaxCallback, this)),
		  tense_service(SCServoBus::Servo::nh_priv.advertiseService("tense", &SCServoBus::Servo::tenseCallback, this)),
		  this_name(getMessageName())
	{
		std::stringstream str;

		str << "Servo " << (int)id << " Status";

		diag.setHardwareIDf("Servo '%hhu' on '%s'", id, parent->port.c_str());
		diag.add(str.str(), this, &SCServoBus::Servo::queryDiagnostics);
		diag.force_update();

		SCServoBus::Servo::mergeSettings();

		dyn_re_srv = new dynamic_reconfigure::Server<ft_scservo_driver::ServoConfig>(dyn_re_mutex, SCServoBus::Servo::nh_priv);
		dyn_re_srv->setCallback(dyn_re_cb);

		ROS_DEBUG_NAMED(this_name, "Added servo at id '%hhu'", id);
	}

	SCServoBus::Servo::~Servo()
	{
		delete dyn_re_srv;
	}

	bool SCServoBus::Servo::ping()
	{
		int ret;

		boost::recursive_mutex::scoped_lock lock(parent->io_mutex);

		ret = sc_ping(parent->scd, id);

		switch (ret)
		{
		case 0:
			return false;
		case 1:
			return true;
		default:
			throw Exception((enum SC_ERROR)ret);
		}
	}

	void SCServoBus::Servo::diagUpdate()
	{
		diag.update();
	}

	void SCServoBus::Servo::dynReCallback(ft_scservo_driver::ServoConfig &config, uint32_t level)
	{
		int ret;

		boost::recursive_mutex::scoped_lock lock(parent->io_mutex);

		ROS_DEBUG_NAMED(this_name, "Dynamic Reconfigure Callback");

		last_settings.min_angle_limit = config.min_angle_limit;
		last_settings.max_angle_limit = config.max_angle_limit;
		last_settings.limit_temperature = config.limit_temperature;
		last_settings.max_limit_voltage = (uint8_t)(config.max_limit_voltage * 10 + 0.5);
		last_settings.min_limit_voltage = (uint8_t)(config.min_limit_voltage * 10 + 0.5);
		last_settings.max_torque = config.max_torque;
		last_settings.compliance_p = config.compliance_p;
		last_settings.compliance_d = config.compliance_d;
		last_settings.compliance_i = config.compliance_i;
		last_settings.imax = config.imax;

		rad_offset = config.rad_offset;
		rad_per_tick = config.rad_per_tick;

		ret = sc_write_settings(parent->scd, id, &last_settings);
		if (ret < SC_SUCCESS)
		{
			ROS_ERROR_NAMED(this_name, "Failed to update servo settings: %s", sc_strerror(ret));
		}

		// Propagate back to DynRe so the precision matches
		config.max_limit_voltage = last_settings.max_limit_voltage / 10.0;
		config.min_limit_voltage = last_settings.min_limit_voltage / 10.0;
	}

	ft_scservo_driver::ServoConfig SCServoBus::Servo::getConfig()
	{
		ft_scservo_driver::ServoConfig cfg;
		int ret;

		ret = sc_read_settings(parent->scd, id, &last_settings);
		if (ret < SC_SUCCESS)
		{
			throw Exception((enum SC_ERROR)ret);
		}

		cfg.min_angle_limit = last_settings.min_angle_limit;
		cfg.max_angle_limit = last_settings.max_angle_limit;
		cfg.limit_temperature = last_settings.limit_temperature;
		cfg.max_limit_voltage = last_settings.max_limit_voltage / 10.0;
		cfg.min_limit_voltage = last_settings.min_limit_voltage / 10.0;
		cfg.max_torque = last_settings.max_torque;
		cfg.compliance_p = last_settings.compliance_p;
		cfg.compliance_d = last_settings.compliance_d;
		cfg.compliance_i = last_settings.compliance_i;
		cfg.imax = last_settings.imax;

		cfg.rad_offset = rad_offset;
		cfg.rad_per_tick = rad_per_tick;

		return cfg;
	}

	std::string SCServoBus::Servo::getMessageName() const
	{
		std::stringstream str;

		str << "scservo_driver" << ".servo_" << (int)id;

		return str.str();
	}

	std::string SCServoBus::Servo::getNodeHandleName() const
	{
		std::stringstream str;

		str << "servo_" << (int)id;

		return str.str();
	}

	void SCServoBus::Servo::getStatus(double &velocity, double &position, double &effort)
	{
		int ret;
		struct sc_status status;

		ret = sc_read_status(parent->scd, id, &status);
		if (ret < SC_SUCCESS)
		{
			throw Exception((enum SC_ERROR)ret);
		}

		velocity = status.present_speed;
		position = status.present_position * rad_per_tick - rad_offset;
		effort = status.present_load;
	}

	void SCServoBus::Servo::mergeSettings()
	{
		ROS_DEBUG_NAMED(this_name, "Merging settings for servo at id '%hhu'", id);

		boost::recursive_mutex::scoped_lock lock(dyn_re_mutex);

		// Start with the settings already on the servo
		ft_scservo_driver::ServoConfig cfg = SCServoBus::Servo::getConfig();

		// Update any settings in the parameter server
		cfg.__fromServer__(nh_priv);
		cfg.__clamp__();

		// Update the parameter server
		if (dyn_re_srv != NULL)
		{
			dyn_re_srv->updateConfig(cfg);
		}
		else
		{
			cfg.__toServer__(nh_priv);
		}
	}

	void SCServoBus::Servo::queryDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
	{
		int ret;
		struct sc_diag diag;
		struct sc_status status;
		uint8_t torque_enable;

		ROS_DEBUG_NAMED(this_name, "Getting diagnostics for servo at id '%hhu'", id);

		ret = sc_read_status_and_diag(parent->scd, id, &status, &diag);
		if (ret < SC_SUCCESS)
		{
			throw Exception((enum SC_ERROR)ret);
		}

		ret = sc_read_torque_enable(parent->scd, id, &torque_enable);
		if (ret < SC_SUCCESS)
		{
			throw Exception((enum SC_ERROR)ret);
		}

		stat.addf("Fault Code", "%s (%d)", sc_strfault((enum SC_FAULT)diag.error), (int)diag.error);
		stat.add("Load", status.present_load);
		stat.add("Position", status.present_position);
		stat.add("Speed", status.present_speed);
		stat.add("Temperature", (int)diag.temperature);
		stat.add("Torque Enabled", (bool)torque_enable);
		stat.add("Voltage", diag.voltage / 10.0f);

		if (diag.error == 0)
		{
			stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Servo Communication OK");
		}
		else
		{
			stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Servo is reporting error(s)");
		}
	}

	bool SCServoBus::Servo::relaxCallback(std_srvs::Empty::Request &reques, std_srvs::Empty::Response &response)
	{
		int ret;

		boost::recursive_mutex::scoped_lock lock(parent->io_mutex);

		ROS_DEBUG_NAMED(this_name, "Relaxing servo at id '%hhu'", id);

		ret = sc_write_torque_enable(parent->scd, id, 0);
		if (ret < SC_SUCCESS)
		{
			ROS_ERROR_NAMED(this_name, "Servo Communication Failure: %s", sc_strerror(ret));
			return false;
		}

		return true;
	}

	bool SCServoBus::Servo::tenseCallback(std_srvs::Empty::Request &reques, std_srvs::Empty::Response &response)
	{
		int ret;

		boost::recursive_mutex::scoped_lock lock(parent->io_mutex);

		ROS_DEBUG_NAMED(this_name, "Tensing servo at id '%hhu'", id);

		ret = sc_write_torque_enable(parent->scd, id, 1);
		if (ret < SC_SUCCESS)
		{
			ROS_ERROR_NAMED(this_name, "Servo Communication Failure: %s", sc_strerror(ret));
			return false;
		}

		return true;
	}

	void SCServoBus::Servo::setEnableTorque(bool enable)
	{
		int ret;

		ret = sc_write_torque_enable(parent->scd, id, enable ? 1 : 0);
		if (ret < SC_SUCCESS)
		{
			throw Exception((enum SC_ERROR)ret);
		}
	}

	void SCServoBus::Servo::setPosition(double position)
	{
		uint16_t pos;
		int ret;

		position = (position + rad_offset) / rad_per_tick;

		if (position < 0 || position > 1023)
		{
			throw Exception(SC_ERROR_INVALID_PARAM);
		}

		pos = (position + 0.5);

		ret = sc_write_goal(parent->scd, id, 25, pos);
		if (ret < SC_SUCCESS)
		{
			throw Exception((enum SC_ERROR)ret);
		}
	}
}
