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

#include <boost/thread/mutex.hpp>

#include <scservo/scservo.h>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <dynamic_reconfigure/server.h>
#include <std_srvs/Empty.h>

#include "ft_scservo_driver/ServoConfig.h"

namespace ft_scservo_driver
{
	class Exception : public ros::Exception
	{
	public:
		Exception(const enum SC_ERROR sc_error);

		const enum SC_ERROR sc_error;
	};

	class SCServoBus
	{
	private:
		friend class Servo;
		class Servo
		{
		public:
			Servo(class SCServoBus *parent, const ros::NodeHandle &nh_priv, const uint8_t id);
			~Servo();

			void diagUpdate();
			void getStatus(double &velocity, double &position, double &effort);
			bool ping();
			void setEnableTorque(bool enable);
			void setPosition(double position);

			const uint8_t id;

		private:
			void dynReCallback(ft_scservo_driver::ServoConfig &config, uint32_t level);
			ft_scservo_driver::ServoConfig getConfig();
			std::string getMessageName() const;
			std::string getNodeHandleName() const;
			void mergeSettings();
			void queryDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
			bool relaxCallback(std_srvs::Empty::Request &reques, std_srvs::Empty::Response &response);
			bool tenseCallback(std_srvs::Empty::Request &reques, std_srvs::Empty::Response &response);

			ros::NodeHandle nh_priv;
			diagnostic_updater::Updater diag;
			const dynamic_reconfigure::Server<ft_scservo_driver::ServoConfig>::CallbackType dyn_re_cb;
			boost::recursive_mutex dyn_re_mutex;
			dynamic_reconfigure::Server<ft_scservo_driver::ServoConfig> *dyn_re_srv;
			struct sc_settings last_settings;
			class SCServoBus *parent;
			double rad_offset;
			double rad_per_tick;
			ros::ServiceServer relax_service;
			ros::ServiceServer tense_service;
			const std::string this_name;
		};

	public:
		SCServoBus(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv);
		~SCServoBus();

		void close();
		void getStatus(int id, double &velocity, double &position, double &effort);
		void open();
		void setEnableTorque(int id, bool enable);
		void setPosition(int id, double position);
		void start();
		bool stat();
		void stop();

		static const int default_baud;
		static const std::string default_port;
		static const int default_timeout;

	private:
		void diagTimerCallback(const ros::TimerEvent &event);

		bool active;
		int baud;
		ros::Timer diag_timer;
		boost::recursive_mutex io_mutex;
		const ros::NodeHandle nh;
		const ros::NodeHandle nh_priv;
		std::string port;
		int scd;
		std::vector<int> servo_list;
		std::map<int, SCServoBus::Servo *> servos;
		const std::string this_name;
		int timeout;
	};
}

