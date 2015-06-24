#!/usr/bin/env python
from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()

# Servo params
gen.add("min_angle_limit",	int_t,		0,	"Minimum Angle Limit (ticks)",	0,	0,	1023);
gen.add("max_angle_limit",	int_t,		0,	"Maximum Angle Limit (ticks)",	1023,	0,	1023);
gen.add("limit_temperature",	int_t,		0,	"Temperature Limit",		0,	0,	255);
gen.add("max_limit_voltage",	double_t,	0,	"Maximum Voltage Limit",	0,	0.0,	25.5);
gen.add("min_limit_voltage",	double_t,	0,	"Minimum Voltage Limit",	0.0,	0.0,	25.5);
gen.add("max_torque",		int_t,		0,	"Maximum Torque",		0,	0,	1023);
gen.add("compliance_p",		int_t,		0,	"Proportional Gain",		0,	0,	255);
gen.add("compliance_d",		int_t,		0,	"Derivative Gain",		0,	0,	255);
gen.add("compliance_i",		int_t,		0,	"Integral Gain",		0,	0,	255);
gen.add("imax",			int_t,		0,	"Maximum Integral Value",	0,	0,	255);

# Driver params
gen.add("rad_offset",		double_t,	0,	"Radians center offset",	-1.890499397,	-3.780,	0.0);
gen.add("rad_per_tick",		double_t,	0,	"Radians per servo tick",	-0.003695991,	-0.005,	-0.001);

exit(gen.generate("ft_scservo_driver", "scservo_driver", "Servo"))
