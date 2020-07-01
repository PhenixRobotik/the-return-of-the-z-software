#pragma once

extern const double Pi;

// Robot mechanical caracteristics, dimensions in mm

extern const double Encoders_Axis_Distance;
extern const double Encoders_Wheel_Diameter_l;
extern const double Encoders_Wheel_Diameter_r;
extern const double Encoders_Dist_Per_Step_l;
extern const double Encoders_Dist_Per_Step_r;
extern const double Encoders_Theta_Per_Diff_l;
extern const double Encoders_Theta_Per_Diff_r;

// PID Calibration
#include "pid.h"

extern const PID_Configuration
  PID_Configuration_sigma,
  PID_Configuration_theta;
