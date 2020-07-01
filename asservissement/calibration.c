#include "calibration.h"

#include "lowlevel/encoders.h"

const double Pi =  3.14159265359;

// Robot mechanical caracteristics, dimensions in mm

const double Encoders_Axis_Distance  = 193.0;
const double Encoders_Wheel_Diameter_l = 62;
const double Encoders_Wheel_Diameter_r = 60.0;
const double Encoders_Dist_Per_Step_l  = Encoders_Wheel_Diameter_l * Pi / ENCODER_PERIOD;
const double Encoders_Dist_Per_Step_r  = Encoders_Wheel_Diameter_r * Pi / ENCODER_PERIOD;
const double Encoders_Theta_Per_Diff_l = Encoders_Dist_Per_Step_l / Encoders_Axis_Distance;
const double Encoders_Theta_Per_Diff_r = Encoders_Dist_Per_Step_r / Encoders_Axis_Distance;


/*
for PID calibration, set one of the 2 PID to zero (Kp,Ki,Kd)
and follow: https://fr.wikipedia.org/wiki/M%C3%A9thode_de_Ziegler-Nichols
put Ki and Kd to 0,
set the pid orders to 0
increase Kp until oscillation (Kp is Ku) but no divergence and mesure the oscillation period Tu
set Kp to 0.6*Ku
set Ki to Kp*2/Tu
set Kd to Kp*Tu/8
*/
const PID_Configuration PID_Configuration_sigma = {//oscillation at Ku=0.05 period 0.25s
  .Te = 0.01,//in seconds

  .Kp = 0.05*0.6,
  .Ki = 0.05*0.6*2/0.25,
  .Kd = 0.05*0.6*0.25/8,

  .max_eps = 100000000,
  .position_tolerance = 3,//in mm
  .speed_tolerance = 1 //in mm/s
};

const PID_Configuration PID_Configuration_theta = {//oscillation at Ku=0.05 period 0.25s
  .Te = 0.01,

  .Kp = 0.05*0.6,
  .Ki = 0.05*0.6*2/0.25,
  .Kd = 0.05*0.6*0.25/8,

  .max_eps = 350,
  .position_tolerance =  Encoders_Axis_Distance*1.0*Pi/180.0,//in rad
  .speed_tolerance =  Encoders_Axis_Distance*0.5*Pi/180 //in rad/s
};
