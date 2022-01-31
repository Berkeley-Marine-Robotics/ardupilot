#pragma once

// @file AC_VeclocityControl.h
// BMR Velocity control library


       
#include <AP_Param/AP_Param.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_InertialNav/AP_InertialNav.h>      // Inertial Navigation library
//#include <AP_Motors/AP_Motors.h>                // motors library
//#include <AP_Motors/AP_MotorsMulticopter.h>




class AC_VelocityControl{
public:

    // Constructor
    AC_VelocityControl(AP_AHRS_View & ahrs, const AP_InertialNav& inav,
                            //AP_Motors & motors, 
                            float dt,
                            float Px, float Ix, float Dx,
                            float Py, float Iy, float Dy,
                            float Pz, float Iz, float Dz);

    // Initialize control
    void init_velocity_control();

    // Set target velocity
    void set_target_velocity(const Vector3f& velocity);

    // Run control
    Vector3f update_velocity_control();

    static const struct AP_Param::GroupInfo var_info[];


protected:

    // Parameters
    float _dt;

    // Inputs/Outputs
    Vector3f _vel_target;
    Vector3f _vel_meas;
    Vector3f _tau;

    // Variables
    Vector3f _error;
    Vector3f _error_integrator;
    Vector3f _error_derivative;
    Vector3f _last_error;

    // Gains
    AP_Float K_p_x;
    AP_Float K_i_x;
    AP_Float K_d_x;

    AP_Float K_p_y;
    AP_Float K_i_y;
    AP_Float K_d_y;

    AP_Float K_p_z;
    AP_Float K_i_z;
    AP_Float K_d_z;



    // references to inertial nav and ahrs libraries
    AP_AHRS_View &                  _ahrs;
    const AP_InertialNav &          _inav;
    // AP_Motors &               _motors; //AP_MotorsMulticopter &          _motors_multi; 




};