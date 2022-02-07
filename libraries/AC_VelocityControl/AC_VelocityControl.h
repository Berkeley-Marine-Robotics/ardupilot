#pragma once

// @file AC_VeclocityControl.h
// BMR Velocity control library

#include <cmath>
#include <AP_Logger/AP_Logger.h>        
#include <AP_Param/AP_Param.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_InertialNav/AP_InertialNav.h>      // Inertial Navigation library
#include <AP_Motors/AP_Motors.h>                // motors library
//#include <AP_Motors/AP_MotorsMulticopter.h>


// Velocity control default definitions
#define VEL_X_P_DEFAULT 1.0f
#define VEL_X_I_DEFAULT 1.0f
#define VEL_X_D_DEFAULT 1.0f

#define VEL_Y_P_DEFAULT 1.0f
#define VEL_Y_I_DEFAULT 1.0f
#define VEL_Y_D_DEFAULT 1.0f

#define VEL_Z_P_DEFAULT 1.0f
#define VEL_Z_I_DEFAULT 1.0f
#define VEL_Z_D_DEFAULT 1.0f

#define VEL_AVOID_X_P_DEFAULT 1.0f
#define VEL_AVOID_Y_P_DEFAULT 1.0f
#define VEL_AVOID_Z_P_DEFAULT 1.0f

#define DIST_AVOID_NORM_DEFAULT 1.0f


class AC_VelocityControl{
public:

    // Constructor
    AC_VelocityControl(AP_AHRS_View & ahrs, const AP_InertialNav& inav,
                            AP_Motors & motors, 
                            float dt);
                            // float Px, float Ix, float Dx,
                            // float Py, float Iy, float Dy,
                            // float Pz, float Iz, float Dz);

    // Initialize control
    void init_velocity_control();

    // Set target velocity
    void set_target_velocity(const Vector3f& velocity);
    void set_last_target_velocity(const Vector3f& velocity);

    // Update control
    void update_velocity_control();

    // Send commands to motors
    void velocity_controller_run();

    // Save last target velocity;
    void save_target_velocity();

    // Load last target velocity
    void load_last_target_velocity();

    // Set measured velocity directly from DVL
    void set_measured_velocity(const Vector3f& velocity);

    // Set measured distance from DVL
    void set_measured_distance(const float& dist);


    // Log data
    void log_data();

    static const struct AP_Param::GroupInfo var_info[];


protected:

    // Parameters
    float _dt;
    int _flag_override;
    const float _taux_max;
    const float _tauy_max;
    const float _tauz_max;

    // Inputs/Outputs
    Vector3f _vel_target;
    Vector3f _vel_meas;
    Vector3f _tau;
    Vector3f _last_vel_target;

    // Variables
    Vector3f _error;
    Vector3f _error_integrator;
    Vector3f _error_derivative;
    Vector3f _last_error;

    // Gains
    AP_Float _K_p_x;
    AP_Float _K_i_x;
    AP_Float _K_d_x;

    AP_Float _K_p_y;
    AP_Float _K_i_y;
    AP_Float _K_d_y;

    AP_Float _K_p_z;
    AP_Float _K_i_z;
    AP_Float _K_d_z;

    // Avoidance Gains
    AP_Float _K_avoid_x;
    AP_Float _K_avoid_y;
    AP_Float _K_avoid_z;

    // Hull following
    AP_Float _d_avoid; // maximum avoidance distance
    Vector3f _d_meas; // Measured distance
    Vector3f _vel_avoid;  
    Vector3f _error_avoid;

    // references to inertial nav and ahrs libraries
    AP_AHRS_View &                  _ahrs;
    const AP_InertialNav &          _inav;
    AP_Motors &               _motors; //AP_MotorsMulticopter &          _motors_multi; 




};