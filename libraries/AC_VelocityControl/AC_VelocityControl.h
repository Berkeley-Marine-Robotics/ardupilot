#pragma once

// @file AC_VeclocityControl.h
// BMR Velocity control library

#include <AP_Logger/AP_Logger.h>        
#include <AP_Param/AP_Param.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_InertialNav/AP_InertialNav.h>      // Inertial Navigation library
#include <AP_Motors/AP_Motors.h>                // motors library


// Velocity control default definitions
#define VEL_X_P_DEFAULT 1.0f
#define VEL_X_I_DEFAULT 0.3f
#define VEL_X_D_DEFAULT 0.01f

#define VEL_Y_P_DEFAULT 1.0f
#define VEL_Y_I_DEFAULT 0.4f
#define VEL_Y_D_DEFAULT 0.01f

#define VEL_Z_P_DEFAULT 0.3f
#define VEL_Z_I_DEFAULT 0.001f
#define VEL_Z_D_DEFAULT 0.01f

#define DIS_A_DEFAULT 1.0f


class AC_VelocityControl{
public:

    // Constructor
    AC_VelocityControl(AP_AHRS_View & ahrs, const AP_InertialNav& inav,
                            AP_Motors & motors, 
                            float dt);

    // Initialize control
    void init_velocity_control();

    // Update control
    void update_velocity_control();

    // Send commands to motors
    void velocity_controller_run();

    // Set target velocity
    void set_target_velocity(const Vector3f& velocity);
    // void set_last_target_velocity(const Vector3f& velocity);

    // Save last target velocity;
    // void save_target_velocity();

    // Load last target velocity
    // void load_last_target_velocity();

    // Get measured velocity directly from DVL
    void get_measured_velocity(const Vector3f& velocity);

    // Get measured distance from DVL
    void get_measured_distance(const float& dist);

    // Log data
    void log_data();

    static const struct AP_Param::GroupInfo var_info[];


protected:

    // Parameters
    float _dt;
    const float _taux_max;
    const float _tauy_max;
    const float _tauz_max;

    // Control Inputs/Outputs
    Vector3f _vel_target;
    Vector3f _vel_meas;

    AP_Float _d_target;          // maximum avoidance distance
    Vector3f _d_meas;           // Measured distance 

    Vector3f _tau;

    // Velocity Control Errors
    Vector3f _error;
    Vector3f _error_integrator;
    Vector3f _error_derivative;
    Vector3f _last_error;

    // Control Gains
    AP_Float _K_p_x;
    AP_Float _K_i_x;
    AP_Float _K_d_x;

    AP_Float _K_p_y;
    AP_Float _K_i_y;
    AP_Float _K_d_y;

    AP_Float _K_p_z;
    AP_Float _K_i_z;
    AP_Float _K_d_z;

    // Position Control Errors
    float _error_pos_z;
    float _error_integrator_pos_z;
    float _error_derivative_pos_z;
    float _last_error_pos_z;

    // Kalman filter parameters
    // AP_Float _z_est;
    // AP_Float _P;

    // Pilot override
    // int _flag_override;
    // Vector3f _last_vel_target;    

    // references to inertial nav and ahrs libraries
    AP_AHRS_View &                  _ahrs;
    const AP_InertialNav &          _inav;
    AP_Motors &               _motors; //AP_MotorsMulticopter &          _motors_multi; 
};