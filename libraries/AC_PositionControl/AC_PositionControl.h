
/*
@file AC_PositionControl.h
 BMR Position control library
 This library is created towards the swarm control for sub
 Idea is to track the position. We are talking about body frame for position tracking.
 x Direction: UUV1 will track the position of leader boat and UUV2 will track the position of UUV1
 y Direction:  Both of them will track distance from boat hull.
 Z Direction: Both of them will maintain given specific depths.
*/

#pragma once

#include <AP_Logger/AP_Logger.h>
#include <AP_Param/AP_Param.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_InertialNav/AP_InertialNav.h>
#include <AP_Motors/AP_Motors.h>

// Position control default gain definitions
// TODO: Find the appropriate values, right now just copied from AC_VelocityControl.h
#define POS_X_P_DEFAULT 1.0f
#define POS_X_I_DEFAULT 0.3f
#define POS_X_D_DEFAULT 0.01f

#define POS_Y_P_DEFAULT 1.0f
#define POS_Y_I_DEFAULT 0.4f
#define POS_Y_D_DEFAULT 0.01f

#define POS_Z_P_DEFAULT 1.0f
#define POS_Z_I_DEFAULT 0.001F
#define POS_Z_D_DEFAULT 0.01f


class AC_PositionControl{
public:

    // Constructor
    AC_PositionControl(AP_AHRS_View& ahrs,
                       const AP_InertialNav& inav,
                       AP_Motors& motors,
                       float dt);

    // Initialize Control
    void init_position_control();

    // Update Control
    void update_position_control();

    // Send commands to motors
    void run_position_control();

    // Get target position
    void set_target_position(const Vector3f& target_position);

    // Get measured position
    void get_measured_position (const Vector3f& measured_position);
//                                const float measured_position_x, // from laser sensor
//                                const float measured_position_y, // from DVL, distance from hull
//                                const float measured_position_z); // from DVL, distance from water surface

    // Log data
    void log_data();

    static const struct AP_Param::GroupInfo var_info[];

protected:
    float _dt;
    const float _Fx_max;
    const float _Fy_max;
    const float _Fz_max;

    // measured/target positions
    Vector3f _pos_target;
    Vector3f _pos_measured;

    Vector3f _F;

    // Errors
    Vector3f _error;
    Vector3f _error_integral;
    Vector3f _error_derivative;
    Vector3f _previous_error;


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

    // references to inertial nav and ahrs libraries
    AP_AHRS_View& _ahrs;
    const AP_InertialNav& _inav;
    AP_Motors& _motors;

    //For Printing
    int count{0};
    Vector3f _current_position_from_home;
    Vector3f _current_position_from_origin;

    // HUll Location
    float   hull_location{20};

};





