// ArduSub position hold flight mode
// GPS required
// Jacob Walser August 2016

#include "Sub.h"
#include <cmath>

#if POSHOLD_ENABLED == ENABLED

// poshold_init - initialise PosHold controller
bool Sub::poshold_init()
{

    //// From PosHold mode
    // // fail to initialise PosHold mode if no GPS lock
    // if (!position_ok()) {
    //     return false;
    // }
    // pos_control.init_vel_controller_xyz();
    // pos_control.set_desired_velocity_xy(0, 0);
    // pos_control.set_target_to_stopping_point_xy();

    // // initialize vertical speeds and acceleration
    // pos_control.set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    // pos_control.set_max_accel_z(g.pilot_accel_z);

    // // initialise position and desired velocity
    // pos_control.set_alt_target(inertial_nav.get_altitude());
    // pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());

    // Stop all thrusters
    // attitude_control.set_throttle_out(0.5f ,true, g.throttle_filt);

    // pos_control.relax_alt_hold_controllers();

    // Attitude control initialization
    attitude_control.relax_attitude_controllers();
    last_pilot_heading = ahrs.yaw_sensor;

    //// From Manual mode
    // set target altitude to zero for reporting
    // pos_control.set_alt_target(0);

    // attitude hold inputs become thrust inputs in manual mode
    // set to neutral to prevent chaotic behavior (esp. roll/pitch)
    set_neutral_controls();

    //// PID initialization
    // TBD...


    return true;
}

// assign PID gais for each degree-of-freedom

float K_p_x= 200;
float K_i_x= 20;
float K_d_x= 20;

float K_p_y= 200;
float K_i_y= 20;
float K_d_y= 20;

float K_p_z= 200;
float K_i_z= 20;
float K_d_z= 20;

float error_integrator_x = 0;
float error_integrator_y = 0;
float error_integrator_z = 0;

float _last_t = 0;

float _last_error_x =  0;
float _last_error_y =  0;
float _last_error_z =  0;

// poshold_run - runs the PosHold controller
// should be called at 100hz or more
void Sub::poshold_run()
{
    uint32_t tnow = AP_HAL::millis();
    uint32_t dt = tnow - _last_t;

    // When unarmed, disable motors and stabilization
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when not auto-armed (i.e. on the ground, pilot has never raised throttle)
        attitude_control.set_throttle_out(0.5f ,true, g.throttle_filt); // 0 in Manual mode??
        attitude_control.relax_attitude_controllers();
        // pos_control.set_target_to_stopping_point_xy();
        // pos_control.relax_alt_hold_controllers();
        last_pilot_heading = ahrs.yaw_sensor;
        return;
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // XYZ control
    float pilot_lateral = channel_lateral->norm_input();
    float pilot_forward = channel_forward->norm_input();
    float pilot_down = channel_throttle->norm_input();
    printf("Pilot commands\n");
    printf("forward: %.2f\n", pilot_forward);
    printf("lateral: %.2f\n", pilot_lateral);
    printf("down: %.2f\n", pilot_down);

    if (fabsf(pilot_lateral) > 0.05f || fabsf(pilot_forward) > 0.05f || fabsf(pilot_down-0.5f) > 0.05f) {
        /// Pilot override
        motors.set_throttle(pilot_down);
        motors.set_forward(pilot_forward);
        motors.set_lateral(pilot_lateral);
    } else{
        //// Velocity control
        // Print target velocity
        printf("Target velocity in body frame:\n");
        printf("Vx target: %.2f cm/s\n", _vel_target.x);
        printf("Vy target: %.2f cm/s\n", _vel_target.y);
        printf("Vz target: %.2f cm/s\n", _vel_target.z);
        // Get measured velocity
        // EKF
        float velEKF_x = inertial_nav.get_velocity().x;
        float velEKF_y = inertial_nav.get_velocity().y;
        _vel_meas.x =  velEKF_x * ahrs_view.cos_yaw() + velEKF_y * ahrs_view.sin_yaw();
        _vel_meas.y = -velEKF_x * ahrs_view.sin_yaw() + velEKF_y* ahrs_view.cos_yaw();
        _vel_meas.z = inertial_nav.get_velocity().z;   
        printf("Horizontal estimated velocity in body frame:\n");
        printf("Vx estimate: %.2f cm/s\n", _vel_meas.x);
        printf("Vy estimate: %.2f cm/s\n", _vel_meas.y);   
        printf("Vz estimate: %.2f cm/s\n", _vel_meas.z);   

        /// PID Control
        Vector3f error = _vel_target - _vel_meas; 

        error_integrator_x += error.x*dt;
        error_integrator_y += error.y*dt;
        error_integrator_z += error.z*dt;

        float error_derivative_x = (error.x - _last_error_x) / dt;
        float error_derivative_y = (error.y - _last_error_y) / dt;
        float error_derivative_z = (error.z - _last_error_z) / dt;
        
        float tau_x = K_p_x*error.x + K_i_x*error_integrator_x + K_d_x*error_derivative_x;
        float tau_y = K_p_x*error.y + K_i_y*error_integrator_y + K_d_y*error_derivative_y;
        float tau_z = K_p_x*error.z + K_i_z*error_integrator_z + K_d_z*error_derivative_z;
        printf("Control inputs:\n");
        printf("tau_x: %.2f N\n", tau_x);
        printf("tau_y: %.2f N\n", tau_y);   
        printf("tau_z: %.2f N\n", tau_z); 

        // Send Control to motors -> Overriden by attitude control??
        motors.set_forward(tau_x);
        motors.set_lateral(tau_y);
        motors.set_throttle(tau_z+0.5f);

        // Update
        _last_t= tnow;
        _last_error_x= error.x;
        _last_error_y= error.y;
        _last_error_z= error.z;
    
    }

    ///////////////////////
    // // update xy outputs //
    // float pilot_lateral = channel_lateral->norm_input();
    // float pilot_forward = channel_forward->norm_input();

    // float lateral_out = 0;
    // float forward_out = 0;

    // pos_control.set_desired_velocity_xy(0,0);

    // if (position_ok()) {
    //     // Allow pilot to reposition the sub
    //     if (fabsf(pilot_lateral) > 0.1 || fabsf(pilot_forward) > 0.1) {
    //         pos_control.set_target_to_stopping_point_xy();
    //     }
    //     translate_pos_control_rp(lateral_out, forward_out);
    //     pos_control.update_xy_controller();
    // } else {
    //     pos_control.init_vel_controller_xyz();
    //     pos_control.set_desired_velocity_xy(0, 0);
    //     pos_control.set_target_to_stopping_point_xy();
    // }
    // motors.set_forward(forward_out + pilot_forward);
    // motors.set_lateral(lateral_out + pilot_lateral);
    /////////////////////
    // Update attitude //

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    // update attitude controller targets
    if (!is_zero(target_yaw_rate)) { // call attitude controller with rate yaw determined by pilot input
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        last_pilot_heading = ahrs.yaw_sensor;
        last_pilot_yaw_input_ms = tnow; // time when pilot last changed heading

    } else { // hold current heading

        // this check is required to prevent bounce back after very fast yaw maneuvers
        // the inertia of the vehicle causes the heading to move slightly past the point when pilot input actually stopped
        if (tnow < last_pilot_yaw_input_ms + 250) { // give 250ms to slow down, then set target heading
            target_yaw_rate = 0; // Stop rotation on yaw axis

            // call attitude controller with target yaw rate = 0 to decelerate on yaw axis
            attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
            last_pilot_heading = ahrs.yaw_sensor; // update heading to hold

        } else { // call attitude controller holding absolute absolute bearing
            attitude_control.input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, last_pilot_heading, true);
        }
    }

    // // Update z axis //
    // control_depth();
}

// Set control target velocity
void Sub::poshold_set_velocity(const Vector3f& velocity)
{
    _vel_target = velocity;
}

// Get velocity from direct DVL measurements
void Sub::poshold_send_dvl(const float& dt, const Vector3f &delAng, 
    const Vector3f &delPos,float quality)
{
    // // DIRECT DVL MEASUREMENTS
    // _vel_meas = Vector3f(delPos.x/dt, delPos.y/dt, delPos.z/dt);

}



#endif  // POSHOLD_ENABLED == ENABLED
