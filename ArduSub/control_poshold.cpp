// ArduSub position hold flight mode
// GPS required
// Jacob Walser August 2016

#include "Sub.h"

#if POSHOLD_ENABLED == ENABLED

// poshold_init - initialise PosHold controller
bool Sub::poshold_init()
{
    // // fail to initialise PosHold mode if no GPS lock
    // if (!position_ok()) {
    //     return false;
    // }
    pos_control.init_vel_controller_xyz();
    pos_control.set_desired_velocity_xy(0, 0);
    // pos_control.set_target_to_stopping_point_xy();

    // initialize vertical speeds and acceleration
    pos_control.set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control.set_max_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    pos_control.set_alt_target(inertial_nav.get_altitude()); //-> Not needed
    // pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z()); //-> initialize to 0
    pos_control.set_desired_velocity_z(0);

    // Stop all thrusters
    attitude_control.set_throttle_out(0.5f ,true, g.throttle_filt);
    attitude_control.relax_attitude_controllers();
    pos_control.relax_alt_hold_controllers();

    last_pilot_heading = ahrs.yaw_sensor;

    return true;
}

// poshold_run - runs the PosHold controller
// should be called at 100hz or more
void Sub::poshold_run()
{
    uint32_t tnow = AP_HAL::millis();
    // When unarmed, disable motors and stabilization
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when not auto-armed (i.e. on the ground, pilot has never raised throttle)
        attitude_control.set_throttle_out(0.5f ,true, g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        pos_control.set_target_to_stopping_point_xy();
        pos_control.relax_alt_hold_controllers();
        last_pilot_heading = ahrs.yaw_sensor;
        return;
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // ++++++++++++++++++++++++ Tests >= D3 ++++++++++++++++++++++
        ///////////////////////
    // NEW update xy outputs //
    // Get pilot input
    float pilot_lateral = channel_lateral->norm_input();
    float pilot_forward = channel_forward->norm_input();
    // // Set desired velocity
    // pos_control.set_desired_velocity_xy(0,0);
    // Velocity control or Pilot override
    if (fabsf(pilot_forward) > 0.1 || fabsf(pilot_lateral) > 0.1 ) { // Forward/lateral input above 10%
        motors.set_forward(pilot_forward);
        motors.set_lateral(pilot_lateral);
    } else { // hold speed
        pos_control.update_xy_controller();      
    }   


    // ///////////////////////
    // // OLD update xy outputs //
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
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 

    // -------------------- Tests D1, D2, ..., D14 ---------------
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
    // -----------------------------------------------------------------

    // =========================== Tests D2, D3, ..., D14 ==============
    // Update z axis //
    // control_depth();
    // =================================================================
}
// guided_set_velocity - sets guided mode's target velocity
void Sub::poshold_set_velocity(const Vector3f& velocity)
{
    // // check we are in velocity control mode
    // if (guided_mode != Guided_Velocity) {
    //     guided_vel_control_start();
    // }

    // vel_update_time_ms = AP_HAL::millis();

    // set position controller velocity target
    pos_control.set_desired_velocity(velocity);
}

void Sub::poshold_send_dvl(const float& dt, const Vector3f &delAng, 
    const Vector3f &delPos,float quality)
{
    // DIRECT DVL MEASUREMENTS
    // Vector3f meas_velocity = Vector3f(delPos.x/dt, delPos.y/dt, delPos.z/dt);
    // pos_control.set_vehicle_velocity(meas_velocity);

}


#endif  // POSHOLD_ENABLED == ENABLED
