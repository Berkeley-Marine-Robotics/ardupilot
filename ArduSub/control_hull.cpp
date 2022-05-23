// BMR Hull following flight mode
// DVL required - Deadk reckoning
#include "Sub.h"

// hull_init - initialize hull following controller
bool Sub::hull_init()
{

    // Attitude control initialization
    attitude_control.relax_attitude_controllers();
    last_pilot_heading = ahrs.yaw_sensor;

    // attitude hold inputs become thrust inputs in manual mode
    // set to neutral to prevent chaotic behavior (esp. roll/pitch)
    set_neutral_controls();

    //// PID Velocity initialization
//    velocity_control.init_velocity_control();

    //// PID Position Initialization
    position_control.init_position_control();

	return true;
}


// hull run - runs hull following controller
void Sub::hull_run()
{

	uint32_t tnow = AP_HAL::millis();
    // uint32_t dt = tnow - _last_t;
	
	////////////////// Motors //////////////////
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
	////////////////////////////////////////////


    ////////////////// Velocity Control //////////////////
    // Velocity control
//    velocity_control.update_velocity_control();

    // Log velocity control data
//    velocity_control.log_data();

    ////////////////// Position Control //////////////////
    // Position control
    position_control.update_position_control();

    // Log position control data
    position_control.log_data();

    //////////////////////////////////////////////////////

    ////////////////// Update attitude //////////////////
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
    /////////////////////////////////////////////////////

}

// Set control target velocity
void Sub::hull_set_target_velocity(const Vector3f& velocity)
{
    velocity_control.set_target_velocity(velocity);
}

void Sub::hull_set_target_position(const Vector3f& position)
{
    position_control.set_target_position(position);
}

void Sub::hull_get_measured_position(const Vector3f& position)
{
    position_control.get_measured_position(position);
}

// Get velocity from direct DVL measurements -> Comment out function contents for EKF
void Sub::hull_get_dvl_vel(const float& dt, const Vector3f &delAng, 
    const Vector3f &delPos,float quality)
{
    Vector3f vel_dvl = Vector3f(delPos.x/dt, delPos.y/dt, delPos.z/dt);
    velocity_control.get_measured_velocity(vel_dvl);

    AP::logger().Write("GETV", "TimeUS,DX,DY,DZ", "Qfff",
    	AP_HAL::micros64(),
        (double)delPos.x,
        (double)delPos.y,
        (double)delPos.z
        );
}

// Get distance from DVL in meters
void Sub::hull_get_dvl_alt(const float &dist)
{
    velocity_control.get_measured_distance(dist);

    AP::logger().Write("GETA", "TimeUS,altfloat3", "Qf",
        AP_HAL::micros64(),
        (double) dist);

}
