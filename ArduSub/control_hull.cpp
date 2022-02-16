// BMR Hull following flight mode
// DVL required - Deadk reckoning
#include "Sub.h"

// hull_init - initialize hull following controller
bool Sub::hull_init()
{

	// TBD

	return true;
}


// hull run - runs hull following controller
void Sub::hull_run()
{
	velocity_control.log_data();

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