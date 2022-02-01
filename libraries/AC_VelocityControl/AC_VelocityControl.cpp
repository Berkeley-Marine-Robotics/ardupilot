
#include "AC_VelocityControl.h"


// table of user settable parameters
const AP_Param::GroupInfo AC_VelocityControl::var_info[] = {

    // @Param: VEL_X_P
    // @DisplayName: Kp_x
    // @Description: PID Control gain
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("VEL_X_P", 1, AC_VelocityControl, _K_p_x, VEL_X_P_DEFAULT),

    // @Param: VEL_X_I
    // @DisplayName: Ki_x
    // @Description: PID Control gain
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("VEL_X_I", 2, AC_VelocityControl, _K_i_x, VEL_X_I_DEFAULT),


    // @Param: VEL_X_D
    // @DisplayName: Kd_x
    // @Description: PID Control gain
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("VEL_X_D", 3, AC_VelocityControl, _K_d_x, VEL_X_D_DEFAULT),

    // @Param: VEL_Y_P
    // @DisplayName: Kp_y
    // @Description: PID Control gain
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("VEL_Y_P", 4, AC_VelocityControl, _K_p_y, VEL_Y_P_DEFAULT),

    // @Param: VEL_Y_I
    // @DisplayName: Ki_y
    // @Description: PID Control gain
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("VEL_Y_I", 5, AC_VelocityControl, _K_i_y, VEL_Y_I_DEFAULT),

    // @Param: VEL_Y_D
    // @DisplayName: Kd_y
    // @Description: PID Control gain
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("VEL_Y_D", 6, AC_VelocityControl, _K_d_y, VEL_Y_D_DEFAULT),

    // @Param: VEL_Z_P
    // @DisplayName: Kp_z
    // @Description: PID Control gain
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("VEL_Z_P", 7, AC_VelocityControl, _K_p_z, VEL_Z_P_DEFAULT),

    // @Param: VEL_Z_I
    // @DisplayName: Ki_z
    // @Description: PID Control gain
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("VEL_Z_I", 8, AC_VelocityControl, _K_i_z, VEL_Z_I_DEFAULT),

    // @Param: VEL_Z_D
    // @DisplayName: Kd_z
    // @Description: PID Control gain
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("VEL_Z_D", 9, AC_VelocityControl, _K_d_z, VEL_Z_D_DEFAULT),

    AP_GROUPEND
};


AC_VelocityControl::AC_VelocityControl(AP_AHRS_View & ahrs, const AP_InertialNav& inav,
                            AP_Motors & motors, 
                            float dt):
        _dt(dt),
        _ahrs(ahrs),
        _inav(inav),
        _motors(motors)
        {
           AP_Param::setup_object_defaults(this, var_info);
        }


void AC_VelocityControl::init_velocity_control()
{

    _error_integrator.x = 0;
    _error_integrator.y = 0;
    _error_integrator.z = 0;

    _last_error.x =  0;
    _last_error.y =  0;
    _last_error.z =  0;

}

void AC_VelocityControl::set_target_velocity(const Vector3f& velocity)
{
    _vel_target = velocity;   
}

//// Velocity control
void AC_VelocityControl::update_velocity_control()
{

        // Print PID gains
        printf("PID Control Gains:\n");
        printf("X\n");
        printf("Kpx: %.2f\n", _K_p_x.get());   
        printf("Kix: %.2f\n", _K_i_x.get());   
        printf("Kdx: %.2f\n", _K_d_x.get());    
        printf("Y\n");
        printf("Kpy: %.2f\n", _K_p_y.get());   
        printf("Kiy: %.2f\n", _K_i_y.get());   
        printf("Kdy: %.2f\n", _K_d_y.get());   
        printf("Z\n");
        printf("Kpz: %.2f\n", _K_p_z.get());   
        printf("Kiz: %.2f\n", _K_i_z.get());   
        printf("Kdz: %.2f\n", _K_d_z.get());   

        // Print target velocity
        printf("Target velocity in body frame:\n");
        printf("Vx target: %.2f cm/s\n", _vel_target.x);
        printf("Vy target: %.2f cm/s\n", _vel_target.y);
        printf("Vz target: %.2f cm/s\n", _vel_target.z);
        // Get measured velocity
        // EKF
        float velEKF_x = _inav.get_velocity().x;
        float velEKF_y = _inav.get_velocity().y;
        _vel_meas.x =  velEKF_x * _ahrs.cos_yaw() + velEKF_y * _ahrs.sin_yaw();
        _vel_meas.y = -velEKF_x * _ahrs.sin_yaw() + velEKF_y* _ahrs.cos_yaw();
        _vel_meas.z = _inav.get_velocity().z;   
        printf("Horizontal estimated velocity in body frame:\n");
        printf("Vx estimate: %.2f cm/s\n", _vel_meas.x);
        printf("Vy estimate: %.2f cm/s\n", _vel_meas.y);   
        printf("Vz estimate: %.2f cm/s\n", _vel_meas.z);   

        /// PID Control
        _error = _vel_target - _vel_meas; 

        _error_integrator.x += _error.x*_dt;
        _error_integrator.y += _error.y*_dt;
        _error_integrator.z += _error.z*_dt;

        _error_derivative.x = (_error.x - _last_error.x) / _dt;
        _error_derivative.y = (_error.y - _last_error.y) / _dt;
        _error_derivative.z = (_error.z - _last_error.z) / _dt;
        
        _tau.x = _K_p_x*_error.x + _K_i_x*_error_integrator.x + _K_d_x*_error_derivative.x;
        _tau.y = _K_p_y*_error.y + _K_i_y*_error_integrator.y + _K_d_y*_error_derivative.y;
        _tau.z = _K_p_z*_error.z + _K_i_z*_error_integrator.z + _K_d_z*_error_derivative.z;

        printf("Control inputs:\n");
        printf("tau_x: %.2f N\n", _tau.x);
        printf("tau_y: %.2f N\n", _tau.y);   
        printf("tau_z: %.2f N\n", _tau.z); 

        // Update
        _last_error.x= _error.x;
        _last_error.y= _error.y;
        _last_error.z= _error.z;

}

void AC_VelocityControl::velocity_controller_run()
{
    // Send Control to motors -> Overriden by attitude control??
    _motors.set_forward(_tau.x);
    _motors.set_lateral(_tau.y);
    _motors.set_throttle(_tau.z+0.5f);
}
