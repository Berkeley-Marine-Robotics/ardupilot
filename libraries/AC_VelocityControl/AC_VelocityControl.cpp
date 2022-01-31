
#include "AC_VelocityControl.h"


// table of user settable parameters
const AP_Param::GroupInfo AC_VelocityControl::var_info[] = {

   // AP_GROUPINFO("P",    0, AC_VelocityControl, _dt, 0),

    AP_GROUPEND
};


AC_VelocityControl::AC_VelocityControl(AP_AHRS_View & ahrs, const AP_InertialNav& inav,
                                //AP_Motors & motors, 
                            float dt,
                            float Px, float Ix, float Dx,
                            float Py, float Iy, float Dy,
                            float Pz, float Iz, float Dz):
        _dt(dt),
        _ahrs(ahrs),
        _inav(inav)
        {
           AP_Param::setup_object_defaults(this, var_info);

           K_p_x = Px;
           K_i_x = Ix;
           K_d_x = Dx;
           K_p_y = Py;
           K_i_y = Iy;
           K_d_y = Dy;
           K_p_z = Pz;
           K_i_z = Iz;
           K_d_z = Dz;
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
Vector3f AC_VelocityControl::update_velocity_control()
{

        // Print PID gains
        printf("PID Control Gains:\n");
        printf("X\n");
        printf("Kpx: %.2f\n", K_p_x.get());   
        printf("Kix: %.2f\n", K_i_x.get());   
        printf("Kdx: %.2f\n", K_d_x.get());    
        printf("Y\n");
        printf("Kpy: %.2f\n", K_p_y.get());   
        printf("Kiy: %.2f\n", K_i_y.get());   
        printf("Kdy: %.2f\n", K_d_y.get());   
        printf("Z\n");
        printf("Kpz: %.2f\n", K_p_z.get());   
        printf("Kiz: %.2f\n", K_i_z.get());   
        printf("Kdz: %.2f\n", K_d_z.get());   

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
        
        _tau.x = K_p_x*_error.x + K_i_x*_error_integrator.x + K_d_x*_error_derivative.x;
        _tau.y = K_p_y*_error.y + K_i_y*_error_integrator.y + K_d_y*_error_derivative.y;
        _tau.z = K_p_z*_error.z + K_i_z*_error_integrator.z + K_d_z*_error_derivative.z;
        printf("Control inputs:\n");
        printf("tau_x: %.2f N\n", _tau.x);
        printf("tau_y: %.2f N\n", _tau.y);   
        printf("tau_z: %.2f N\n", _tau.z); 


        // Update
        _last_error.x= _error.x;
        _last_error.y= _error.y;
        _last_error.z= _error.z;

        return _tau;


}

