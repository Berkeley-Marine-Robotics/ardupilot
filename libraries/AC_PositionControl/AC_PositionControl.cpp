#include "AC_PositionControl.h"
#include <iostream>

// table of user settable parameters
const AP_Param::GroupInfo AC_PositionControl::var_info[] = {

        // @param: POS_X_P
        // @DisplayName: Kp_x
        // @Description: PID Control Gaingit st
        // @Range: 0 100
        // @User: Standard
        AP_GROUPINFO("POS_X_P", 1, AC_PositionControl, _K_p_x, POS_X_P_DEFAULT),

        // @param: POS_X_I
        // @DisplayName: Ki_x
        // @Description: PID Control Gain
        // @Range: 0 100
        // @User: Standard
        AP_GROUPINFO("POS_X_I", 2, AC_PositionControl, _K_i_x, POS_X_I_DEFAULT),

        // @param: POS_X_D
        // @DisplayName: Kd_x
        // @Description: PID Control Gain
        // @Range: 0 100
        // @User: Standard
        AP_GROUPINFO("POS_X_D", 3, AC_PositionControl, _K_d_x, POS_X_D_DEFAULT),

        // @param: POS_Y_P
        // @DisplayName: Kp_y
        // @Description: PID Control Gain
        // @Range: 0 100
        // @User: Standard
        AP_GROUPINFO("POS_Y_P", 4, AC_PositionControl, _K_p_y, POS_Y_P_DEFAULT),

        // @param: POS_Y_I
        // @DisplayName: Ki_y
        // @Description: PID Control Gain
        // @Range: 0 100
        // @User: Standard
        AP_GROUPINFO("POS_Y_I", 5, AC_PositionControl, _K_i_y, POS_Y_I_DEFAULT),

        // @param: POS_Y_D
        // @DisplayName: Kd_y
        // @Description: PID Control Gain
        // @Range: 0 100
        // @User: Standard
        AP_GROUPINFO("POS_Y_D", 6, AC_PositionControl, _K_d_y, POS_Y_D_DEFAULT),

        // @param: POS_Z_P
        // @DisplayName: Kp_z
        // @Description: PID Control Gain
        // @Range: 0 100
        // @User: Standard
        AP_GROUPINFO("POS_Z_P", 7, AC_PositionControl, _K_p_z, POS_Z_P_DEFAULT),

        // @param: POS_Z_I
        // @DisplayName: Ki_z
        // @Description: PID Control Gain
        // @Range: 0 100
        // @User: Standard
        AP_GROUPINFO("POS_Z_I", 8, AC_PositionControl, _K_i_z, POS_Z_I_DEFAULT),

        // @param: POS_Z_D
        // @DisplayName: Kd_z
        // @Description: PID Control Gain
        // @Range: 0 100
        // @User: Standard
        AP_GROUPINFO("POS_Z_D", 9, AC_PositionControl, _K_d_z, POS_Z_D_DEFAULT),

        AP_GROUPEND
};

AC_PositionControl::AC_PositionControl(AP_AHRS_View &ahrs,
                                       const AP_InertialNav &inav,
                                       AP_Motors &motors,
                                       float dt) :

        _dt(dt),
        _ahrs(ahrs),
        _inav(inav),
        _motors(motors),
        _Fx_max(9 * 9.81),
        _Fy_max(9 * 9.81),
        _Fz_max(7 * 9.81) {
    AP_Param::setup_object_defaults(this, var_info);
}

void AC_PositionControl::init_position_control() {
    // Target position initialization
    _pos_target.x = 0;
    _pos_target.y = 0;
    _pos_target.z = 0;

    // Position Control initialization
    _error.x = 0;
    _error.y = 0;
    _error.z = 0;

    _error_integral.x = 0;
    _error_integral.y = 0;
    _error_integral.z = 0;

    _error_derivative.x = 0;
    _error_derivative.y = 0;
    _error_derivative.z = 0;

    _previous_error.x = 0;
    _previous_error.y = 0;
    _previous_error.z = 0;

    /*std::cout << "KPX: " << _K_p_x << '\n';
    std::cout << "KIX: " << _K_i_x << '\n';
    std::cout << "KDX: " << _K_d_x << '\n';

    std::cout << "KPY: " << _K_p_y << '\n';
    std::cout << "KIY: " << _K_i_y << '\n';
    std::cout << "KDY: " << _K_d_y << '\n';

    std::cout << "KPZ: " << _K_p_z << '\n';
    std::cout << "KIZ: " << _K_i_z << '\n';
    std::cout << "KDZ: " << _K_d_z << '\n';
*/
}

void AC_PositionControl::update_position_control() {

//    std::cout << "update: " << '\n';

    // PID Control Implementation
    _error = _pos_measured - _pos_target;

    // Update Integral
    _error_integral.x += _error.x * _dt;
    _error_integral.y += _error.y * _dt;
    _error_integral.z += _error.z * _dt;

    // Update Derivative
    // x
//    float derivative_x = (_error.x - _previous_error.x) / _dt;
//    _error_derivative.x = 0.5 * (_error_derivative.x + derivative_x);
    _error_derivative.x = (_error.x - _previous_error.x) / _dt;

    // y
//    float derivative_y = (_error.y - _previous_error.y) / _dt;
//    _error_derivative.y = 0.5 * (_error_derivative.y + derivative_y);
    _error_derivative.y = (_error.y - _previous_error.y) / _dt;

    // z
//    float derivative_z = (_error.z - _previous_error.z) / _dt;
//    _error_derivative.z = 0.5 * (_error_derivative.z + derivative_z);
    _error_derivative.z = (_error.z - _previous_error.z) / _dt;


    // Compute Control Inputs
    _F.x = _K_p_x * _error.x + _K_i_x * _error_integral.x + _K_d_x * _error_derivative.x;
    _F.y = _K_p_y * _error.y + _K_i_y * _error_integral.y + _K_d_y * _error_derivative.y;
    _F.z = _K_p_z * _error.z + _K_i_z * _error_integral.z + _K_d_z * _error_derivative.z;

    // Normalize Control Inputs
    _F.x = _F.x / _Fx_max;
    _F.y = _F.y / _Fy_max;
    _F.z = _F.z / _Fz_max;

    // Update previous error
    _previous_error = _error;

    // _inav.get_position() is same as What are we getting from the following function
    bool status1 = _ahrs.get_relative_position_NED_home(_current_position_from_home);

    bool status2 = _ahrs.get_relative_position_NED_origin(_current_position_from_origin);

    count++;
    if (count%1000 == 0 && status1  && status2){

//        std::cout << "TargetX: " << _pos_target.x << '\n';
//        std::cout << "TargetY: " << _pos_target.y << '\n';
//        std::cout << "TargetZ: " << _pos_target.z << '\n';
//
//        std::cout << "MeasuredX: " << _pos_measured.x << '\n';
//        std::cout << "MeasuredY: " << _pos_measured.y << '\n';
//        std::cout << "MeasuredZ: " << _pos_measured.z << '\n';


//        std::cout << "ErrorX: " << _error.x << '\n';
//        std::cout << "ErrorY: " << _error.y << '\n';
//        std::cout << "ErrorZ: " << _error.z << '\n';
//
//        std::cout << "FX: " << _F.x  << '\n';
//        std::cout << "FY: " << _F.y  << '\n';
//        std::cout << "FZ: " << _F.z  << '\n';

        /*std::cout << "From inav_X: " << _inav.get_position().x << '\n';
        std::cout << "From inav_Y: " << _inav.get_position().y << '\n';
        std::cout << "From inav_Z: " << _inav.get_position().z << '\n';

        std::cout << std::endl;

        std::cout << "From ahrs_home_X: " << _current_position_from_home.x << '\n';
        std::cout << "From ahrs_home_Y: " << _current_position_from_home.y << '\n';
        std::cout << "From ahrs_home_Z: " << _current_position_from_home.z << '\n';

        std::cout << std::endl;

        std::cout << "From ahrs_origin_X: " << _current_position_from_origin.x << '\n';
        std::cout << "From ahrs_origin_Y: " << _current_position_from_origin.y << '\n';
        std::cout << "From ahrs_origin_Z: " << _current_position_from_origin.z << '\n';

        std::cout << std::endl;
*/

    }

}

void AC_PositionControl::run_position_control() {

//    std:: cout << "run" << '\n';

    float body_Fx = _F.x*_ahrs.cos_yaw() + _F.y*_ahrs.sin_yaw();
    float body_Fy = -_F.x*_ahrs.sin_yaw() + _F.y*_ahrs.cos_yaw();

    _F.x = body_Fx;
    _F.y = body_Fy;

    _motors.set_forward(_F.x);
    _motors.set_lateral(_F.y);
    _motors.set_throttle(_F.z);
}

void AC_PositionControl::set_target_position(const Vector3f &target_position) {
    _pos_target = target_position;
}

void AC_PositionControl::get_measured_position(const Vector3f& measured_position){
//                                                const float measured_position_x,
//                                               const float measured_position_y,
//                                               const float measured_position_z) {
    _pos_measured.x = measured_position.x;
    _pos_measured.y = hull_location - _inav.get_position().y;
    _pos_measured.z = _inav.get_position().z; // depth reading are negative
//      _pos_measured = measured_position;
}

void AC_PositionControl::log_data() {
    // Although names suggest the absolute position, it is relative distance from uuv/boat (x),
    // distance from hull(y), distance from water surface (z)
    // Position target, measured and Force in x
    AP::logger().Write("POSX", "TimeUS, PosTargetX, PosMeasuredX, Fx", "Qfff",
                       AP_HAL::micros64(),
                       (double) _pos_target.x,
                       (double) _pos_measured.x,
                       (double) _F.x);

    // Position target, measured and Force in y
    AP::logger().Write("POSY", "TimeUS, PosTargetY, PosMeasuredY, Fy", "Qfff",
                       AP_HAL::micros64(),
                       (double) _pos_target.y,
                       (double) _pos_measured.y,
                       (double) _F.y);

    AP::logger().Write("POSZ", "TimeUS, PosTargetZ, PosMeasuredZ, Fz", "Qfff",
                       AP_HAL::micros64(),
                       (double) _pos_target.z,
                       (double) _pos_measured.z,
                       (double) _F.z);

    // Position: Log Errors in X
    AP::logger().Write("ERPX", "TimeUS, ErrX, ErrIntX, ErrDerX", "Qfff",
                       AP_HAL::micros64(),
                       (double) _error.x,
                       (double) _error_integral.x,
                       (double) _error_derivative.x);

    // Velocity: Log Errors in Y
    AP::logger().Write("ERPY", "TimeUS, ErrY, ErrIntY, ErrDerY", "Qfff",
                       AP_HAL::micros64(),
                       (double) _error.y,
                       (double) _error_integral.y,
                       (double) _error_derivative.y);

    // Velocity: Log Errors in Z
    AP::logger().Write("ERPZ", "TimeUS, ErrZ, ErrIntZ, ErrDerZ", "Qfff",
                       AP_HAL::micros64(),
                       (double) _error.z,
                       (double) _error_integral.z,
                       (double) _error_derivative.z);

    // Log gains
    AP::logger().Write("PID", "TimeUS, PX, IX, DX, PY, IY, DY, PZ, IZ, DZ", "Qfffffffff",
                       AP_HAL::micros64(),
                       (double) _K_p_x.get(),
                       (double) _K_i_x.get(),
                       (double) _K_d_x.get(),
                       (double) _K_p_y.get(),
                       (double) _K_i_y.get(),
                       (double) _K_d_y.get(),
                       (double) _K_p_z.get(),
                       (double) _K_i_z.get(),
                       (double) _K_d_z.get());
}

