
#include "AC_CustomControl_MC.h"
#include "AC_CustomControl_config.h"
#include <GCS_MAVLink/GCS.h>

#if AP_CUSTOMCONTROL_MC_ENABLED

// Initialize TDC_Controller with default parameters
TDC_Controller::TDC_Controller() :
    _lower_limit(-1.0f),
    _upper_limit(1.0f)
{
    // Default control effectiveness inverse parameters
    _g_bar_inverse[0] = 1.0f / 8000.0f;  // Roll
    _g_bar_inverse[1] = 1.0f / 8000.0f;  // Pitch
    _g_bar_inverse[2] = 1.0f / 10000.0f;  // Yaw


    // Default gain matrices                            
    // Initialize all state and control variables to zero
    _x_1.zero();
    _x_2.zero();
    _x_2_dot.zero();
    _x_2_dot_old.zero();
    _u_curr.zero();
    _delta_u.zero();
    _q_r.zero();
    _q_r_dot.zero();
    _q_r_dot_dot.zero();
    _x_1_old.zero();
    _x_2_old.zero();
    _q_r_old.zero();
    _q_r_old_old.zero();
}

// Initialize controller with specific parameters
void TDC_Controller::initialize(float tdc_k1_roll, float tdc_k1_pitch, float tdc_k1_yaw,
                                 float tdc_k2_roll, float tdc_k2_pitch, float tdc_k2_yaw,        
                               float g_bar_roll, float g_bar_pitch, float g_bar_yaw,
                               float lower_limit, float upper_limit)
{
    _lower_limit = lower_limit;
    _upper_limit = upper_limit;
    
    // Set control effectiveness parameters
    set_control_effectiveness(g_bar_roll, g_bar_pitch, g_bar_yaw);
    set_gain_matrices(tdc_k1_roll, tdc_k1_pitch, tdc_k1_yaw,
                                 tdc_k2_roll, tdc_k2_pitch, tdc_k2_yaw);
    
    // Reset all state variables
    reset();
}

// Convert body angular rates to Euler rates
Vector3f TDC_Controller::bodyToEulerRates(const Vector3f& att_eul, const Vector3f& angular_rates)
{
    // Extract Euler angles
    float roll = att_eul.x;   // phi
    float pitch = att_eul.y;  // theta
    
    // Precompute trigonometric values
    float tan_pitch = tanf(pitch);
    float cos_pitch = cosf(pitch);
    float sin_roll = sinf(roll);
    float cos_roll = cosf(roll);
    
    // Convert body rates (p,q,r) to Euler rates (phi_dot, theta_dot, psi_dot)
    float euler_rate_roll = angular_rates.x + 
                           angular_rates.y * sin_roll * tan_pitch + 
                           angular_rates.z * cos_roll * tan_pitch;
                           
    float euler_rate_pitch = angular_rates.y * cos_roll - 
                            angular_rates.z * sin_roll;
                            
    float euler_rate_yaw = angular_rates.y * sin_roll / cos_pitch + 
                          angular_rates.z * cos_roll / cos_pitch;
    
    return Vector3f(euler_rate_roll, euler_rate_pitch, euler_rate_yaw);
}


// Main TDC update function with variable dt
void TDC_Controller::update(const Vector3f& att_target_eul, const Vector3f& att_eul, 
                           const Vector3f& angular_rates, float dt)
{

    
    // Ensure dt is positive
    if (dt <= 0.0f) {
        dt = 0.0025f;  // Fall back to 400Hz if dt is invalid
    }
    _q_r = att_target_eul;
    _x_1 = att_eul;
    
    //Convert body angular rates to Euler rates
    _x_2 = bodyToEulerRates(att_eul, angular_rates);
    _q_r_dot = (_q_r - _q_r_old) / dt;
    _q_r_dot_dot = (_q_r - _q_r_old*2.0f + _q_r_old_old) / (dt * dt);
    _x_2_dot = (_x_2 - _x_2_old) / dt;
    

    // Backstepping control 
    /*Vector3f temp = _q_r_dot_dot + 
                   _q_r_dot * (_k1 + _k2) + 
                   _q_r * (_k1 * _k2) - 
                   _x_1 * (_k1 * _k2) - 
                   _x_2 * (_k1 + _k2) - 
                   _x_2_dot_old;  // Time delay control term */
    Vector3f temp = _q_r_dot_dot +
                    (_k1_matrix + _k2_matrix) * _q_r_dot +
                    (_k1_matrix * _k2_matrix) * _q_r -
                    (_k1_matrix * _k2_matrix) * _x_1 -
                    (_k1_matrix + _k2_matrix) * _x_2 -
                    _x_2_dot_old;


    _delta_u.x = temp.x * _g_bar_inverse[0];
    _delta_u.y = temp.y * _g_bar_inverse[1];
    _delta_u.z = temp.z * _g_bar_inverse[2];
    

    _u_curr += _delta_u;
    
    // Apply saturation to prevent control output from exceeding physical limits
    _u_curr.x = saturate(_u_curr.x, _lower_limit, _upper_limit);
    _u_curr.y = saturate(_u_curr.y, _lower_limit, _upper_limit);
    _u_curr.z = saturate(_u_curr.z, _lower_limit, _upper_limit);
    
}

// Refresh old values for next control cycle
void TDC_Controller::refresh()
{
    _x_1_old = _x_1;
    _x_2_old = _x_2;
    _x_2_dot_old = _x_2_dot;
    _q_r_old_old = _q_r_old;
    _q_r_old = _q_r;
}

// Reset controller states
void TDC_Controller::reset()
{
    _x_1.zero();
    _x_2.zero();
    _x_2_dot.zero();
    _x_2_dot_old.zero();
    _u_curr.zero();
    _delta_u.zero();
    _q_r.zero();
    _q_r_dot.zero();
    _q_r_dot_dot.zero();
    _x_1_old.zero();
    _x_2_old.zero();
    _q_r_old.zero();
    _q_r_old_old.zero();
}

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_MC::var_info[] = {
    AP_GROUPINFO("TDC_K1_R", 41, AC_CustomControl_MC, _tdc_k1_roll, 1.0f),
    AP_GROUPINFO("TDC_K1_P", 42, AC_CustomControl_MC, _tdc_k1_pitch, 1.0f),
    AP_GROUPINFO("TDC_K1_Y", 43, AC_CustomControl_MC, _tdc_k1_yaw, 1.0f),
    AP_GROUPINFO("TDC_K2_R", 44, AC_CustomControl_MC, _tdc_k2_roll, 50.0f),
    AP_GROUPINFO("TDC_K2_P", 45, AC_CustomControl_MC, _tdc_k2_pitch, 50.0f),
    AP_GROUPINFO("TDC_K2_Y", 46, AC_CustomControl_MC, _tdc_k2_yaw, 50.0f),
    AP_GROUPINFO("TDC_GBAR_R", 47, AC_CustomControl_MC, _tdc_g_bar_roll, 8000.0f),
    AP_GROUPINFO("TDC_GBAR_P", 48, AC_CustomControl_MC, _tdc_g_bar_pitch, 8000.0f),
    AP_GROUPINFO("TDC_GBAR_Y", 49, AC_CustomControl_MC, _tdc_g_bar_yaw, 10000.0f),
    AP_GROUPINFO("TDC_LIMIT", 50, AC_CustomControl_MC, _tdc_limit, 1.0f),

    
    AP_GROUPEND
};

// Initialize the controller
AC_CustomControl_MC::AC_CustomControl_MC(AC_CustomControl &frontend, AP_AHRS_View *&ahrs, AC_AttitudeControl *&att_control, AP_MotorsMulticopter *&motors, float dt) : 
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt)
{
    AP_Param::setup_object_defaults(this, var_info);
    _tdc_controller.initialize(
        _tdc_k1_roll,
        _tdc_k1_pitch,
        _tdc_k1_yaw,
        _tdc_k2_roll,
        _tdc_k2_pitch,
        _tdc_k2_yaw,
        _tdc_g_bar_roll,
        _tdc_g_bar_pitch,
        _tdc_g_bar_yaw,
        -_tdc_limit,
        _tdc_limit
    );

  
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "(GCS) TDC Controller initialized, nominal dt: %4.3f", dt);
    //printf("(Printf) TDC Controller initialized, nominal dt: %4.3f", dt);
    //gcs().send_text(MAV_SEVERITY_WARNING, "Custom Control!");
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TDC Parameters registered");

    // Test if parameters are accessible
    //printf("TDC_K1_R value: %f", (double)_tdc_k1_roll.get());

}

// Main controller update function
Vector3f AC_CustomControl_MC::update()
{
    AP_Param::setup_object_defaults(this, var_info);

    // Reset controller based on spool state
    switch (_motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
    case AP_Motors::SpoolState::GROUND_IDLE:
        // Reset controller when on ground to prevent build-up
        reset();
        break;
        
    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // Normal operation when off the ground
        break;
    }
    Vector3f attitude_eul = Vector3f(_ahrs->roll, _ahrs->pitch, _ahrs->yaw);
    Vector3f attitude_target_eul = _att_control->get_att_target_euler_rad();
    Vector3f angular_rates = _ahrs->get_gyro_latest();

    _tdc_controller.set_control_effectiveness(_tdc_g_bar_roll, _tdc_g_bar_pitch, _tdc_g_bar_yaw);
    _tdc_controller.set_output_limits(-_tdc_limit, _tdc_limit);
    _tdc_controller.set_gain_matrices(_tdc_k1_roll, _tdc_k1_pitch, _tdc_k1_yaw,
                                 _tdc_k2_roll, _tdc_k2_pitch, _tdc_k2_yaw);
    float current_dt = _dt;
    _tdc_controller.update(attitude_target_eul, attitude_eul, angular_rates, current_dt);
    _tdc_controller.refresh();
    return _tdc_controller.get_control_output(); 
}

// Reset controller
void AC_CustomControl_MC::reset()
{
    _tdc_controller.reset();
}

#endif // AP_CUSTOMCONTROL_MC_ENABLED