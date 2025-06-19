#pragma once

#include "AC_CustomControl_Backend.h"
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <AP_Motors/AP_MotorsMulticopter.h>

#if AP_CUSTOMCONTROL_MC_ENABLED

// Time-Delay Control (TDC) controller for attitude control
class TDC_Controller {
public:
    TDC_Controller();
    
    // Initialize with specific parameters
    void initialize(float tdc_k1_roll, float tdc_k1_pitch, float tdc_k1_yaw,
                                 float tdc_k2_roll, float tdc_k2_pitch, float tdc_k2_yaw,        
                               float g_bar_roll, float g_bar_pitch, float g_bar_yaw,
                               float lower_limit, float upper_limit);
    
    // Main controller update - passing dt with each call
    void update(const Vector3f& att_target_eul, const Vector3f& att_eul, 
                const Vector3f& angular_rates, float dt);
    
    // Convert body angular rates to Euler rates
    Vector3f bodyToEulerRates(const Vector3f& att_eul, const Vector3f& angular_rates);
    
    // Refresh internal state variables
    void refresh();
    
    // Reset all controller states
    void reset();
    
    // Saturate a value between lower and upper limits
    float saturate(float value, float lower, float upper) {
        return constrain_float(value, lower, upper);
    }
    
    // Set controller gains
    void set_gain_matrices(float k1_roll, float k1_pitch, float k1_yaw,
                      float k2_roll, float k2_pitch, float k2_yaw) {
    _k1_matrix.identity();
    _k1_matrix.a.x = k1_roll;   // [0,0]
    _k1_matrix.b.y = k1_pitch;  // [1,1] 
    _k1_matrix.c.z = k1_yaw;    // [2,2]
    
    _k2_matrix.identity();
    _k2_matrix.a.x = k2_roll;   // [0,0]
    _k2_matrix.b.y = k2_pitch;  // [1,1]
    _k2_matrix.c.z = k2_yaw;    // [2,2]
    }
    
    // Set control effectiveness parameters
    void set_control_effectiveness(float g_bar_roll, float g_bar_pitch, float g_bar_yaw) {
        _g_bar_inverse[0] = (g_bar_roll > 0.0f) ? (1.0f / g_bar_roll) : 0.0f;
        _g_bar_inverse[1] = (g_bar_pitch > 0.0f) ? (1.0f / g_bar_pitch) : 0.0f;
        _g_bar_inverse[2] = (g_bar_yaw > 0.0f) ? (1.0f / g_bar_yaw) : 0.0f;
    }
    
    // Set output limits
    void set_output_limits(float lower, float upper) {
        _lower_limit = lower;
        _upper_limit = upper;
    }
    
    // Get current control output
    const Vector3f& get_control_output() const { return _u_curr; }
    
private:
    // Controller gains
    //float _k1;        // First backstepping gain
    //float _k2;        // Second backstepping gain

    Matrix3f _k1_matrix;  // 3x3 diagonal matrix for k1
    Matrix3f _k2_matrix;  // 3x3 diagonal matrix for k2

    // Add individual diagonal element parameters
    //AP_Float _tdc_k1_roll, _tdc_k1_pitch, _tdc_k1_yaw;
    //AP_Float _tdc_k2_roll, _tdc_k2_pitch, _tdc_k2_yaw;
    
    // Control effectiveness inverse (1/g_bar)
    float _g_bar_inverse[3];
    
    // Output limits
    float _lower_limit;
    float _upper_limit;
    
    // State variables
    Vector3f _x_1;           // Current attitude (Euler angles)
    Vector3f _x_2;           // Current Euler angle rates
    Vector3f _x_2_dot;       // Derivative of current Euler angle rates
    Vector3f _x_2_dot_old;   // Previous derivative of Euler angle rates
    
    // Control variables
    Vector3f _u_curr;        // Current control output
    Vector3f _delta_u;       // Control increment
    
    // Reference variables
    Vector3f _q_r;           // Reference attitude (target)
    Vector3f _q_r_dot;       // Reference attitude rate
    Vector3f _q_r_dot_dot;   // Reference attitude acceleration
    
    // Previous values for numerical differentiation
    Vector3f _x_1_old;       // Previous attitude
    Vector3f _x_2_old;       // Previous Euler angle rates
    Vector3f _q_r_old;       // Previous reference attitude
    Vector3f _q_r_old_old;   // Two steps ago reference attitude
};

// ArduPilot custom controller backend for multicopters
class AC_CustomControl_MC : public AC_CustomControl_Backend {
public:
    AC_CustomControl_MC(AC_CustomControl &frontend, AP_AHRS_View *&ahrs, AC_AttitudeControl *&att_control, AP_MotorsMulticopter *&motors, float dt);
    
    // Update the controller and return attitude rate corrections
    Vector3f update() override;
    
    // Reset the controller
    void reset() override;
    
    // User-adjustable parameters
    static const struct AP_Param::GroupInfo var_info[];
    
private:
    // Parameters
    AP_Float _tdc_k1_roll;
    AP_Float _tdc_k1_pitch; 
    AP_Float _tdc_k1_yaw; 
    AP_Float _tdc_k2_roll; 
    AP_Float _tdc_k2_pitch; 
    AP_Float _tdc_k2_yaw; 

    AP_Float _tdc_g_bar_roll;     // Control effectiveness for roll
    AP_Float _tdc_g_bar_pitch;    // Control effectiveness for pitch
    AP_Float _tdc_g_bar_yaw;      // Control effectiveness for yaw
    AP_Float _tdc_limit;          // Output limit in radians
    float _dt;
    //Matrix3f _k1_matrix, _k2_matrix;
    
    // Controller instance
    TDC_Controller _tdc_controller;
};


#endif // AP_CUSTOMCONTROL_MC_ENABLED