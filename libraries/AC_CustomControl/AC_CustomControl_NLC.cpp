#include "AC_CustomControl_config.h"

#if AP_COPTER_CUSTOMCONTROL_NLC_ENABLED

#include "AC_CustomControl_NLC.h"

#include <GCS_MAVLink/GCS.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_NLC::var_info[] = {
    // @Param: PARAM1
    // @DisplayName: NLC param1
    // @Description: Dummy parameter for NLC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM1", 1, AC_CustomControl_NLC, param1, 0.0f),

    // @Param: PARAM2
    // @DisplayName: NLC param2
    // @Description: Dummy parameter for NLC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM2", 2, AC_CustomControl_NLC, param2, 0.0f),

    // @Param: PARAM3
    // @DisplayName: NLC param3
    // @Description: Dummy parameter for NLC custom controller backend
    // @User: Advanced
    AP_GROUPINFO("PARAM3", 3, AC_CustomControl_NLC, param3, 0.0f),

    AP_GROUPEND
};

// initialize in the constructor
AC_CustomControl_NLC::AC_CustomControl_NLC(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// update controller
// return roll, pitch, yaw controller output
Vector3f AC_CustomControl_NLC::update(void)
{
    Quaternion att_quat_current;
    Quaternion att_quat_err;
    Vector3f rate_error;

    // Get current attitude from _att_control and current rate from Attitude Heading Reference System (AHRS)
    _ahrs->get_quat_body_to_ned(att_quat_current);
    Vector3f rate_current = _ahrs->get_gyro();

    // Get target attitude and rate 
    Quaternion att_quat_target = _att_control->get_attitude_target_quat();
    Vector3f rate_target = _att_control->get_rate_ef_target_rads();

    // Calculate attitude error and rate error
    att_quat_err = att_quat_target * att_quat_current.inverse();
    rate_error = rate_target - rate_current;

    // Ensure the scalar part of the quaternion error is non-negative to avoid ambiguity in rotation representation
    if (att_quat_err.q1 <0){
        att_quat_err.q1 = -att_quat_err.q1;
        att_quat_err.q2 = -att_quat_err.q2;
        att_quat_err.q3 = -att_quat_err.q3;
        att_quat_err.q4 = -att_quat_err.q4;
    }

    // reset controller based on spool state
    switch (_motors->get_spool_state()) {
        case AP_Motors::SpoolState::SHUT_DOWN:
        case AP_Motors::SpoolState::GROUND_IDLE:
            // We are still at the ground. Reset custom controller to avoid
            // build up, ex: integrator
            reset();
            break;

        case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        case AP_Motors::SpoolState::SPOOLING_UP:
        case AP_Motors::SpoolState::SPOOLING_DOWN:
            // we are off the ground

            // Build sliding surface based on errors and parameters
            

            // Compute control output using NLC algorithm
            break;
    }

    // arducopter main attitude controller already ran
    // we don't need to do anything else

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NLC custom controller working.");
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Att  eror: %f, %f, %f, %f", att_quat_err.q1, att_quat_err.q2, att_quat_err.q3, att_quat_err.q4);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Rate error: %f, %f, %f", rate_error.x, rate_error.y, rate_error.z);

    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Tagt attitude : %f, %f, %f", att_quat_target.q2, att_quat_target.q3, att_quat_target.q4);
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Curr attitude: %f, %f, %f", att_quat_current.q2, att_quat_current.q3, att_quat_current.q4);

    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Tagt rate: %f, %f, %f", rate_target.x, rate_target.y, rate_target.z );
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Curr rate: %f, %f, %f", rate_current.x, rate_current.y, rate_current.z );

    // return what arducopter main controller outputted
    return Vector3f{_motors->get_roll(), _motors->get_pitch(), _motors->get_yaw()};
}

// reset controller to avoid build up on the ground
// or to provide bumpless transfer from arducopter main controller
void AC_CustomControl_NLC::reset(void)
{
}

void AC_CustomControl_NLC::init(void)
{
    // load parameters from EEPROM
    //if (_frontend._backend_var_info[get_type()]) {
    //    AP_Param::load_object_from_eeprom(this, _frontend._backend_var_info[get_type()]);
    //}
}

#endif  // AP_COPTER_CUSTOMCONTROL_NLC_ENABLED
