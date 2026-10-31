#include <AP_HAL/AP_HAL.h>

#include "AC_CustomControl.h"

#if AP_COPTER_CUSTOMCONTROL_ENABLED

#include "AC_CustomControl_Backend.h"
#include "AC_CustomControl_Empty.h"
#include "AC_CustomControl_PID.h"
#include "AC_CustomControl_NLC.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <SRV_Channel/SRV_Channel.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl::var_info[] = {
    // @Param: _TYPE
    // @DisplayName: Custom control type
    // @Description: Custom control type to be used
    // @Values: 0:None, 1:Empty, 2:PID, 3:NLC
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_TYPE", 1, AC_CustomControl, _controller_type, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: _AXIS_MASK
    // @DisplayName: Custom Controller bitmask
    // @Description: Custom Controller bitmask to chose which axis to run
    // @Bitmask: 0:Roll, 1:Pitch, 2:Yaw
    // @User: Advanced
    AP_GROUPINFO("_AXIS_MASK", 2, AC_CustomControl, _custom_controller_mask, 0),


    // @Param: _OPTIONS
    // @DisplayName: Custom Controller options
    // @Description: Custom Controller option bits
    // @Bitmask: 0:Log per-motor output (CCMO) while armed
    // @User: Advanced
    AP_GROUPINFO("_OPTIONS", 3, AC_CustomControl, _options, 0),

    // parameters for empty controller. only used as a template, no need for param table 
    // AP_SUBGROUPVARPTR(_backend, "1_", 6, AC_CustomControl, _backend_var_info[0]),

    // @Group: 1_
    // @Path: AC_CustomControl_Empty.cpp
    AP_SUBGROUPVARPTR(_backend, "1_", 6, AC_CustomControl, _backend_var_info[0]),

    // @Group: 2_
    // @Path: AC_CustomControl_PID.cpp
    AP_SUBGROUPVARPTR(_backend, "2_", 7, AC_CustomControl, _backend_var_info[1]),

    // @Group: 3_
    // @Path: AC_CustomControl_NLC.cpp
    AP_SUBGROUPVARPTR(_backend, "3_", 8, AC_CustomControl, _backend_var_info[2]),

    AP_GROUPEND
};

const struct AP_Param::GroupInfo *AC_CustomControl::_backend_var_info[CUSTOMCONTROL_MAX_TYPES];

AC_CustomControl::AC_CustomControl(AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors) :
    _ahrs(ahrs),
    _att_control(att_control),
    _motors(motors)
{
    AP_Param::setup_object_defaults(this, var_info);
    // initialize the fstrate parameter pointer and type
    _fstrate_param = nullptr;
    _fstrate_param_type = AP_PARAM_NONE;
    _last_fstrate_warn_ms = 0;
}

void AC_CustomControl::init(void)
{
    _dt = AP::scheduler().get_loop_period_s();
    //FSTRATE_ENABLE exists on builds with fast rate thread, if not found nullptr is returned
    _fstrate_param = AP_Param::find("FSTRATE_ENABLE", &_fstrate_param_type);

    switch (CustomControlType(_controller_type))
    {
        case CustomControlType::CONT_NONE:
            break;
        case CustomControlType::CONT_EMPTY: // This is template backend. Don't initialize it.
            // This is template backend. Don't initialize it.
            // _backend = NEW_NOTHROW AC_CustomControl_Empty(*this, _ahrs, _att_control, _motors, _dt);
            // _backend_var_info[get_type()] = AC_CustomControl_Empty::var_info;
            break;
        case CustomControlType::CONT_PID:
            _backend = NEW_NOTHROW AC_CustomControl_PID(*this, _ahrs, _att_control, _motors, _dt);
            _backend_var_info[get_type()] = AC_CustomControl_PID::var_info;
            break;
        case CustomControlType::CONT_NLC:
            _backend = NEW_NOTHROW AC_CustomControl_NLC(*this, _ahrs, _att_control, _motors, _dt);
            _backend_var_info[get_type()] = AC_CustomControl_NLC::var_info;
            break;
        default:
            return;
    }

    if (_backend && _backend_var_info[get_type()]) {
        AP_Param::load_object_from_eeprom(_backend, _backend_var_info[get_type()]);
    }
}

// run custom controller if it is activated by RC switch and appropriate type is selected
void AC_CustomControl::update(void)
{
    if(_custom_controller_active && fast_rate_enabled()) {
        static uint32_t now_ms = AP_HAL::millis(); // get time in milliseconds
        // log a warning every 5 seconds if the custom controller is active but the fast loop is not enabled
        if (now_ms - _last_fstrate_warn_ms > 2000) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CC: FSTRATE_ENABLE != 0, Custom Controller disabled");
            _last_fstrate_warn_ms = now_ms;
        }
        // in fast rate loop mode, the PID is running in a separate thread, potential overwrite of motor output
        set_custom_controller(false);
        return;
    }

    if (is_safe_to_run()) {
        Vector3f motor_out_rpy;

        motor_out_rpy = _backend->update();

        motor_set(motor_out_rpy);
    }
}

// choose which axis to apply custom controller output
void AC_CustomControl::motor_set(Vector3f rpy) {
    // set the FeedForward FF = 0 to avoid any unexpected behavior on the custom controller output.
    //ensures that the custom controller has full control over axis without any interference from the FF component.
    Vector3f i_scale{1.0f, 1.0f, 1.0f};
    if (_custom_controller_mask & (uint8_t)CustomControlOption::ROLL) {
        _motors->set_roll(rpy.x);
        //set the feedforward calculated from PID to avoid interference with custom controller
        _motors->set_roll_ff(0.0f);
        // mirror the active controller integrator into the rate PID so that
        //checks reading it (yaw imbalance) keep working
        float cc_Ix = _backend->get_integrator(0);
        _att_control->get_rate_roll_pid().set_integrator(cc_Ix);
        i_scale.x = 0.0f;
    }
    if (_custom_controller_mask & (uint8_t)CustomControlOption::PITCH) {
        _motors->set_pitch(rpy.y);
        //set the feedforward calculated from PID to avoid interference with custom controller
        _motors->set_pitch_ff(0.0f);
        // mirror the active controller integrator into the rate PID so that
        //checks reading it (yaw imbalance) keep working
        float cc_Iy = _backend->get_integrator(1);
        _att_control->get_rate_pitch_pid().set_integrator(cc_Iy);
        i_scale.y = 0.0f;
    }
    if (_custom_controller_mask & (uint8_t)CustomControlOption::YAW) {
        _motors->set_yaw(rpy.z);
        //set the feedforward calculated from PID to avoid interference with custom controller
        _motors->set_yaw_ff(0.0f);
        // mirror the active controller integrator into the rate PID so that
        //checks reading it (yaw imbalance) keep working
        float cc_Iz = _backend->get_integrator(2);
        _att_control->get_rate_yaw_pid().set_integrator(cc_Iz);
        i_scale.z = 0.0f;
    }
    _att_control->set_I_scale_mult(i_scale);
}

// move main controller's target to current states, reset filters,
// and move integrator to motor output
// to allow smooth transition to the primary controller
void AC_CustomControl::reset_main_att_controller(void)
{
    // reset attitude and rate target, if feedforward is enabled
    if (_att_control->get_bf_feedforward()) {
        _att_control->relax_attitude_controllers();
    }

    _att_control->get_rate_roll_pid().set_integrator(0.0);
    _att_control->get_rate_pitch_pid().set_integrator(0.0);
    _att_control->get_rate_yaw_pid().set_integrator(0.0);
}

void AC_CustomControl::set_custom_controller(bool enabled)
{
    // double logging switch makes the state change very clear in the log
    log_switch();

    _custom_controller_active = false;

    // don't allow accidental main controller reset without active custom controller
    if (_controller_type == CustomControlType::CONT_NONE) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Custom controller is not enabled");
        return;
    }

    // controller type is out of range
    if (_controller_type > CUSTOMCONTROL_MAX_TYPES) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Custom controller type is out of range");
        return;
    }

    // backend is not created
    if (_backend == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Reboot to enable selected custom controller");
        return;
    }

    if (_custom_controller_mask == 0 && enabled) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Axis mask is not set");
        return;
    }

    //Block Custom Controller activation when FSTRATE_ENABLE = 1, due to potential overwrite of motor output by PID in fast loop mode
    if (enabled && fast_rate_enabled()) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CC: Set FSTRATE_ENABLE=0 to enable Custom controller");
        return;
    }

    // reset main controller
    if (!enabled) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Custom controller is OFF");
        // don't reset if the empty backend is selected
        if (_controller_type > CustomControlType::CONT_EMPTY) {
            reset_main_att_controller();
        }
    }

    if (enabled && _controller_type > CustomControlType::CONT_NONE) {
        // reset custom controller filter, integrator etc.
        _backend->reset();
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Custom controller is ON");
    }

    _custom_controller_active = enabled;

    // log successful switch
    log_switch();
}

// check that RC switch is on, backend is not changed mid flight and controller type is selected
bool AC_CustomControl::is_safe_to_run(void) {
    if (_custom_controller_active && (_controller_type > CustomControlType::CONT_NONE)
        && (_controller_type <= CUSTOMCONTROL_MAX_TYPES) && _backend != nullptr)
    {
        return true;
    }

    return false;
}

// log when the custom controller is switch into
void AC_CustomControl::log_switch(void) {
    // @LoggerMessage: CC
    // @Description: Custom Controller data
    // @Field: TimeUS: Time since system startup
    // @Field: Type: controller type
    // @FieldValueEnum: Type: AC_CustomControl::CustomControlType
    // @Field: Act: true if controller is active
    AP::logger().Write("CC", "TimeUS,Type,Act","QBB",
                            AP_HAL::micros64(),
                            _controller_type,
                            _custom_controller_active);
}

bool AC_CustomControl::fast_rate_enabled(void)
{
    if (_fstrate_param == nullptr) {
        return false;
    }
    if (is_zero(_fstrate_param->cast_to_float(_fstrate_param_type))) {
        
        return false;
    }
    return true;
}

// log what the motors library produced from the current mixer inputs, per motor.
// Called by the vehicle right after motors->output(), in the same loop.
void AC_CustomControl::log_motor_output(void)
{
#if HAL_LOGGING_ENABLED
    if (!option_set(Options::LOG_MOTORS) || !_motors->armed()) {
        return;
    }
    const uint64_t now_us = AP_HAL::micros64();
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (!_motors->is_motor_enabled(i)) {
            continue;
        }
        uint16_t pwm = 0;
        SRV_Channels::get_output_pwm(SRV_Channels::get_motor_function(i), pwm);

        // @LoggerMessage: CCMO
        // @Description: Custom controller per-motor output chain
        // @Field: TimeUS: Time since system startup
        // @Field: I: motor index (0 based, MOTn = I+1)
        // @Field: CC: 1 if the custom controller is active
        // @Field: Thr: mixer thrust demand for this motor (_thrust_rpyt_out), 0..1
        // @Field: Out: actuator command after thrust linearisation and slew (_actuator), 0..1
        // @Field: PWM: PWM written to this motor output
        AP::logger().WriteStreaming("CCMO", "TimeUS,I,CC,Thr,Out,PWM", "s#---Y", "F----0", "QBBffH",
                                    now_us,
                                    i,
                                    uint8_t(_custom_controller_active),
                                    _motors->get_thrust_rpyt_out(i),
                                    _motors->get_actuator(i),
                                    pwm);
    }
#endif
}

void AC_CustomControl::set_notch_sample_rate(float sample_rate)
{
#if AP_FILTER_ENABLED
    if (_backend != nullptr) {
        _backend->set_notch_sample_rate(sample_rate);
    }
#endif
}

#endif  // AP_COPTER_CUSTOMCONTROL_ENABLED
