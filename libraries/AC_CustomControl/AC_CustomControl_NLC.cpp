#include "AC_CustomControl_config.h"

#if AP_COPTER_CUSTOMCONTROL_NLC_ENABLED

#include "AC_CustomControl_NLC.h"

#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>

// default gains, normalised mixer units (-1..1)
// Tuned in SITL (default "quad" model, Tools/NLC/sitl_rms_test.py).
// They are a starting point only: retune for a real airframe.
#define NLC_LAMBDA_RP_DEFAULT   4.5f     // same as ATC_ANG_RLL_P / ATC_ANG_PIT_P
#define NLC_LAMBDA_Y_DEFAULT    4.5f     // same as ATC_ANG_YAW_P
#define NLC_K1_RP_DEFAULT       0.07f
#define NLC_K1_Y_DEFAULT        0.40f
#define NLC_K2_RP_DEFAULT       2.0f
#define NLC_K2_Y_DEFAULT        1.0f
#define NLC_VMAX_DEFAULT        0.50f
#define NLC_EPS_DEFAULT         0.5f     // rad/s, 0 = pure discontinuous STA

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_NLC::var_info[] = {
    // @Param: LAM_R
    // @DisplayName: NLC roll sliding surface slope
    // @Description: Slope lambda of the roll sliding surface s = lambda*e + w_ff - w. Converts attitude error (rad) into a reference rate (rad/s)
    // @Range: 1 12
    // @Units: 1/s
    // @User: Advanced
    AP_GROUPINFO("LAM_R", 1, AC_CustomControl_NLC, _lambda_roll, NLC_LAMBDA_RP_DEFAULT),

    // @Param: LAM_P
    // @DisplayName: NLC pitch sliding surface slope
    // @Description: Slope lambda of the pitch sliding surface
    // @Range: 1 12
    // @Units: 1/s
    // @User: Advanced
    AP_GROUPINFO("LAM_P", 2, AC_CustomControl_NLC, _lambda_pitch, NLC_LAMBDA_RP_DEFAULT),

    // @Param: LAM_Y
    // @DisplayName: NLC yaw sliding surface slope
    // @Description: Slope lambda of the yaw sliding surface
    // @Range: 1 12
    // @Units: 1/s
    // @User: Advanced
    AP_GROUPINFO("LAM_Y", 3, AC_CustomControl_NLC, _lambda_yaw, NLC_LAMBDA_Y_DEFAULT),

    // @Param: K1_R
    // @DisplayName: NLC roll STA gain k1
    // @Description: Super-Twisting square-root (continuous) gain for roll: u = k1*|s|^0.5*sign(s) + v
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("K1_R", 4, AC_CustomControl_NLC, _k1_roll, NLC_K1_RP_DEFAULT),

    // @Param: K1_P
    // @DisplayName: NLC pitch STA gain k1
    // @Description: Super-Twisting square-root (continuous) gain for pitch
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("K1_P", 5, AC_CustomControl_NLC, _k1_pitch, NLC_K1_RP_DEFAULT),

    // @Param: K1_Y
    // @DisplayName: NLC yaw STA gain k1
    // @Description: Super-Twisting square-root (continuous) gain for yaw
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("K1_Y", 6, AC_CustomControl_NLC, _k1_yaw, NLC_K1_Y_DEFAULT),

    // @Param: K2_R
    // @DisplayName: NLC roll STA gain k2
    // @Description: Super-Twisting integral (discontinuous) gain for roll: dv/dt = k2*sign(s)
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("K2_R", 7, AC_CustomControl_NLC, _k2_roll, NLC_K2_RP_DEFAULT),

    // @Param: K2_P
    // @DisplayName: NLC pitch STA gain k2
    // @Description: Super-Twisting integral (discontinuous) gain for pitch
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("K2_P", 8, AC_CustomControl_NLC, _k2_pitch, NLC_K2_RP_DEFAULT),

    // @Param: K2_Y
    // @DisplayName: NLC yaw STA gain k2
    // @Description: Super-Twisting integral (discontinuous) gain for yaw
    // @Range: 0 5
    // @User: Advanced
    AP_GROUPINFO("K2_Y", 9, AC_CustomControl_NLC, _k2_yaw, NLC_K2_Y_DEFAULT),

    // @Param: VMAX
    // @DisplayName: NLC integral state limit
    // @Description: Maximum absolute value of the Super-Twisting integral state v (mixer units)
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("VMAX", 10, AC_CustomControl_NLC, _vmax, NLC_VMAX_DEFAULT),

    // @Param: EPS
    // @DisplayName: NLC sign boundary layer
    // @Description: Boundary layer width for the sign function used in the integral term, sign(s) ~ s/(|s|+EPS). 0 gives the pure discontinuous Super-Twisting algorithm
    // @Range: 0 1
    // @Units: rad/s
    // @User: Advanced
    AP_GROUPINFO("EPS", 11, AC_CustomControl_NLC, _eps, NLC_EPS_DEFAULT),

    // @Param: OPT
    // @DisplayName: NLC options
    // @Description: Option bits for the NLC controller
    // @Bitmask: 0:Log CNLC every loop, 1:Disable integrator freeze on mixer saturation
    // @User: Advanced
    AP_GROUPINFO("OPT", 12, AC_CustomControl_NLC, _options, 1),

    AP_GROUPEND
};

// initialize in the constructor
AC_CustomControl_NLC::AC_CustomControl_NLC(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt),
    _dt(dt)
{
    AP_Param::setup_object_defaults(this, var_info);
}

// sign function, optionally smoothed with a boundary layer
float AC_CustomControl_NLC::sgn(float s) const
{
    const float eps = _eps.get();
    if (is_positive(eps)) {
        return s / (fabsf(s) + eps);
    }
    if (is_positive(s)) {
        return 1.0f;
    }
    if (is_negative(s)) {
        return -1.0f;
    }
    return 0.0f;
}

// one discrete-time Super-Twisting step (explicit Euler) for one axis
// returns total output u, and the square-root term in u_p
float AC_CustomControl_NLC::sta_axis(uint8_t axis, float s, float k1, float k2, bool saturated, float &u_p)
{
    // continuous term: k1 * |s|^(1/2) * sign(s)
    u_p = k1 * safe_sqrt(fabsf(s)) * (is_negative(s) ? -1.0f : 1.0f);

    // integral term: dv/dt = k2 * sign(s)
    const float dv = k2 * sgn(s) * _dt;

    // conditional integration: when the mixer reports this axis is saturated,
    // do not let v grow further in the direction that drives the saturation
    const bool pushing_into_limit = saturated && (dv * _u_last[axis] > 0.0f);
    if (!pushing_into_limit || option_set(Option::NO_SAT_FREEZE)) {
        _v[axis] += dv;
    }

    // clamp integral state
    const float vmax = fabsf(_vmax.get());
    _v[axis] = constrain_float(_v[axis], -vmax, vmax);

    const float u = constrain_float(u_p + _v[axis], -1.0f, 1.0f);
    _u_last[axis] = u;
    return u;
}

// update controller
// return roll, pitch, yaw controller output
Vector3f AC_CustomControl_NLC::update(void)
{
    // reset controller based on spool state
    switch (_motors->get_spool_state()) {
        case AP_Motors::SpoolState::SHUT_DOWN:
        case AP_Motors::SpoolState::GROUND_IDLE:
            // still on the ground: avoid build-up of the integral state
            reset_integrators();
            break;

        case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        case AP_Motors::SpoolState::SPOOLING_UP:
        case AP_Motors::SpoolState::SPOOLING_DOWN:
            // we are off the ground
            break;
    }

    // ---- attitude error, same definition used by the ArduCopter main controller ----
    Quaternion attitude_body;
    _ahrs->get_quat_body_to_ned(attitude_body);
    Quaternion attitude_target = _att_control->get_attitude_target_quat();

    // x,y: thrust-vector tilt error, z: heading error. Body frame, rad.
    Vector3f attitude_error;
    float thrust_angle_rad, thrust_error_angle_rad;
    _att_control->thrust_heading_rotation_angles(attitude_target, attitude_body, attitude_error, thrust_angle_rad, thrust_error_angle_rad);

    // angular velocity feed-forward (from input shaping) rotated into the body frame
    const Quaternion rotation_target_to_body = attitude_body.inverse() * attitude_target;
    const Vector3f ang_vel_ff = rotation_target_to_body * _att_control->get_attitude_target_ang_vel();

    // ---- sliding surface s = lambda*e + w_ff - w ----
    const Vector3f lambda{_lambda_roll.get(), _lambda_pitch.get(), _lambda_yaw.get()};
    const Vector3f k1{_k1_roll.get(), _k1_pitch.get(), _k1_yaw.get()};
    const Vector3f k2{_k2_roll.get(), _k2_pitch.get(), _k2_yaw.get()};

    const Vector3f gyro = _ahrs->get_gyro_latest();

    Vector3f w_ref;
    Vector3f s;
    for (uint8_t i = 0; i < 3; i++) {
        w_ref[i] = lambda[i] * attitude_error[i] + ang_vel_ff[i];
        s[i] = w_ref[i] - gyro[i];
    }

    // mixer saturation flags from the previous loop
    const bool sat[3] = { _motors->limit.roll, _motors->limit.pitch, _motors->limit.yaw };

    // ---- Super-Twisting ----
    Vector3f u;
    Vector3f u_p;
    for (uint8_t i = 0; i < 3; i++) {
        u[i] = sta_axis(i, s[i], k1[i], k2[i], sat[i], u_p[i]);
    }

    if (option_set(Option::LOG)) {
        for (uint8_t i = 0; i < 3; i++) {
            log_axis(i, w_ref[i], gyro[i], s[i], u_p[i], u[i]);
        }
    }

    return u;
}

void AC_CustomControl_NLC::log_axis(uint8_t axis, float w_ref, float w, float s, float u_p, float u)
{
#if HAL_LOGGING_ENABLED
    // @LoggerMessage: CNLC
    // @Description: Custom controller NLC (Super-Twisting sliding mode) per-axis data
    // @Field: TimeUS: Time since system startup
    // @Field: I: axis (0 roll, 1 pitch, 2 yaw)
    // @Field: Tar: reference body rate w_ref = lambda*e + w_ff
    // @Field: Act: measured body rate
    // @Field: S: sliding variable s = w_ref - w
    // @Field: UP: square-root term k1*|s|^0.5*sign(s)
    // @Field: V: integral state v
    // @Field: U: total output u (mixer units)
    AP::logger().WriteStreaming("CNLC", "TimeUS,I,Tar,Act,S,UP,V,U", "s#EEE---", "F-000000", "QBffffff",
                                AP_HAL::micros64(),
                                axis,
                                w_ref,
                                w,
                                s,
                                u_p,
                                _v[axis],
                                u);
#endif
}

// called when switching to the custom controller:
// initialise integral state with the main controller output so the
// first STA output is close to what the vehicle was already commanding
void AC_CustomControl_NLC::reset(void)
{
    _v.x = _motors->get_roll();
    _v.y = _motors->get_pitch();
    _v.z = _motors->get_yaw();
    _u_last = _v;
}

#endif  // AP_COPTER_CUSTOMCONTROL_NLC_ENABLED
