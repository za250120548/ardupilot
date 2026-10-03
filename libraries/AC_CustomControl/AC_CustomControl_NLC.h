#pragma once

/// @file    AC_CustomControl_NLC.h
/// @brief   ArduCopter custom controller backend: Non-Linear Controller (NLC)
///          implementing a Super-Twisting Algorithm (STA), a second-order
///          (high-order) sliding mode controller, for the attitude/rate loop.
///
/// Control law, per body axis i in {roll, pitch, yaw}:
///
///   Attitude error (rad, body frame, ArduPilot thrust/heading decomposition):
///       e_i
///   Angular-velocity feed-forward from ArduPilot's input shaping (rad/s):
///       w_ff_i
///   Sliding surface (rad/s):
///       s_i = lambda_i * e_i + w_ff_i - w_i
///           = (w_ref_i - w_i)            with  w_ref_i = lambda_i * e_i + w_ff_i
///       which is the classic first-order surface s = de/dt + lambda*e
///       written with the measurable rate error.
///   Super-Twisting control (normalised mixer units, -1..1):
///       u_i  = k1_i * |s_i|^(1/2) * sign(s_i) + v_i
///       dv_i/dt = k2_i * sign(s_i)
///
///   Practical additions for flight software (all can be disabled):
///     - |v_i| <= VMAX                    (integrator clamp)
///     - conditional integration when the mixer reports saturation on that axis
///     - optional boundary-layer sign:  sign(s) ~ s / (|s| + EPS)
///     - u_i constrained to [-1, 1]
///     - bumpless transfer: v_i is initialised with the main controller output

#include "AC_CustomControl_config.h"

#if AP_COPTER_CUSTOMCONTROL_NLC_ENABLED

#include "AC_CustomControl_Backend.h"

class AC_CustomControl_NLC : public AC_CustomControl_Backend {
public:
    AC_CustomControl_NLC(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt);

    // run the controller, return roll, pitch, yaw mixer inputs (-1..1)
    Vector3f update(void) override;

    // called by the frontend when the custom controller is switched on:
    // initialise the STA integral state from the current mixer input (bumpless)
    void reset(void) override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    // bits for the OPTIONS parameter
    enum class Option : uint8_t {
        LOG          = (1U << 0),   // log CNLC message every loop
        NO_SAT_FREEZE = (1U << 1),  // disable integrator freeze on mixer saturation
    };

protected:
    // one Super-Twisting step for a single axis
    float sta_axis(uint8_t axis, float s, float k1, float k2, bool saturated, float &u_p);

    // smooth or discontinuous sign function depending on EPS
    float sgn(float s) const;

    bool option_set(Option opt) const { return (uint8_t(_options.get()) & uint8_t(opt)) != 0; }

    // write the CNLC log message for one axis
    void log_axis(uint8_t axis, float w_ref, float w, float s, float u_p, float u);

    // zero the STA integral states (used on the ground)
    void reset_integrators(void) { _v.zero(); }

    float _dt;          // loop period (s)
    Vector3f _v;        // STA integral state per axis
    Vector3f _u_last;   // last output, used for saturation-aware integration

    // ---- parameters ----
    // sliding surface slope (attitude error -> rate), 1/s
    AP_Float _lambda_roll;
    AP_Float _lambda_pitch;
    AP_Float _lambda_yaw;
    // STA proportional (square-root) gain
    AP_Float _k1_roll;
    AP_Float _k1_pitch;
    AP_Float _k1_yaw;
    // STA integral (discontinuous) gain
    AP_Float _k2_roll;
    AP_Float _k2_pitch;
    AP_Float _k2_yaw;
    // integrator limit (mixer units)
    AP_Float _vmax;
    // boundary layer width for the sign function (rad/s); 0 = pure sign
    AP_Float _eps;
    // option bits
    AP_Int8  _options;
};

#endif  // AP_COPTER_CUSTOMCONTROL_NLC_ENABLED
