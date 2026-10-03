#pragma once

/// @file    AC_CustomControl_NLC.h
/// @brief   ArduCopter custom controller backend for Not Linear Controller (NLC)

#include "AC_CustomControl_config.h"

#if AP_COPTER_CUSTOMCONTROL_NLC_ENABLED

#include "AC_CustomControl_Backend.h"

class AC_CustomControl_NLC : public AC_CustomControl_Backend {
public:
    AC_CustomControl_NLC(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt);


    Vector3f update(void) override;
    void reset(void) override;
    void init(void);

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // declare parameters here
    AP_Float param1;
    AP_Float param2;
    AP_Float param3;
};

#endif  // AP_COPTER_CUSTOMCONTROL_NLC_ENABLED
