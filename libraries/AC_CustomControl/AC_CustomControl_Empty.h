#pragma once

#include "AC_CustomControl_config.h"

#if AP_COPTER_CUSTOMCONTROL_EMPTY_ENABLED

#include "AC_CustomControl_Backend.h"

class AC_CustomControl_Empty : public AC_CustomControl_Backend {
public:
    AC_CustomControl_Empty(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt);


    Vector3f update(void) override;
    void reset(void) override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    // Get integrator per axis return true if integrator is used
    // axis value is Iroll=0, Ipitch = 1, Iyaw=2, mixer inputs (-1..1)
    bool get_integrator(uint8_t axis, float &value) const override;

protected:
    // declare parameters here
    AP_Float param1;
    AP_Float param2;
    AP_Float param3;
};

#endif  // AP_COPTER_CUSTOMCONTROL_EMPTY_ENABLED
