#include "AutoHoverMode.h"
#include "Controller.h"
#include "Data.h"
#include "Attitude.h"
#include "Config.h"
#include "InputCapture.h"
#include "Timers.h"
#include "OffMode.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

static bool hovering;
#define HOVER_ENGAGE    (60 * M_PI / 180.0)

void AutoHoverInit(void) {
    hovering = false;
}

void AutoHoverAuto(void) {
    float ailTarget;
    float elevTarget;
    int stickOffset;
    AttitudeData currentAttitude;
    Vector ypr;

    __builtin_set_isr_state(3);
    __builtin_enable_interrupts();
    currentAttitude = attitude;
    __builtin_set_isr_state(0);
    __builtin_enable_interrupts();
    ypr = toYPR(attitude.qAttitude);
    if (!hovering && ypr.pitch >= HOVER_ENGAGE) {
        hovering = true;
        SetTarget(ELEV_CONTROLLER, 90 * M_PI / 180.0);
        SetTarget(RUD_CONTROLLER, 0);
        SetTarget(AIL_CONTROLLER, 0);
    } else if (hovering && ypr.pitch < HOVER_ENGAGE) {
        hovering = false;
    }
    
    if (hovering) {
        
    } else {
        OffMode();
    }
    
    
}

void AutoHoverManual(void) {

}
