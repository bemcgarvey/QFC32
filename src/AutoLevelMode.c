

#include "AutoLevelMode.h"
#include "Controller.h"
#include "Data.h"
#include "Attitude.h"
#include "Config.h"
#include "InputCapture.h"
#include "Timers.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

static uint16_t calculateAilServo(float roll, float rate);
static uint16_t calculateElevServo(float pitch, float rate, float roll);
static uint16_t calculateRudServo(float yaw, float rate, float roll, int stick);

static int centerTime;
static bool leveling;
static bool limitingRoll;
static bool limitingPitch;

#define OUTPUT_SCALE    1000
#define CENTER_TIME     200  //200 ms

void AutoLevelInit(void) {
    centerTime = systemTickCount;
    leveling = false;
    limitingRoll = false;
    limitingPitch = false;
    SetTarget(AIL_CONTROLLER, 0.0);
    SetTarget(ELEV_CONTROLLER, 0.0);
}

void AutoLevelAuto(void) {
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
    ypr = toYPR(currentAttitude.qAttitude);

    if (ailUpdate && pCurrentMode->aileron.enabled) {
        ailUpdate = false;
        stickOffset = inputChannels.ailIn - controllers[AIL_CONTROLLER].center;
        stickOffset *= controllers[AIL_CONTROLLER].direction;
        if (stickOffset < 0) {
            ailTarget = stickOffset / (float) (controllers[AIL_CONTROLLER].center - controllers[AIL_CONTROLLER].min);
        } else {
            ailTarget = stickOffset / (float) (controllers[AIL_CONTROLLER].max - controllers[AIL_CONTROLLER].center);
        }
        ailTarget *= pCurrentMode->maxRoll;
        SetTarget(AIL_CONTROLLER, ailTarget);
        outputChannels.ailOut = calculateAilServo(ypr.roll, currentAttitude.gyroRates.rollRate);
    } else if (ailUpdate) {
        ailUpdate = false;
        outputChannels.ailOut = inputChannels.ailIn;
    }
    if (elevUpdate && pCurrentMode->elevator.enabled) {
        elevUpdate = false;
        stickOffset = inputChannels.elevIn - controllers[ELEV_CONTROLLER].center;
        stickOffset *= controllers[ELEV_CONTROLLER].direction;
        if (stickOffset < 0) {
            elevTarget = stickOffset / (float) (controllers[ELEV_CONTROLLER].center - controllers[ELEV_CONTROLLER].min);
        } else {
            elevTarget = stickOffset / (float) (controllers[ELEV_CONTROLLER].max - controllers[ELEV_CONTROLLER].center);
        }
        elevTarget *= pCurrentMode->maxPitch;
        SetTarget(ELEV_CONTROLLER, elevTarget);
        outputChannels.elevOut = calculateElevServo(ypr.pitch, currentAttitude.gyroRates.pitchRate, ypr.roll);
    } else if (elevUpdate) {
        elevUpdate = false;
        outputChannels.elevOut = inputChannels.elevIn;
    }

    //TODO If rudder enabled apply rate controller?
    if (rudUpdate) {
        rudUpdate = false;
        outputChannels.rudOut = inputChannels.rudIn;
    }
    controllers[AIL_CONTROLLER].lastOutput = outputChannels.ailOut;
    controllers[ELEV_CONTROLLER].lastOutput = outputChannels.elevOut;
    controllers[RUD_CONTROLLER].lastOutput = outputChannels.rudOut;
}

uint16_t calculateAilServo(float roll, float rate) {
    float out;
    float error;
    uint16_t servoOut = 0;
    int adjust;
    ControllerData *p;
    p = &controllers[AIL_CONTROLLER];
    error = p->target - roll;
    out = p->kp * error + p->ki * p->errorSum - p->kd * rate;
    adjust = (int16_t) (out * OUTPUT_SCALE);
    //TODO Compare adjust to MAX_ADJUST which is set by Config->ResponseRate
    //if greater then max set to max
    adjust *= p->direction;
    servoOut = p->lastOutput + adjust;
    if (servoOut > p->max) {
        servoOut = p->max;
    } else if (servoOut < p->min) {
        servoOut = p->min;
    } else {
        p->errorSum += error;
    }
    return servoOut;
}

static uint16_t calculateElevServo(float pitch, float rate, float roll) {
    float out;
    float error;
    uint16_t servoOut = 0;
    int adjust;
    ControllerData *p;
    p = &controllers[ELEV_CONTROLLER];
    error = p->target - pitch;
    out = p->kp * error + p->ki * p->errorSum - p->kd * rate;
    out *= cos(roll); //adjust for roll angle knife edge or inverted.
    adjust = (int16_t) (out * OUTPUT_SCALE);
    //TODO Compare adjust to MAX_ADJUST which is set by Config->ResponseRate
    //if greater then max set to max
    adjust *= p->direction;
    servoOut = p->lastOutput + adjust;
    if (servoOut > p->max) {
        servoOut = p->max;
    } else if (servoOut < p->min) {
        servoOut = p->min;
    } else {
        p->errorSum += error;
    }
    return servoOut;
}

void AutoLevelManual(void) {
    int ailStickOffset;
    int elevStickOffset;
    AttitudeData currentAttitude;
    Vector ypr;

    __builtin_set_isr_state(3);
    __builtin_enable_interrupts();
    currentAttitude = attitude;
    __builtin_set_isr_state(0);
    __builtin_enable_interrupts();
    ypr = toYPR(currentAttitude.qAttitude);

    //Check for centered controls
    if (ailUpdate || elevUpdate) {
        ailStickOffset = abs(inputChannels.ailIn - config.ailServo.center);
        elevStickOffset = abs(inputChannels.elevIn - config.elevServo.center);
        if (ailStickOffset > pCurrentMode->aileron.deadband || elevStickOffset > pCurrentMode->elevator.deadband) {
            leveling = false;
            centerTime = systemTickCount;
        } else if (!leveling && systemTickCount - centerTime > CENTER_TIME) {
            leveling = true;
            limitingRoll = limitingPitch = false;
            SetTarget(AIL_CONTROLLER, 0.0);
            SetTarget(ELEV_CONTROLLER, 0.0);
        }
    }
    //Check for out of range attitude
    if (!leveling) {
        if (!limitingRoll && fabs(ypr.roll) > pCurrentMode->maxRoll) {
            limitingRoll = true;
            if (ypr.roll > 0)
                SetTarget(AIL_CONTROLLER, pCurrentMode->maxRoll);
            else
                SetTarget(AIL_CONTROLLER, -(pCurrentMode->maxRoll));
        }
        if (!limitingPitch && fabs(ypr.pitch) > pCurrentMode->maxPitch) {
            limitingPitch = true;
            if (ypr.pitch > 0)
                SetTarget(ELEV_CONTROLLER, pCurrentMode->maxPitch);
            else
                SetTarget(ELEV_CONTROLLER, -(pCurrentMode->maxPitch));
        }
    }
    //Calculate aileron servo output
    if ((leveling || limitingRoll) && ailUpdate && pCurrentMode->aileron.enabled) {
        ailUpdate = false;
        uint16_t out = calculateAilServo(ypr.roll, currentAttitude.gyroRates.rollRate);
        if (limitingRoll) {
            if (abs(inputChannels.ailIn - config.ailServo.center) < abs(out - config.ailServo.center)) {
                out = inputChannels.ailIn;
                limitingRoll = false;
            }
        }
        outputChannels.ailOut = out;
    } else if (ailUpdate) {
        ailUpdate = false;
        outputChannels.ailOut = inputChannels.ailIn;
    }
    //Elevator servo output
    if ((leveling || limitingPitch) && elevUpdate && pCurrentMode->elevator.enabled) {
        elevUpdate = false;
        uint16_t out = calculateElevServo(ypr.pitch, currentAttitude.gyroRates.pitchRate, ypr.roll);
        if (limitingPitch) {
            if (abs(inputChannels.elevIn - config.elevServo.center) < abs(out - config.elevServo.center)) {
                out = inputChannels.elevIn;
                limitingPitch = false;
            }
        }
        outputChannels.elevOut = out;
    } else if (elevUpdate) {
        elevUpdate = false;
        outputChannels.elevOut = inputChannels.elevIn;
    }
    //TODO If rudder enabled apply rate controller?
    if (rudUpdate) {
        rudUpdate = false;
        outputChannels.rudOut = inputChannels.rudIn;
    }
    controllers[AIL_CONTROLLER].lastOutput = outputChannels.ailOut;
    controllers[ELEV_CONTROLLER].lastOutput = outputChannels.elevOut;
    controllers[RUD_CONTROLLER].lastOutput = outputChannels.rudOut;
}