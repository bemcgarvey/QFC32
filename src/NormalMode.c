#include "NormalMode.h"
#include "Controller.h"
#include "Data.h"
#include "Attitude.h"
#include "Config.h"
#include "InputCapture.h"
#include "Timers.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

static uint16_t calculateAilServo(float rate);
static uint16_t calculateElevServo(float rate);
static uint16_t calculateRudServo(float rate);

static uint16_t calculateAilServoRateOnly(float rate);
static uint16_t calculateElevServoRateOnly(float rate);
static uint16_t calculateRudServoRateOnly(float rate);

#define OUTPUT_SCALE    1000

void NormalInit(void) {
    SetTarget(AIL_CONTROLLER, 0.0);
    SetTarget(ELEV_CONTROLLER, 0.0);
}

void NormalAuto(void) {
    float ailTarget;
    float elevTarget;
    float rudTarget;
    int stickOffset;
    AttitudeData currentAttitude;
    //Vector ypr;

    __builtin_set_isr_state(3);
    __builtin_enable_interrupts();
    currentAttitude = attitude;
    __builtin_set_isr_state(0);
    __builtin_enable_interrupts();
    //ypr = toYPR(currentAttitude.qAttitude);

    if (ailUpdate && pCurrentMode->aileron.enabled) {
        ailUpdate = false;
        stickOffset = inputChannels.ailIn - controllers[AIL_CONTROLLER].center;
        if (stickOffset < 0) {
            ailTarget = stickOffset / (float) (controllers[AIL_CONTROLLER].center - controllers[AIL_CONTROLLER].min);
        } else {
            ailTarget = stickOffset / (float) (controllers[AIL_CONTROLLER].max - controllers[AIL_CONTROLLER].center);
        }
        ailTarget *= pCurrentMode->maxRoll;
        SetTarget(AIL_CONTROLLER, ailTarget);
        outputChannels.ailOut = calculateAilServo(currentAttitude.gyroRates.rollRate);
    } else if (ailUpdate) {
        ailUpdate = false;
        outputChannels.ailOut = inputChannels.ailIn;
    }
    if (elevUpdate && pCurrentMode->elevator.enabled) {
        elevUpdate = false;
        stickOffset = inputChannels.elevIn - controllers[ELEV_CONTROLLER].center;
        if (stickOffset < 0) {
            elevTarget = stickOffset / (float) (controllers[ELEV_CONTROLLER].center - controllers[ELEV_CONTROLLER].min);
        } else {
            elevTarget = stickOffset / (float) (controllers[ELEV_CONTROLLER].max - controllers[ELEV_CONTROLLER].center);
        }
        elevTarget *= pCurrentMode->maxPitch;
        SetTarget(ELEV_CONTROLLER, elevTarget);
        outputChannels.elevOut = calculateElevServo(currentAttitude.gyroRates.pitchRate);
    } else if (elevUpdate) {
        elevUpdate = false;
        outputChannels.elevOut = inputChannels.elevIn;
    }
    if (rudUpdate && pCurrentMode->rudder.enabled) {
        rudUpdate = false;
        stickOffset = inputChannels.rudIn - controllers[RUD_CONTROLLER].center;
        if (stickOffset < 0) {
            rudTarget = stickOffset / (float) (controllers[RUD_CONTROLLER].center - controllers[RUD_CONTROLLER].min);
        } else {
            rudTarget = stickOffset / (float) (controllers[RUD_CONTROLLER].max - controllers[RUD_CONTROLLER].center);
        }
        rudTarget *= pCurrentMode->maxYaw;
        SetTarget(RUD_CONTROLLER, rudTarget);
        outputChannels.rudOut = calculateRudServo(currentAttitude.gyroRates.yawRate);
    } else if (rudUpdate) {
        //TODO Should rudder use rate gyro if not enabled?
        rudUpdate = false;
        outputChannels.rudOut = inputChannels.rudIn;
    }
    controllers[AIL_CONTROLLER].lastOutput = outputChannels.ailOut;
    controllers[ELEV_CONTROLLER].lastOutput = outputChannels.elevOut;
    controllers[RUD_CONTROLLER].lastOutput = outputChannels.rudOut;
}

void NormalManual(void) {
    AttitudeData currentAttitude;
    
    __builtin_set_isr_state(3);
    __builtin_enable_interrupts();
    currentAttitude = attitude;
    __builtin_set_isr_state(0);
    __builtin_enable_interrupts();
    
    if (ailUpdate && pCurrentMode->aileron.enabled) {
        ailUpdate = false;
        outputChannels.ailOut = calculateAilServoRateOnly(currentAttitude.gyroRates.rollRate);
    } else if (ailUpdate) {
        ailUpdate = false;
        outputChannels.ailOut = inputChannels.ailIn;
    }
    if (elevUpdate && pCurrentMode->elevator.enabled) {
        elevUpdate = false;
        outputChannels.elevOut = calculateElevServoRateOnly(currentAttitude.gyroRates.pitchRate);
    } else if (elevUpdate) {
        elevUpdate = false;
        outputChannels.elevOut = inputChannels.elevIn;
    }
    if (rudUpdate && pCurrentMode->rudder.enabled) {
        rudUpdate = false;
        outputChannels.rudOut = calculateRudServoRateOnly(currentAttitude.gyroRates.yawRate);
    } else if (rudUpdate) {
        rudUpdate = false;
        outputChannels.rudOut = inputChannels.rudIn;
    }
    controllers[AIL_CONTROLLER].lastOutput = outputChannels.ailOut;
    controllers[ELEV_CONTROLLER].lastOutput = outputChannels.elevOut;
    controllers[RUD_CONTROLLER].lastOutput = outputChannels.rudOut;
    
}

uint16_t calculateAilServo(float rate) {
    float out;
    float error;
    float dError;

    uint16_t servoOut = 0;
    int adjust;
    ControllerData *p;
    p = &controllers[AIL_CONTROLLER];
    error = p->target - rate;
    dError = error - p->lastError;
    p->lastError = error;
    out = p->kp * error + p->ki * p->errorSum + p->kd * dError;
    adjust = (int16_t) (out * OUTPUT_SCALE);
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

static uint16_t calculateElevServo(float rate) {

    return 1500 * 3;
}

static uint16_t calculateRudServo(float rate) {
    
    return 1500 * 3;
}

static uint16_t calculateAilServoRateOnly(float rate) {
    float error;
    float dRate;
    uint16_t servoOut;
    int adjust;
    
    ControllerData *p;
    p = &controllers[AIL_CONTROLLER];
    error = attitude.gyroAvg.rollRate - rate;
    dRate = error - p->lastError;
    p->lastError = error;
    adjust = error * p->kp + p->errorSum * p->ki + dRate * p->kd;
    adjust *= OUTPUT_SCALE;
    adjust *= p->direction;
    servoOut = inputChannels.ailIn + adjust;
    if (servoOut > p->max) {
        servoOut = p->max;
    } else if (servoOut < p->min) {
        servoOut = p->min;
    } else {
        p->errorSum += error;
    }
    return servoOut;
}

static uint16_t calculateElevServoRateOnly(float rate) {
    return 1500 * 3;
}

static uint16_t calculateRudServoRateOnly(float rate) {
    return 1500 * 3;
}