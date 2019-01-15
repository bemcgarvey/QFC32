//*******************************************************************//
// File: Attitude.c                                                  //
// Author: Brad McGarvey                                             //
// Project: QFC32 Flight Controller                                  //
//                                                                   //
// Compiler: XC32                                                    //
// MCU: PIC32MX270F256B                                              // 
//                                                                   //
// Description: Attitude data and calculations                       //
//                                                                   //
//*******************************************************************// 

#include <xc.h>
#include <Attitude.h>
#include <Data.h>
#include <MPU6050.h>

volatile AttitudeData attitude;

#define AVG_COUNT   10
static float ailRates[AVG_COUNT];
static float elevRates[AVG_COUNT];
static float rudRates[AVG_COUNT];

static int avgIndex;

void InitAttitude(void) {
    ConvertRawData();
    Vector v = AccelYPR();
    v.yaw = 0;
    attitude.qAttitude = toQuaternion(v);
    avgIndex = 0;
    int i;
    for (i = 0; i < AVG_COUNT; ++i) {
        ailRates[i] = 0;
        elevRates[i] = 0;
        rudRates[i] = 0;
    }
    attitude.gyroAvg.rollRate = 0;
    attitude.gyroAvg.pitchRate = 0;
    attitude.gyroAvg.yawRate = 0;
}

void UpdateAttitude(void) {
    Vector correction_Body, correction_World;
    Vector Accel_Body, Accel_World;
    Quaternion incrementalRotation;
    Vector gyroVec;
    static Vector VERTICAL = {0, 0, 1};

    //Update rolling average before rates are updated;
    attitude.gyroAvg.rollRate -= ailRates[avgIndex] / AVG_COUNT;
    ailRates[avgIndex] = attitude.gyroRates.rollRate;
    attitude.gyroAvg.rollRate += ailRates[avgIndex] / AVG_COUNT;
    ++avgIndex;
    if (avgIndex == AVG_COUNT) {
        avgIndex = 0;
    }
    
    ConvertRawData();
    attitude.gyroRates.rollRate = rawData.gyro_x * GYRO_SCALE_RAD;
    attitude.gyroRates.pitchRate = rawData.gyro_y * GYRO_SCALE_RAD;
    attitude.gyroRates.yawRate = rawData.gyro_z * GYRO_SCALE_RAD;
    
    Accel_Body.x = rawData.accel_x * ACCEL_SCALE;
    Accel_Body.y = rawData.accel_y * ACCEL_SCALE;
    Accel_Body.z = rawData.accel_z * ACCEL_SCALE;
    if (rawData.accel_z < 0) {
        attitude.zSign = -1;
    } else {
        attitude.zSign = 1;
    }
    Accel_World = RotateL(attitude.qAttitude, Accel_Body); // rotate accel from body frame to world frame
    correction_World = VecCrossProduct(Accel_World, VERTICAL); // cross product to determine error
    correction_Body = RotateR(correction_World, attitude.qAttitude); // rotate correction vector to body frame
    gyroVec = VecSum(attitude.gyroRates, correction_Body); // add correction vector to gyro data
    incrementalRotation = QuaternionInt(gyroVec, DT); // create incremental rotation quaternion
    attitude.qAttitude = MulQuaternion(incrementalRotation, attitude.qAttitude); // quaternion integration
}

float AccelPitch(void) {
    return atan2(-rawData.accel_x, sqrt(rawData.accel_y * rawData.accel_y + rawData.accel_z * rawData.accel_z));
}

float AccelRoll(void) {
    return atan2(rawData.accel_y, rawData.accel_z);
}

float AccelYaw(void) {
    //Only works while nose up
    return atan2(rawData.accel_y, -rawData.accel_x);
}

Vector AccelYPR(void) {
    Vector ypr;
    ypr.yaw = atan2(rawData.accel_y, rawData.accel_x);
    ypr.pitch = atan2(-rawData.accel_x, sqrt(rawData.accel_y * rawData.accel_y + rawData.accel_z * rawData.accel_z));
    ypr.roll = atan2(rawData.accel_y, rawData.accel_z);
    return ypr;
}

Quaternion MulQuaternion(Quaternion lhs, Quaternion rhs) {
    Quaternion prod;
    prod.x = lhs.w * rhs.x + lhs.z * rhs.y - lhs.y * rhs.z + lhs.x * rhs.w;
    prod.y = lhs.w * rhs.y + lhs.x * rhs.z + lhs.y * rhs.w - lhs.z * rhs.x;
    prod.z = lhs.y * rhs.x - lhs.x * rhs.y + lhs.w * rhs.z + lhs.z * rhs.w;
    prod.w = lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z;
    return prod;
}

Quaternion ScalerMulQuaternion(float a, Quaternion q) {
    q.w *= a;
    q.x *= a;
    q.y *= a;
    q.z *= a;
    return q;
}

Quaternion QuaternionInt(Vector v, int t) {
    float dt2;
    Quaternion a;
    dt2 = t * 0.0000005f;
    a.x = v.x * dt2;
    a.y = v.y * dt2;
    a.z = v.z * dt2;
    a.w = 1.0 - 0.5 * (a.x * a.x + a.y * a.y + a.z * a.z);
    return a;
}

Vector ScalerMulVector(float a, Vector v) {
    v.x *= a;
    v.y *= a;
    v.z *= a;
    return v;
}

Vector VecSum(Vector lhs, Vector rhs) {
    lhs.x += rhs.x;
    lhs.y += rhs.y;
    lhs.z += rhs.z;
    return lhs;
}

Vector VecCrossProduct(Vector lhs, Vector rhs) {
    Vector cross;
    cross.x = lhs.y * rhs.z - lhs.z * rhs.y;
    cross.y = lhs.z * rhs.x - lhs.x * rhs.z;
    cross.z = lhs.x * rhs.y - lhs.y * rhs.x;
    return cross;
}

Vector RotateR(Vector v, Quaternion q) {
    Vector r = {-q.x, -q.y, -q.z};
    return VecSum(v, VecCrossProduct(VecSum(r, r), VecSum(VecCrossProduct(r, v), ScalerMulVector(q.w, v))));

}

Vector RotateL(Quaternion q, Vector v) {
    Vector r = {q.x, q.y, q.z};
    return VecSum(v, VecCrossProduct(VecSum(r, r), VecSum(VecCrossProduct(r, v), ScalerMulVector(q.w, v))));
}

void NormalizeQuat(Quaternion *q) {
    float d = 1.0 / sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    q->w *= d;
    q->x *= d;
    q->y *= d;
    q->z *= d;
}

Quaternion toQuaternion(Vector v) {
    Quaternion q;
    float cy = cos(v.z * 0.5);
    float sy = sin(v.z * 0.5);
    float cr = cos(v.x * 0.5);
    float sr = sin(v.x * 0.5);
    float cp = cos(v.y * 0.5);
    float sp = sin(v.y * 0.5);

    q.w = cy * cr * cp + sy * sr * sp;
    q.x = cy * sr * cp - sy * cr * sp;
    q.y = cy * cr * sp + sy * sr * cp;
    q.z = sy * cr * cp - cy * sr * sp;
    return q;
}

Vector toYPR(Quaternion q) {
    Vector ypr;
    ypr.roll = atan2(q.y * q.z + q.w * q.x, 0.5f - (q.x * q.x + q.y * q.y));
    ypr.pitch = asin(-2.0f * (q.x * q.z - q.w * q.y));
    ypr.yaw = 0.0;
    return ypr;
}



