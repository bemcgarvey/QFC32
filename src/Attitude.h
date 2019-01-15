//*******************************************************************//
// File: Attitude.h                                                  //
// Author: Brad McGarvey                                             //
// Project: QFC32 Flight Controller                                  //
//                                                                   //
// Compiler: XC32                                                    //
// MCU: PIC32MX270F256B                                              // 
//                                                                   //
// Description: Attitude data and calculations                       //
//                                                                   //
//*******************************************************************// 

#ifndef ATTITUDE_H
#define	ATTITUDE_H

#include <stdint.h>
#include <math.h>

typedef union { 
    struct {
        float x;
        float y;
        float z;
    };
    struct {
        float roll;
        float pitch;
        float yaw;
    };
    struct {
        float rollRate;
        float pitchRate;
        float yawRate;
    };
} Vector;

typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quaternion;

typedef struct {
    Quaternion qAttitude;
    Vector gyroRates;
    Vector gyroAvg;
    int zSign;  //sign of z axis acceleration: 1 = upright, -1 = inverted
} AttitudeData;

extern volatile AttitudeData attitude;

#define RAD_TO_DEGREES     (180.0 / M_PI)

#ifdef	__cplusplus
extern "C" {
#endif
    Quaternion MulQuaternion(Quaternion lhs, Quaternion rhs);
    Quaternion ScalerMulQuaternion(float a, Quaternion q);
    Quaternion SumQuaternion(Quaternion lhs, Quaternion rhs);
    Quaternion QuaternionInt(Vector v, int t);
    Vector ScalerMulVector(float a, Vector v);
    Vector VecSum(Vector lhs, Vector rhs);
    Vector VecCrossProduct(Vector lhs, Vector rhs);
    Vector RotateR(Vector v, Quaternion q);
    Vector RotateL(Quaternion q, Vector v);
    void NormalizeQuat(Quaternion *q);
    Quaternion toQuaternion(Vector v);
    Vector toYPR(Quaternion q);
    
    void InitAttitude(void);
    void UpdateAttitude(void);

    float AccelPitch(void);
    float AccelRoll(void);
    float AccelYaw(void);
    Vector AccelYPR(void);
    
#ifdef	__cplusplus
}
#endif

#endif	/* ATTITUDE_H */

