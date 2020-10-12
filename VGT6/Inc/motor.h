/*
 * motor.h
 *
 *  Created on: 2020. 1. 3.
 *      Author: cctv1
 */
#include <stdio.h>
#ifdef   MOTOR_H_
   #ifndef _MOTOR_GLOBAL_
      #define _MOTOR_GLOBAL_
   #endif
#else
   #ifndef _MOTOR_GLOBAL_
      #define _MOTOR_GLOBAL_   extern
   #endif
#endif


#define UP      3
#define FALL   2
#define ON      1
#define ON_L   1L
#define OFF      0
#define TRUE    1
#define FALSE    0
#define HIGH   1
#define LOW      0
//#define NULL   (void *)0


typedef volatile struct Motor_Variable
{
   uint16_t   U16Qep_Sample,
            U16Tick;

   int16_t   i16QepVal,
         Stop_Flag;
   float   f_Tick_Distance,
         f_Distace_Sum,
         f_Kp,
         f_Ki,
         f_Kd,
         f_User_Distacne,
         f_Remaning_Disatance,
         f_Current_Velocity,
         f_StopDistance,
         f_Decel_Velocity,
         f_Next_Velocity,
         f_User_Velocity,
         f_ErrVelocity[4],
         f_ErrVelocitySum,
         f_proportionalterm,
         f_derivativeterm,
         f_integralterm,
         f_pidoutterm,
         f_pidoutresult,
         f_encoder,
         f_Current_Velocity_temp[100],
         f_Current_Velocity_av,
         f_posadjrate;

   float   f_Accel;

}Motor_Val;

typedef volatile struct Angle_Variable
{
   float f_Err_angle_roll[4],
        f_Err_angle_sum_roll,
        f_proportionalterm_roll,
        f_derivativeterm_roll,
        f_integralterm_roll,
        f_pidoutterm_roll,
        f_Current_angle_roll,
        f_Next_angle_roll,
        f_Err_angle_yaw[4],
        f_Err_angle_sum_yaw,
        f_proportionalterm_yaw,
        f_derivativeterm_yaw,
        f_integralterm_yaw,
        f_pidoutterm_yaw,
        f_Current_angle_yaw,
        f_Next_angle_yaw,
        f_kp_roll,
        f_ki_roll,
        f_kd_roll,
        f_kp_yaw,
        f_ki_yaw,
        f_roll_lifo[100],
        f_roll_sog,
        f_kd_yaw;

}Angle_Val;
_MOTOR_GLOBAL_ float f_roll,
      f_pitch,
      f_yaw;

_MOTOR_GLOBAL_ Angle_Val Angle;
_MOTOR_GLOBAL_ Motor_Val L_Motor;
_MOTOR_GLOBAL_ Motor_Val R_Motor;
_MOTOR_GLOBAL_ void InitMotor_R(  Motor_Val *pmotor);
_MOTOR_GLOBAL_ void InitMotor_L(  Motor_Val *pmotor);
_MOTOR_GLOBAL_ volatile uint16_t gMovestate,
                        g_u16motortic,
                        gStopcount,
                        gUint16user_speed,
                        gUint16speedcnt,
                        adcval[3],
                        g_uint16_pwm_flag;
_MOTOR_GLOBAL_ volatile int16_t g_servo_pulse;

_MOTOR_GLOBAL_ volatile uint32_t gUserTimeCnt;
_MOTOR_GLOBAL_ volatile uint32_t gsamplecount;
_MOTOR_GLOBAL_ volatile uint8_t gSencount;

_MOTOR_GLOBAL_ volatile int g_int32_sen_cnt;
