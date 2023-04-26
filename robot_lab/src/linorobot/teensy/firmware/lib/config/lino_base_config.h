#ifndef LINO_BASE_CONFIG_H
#define LINO_BASE_CONFIG_H
#define LINO_BASE MECANUM         // Mecanum 

//uncomment the motor driver you're using
#define USE_L298_DRIVER
#define USE_MPU6050_IMU

#define DEBUG 1


//thong so cua dong co vs arduino 
#define MAX_RPM 1000              
#define COUNTS_PER_REV 1000      
#define WHEEL_DIAMETER 0.076       
#define PWM_BITS 8              
#define LR_WHEELS_DISTANCE 0.5  
#define FR_WHEELS_DISTANCE 0.4   
#define MAX_STEERING_ANGLE 0.415 
 
//encode
#define MOTOR1_ENCODER_A 2
#define MOTOR1_ENCODER_B 4 

#define MOTOR2_ENCODER_A 3
#define MOTOR2_ENCODER_B 7 

#define MOTOR3_ENCODER_A 19
#define MOTOR3_ENCODER_B 8 

#define MOTOR4_ENCODER_A 18
#define MOTOR4_ENCODER_B 9

//MOTOR PINS
#ifdef USE_L298_DRIVER
  #define MOTOR_DRIVER L298

  #define MOTOR1_PWM 11
  #define MOTOR1_IN_A 24
  #define MOTOR1_IN_B 25

  #define MOTOR2_PWM 5
  #define MOTOR2_IN_A 26
  #define MOTOR2_IN_B 27

  #define MOTOR3_PWM 6
  #define MOTOR3_IN_A 28
  #define MOTOR3_IN_B 29

  #define MOTOR4_PWM 46
  #define MOTOR4_IN_A 30
  #define MOTOR4_IN_B 31

  #define PWM_MAX pow(2, PWM_BITS) - 6
  #define PWM_MIN -PWM_MAX
#endif 

#define STEERING_PIN 7

#endif
