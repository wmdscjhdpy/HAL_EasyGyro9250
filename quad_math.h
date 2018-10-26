

#ifndef __QUAD_MATH_H__
#define __QUAD_MATH_H__
#include <math.h>
#include "stm32f4xx_hal.h"
#define MATH_TIMES                      10
#define CONSTANTS_ONE_G					9.80665f		/* m/s^2		*/
#define CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C		1.225f			/* kg/m^3		*/
#define CONSTANTS_AIR_GAS_CONST				287.1f 			/* J/(kg * K)		*/
#define CONSTANTS_ABSOLUTE_NULL_CELSIUS			-273.15f		/* 癈			*/
#define CONSTANTS_RADIUS_OF_EARTH			6371000			/* meters (m)		*/

#define IMU_SAMPLE_RATE 			1000

#define IMU_FILTER_CUTOFF_FREQ	10
#define qq30  1073741824.0f

//校准时间
#define ACC_CALC_TIME  1000//ms
#define GYRO_CALC_TIME   1000000l	//us
#define usTicks 100

#define Math_PERIOD (10)
typedef float  quad[4];
typedef float  vector3f[3];	//不可作为返回值，指针
typedef float  matrix3f[3][3];

 
typedef struct mat3_tt
{
float m[3][3];
}mat3;

typedef struct vec3_tt
{
float v[3];
}vec3;


enum{ROLL,PITCH,YAW,THROTTLE};
enum{X,Y,Z};

typedef struct IMU_tt
{
uint8_t caliPass;
uint8_t caliGyro;
uint8_t ready;
int16_t accADC[3];
int16_t gyroADC[3];
int16_t magADC[3];
float 	accRaw[3];		//m/s^2
float 	gyroRaw[3];		//rad/s 
float 	magRaw[3];		//
float   accOffset[3];		//m/s^2
float   gyroOffset[3]; 
float 	gyroADCOffset[3];
float   accb[3];		//filted, in body frame
float   accg[3];
float   gyro[3];
float   DCMgb[3][3];
float   q[4];
float   roll;				//deg
float   pitch;
float 	yaw;
float   rollRad;				//rad
float   pitchRad;
float 	yawRad;
}imu_t;

//用来存到flash里面
//typedef struct{
//	float   accOffset[3];		//m/s^2
//	float   gyroOffset[3]; 
//}Flash_Param_t;
 


#define CONSTANTS_ONE_G					9.80665f		/* m/s^2		*/
#define CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C		1.225f			/* kg/m^3		*/
#define CONSTANTS_AIR_GAS_CONST				287.1f 			/* J/(kg * K)		*/
#define CONSTANTS_ABSOLUTE_NULL_CELSIUS			-273.15f		/* 癈			*/
#define CONSTANTS_RADIUS_OF_EARTH			6371000			/* meters (m)		*/
void QuatUpdate(long q[]);
extern float z_est[];	// estimate z Vz  Az
extern float accel_NED[];
extern volatile float accFilted[3],gyroFilted[3];
extern float DCMbg[3][3],DCMgb[3][3];
extern float accZoffsetTemp;
extern float IMU_Pitch,IMU_Roll,IMU_Yaw;
extern imu_t imu;
extern uint8_t imuCaliFlag;
extern 	float euler[];	//rad
extern float MagYaw;
extern float AccVector;
static void eular2DCM(float DCM[3][3],float roll,float pitch,float yaw);

void IMU_Init(void);
void IMU_Process(void);
uint8_t IMU_Calibrate(void);
void ReadIMUSensorHandle(void);
uint8_t IMUCheck(void);

void getMagYaw(float *euler_Pitch_in,float *euler_Roll_in,float *euler_Yaw_out);
/* Function prototypes */
float invSqrt(float number);
static void NonlinearSO3AHRSinit(float ax, float ay, float az, float mx, float my, float mz);
static void NonlinearSO3AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, float dt);
//uint32_t micros(void);
extern uint8_t SaveFlashFlag;
void ReadGyroOffsetFromFlash(void);
void SaveGyroOffset2Flash(void);
void IMUSO3Thread(void);

#endif
