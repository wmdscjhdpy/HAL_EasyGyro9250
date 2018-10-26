#ifndef WMD_SPI_9250
#define WMD_SPI_9250

#include "spi.h"
#include "quad_math.h"
#define MOTION_DRIVER_TARGET_MSP430     //借用MSP430来装载驱动
#define MPU9250
#define SPI_MPU_TAKE// 如果不用SPI方式 一定要注释这段
#define mpu_write_mem spi_mem_send
#define mpu_read_mem spi_mem_read
#define u8 unsigned char



#define MPU9250A_2g       (0.000061035156f) // 0.000061035156 g/LSB
#define MPU9250A_4g       (0.000122070312f) // 0.000122070312 g/LSB
#define MPU9250A_8g       (0.000244140625f) // 0.000244140625 g/LSB
#define MPU9250A_16g      (0.000488281250f) // 0.000488281250 g/LSB

#define MPU9250G_250dps   (0.007633587786f) // 0.007633587786 dps/LSB
#define MPU9250G_500dps   (0.015267175572f) // 0.015267175572 dps/LSB
#define MPU9250G_1000dps  (0.030487804878f) // 0.030487804878 dps/LSB
#define MPU9250G_2000dps  (0.060975609756f) // 0.060975609756 dps/LSB

#define MPU9250M_4800uT   (0.6f)            // 0.6 uT/LSB

#define MPU9250T_85degC   (0.002995177763f) // 0.002995177763 degC/LSB


#define MPU6500_I2C_ADDR            ((u8)0xD0)
#define MPU6500_Device_ID           ((u8)0x71)  // In MPU9250

#define MPU6500_SELF_TEST_XG        ((u8)0x00)
#define MPU6500_SELF_TEST_YG        ((u8)0x01)
#define MPU6500_SELF_TEST_ZG        ((u8)0x02)
#define MPU6500_SELF_TEST_XA        ((u8)0x0D)
#define MPU6500_SELF_TEST_YA        ((u8)0x0E)
#define MPU6500_SELF_TEST_ZA        ((u8)0x0F)
#define MPU6500_XG_OFFSET_H         ((u8)0x13)
#define MPU6500_XG_OFFSET_L         ((u8)0x14)
#define MPU6500_YG_OFFSET_H         ((u8)0x15)
#define MPU6500_YG_OFFSET_L         ((u8)0x16)
#define MPU6500_ZG_OFFSET_H         ((u8)0x17)
#define MPU6500_ZG_OFFSET_L         ((u8)0x18)
#define MPU6500_SMPLRT_DIV          ((u8)0x19)
#define MPU6500_CONFIG              ((u8)0x1A)
#define MPU6500_GYRO_CONFIG         ((u8)0x1B)
#define MPU6500_ACCEL_CONFIG        ((u8)0x1C)
#define MPU6500_ACCEL_CONFIG_2      ((u8)0x1D)
#define MPU6500_LP_ACCEL_ODR        ((u8)0x1E)
#define MPU6500_MOT_THR             ((u8)0x1F)
#define MPU6500_FIFO_EN             ((u8)0x23)
#define MPU6500_I2C_MST_CTRL        ((u8)0x24)
#define MPU6500_I2C_SLV0_ADDR       ((u8)0x25)
#define MPU6500_I2C_SLV0_REG        ((u8)0x26)
#define MPU6500_I2C_SLV0_CTRL       ((u8)0x27)
#define MPU6500_I2C_SLV1_ADDR       ((u8)0x28)
#define MPU6500_I2C_SLV1_REG        ((u8)0x29)
#define MPU6500_I2C_SLV1_CTRL       ((u8)0x2A)
#define MPU6500_I2C_SLV2_ADDR       ((u8)0x2B)
#define MPU6500_I2C_SLV2_REG        ((u8)0x2C)
#define MPU6500_I2C_SLV2_CTRL       ((u8)0x2D)
#define MPU6500_I2C_SLV3_ADDR       ((u8)0x2E)
#define MPU6500_I2C_SLV3_REG        ((u8)0x2F)
#define MPU6500_I2C_SLV3_CTRL       ((u8)0x30)
#define MPU6500_I2C_SLV4_ADDR       ((u8)0x31)
#define MPU6500_I2C_SLV4_REG        ((u8)0x32)
#define MPU6500_I2C_SLV4_DO         ((u8)0x33)
#define MPU6500_I2C_SLV4_CTRL       ((u8)0x34)
#define MPU6500_I2C_SLV4_DI         ((u8)0x35)
#define MPU6500_I2C_MST_STATUS      ((u8)0x36)
#define MPU6500_INT_PIN_CFG         ((u8)0x37)
#define MPU6500_INT_ENABLE          ((u8)0x38)
#define MPU6500_INT_STATUS          ((u8)0x3A)
#define MPU6500_ACCEL_XOUT_H        ((u8)0x3B)
#define MPU6500_ACCEL_XOUT_L        ((u8)0x3C)
#define MPU6500_ACCEL_YOUT_H        ((u8)0x3D)
#define MPU6500_ACCEL_YOUT_L        ((u8)0x3E)
#define MPU6500_ACCEL_ZOUT_H        ((u8)0x3F)
#define MPU6500_ACCEL_ZOUT_L        ((u8)0x40)
#define MPU6500_TEMP_OUT_H          ((u8)0x41)
#define MPU6500_TEMP_OUT_L          ((u8)0x42)
#define MPU6500_GYRO_XOUT_H         ((u8)0x43)
#define MPU6500_GYRO_XOUT_L         ((u8)0x44)
#define MPU6500_GYRO_YOUT_H         ((u8)0x45)
#define MPU6500_GYRO_YOUT_L         ((u8)0x46)
#define MPU6500_GYRO_ZOUT_H         ((u8)0x47)
#define MPU6500_GYRO_ZOUT_L         ((u8)0x48)
#define MPU6500_EXT_SENS_DATA_00    ((u8)0x49)
#define MPU6500_EXT_SENS_DATA_01    ((u8)0x4A)
#define MPU6500_EXT_SENS_DATA_02    ((u8)0x4B)
#define MPU6500_EXT_SENS_DATA_03    ((u8)0x4C)
#define MPU6500_EXT_SENS_DATA_04    ((u8)0x4D)
#define MPU6500_EXT_SENS_DATA_05    ((u8)0x4E)
#define MPU6500_EXT_SENS_DATA_06    ((u8)0x4F)
#define MPU6500_EXT_SENS_DATA_07    ((u8)0x50)
#define MPU6500_EXT_SENS_DATA_08    ((u8)0x51)
#define MPU6500_EXT_SENS_DATA_09    ((u8)0x52)
#define MPU6500_EXT_SENS_DATA_10    ((u8)0x53)
#define MPU6500_EXT_SENS_DATA_11    ((u8)0x54)
#define MPU6500_EXT_SENS_DATA_12    ((u8)0x55)
#define MPU6500_EXT_SENS_DATA_13    ((u8)0x56)
#define MPU6500_EXT_SENS_DATA_14    ((u8)0x57)
#define MPU6500_EXT_SENS_DATA_15    ((u8)0x58)
#define MPU6500_EXT_SENS_DATA_16    ((u8)0x59)
#define MPU6500_EXT_SENS_DATA_17    ((u8)0x5A)
#define MPU6500_EXT_SENS_DATA_18    ((u8)0x5B)
#define MPU6500_EXT_SENS_DATA_19    ((u8)0x5C)
#define MPU6500_EXT_SENS_DATA_20    ((u8)0x5D)
#define MPU6500_EXT_SENS_DATA_21    ((u8)0x5E)
#define MPU6500_EXT_SENS_DATA_22    ((u8)0x5F)
#define MPU6500_EXT_SENS_DATA_23    ((u8)0x60)
#define MPU6500_I2C_SLV0_DO         ((u8)0x63)
#define MPU6500_I2C_SLV1_DO         ((u8)0x64)
#define MPU6500_I2C_SLV2_DO         ((u8)0x65)
#define MPU6500_I2C_SLV3_DO         ((u8)0x66)
#define MPU6500_I2C_MST_DELAY_CTRL  ((u8)0x67)
#define MPU6500_SIGNAL_PATH_RESET   ((u8)0x68)
#define MPU6500_MOT_DETECT_CTRL     ((u8)0x69)
#define MPU6500_USER_CTRL           ((u8)0x6A)
#define MPU6500_PWR_MGMT_1          ((u8)0x6B)
#define MPU6500_PWR_MGMT_2          ((u8)0x6C)
#define MPU6500_FIFO_COUNTH         ((u8)0x72)
#define MPU6500_FIFO_COUNTL         ((u8)0x73)
#define MPU6500_FIFO_R_W            ((u8)0x74)
#define MPU6500_WHO_AM_I            ((u8)0x75)	// ID = 0x71 In MPU9250
#define MPU6500_XA_OFFSET_H         ((u8)0x77)
#define MPU6500_XA_OFFSET_L         ((u8)0x78)
#define MPU6500_YA_OFFSET_H         ((u8)0x7A)
#define MPU6500_YA_OFFSET_L         ((u8)0x7B)
#define MPU6500_ZA_OFFSET_H         ((u8)0x7D)
#define MPU6500_ZA_OFFSET_L         ((u8)0x7E)

/* ---- AK8963 Reg In MPU9250 ----------------------------------------------- */

#define AK8963_I2C_ADDR             ((u8)0x18)
#define AK8963_Device_ID            ((u8)0x48)

// Read-only Reg
#define AK8963_WIA                  ((u8)0x00)
#define AK8963_INFO                 ((u8)0x01)
#define AK8963_ST1                  ((u8)0x02)
#define AK8963_HXL                  ((u8)0x03)
#define AK8963_HXH                  ((u8)0x04)
#define AK8963_HYL                  ((u8)0x05)
#define AK8963_HYH                  ((u8)0x06)
#define AK8963_HZL                  ((u8)0x07)
#define AK8963_HZH                  ((u8)0x08)
#define AK8963_ST2                  ((u8)0x09)
// Write/Read Reg
#define AK8963_CNTL1                ((u8)0x0A)
#define AK8963_CNTL2                ((u8)0x0B)
#define AK8963_ASTC                 ((u8)0x0C)
#define AK8963_TS1                  ((u8)0x0D)
#define AK8963_TS2                  ((u8)0x0E)
#define AK8963_I2CDIS               ((u8)0x0F)
// Read-only Reg ( ROM )
#define AK8963_ASAX                 ((u8)0x10)
#define AK8963_ASAY                 ((u8)0x11)
#define AK8963_ASAZ                 ((u8)0x12)

typedef struct {
	int16_t ax;
	int16_t ay;
	int16_t az;
	
	int16_t gx;
	int16_t gy;
	int16_t gz;

	int16_t temperature;

}Mpu_data_t;

typedef struct MPU9250_Raw_Data
{
    short Accel_X;  //寄存器原生X轴加速度表示值
    short Accel_Y;  //寄存器原生Y轴加速度表示值
    short Accel_Z;  //寄存器原生Z轴加速度表示值
    short Temp;     //寄存器原生温度表示值
    short Gyro_X;   //寄存器原生X轴陀螺仪表示值
    short Gyro_Y;   //寄存器原生Y轴陀螺仪表示值
    short Gyro_Z;   //寄存器原生Z轴陀螺仪表示值
	short Mag_X;
	short Mag_Y;
	short Mag_Z;
}MPU9250_Raw_Data;

//给软解用的结构体
extern MPU9250_Raw_Data  g_Raw_Data,g_Raw_Data_last;
void mpu_read_Raw(void);
void mpu_write_reg(uint8_t addr, uint8_t data);
void mpu9250_init(void);
uint8_t MPU_cheak(void);//检查mpu9250通信是否正常
//以上用于读取原始数据/基本寄存器




//仅适用于HAL库的扩展 宏定义决定spi接口
#define HMPU9250 &hspi1
#define MPU9250ADDR 
//定义传输过程中的超时限制
#define TIMEOUT 10


//以下为dmp用户参数
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define QUAT_ON         (0x04)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (200)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)
    
#define Acceleration_Of_Gravity     9.87F


struct hal_s {
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
    unsigned char motion_int_mode;
};

//函数声明区
uint8_t MPU9250DMP_InitConfig(uint8_t mode);
uint8_t MPU9250DMP_GetEuler(float *Pitch, float *Roll, float *Yaw);
uint8_t MPU9250_GetGyro(float *X, float *Y, float *Z);
uint8_t MPU9250_GetAccel(float *X, float *Y, float *Z);

uint8_t MPU9250DMPU_Selftest(uint8_t mode);

int spi_mem_send(unsigned char reg_addr,unsigned char length, unsigned char *data);
int spi_mem_read(unsigned char reg_addr,unsigned char length, unsigned char *data);
uint8_t spi_buffersend(unsigned char slave_addr, unsigned char reg_addr,unsigned char length, unsigned char const *data);
uint8_t spi_bufferread(unsigned char slave_addr, unsigned char reg_addr,unsigned char length, unsigned char *data);
uint8_t get_ms(unsigned long *count);//dmp欺骗函数
#endif
