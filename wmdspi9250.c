/*
*********************************************************************************************************
						WMD模块化散装库————————MPU9250 SPI方式（HAL限定）
		初始化：①需要SPI的IO口 更改硬件外设号 MPU片选线 按需更改MPU初始化参数(详见各文件头部宏定义)
									②主函数调用void mpu9250_init(void)和IMU_Init()
									③按需加校准函数 uint8_t IMU_Calibrate(void) 
									④周期调用IMUSO3Thread()即可
									使用：直接读取imu结构体的数据即可使用
											2017.11.14		V1.2
*********************************************************************************************************
*/
#include "wmdspi9250.h"
#include "stm32f4xx.h"
#include <string.h>
#include <math.h>
//需要自己转接的3个宏定义
#define MPU_SPI hspi1
#define MPU_EA HAL_GPIO_WritePin(SPI_NSS_GPIO_Port,SPI_NSS_Pin,GPIO_PIN_RESET) //使能SPI的MPU6050
#define MPU_DA HAL_GPIO_WritePin(SPI_NSS_GPIO_Port,SPI_NSS_Pin,GPIO_PIN_SET)   //失能SPI的MPU6050

int16_t tmp;
static uint8_t tx, rx;
uint8_t mpu_data_buf[14];
Mpu_data_t mpu_data;
MPU9250_Raw_Data  g_Raw_Data,g_Raw_Data_last;



//include accel and gyro  and temperature data
/**
  * @brief  send a byte to specific address register of mpu
  *         
  * @param  address and data to write : 
	*
  * @retval none
  */
void mpu_spi_write_reg(uint8_t addr, uint8_t data){
	MPU_EA;
	tx	= addr & 0x7f;
	HAL_SPI_TransmitReceive(&MPU_SPI, &tx, &rx, 1, 100);
	tx  = data;
	HAL_SPI_TransmitReceive(&MPU_SPI, &tx, &rx, 1, 100);
	MPU_DA;
}

void mpu_read_regs(u8 firstAddr, u8 *pBuff, u8 length){
	MPU_EA;
	
	tx = firstAddr | 0x80;
	HAL_SPI_TransmitReceive(&MPU_SPI, &tx, &rx, 1, 0x05);
	HAL_SPI_Receive(&MPU_SPI, pBuff, length, 0x05);
	MPU_DA;
}

void mpu_read_Raw(){
	mpu_read_regs(MPU6500_ACCEL_XOUT_H, mpu_data_buf, 14);
	
	g_Raw_Data.Accel_X = 		mpu_data_buf[0]<<8 | mpu_data_buf[1];
	g_Raw_Data.Accel_Y = 		mpu_data_buf[2]<<8 | mpu_data_buf[3];
	g_Raw_Data.Accel_Z = 		mpu_data_buf[4]<<8 | mpu_data_buf[5];
	
	g_Raw_Data.Gyro_X = 	mpu_data_buf[8]<<8  | mpu_data_buf[9];
	g_Raw_Data.Gyro_Y = 	mpu_data_buf[10]<<8 | mpu_data_buf[11];
	g_Raw_Data.Gyro_Z = 	mpu_data_buf[12]<<8 | mpu_data_buf[13];
	
	g_Raw_Data.Temp = 	mpu_data_buf[6]<<8 | mpu_data_buf[7];
}

#define MPU9250_InitRegNum 8
void mpu9250_init(void)
{
	uint8_t MPU6500_Init_Data[MPU9250_InitRegNum][2] = {
		{0x80, MPU6500_PWR_MGMT_1},     // Reset Device
		{0x01, MPU6500_PWR_MGMT_1},     // Clock Source
		{0x00, MPU6500_PWR_MGMT_2},     // Enable Acc & Gyro
		{0x06, MPU6500_CONFIG},         // LPS_5Hz
		{0x18, MPU6500_GYRO_CONFIG},    // +-2000dps
		{0x10, MPU6500_ACCEL_CONFIG},   // +-8G
		{0x0b, MPU6500_ACCEL_CONFIG_2}, // Set Acc Data Rates
//		{0x30, MPU6500_INT_PIN_CFG},    // 
//		{0x40, MPU6500_I2C_MST_CTRL},   // I2C Speed 348 kHz
		{0x30, MPU6500_USER_CTRL},      // Enable AUX
	};
	for(u8 i=0; i<MPU9250_InitRegNum; i++)
	{
		mpu_spi_write_reg(MPU6500_Init_Data[i][1], MPU6500_Init_Data[i][0]);
		HAL_Delay(5);
  }
	HAL_Delay(500);
}
uint8_t MPU_cheak(void)//检查mpu9250通信是否正常
{
	uint8_t rxbuf;
	mpu_read_regs(0x75,&rxbuf,1);
	return rxbuf;
}
