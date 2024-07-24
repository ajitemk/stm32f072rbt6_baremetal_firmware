/*
 * i2c_Driver.h
 *
 *  Created on: Jul 10, 2024
 *      Author: ADMIN
 */

#ifndef DRIVER_INC_I2C_DRIVER_H_
#define DRIVER_INC_I2C_DRIVER_H_



/*
 * configuration structure for i2c peripheral
 */
typedef struct{
	uint32_t I2C_SCL_Speed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_AckControl;
	uint8_t I2C_FMDutyCycle;
}I2C_Config_t;

/*
 * Handle structure for i2c peripheral
 */
typedef struct{
	I2C2_RegDef_t *pI2Cx;
	I2C2_RegDef_t I2C_Config;
}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM 	100000
#define I2C_SCL_SPEED_FM 	400000
#define I2C_SCL_SPEED_FM2K 	200000

/*
 * i2C ackcontrol
 */

#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0
/*
 * i2C fmdutycycle
 */

#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1

/*********************************************************************************************
 * 								APIs supported by this driver
 * 				for more informatoin about the api check the function definition
 ********************************************************************************************/
/*
 * Peripheral Clock Setup
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);

/*
 * 		i2c init and De-init
 */

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);
#endif /* DRIVER_INC_I2C_DRIVER_H_ */
