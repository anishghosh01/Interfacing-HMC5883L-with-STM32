/*
 * hmc5883l.h
 *
 *  Created on: Oct 9, 2025
 *      Author: anish
 */

#ifndef INC_HMC5883L_H_
#define INC_HMC5883L_H_

#include "stm32f4xx_hal.h"
#include <math.h>

// HMC5883L I2C Address
#define HMC5883L_ADDRESS (0x1E << 1) // 0x3C shifted for 7-bit address

// HMC5883L Register Addresses
#define HMC5883L_REG_CONFIG_A 0x00
#define HMC5883L_REG_CONFIG_B 0x01
#define HMC5883L_REG_MODE 0x02
#define HMC5883L_REG_DATA_X_MSB 0x03

/**
 * @brief  HMC5883L sensor gain settings (scale).
 */
typedef enum {
    HMC5883L_SCALE_0_88GA = 0x00, // Gain: 1370 LSB/Gauss
    HMC5883L_SCALE_1_3GA  = 0x20, // Gain: 1090 LSB/Gauss (default)
    HMC5883L_SCALE_1_9GA  = 0x40, // Gain: 820 LSB/Gauss
    HMC5883L_SCALE_2_5GA  = 0x60, // Gain: 660 LSB/Gauss
    HMC5883L_SCALE_4_0GA  = 0x80, // Gain: 440 LSB/Gauss
    HMC5883L_SCALE_4_7GA  = 0xA0, // Gain: 390 LSB/Gauss
    HMC5883L_SCALE_5_6GA  = 0xC0, // Gain: 330 LSB/Gauss
    HMC5883L_SCALE_8_1GA  = 0xE0  // Gain: 230 LSB/Gauss
} HMC5883L_Scale_t;

/**
 * @brief  HMC5883L sensor data structure.
 */
typedef struct {
    I2C_HandleTypeDef *hi2c; // Pointer to I2C handle
    int16_t rawX;            // Raw X-axis data
    int16_t rawY;            // Raw Y-axis data
    int16_t rawZ;            // Raw Z-axis data
    float scaledX;         // Scaled X-axis data in Gauss
    float scaledY;         // Scaled Y-axis data in Gauss
    float scaledZ;         // Scaled Z-axis data in Gauss
    float scale_factor;      // Scale factor based on gain
} HMC5883L_t;

/**
 * @brief  Initializes the HMC5883L sensor.
 * @param  hmc: Pointer to HMC5883L_t handle structure.
 * @param  hi2c: Pointer to the I2C_HandleTypeDef for communication.
 * @param  scale: The desired gain setting from HMC5883L_Scale_t enum.
 * @retval HAL_StatusTypeDef: HAL status.
 */
HAL_StatusTypeDef HMC5883L_Init(HMC5883L_t *hmc, I2C_HandleTypeDef *hi2c, HMC5883L_Scale_t scale);

/**
 * @brief  Reads raw data from the magnetometer and calculates scaled data.
 * @param  hmc: Pointer to HMC5883L_t handle structure.
 * @retval HAL_StatusTypeDef: HAL status.
 */
HAL_StatusTypeDef HMC5883L_ReadData(HMC5883L_t *hmc);

/**
 * @brief  Calculates the heading in degrees from the sensor data.
 * @param  hmc: Pointer to HMC5883L_t handle structure.
 * @retval float: The calculated heading in degrees (0-360).
 */
float HMC5883L_GetHeading(HMC5883L_t *hmc);


#endif /* INC_HMC5883L_H_ */
