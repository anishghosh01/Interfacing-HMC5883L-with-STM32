/*
 * hmc5883l.c
 *
 *  Created on: Oct 9, 2025
 *      Author: anish
 */

#include "hmc5883l.h"

/**
 * @brief  Initializes the HMC5883L sensor.
 * @param  hmc: Pointer to HMC5883L_t handle structure.
 * @param  hi2c: Pointer to the I2C_HandleTypeDef for communication.
 * @param  scale: The desired gain setting from HMC5883L_Scale_t enum.
 * @retval HAL_StatusTypeDef: HAL status.
 */
HAL_StatusTypeDef HMC5883L_Init(HMC5883L_t *hmc, I2C_HandleTypeDef *hi2c, HMC5883L_Scale_t scale) {
    hmc->hi2c = hi2c;

    // Set scale factor
    switch (scale) {
        case HMC5883L_SCALE_0_88GA: hmc->scale_factor = 1.0f / 1370.0f; break;
        case HMC5883L_SCALE_1_3GA:  hmc->scale_factor = 1.0f / 1090.0f; break;
        case HMC5883L_SCALE_1_9GA:  hmc->scale_factor = 1.0f / 820.0f;  break;
        case HMC5883L_SCALE_2_5GA:  hmc->scale_factor = 1.0f / 660.0f;  break;
        case HMC5883L_SCALE_4_0GA:  hmc->scale_factor = 1.0f / 440.0f;  break;
        case HMC5883L_SCALE_4_7GA:  hmc->scale_factor = 1.0f / 390.0f;  break;
        case HMC5883L_SCALE_5_6GA:  hmc->scale_factor = 1.0f / 330.0f;  break;
        case HMC5883L_SCALE_8_1GA:  hmc->scale_factor = 1.0f / 230.0f;  break;
        default: return HAL_ERROR;
    }

    HAL_StatusTypeDef ret;
    uint8_t reg_data;

    // Configure Register A: 8-sample average, 75 Hz data output rate
    reg_data = 0x78;
    ret = HAL_I2C_Mem_Write(hmc->hi2c, HMC5883L_ADDRESS, HMC5883L_REG_CONFIG_A, 1, &reg_data, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;

    // Configure Register B: Set gain
    reg_data = (uint8_t)scale;
    ret = HAL_I2C_Mem_Write(hmc->hi2c, HMC5883L_ADDRESS, HMC5883L_REG_CONFIG_B, 1, &reg_data, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;

    // Configure Mode Register: Continuous-measurement mode
    reg_data = 0x00;
    ret = HAL_I2C_Mem_Write(hmc->hi2c, HMC5883L_ADDRESS, HMC5883L_REG_MODE, 1, &reg_data, 1, HAL_MAX_DELAY);
    return ret;
}

/**
 * @brief  Reads raw data from the magnetometer and calculates scaled data.
 * @param  hmc: Pointer to HMC5883L_t handle structure.
 * @retval HAL_StatusTypeDef: HAL status.
 */
HAL_StatusTypeDef HMC5883L_ReadData(HMC5883L_t *hmc) {
    uint8_t buffer[6];

    // Read 6 bytes of data starting from the DATA_X_MSB register
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(hmc->hi2c, HMC5883L_ADDRESS, HMC5883L_REG_DATA_X_MSB, 1, buffer, 6, HAL_MAX_DELAY);

    if (ret == HAL_OK) {
        // Combine MSB and LSB for each axis (order is X, Z, Y)
        hmc->rawX = (int16_t)(buffer[0] << 8 | buffer[1]);
        hmc->rawZ = (int16_t)(buffer[2] << 8 | buffer[3]);
        hmc->rawY = (int16_t)(buffer[4] << 8 | buffer[5]);

        // Calculate scaled values in Gauss
        hmc->scaledX = hmc->rawX * hmc->scale_factor;
        hmc->scaledY = hmc->rawY * hmc->scale_factor;
        hmc->scaledZ = hmc->rawZ * hmc->scale_factor;
    }

    return ret;
}

/**
 * @brief  Calculates the heading in degrees from the sensor data.
 * @param  hmc: Pointer to HMC5883L_t handle structure.
 * @retval float: The calculated heading in degrees (0-360).
 */
float HMC5883L_GetHeading(HMC5883L_t *hmc) {
    // Calculate heading from X and Y axes
    float heading = atan2(hmc->scaledY, hmc->scaledX);

    // Convert heading from radians to degrees
    heading *= 180.0f / M_PI;

    // Normalize to 0-360 degrees
    if (heading < 0) {
        heading += 360.0f;
    }

    return heading;
}



