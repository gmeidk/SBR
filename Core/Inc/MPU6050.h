/*
 * MPU6050.h
 *
 *  Created on: May 14, 2021
 *      Authors: Quatela Alessandro, Roberto Giuseppe
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

typedef struct {
	float angleY, Gy;
} Control_Data_t;

int MPU6050_Init(I2C_HandleTypeDef hi2c, int iter, int timeout);
Control_Data_t MPU6050_Data(I2C_HandleTypeDef hi2c, float dt);

#endif /* INC_MPU6050_H_ */
