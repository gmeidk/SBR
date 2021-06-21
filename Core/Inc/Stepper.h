/*
 * Stepper.h
 *
 *  Created on: May 14, 2021
 *      Authors: Quatela Alessandro, Roberto Giuseppe
 */

#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_

float PID_Control(Control_Data_t Control_Data, float dt);
float LQR_Control(Control_Data_t Control_Data, float dt);
void Speed_Actuation(TIM_HandleTypeDef htim_1, TIM_HandleTypeDef htim_2, float velocity);

#endif /* INC_STEPPER_H_ */
