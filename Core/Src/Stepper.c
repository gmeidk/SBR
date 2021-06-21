/*
 * Stepper.c
 *
 *  Created on: May 14, 2021
 *      Authors: Quatela Alessandro, Roberto Giuseppe
 */


#include "main.h"
#include "MPU6050.h"
#include "Stepper.h"
#include <math.h>


#define RESOLUTION 1.8 		// °/step
#define RADIUS 0.045    	// m
#define FCLK 84000000       // Hz
#define MAX_CTRL_INT 1      // m/s
#define MAX_CTRL_ACT 1.8    // m/s
#define MAX_CTRL_ANG 45     // °
#define FREQ_CNG_PSC 0.14   // Hz
#define MAX_ACC 0.18		// 100 m/s^2 * Tc
#define MIN_ARR 5000
#define MAX_ARR 4294967290
#define ABS(x) ((x) > 0) ? (x) : -(x);

typedef struct {
	float Kp, Ki, Kd, Yi;
} Control_Param_t;
Control_Param_t Control_Param = { .Kp=4.4, .Ki=18.0, .Kd=0.2, .Yi=0.0 };
float velocity_old = 0;

// LQR PARAMETERS
double K[4] = {-0.293249385038703,	47.0534994121473,	-6.72510997203132,	9.30922001696825};
float x_pos = 0;

float PID_Control(Control_Data_t Control_Data, float dt) {

	/* Max control angle check */
	if (Control_Data.angleY < -MAX_CTRL_ANG || Control_Data.angleY > MAX_CTRL_ANG) {
		return 0;
	}

	/* Integrative component and anti wind-up filter */
	Control_Param.Yi += ( Control_Data.angleY * M_PI / 180) * dt;

	if (Control_Param.Yi * Control_Param.Ki > MAX_CTRL_INT) {
		Control_Param.Yi = MAX_CTRL_INT / Control_Param.Ki;
	}
	if (Control_Param.Yi * Control_Param.Ki < -MAX_CTRL_INT) {
		Control_Param.Yi = -MAX_CTRL_INT / Control_Param.Ki;
	}

	/* PID control action */
	float velocity = Control_Param.Kp * (Control_Data.angleY * M_PI / 180) + Control_Param.Ki * Control_Param.Yi + Control_Param.Kd * (Control_Data.Gy * M_PI / 180);


	return velocity;
}

void Speed_Actuation(TIM_HandleTypeDef htim_1, TIM_HandleTypeDef htim_2, float velocity) {

	/* Max velocity check */
	if (velocity >  MAX_CTRL_ACT) {
		velocity =  MAX_CTRL_ACT;
	}
	if (velocity < - MAX_CTRL_ACT) {
		velocity = - MAX_CTRL_ACT;
	}

	/* Max acceleration check */
	if ((velocity - velocity_old) > MAX_ACC) {
		velocity = velocity_old + MAX_ACC;
	}
	if ((velocity - velocity_old) < -MAX_ACC) {
		velocity = velocity_old - MAX_ACC;
	}

	velocity_old = velocity;

	/* PWM frequency computation */
	float freq = ABS(velocity / (RADIUS * RESOLUTION * M_PI / 180));


	/* Direction check */
	if (velocity < 0) {
		HAL_GPIO_WritePin(DIR_D10_GPIO_Port, DIR_D10_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(DIR_D10_GPIO_Port, DIR_D10_Pin, GPIO_PIN_SET);
	}

	/* PSC scale check */
	uint16_t PSC;
	if(freq <= FREQ_CNG_PSC) {
		PSC = 60000;
		__HAL_TIM_SET_PRESCALER(&htim_1, PSC - 1);
		__HAL_TIM_SET_PRESCALER(&htim_2, PSC - 1);

	} else {
		PSC = 11;
		__HAL_TIM_SET_PRESCALER(&htim_1, PSC - 1);
		__HAL_TIM_SET_PRESCALER(&htim_2, PSC - 1);
	}

	/* ARR computation */
	uint32_t ARR = (uint32_t) FCLK / (freq * PSC) - 1;

	if (ARR < MIN_ARR) {
		ARR = MIN_ARR;
	}

	if (ARR > MAX_ARR) {
		ARR = 0;
	}


	uint32_t CCR = round(ARR/2);

	/* PWM TIM register update */
	__HAL_TIM_SET_COMPARE(&htim_1,TIM_CHANNEL_1,CCR);
	__HAL_TIM_SET_COMPARE(&htim_2,TIM_CHANNEL_1,CCR);

	__HAL_TIM_SET_AUTORELOAD(&htim_1, ARR);
	__HAL_TIM_SET_AUTORELOAD(&htim_2, ARR);

	htim_1.Instance->EGR = TIM_EGR_UG;
	htim_2.Instance->EGR = TIM_EGR_UG;
}


// LQR CONTROL FUNCTION

float LQR_Control(Control_Data_t Control_Data, float dt) {

	/* Position computation */
	x_pos += velocity_old * dt;	// m

	/* LQR control action */
	float acceleration = x_pos * K[0] + (Control_Data.angleY * M_PI / 180) * K[1] + velocity_old * K[2] + (Control_Data.Gy * M_PI / 180) * K[3]; // x * K [m/s^2]

	/* Computing new velocity from LQR control action */
	float velocity = velocity_old + acceleration * dt;

	return velocity;
}
