/**
 *	Raise your ARM 2015 sample code http://raiseyourarm.com/
 *	Author: Pay it forward club
 *	http://www.payitforward.edu.vn
 *	version 0.0.1
 */

/**
 * @file	speed control.c
 * @brief	speed control
 */

#include "../include.h"
#include "speed_control.h"

//* Private function prototype ----------------------------------------------*/
static void Config_PWM(void);
static void SetPWM(uint32_t ulBaseAddr, uint32_t ulTimer, uint32_t ulFrequency, int32_t ucDutyCycle);
static void speed_update_setpoint(void);
static void speed_control_runtimeout(uint32_t ms);
static void speed_control_stoptimeout(void);
//* Private variables -------------------------------------------------------*/
static real_T Theta[4], Theta_[4] = {-1, 1, 1, 1};
static real_T Theta2[4], Theta2_[4] = {-1, 1, 1, 1};
static int32_t SetPoint[2] = {0, 0};
static int32_t RealSpeedSet[2] = {0, 0};
static float udk = 0;
static TIMER_ID speed_control_timID = INVALID_TIMER_ID;

void speed_control_init(void)
{
	Control_initialize();
	Config_PWM();
	SetPWM(PWM_MOTOR_LEFT, DEFAULT, 0);
	SetPWM(PWM_MOTOR_RIGHT, DEFAULT, 0);
}

/**
 * @brief Init battery sense
 * @note this function must call to calculate speed control
 */
void ProcessSpeedControl(void)
{
	int32_t Velocity[2];
//	SetPoint = 250;
	if (qei_getVelocity(0, &Velocity[0]) == true)
	{
		udk = STR_Indirect(Theta, RealSpeedSet[0], Velocity[0]);
		SetPWM(PWM_MOTOR_RIGHT, DEFAULT, udk);
		Uocluong(udk, Velocity[0], Theta, Theta_);
#ifdef _DEBUG_SPEED_
		bluetooth_print("Right: %d\r\n", Velocity[0]);
#endif
	}
	if (qei_getVelocity(1, &Velocity[1]) == true)
	{
		udk = STR_Indirect2(Theta2, RealSpeedSet[1], Velocity[1]);
		SetPWM(PWM_MOTOR_LEFT, DEFAULT, udk);
		Uocluong2(udk, Velocity[1], Theta2, Theta2_);
#ifdef _DEBUG_SPEED_
		bluetooth_print("Left: %d\r\n", Velocity[1]);
#endif
	}
}

static void Config_PWM(void)
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	ROM_GPIOPinConfigure(GPIO_PB6_T0CCP0);
	ROM_GPIOPinConfigure(GPIO_PB2_T3CCP0);
	ROM_GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_6);

	// Configure timer
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);

	ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM);
	ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, DEFAULT);
	ROM_TimerMatchSet(TIMER0_BASE, TIMER_A, DEFAULT); // PWM
	ROM_TimerEnable(TIMER0_BASE, TIMER_A);

	ROM_TimerConfigure(TIMER3_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM);
	ROM_TimerLoadSet(TIMER3_BASE, TIMER_A, DEFAULT);
	ROM_TimerMatchSet(TIMER3_BASE, TIMER_A, DEFAULT); // PWM
	ROM_TimerControlLevel(TIMER3_BASE, TIMER_A, true);
	ROM_TimerEnable(TIMER3_BASE, TIMER_A);

	ROM_SysCtlPeripheralEnable(DRV_ENABLE_LEFT_CHN_PERIPHERAL);
	ROM_SysCtlPeripheralEnable(DRV_ENABLE_RIGHT_CHN_PERIPHERAL);
	ROM_GPIOPinTypeGPIOOutput(DRV_ENABLE_LEFT_CHN_PORT, DRV_ENABLE_LEFT_CHN_PIN);
	ROM_GPIOPinTypeGPIOOutput(DRV_ENABLE_RIGHT_CHN_PORT, DRV_ENABLE_RIGHT_CHN_PIN);
	ROM_GPIOPinWrite(DRV_ENABLE_LEFT_CHN_PORT, DRV_ENABLE_LEFT_CHN_PIN, 0);
	ROM_GPIOPinWrite(DRV_ENABLE_RIGHT_CHN_PORT, DRV_ENABLE_RIGHT_CHN_PIN, 0);
}

/**
 * @brief Control Hbridge
 */
void speed_Enable_Hbridge(bool Enable)
{
	if (Enable)
	{
		ROM_GPIOPinWrite(DRV_ENABLE_LEFT_CHN_PORT, DRV_ENABLE_LEFT_CHN_PIN, DRV_ENABLE_LEFT_CHN_PIN);
		ROM_GPIOPinWrite(DRV_ENABLE_RIGHT_CHN_PORT, DRV_ENABLE_RIGHT_CHN_PIN, DRV_ENABLE_RIGHT_CHN_PIN);
	}
	else
	{
		ROM_GPIOPinWrite(DRV_ENABLE_LEFT_CHN_PORT, DRV_ENABLE_LEFT_CHN_PIN, 0);
		ROM_GPIOPinWrite(DRV_ENABLE_RIGHT_CHN_PORT, DRV_ENABLE_RIGHT_CHN_PIN, 0);
	}
}

void SetPWM(uint32_t ulBaseAddr, uint32_t ulTimer, uint32_t ulFrequency, int32_t ucDutyCycle)
{
	uint32_t ulPeriod;
	ulPeriod = ROM_SysCtlClockGet() / ulFrequency;
	ROM_TimerLoadSet(ulBaseAddr, ulTimer, ulPeriod);
	if (ucDutyCycle > 90)
		ucDutyCycle = 90;
	else if (ucDutyCycle < -90)
		ucDutyCycle = -90;
	ROM_TimerMatchSet(ulBaseAddr, ulTimer, (100 + ucDutyCycle) * ulPeriod / 200 - 1);
}

/**
 * @brief Control speed
 * @param select motor select
 * @param speed motor speed (encoder pulse / 20ms)
 */
void speed_set(MOTOR_SELECT Select, int32_t speed)
{
//	speed_Enable_Hbridge(true);
	if (Select == MOTOR_RIGHT)
	{
		if (SetPoint[0] != speed)
		{
			SetPoint[0] = speed;
			speed_control_runtimeout(20);
		}
	}
	else if (Select == MOTOR_LEFT)
	{
		if (SetPoint[1] != speed)
		{
			SetPoint[1] = speed;
			speed_control_runtimeout(20);
		}
	}
	if (SetPoint[0]==0 && SetPoint[1]==0 )
	{
		speed_Enable_Hbridge(false);
	}
}


static void speed_update_setpoint(void)
{
	int i;
	speed_control_timID = INVALID_TIMER_ID;

	for (i = 0; i < 2; i++)
	{
		if (RealSpeedSet[i] + 20 < SetPoint[i])
			RealSpeedSet[i] += 20;
		else if (RealSpeedSet[i] > SetPoint[i] + 20)
			RealSpeedSet[i] -= 20;
		else
			RealSpeedSet[i] = SetPoint[i];
	}

	speed_control_runtimeout(20);
}

static void speed_control_runtimeout(uint32_t ms)
{
	speed_control_stoptimeout();
	speed_control_timID = TIMER_RegisterEvent(&speed_update_setpoint, ms);
}

static void speed_control_stoptimeout(void)
{
	if (speed_control_timID != INVALID_TIMER_ID)
		TIMER_UnregisterEvent(speed_control_timID);
	speed_control_timID = INVALID_TIMER_ID;
}

void speed_SetMotorModel(MOTOR_SELECT select, real_T Theta[4])
{
	int i;
	if (select == MOTOR_RIGHT)
	{
		for (i = 0; i < 4; i++)
		{
			Theta_[i] = Theta[i];
		}
	}
	else if (select == MOTOR_LEFT)
	{
		for (i = 0; i < 4; i++)
		{
			Theta2_[i] = Theta[i];
		}
	}
}

void speed_GetMotorModel(MOTOR_SELECT select, real_T Theta[4])
{
	int i;
	if (select == MOTOR_RIGHT)
	{
		for (i = 0; i < 4; i++)
		{
			Theta[i] = Theta_[i];
		}
	}
	else if (select == MOTOR_LEFT)
	{
		for (i = 0; i < 4; i++)
		{
			Theta[i] = Theta2_[i];
		}
	}
}
