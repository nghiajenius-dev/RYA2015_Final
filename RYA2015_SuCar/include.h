/**
 *	Raise your ARM 2015 sample code http://raiseyourarm.com/
 *	Author: Pay it forward club
 *	http://www.payitforward.edu.vn
 *  version 0.0.1
 */

/**
 * @file	include.h
 * @brief	Global include file
 */
#ifndef INCLUDE_H_
#define INCLUDE_H_

#define TARGET_IS_BLIZZARD_RB1

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"
#include "driverlib/qei.h"
#include "driverlib/fpu.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "define.h"
#include "Bluetooth/uartstdio.h"
#include "SystemConfig.h"
#include "Timer/Timer.h"
#include "IR/IR.h"
#include "STR/Uocluong.h"
#include "STR/STR_Indirect.h"
#include "STR/Control_initialize.h"
#include "Bluetooth/ustdlib.h"
#include "Bluetooth/Bluetooth.h"
#include "QEI/qei.h"
#include "Button/Button.h"
#include "STR/speed_control.h"
#include "WallFollow/PID.h"
#include "WallFollow/WallFollow.h"
#include "Buzzer/Buzzer.h"
#include "Battery/Battery.h"
#include "HostComm/hostComm.h"
#include "Eeprom/EEPROM.h"
#include "FloodFill/Maze.h"
#endif /* INCLUDE_H_ */
