/**
 *	Raise your ARM 2015 sample code http://raiseyourarm.com/
 *	Author: Pay it forward club
 *	http://www.payitforward.edu.vn
 *  version 0.0.1
 */

/**
 * @file	define.h
 * @brief	global defination
 */

#ifndef DEFINE_H_
#define DEFINE_H_

//*****************************************************************************
//
// Type of Update Map
//
//*****************************************************************************
#define IS_NORTH_WALL		0x8000
#define IS_SOUTH_WALL		0x4000
#define IS_WEST_WALL		0x2000
#define IS_EAST_WALL		0x1000
#define HAS_GONE			0x0800
#define HAS_GONE2			0x0400
//*****************************************************************************
//
// Type of Available direction
//
//*****************************************************************************
#define AVAIL_LEFT		0x01
#define AVAIL_FL		0x02
#define AVAIL_FR		0x04
#define AVAIL_RIGHT		0x08
#define NOT_AVAIL		0xFE
#define Uht			(double)12.0

#define DEFAULT		20000	//H-Bridge Freq (Hz)

#endif /* DEFINE_H_ */
