/**
 *	Raise your ARM 2015 sample code http://raiseyourarm.com/
 *	Author: Pay it forward club
 *	http://www.payitforward.edu.vn
 *  version 0.0.1
 */

/**
 * @file	WallFollow.c
 * @brief	wall follow controller
 */

#ifndef WALLFOLLOW_WALLFOLLOW_H_
#define WALLFOLLOW_WALLFOLLOW_H_


typedef enum
{
	WALL_FOLLOW_NONE = 0,	//Don't use pid
	WALL_FOLLOW_LEFT,
	WALL_FOLLOW_RIGHT,
	WALL_FOLLOW_BOTH,
	WALL_FOLLOW_AUTO,
	IMAGE_PROCESSING
} WALL_FOLLOW_SELECT;

typedef enum{
	TURN_LEFT = 1,
	TURN_RIGHT,
	TURN_BACK,
	FORWARD,
	BACKWARD,
	NONE
}MOVE;

typedef enum{
	UP_DIR=0,
	RIGHT_DIR=1,
	DOWN_DIR=2,
	LEFT_DIR=3
}DIRECTION;
uint8_t GetAvailDir(void);
extern void wallFollow_init();
extern void pid_Wallfollow_process(void);
extern void pid_Wallfollow_set_follow(WALL_FOLLOW_SELECT follow_sel);
extern WALL_FOLLOW_SELECT Get_Pid_Wallfollow();
extern bool pid_wallfollow(int32_t delta_IR_left, int32_t delta_IR_right, int32_t averageSpeed,
		WALL_FOLLOW_SELECT wall_follow_select);
//bool move(int posLeft,int posRight,int velLeftMax, int velRightMax);
//extern static bool move(int deltaLeft,int deltaRight,int velLeftMax, int velRightMax)

extern void initPos();

#endif /* WALLFOLLOW_WALLFOLLOW_H_ */
