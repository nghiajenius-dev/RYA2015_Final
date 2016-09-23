#ifndef MAZE_H_
#define MAZE_H_


void ClearMaze(void);
void InitMaze(int8_t x0, int8_t y0, int8_t goal_x, int8_t goal_y);
void UpdateMap(uint32_t x, uint32_t y, DIRECTION Direction);
uint8_t GetMin(uint16_t Num0, uint16_t Num1, uint16_t Num2);
MOVE GetBestDir(uint32_t x, uint32_t y, DIRECTION Direction);
void UpdateMaze(int8_t x0, int8_t y0, int8_t goal_x, int8_t goal_y);
void PrintMaze(void);
void LockMap(void);

void InitHome(int8_t x0, int8_t y0, int8_t goal_x, int8_t goal_y);
void UpdateHome(uint32_t x, uint32_t y, DIRECTION Direction);

#endif /* MAZE_H_ */
