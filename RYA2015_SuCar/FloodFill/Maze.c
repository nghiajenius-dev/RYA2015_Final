#include "../include.h"

#define start_X	3
#define start_Y	0

#define goal_X	3
#define goal_Y	13

extern MOVE eMove;
extern DIRECTION currentDir;
extern int32_t robotX,robotY;
extern uint8_t AvailDirection;

uint16_t Array[3];
static uint16_t Maze[7][14];
static bool callibhome = false;

void ClearMaze(void)
{
	uint8_t i, j;
	for (i = 0; i < 7; i++)
	{
		for (j = 0; j < 14; j++)
		{
			Maze[i][j] = 0;
		}
	}
}

void InitMaze(int8_t x0, int8_t y0, int8_t goal_x, int8_t goal_y)
{



	uint8_t i, j;
	uint16_t count = 1;
	//x = x0;
	//y = y0;
	for (i = 0; i < 7; i++)
	{
		for (j = 0; j < 14; j++)
		{
			Maze[i][j] |= 0x00ff;
			Maze[i][j] &= ~HAS_GONE;
		}
	}
	Maze[goal_x][goal_y] = 0;
	while (count)
	{
		count = 0;
		for (i = 0; i < 7; i++)
		{
			for (j = 0; j < 14; j++)
			{
				if ((i > 0) && ((Maze[i][j] & 0xff) < (Maze[i-1][j] & 0xff)) && ((Maze[i - 1][j] & 0xff) == 0x00ff))
				{
					if ((!(Maze[i][j] & IS_WEST_WALL)))
					{
						Maze[i-1][j] &= 0xff00;
						Maze[i-1][j] |= ((Maze[i][j] + 1) & 0xff);
						count++;
					}
				}
				if ((i < 6) && ((Maze[i][j] & 0xff) < (Maze[i+1][j] & 0xff)) && ((Maze[i + 1][j] & 0xff) == 0x00ff))
				{
					if ((!(Maze[i][j] & IS_EAST_WALL)))
					{
						Maze[i+1][j] &= 0xff00;
						Maze[i+1][j] |= ((Maze[i][j] + 1) & 0xff);
						count++;
					}
				}
				if ((j > 0) && ((Maze[i][j] & 0xff) < (Maze[i][j - 1] & 0xff)) && ((Maze[i][j - 1] & 0xff) == 0x00ff))
				{
					if ((!(Maze[i][j] & IS_SOUTH_WALL)))
					{
						Maze[i][j - 1] &= 0xff00;
						Maze[i][j - 1] |= ((Maze[i][j] + 1) & 0xff);
						count++;
					}
				}
				if ((j < 12) && ((Maze[i][j] & 0xff) < (Maze[i][j + 1] & 0xff)) && ((Maze[i][j + 1] & 0xff) == 0x00ff))
				{
					if ((!(Maze[i][j] & IS_NORTH_WALL)))
					{
						Maze[i][j + 1] &= 0xff00;
						Maze[i][j + 1] |= ((Maze[i][j] + 1) & 0xff);
						count++;
					}
				}
			}
		}
	}

	Maze[goal_x][goal_y] = 0;
}

void UpdateMap(uint32_t x, uint32_t y, DIRECTION Direction)
{

	AvailDirection = GetAvailDir();
	Maze[x][y] |= HAS_GONE;
	switch (Direction)
	{
		case UP_DIR:
			if (!(AvailDirection & AVAIL_LEFT))
			{
				Maze[x][y] |= IS_WEST_WALL;
				if (x != 0)
				{
					Maze[x-1][y] |= IS_EAST_WALL;
				}
			}
			if (!(AvailDirection & AVAIL_RIGHT))
			{
				Maze[x][y] |= IS_EAST_WALL;
				if (x != 6)
				{
					Maze[x+1][y] |= IS_WEST_WALL;
				}
			}
			if ((AvailDirection & (AVAIL_FR | AVAIL_FL)) != (AVAIL_FR | AVAIL_FL))
			{
				Maze[x][y] |= IS_NORTH_WALL;
				if (y != 12)
				{
					Maze[x][y+1] |= IS_SOUTH_WALL;
				}
			}
			break;
		case RIGHT_DIR:
			if (!(AvailDirection & AVAIL_LEFT))
			{
				Maze[x][y] |= IS_NORTH_WALL;
				if (y != 12)
				{
					Maze[x][y+1] |= IS_SOUTH_WALL;
				}
			}
			if (!(AvailDirection & AVAIL_RIGHT))
			{
				Maze[x][y] |= IS_SOUTH_WALL;
				if (y != 0)
				{
					Maze[x][y-1] |= IS_NORTH_WALL;
				}
			}
			if ((AvailDirection & (AVAIL_FR | AVAIL_FL)) != (AVAIL_FR | AVAIL_FL))
			{
				Maze[x][y] |= IS_EAST_WALL;
				if (x != 12)
				{
					Maze[x+1][y] |= IS_WEST_WALL;
				}
			}
			break;
		case DOWN_DIR:
			if (!(AvailDirection & AVAIL_LEFT))
			{
				Maze[x][y] |= IS_EAST_WALL;
				if (x != 6)
				{
					Maze[x+1][y] |= IS_WEST_WALL;
				}
			}
			if (!(AvailDirection & AVAIL_RIGHT))
			{
				Maze[x][y] |= IS_WEST_WALL;
				if (x != 0)
				{
					Maze[x-1][y] |= IS_EAST_WALL;
				}
			}
			if ((AvailDirection & (AVAIL_FR | AVAIL_FL)) != (AVAIL_FR | AVAIL_FL))
			{
				Maze[x][y] |= IS_SOUTH_WALL;
				if (y != 0)
				{
					Maze[x][y-1] |= IS_NORTH_WALL;
				}
			}
			break;
		case LEFT_DIR:
			if (!(AvailDirection & AVAIL_LEFT))
			{
				Maze[x][y] |= IS_SOUTH_WALL;
				if (y != 0)
				{
					Maze[x][y-1] |= IS_NORTH_WALL;
				}
			}
			if (!(AvailDirection & AVAIL_RIGHT))
			{
				Maze[x][y] |= IS_NORTH_WALL;
				if (y != 12)
				{
					Maze[x][y+1] |= IS_SOUTH_WALL;
				}
			}
			if ((AvailDirection & (AVAIL_FR | AVAIL_FL)) != (AVAIL_FR | AVAIL_FL))
			{
				Maze[x][y] |= IS_WEST_WALL;
				if (x != 0)
				{
					Maze[x-1][y] |= IS_EAST_WALL;
				}
			}
			break;
	}
}

uint8_t GetMin(uint16_t Num0, uint16_t Num1, uint16_t Num2)
{
		if (Num1 > Num0)
		{
			if (Num0 > Num2)
				return (2);
			else
				return (0);
		}
		else if (Num1 > Num2)
		{
			return(2);
		}
	return (1);
}

MOVE GetBestDir(uint32_t x, uint32_t y, DIRECTION Direction)
{

	AvailDirection = GetAvailDir();
	if ((AvailDirection & (AVAIL_FR | AVAIL_FL)) == (AVAIL_FR | AVAIL_FL))
	{
		switch (Direction)
		{
			case UP_DIR:
				Array[1] = Maze[x][y+1] & (0x00ff | HAS_GONE);
				break;
			case RIGHT_DIR:
				Array[1] = Maze[x+1][y] & (0x00ff | HAS_GONE);
				break;
			case DOWN_DIR:
				Array[1] = Maze[x][y-1] & (0x00ff | HAS_GONE);
				break;
			case LEFT_DIR:
				Array[1] = Maze[x-1][y] & (0x00ff | HAS_GONE);
				break;
		}
	}
	else
	{
		Array[1] = 0xffff;
	}
	if (AvailDirection & AVAIL_RIGHT)
	{
		switch (Direction)
		{
			case UP_DIR:
				Array[2] = Maze[x+1][y] & (0x00ff | HAS_GONE);
				break;
			case RIGHT_DIR:
				Array[2] = Maze[x][y-1] & (0x00ff | HAS_GONE);
				break;
			case DOWN_DIR:
				Array[2] = Maze[x-1][y] & (0x00ff | HAS_GONE);
				break;
			case LEFT_DIR:
				Array[2] = Maze[x][y+1] & (0x00ff | HAS_GONE);
				break;
		}
	}
	else
	{
		Array[2] = 0xffff;
	}
	if (AvailDirection & AVAIL_LEFT)
	{
		switch (Direction)
		{
			case UP_DIR:
				Array[0] = Maze[x-1][y] & (0x00ff | HAS_GONE);
				break;
			case RIGHT_DIR:
				Array[0] = Maze[x][y+1] & (0x00ff | HAS_GONE);
				break;
			case DOWN_DIR:
				Array[0] = Maze[x+1][y] & (0x00ff | HAS_GONE);
				break;
			case LEFT_DIR:
				Array[0] = Maze[x][y-1] & (0x00ff | HAS_GONE);
				break;
		}
	}
	else
	{
		Array[0] = 0xffff;
	}
	AvailDirection = GetAvailDir();
	if (AvailDirection == 0)
		return (TURN_BACK);
	if (AvailDirection == AVAIL_LEFT)
		return (TURN_LEFT);
	if (AvailDirection == AVAIL_RIGHT)
		return (TURN_RIGHT);

	switch(GetMin(Array[0], Array[1], Array[2]))
	{
		case 0:
			return (TURN_LEFT);
		case 1:
			return (FORWARD);
		case 2:
			return (TURN_RIGHT);
	}
	return (FORWARD);


}


void UpdateMaze(int8_t x0, int8_t y0, int8_t goal_x, int8_t goal_y)
{
	uint8_t i, j;
	uint16_t count = 1;
	Maze[goal_x][goal_y] = 0;
		while (count)
		{
			count = 0;
			for (i = 0; i < 7; i++)
			{
				for (j = 0; j < 14; j++)
				{
					if ((i > 0) && ((Maze[i][j] & 0xff) < (Maze[i-1][j] & 0xff)) && ((Maze[i - 1][j] & 0xff) == 0x00ff))
					{
						if ((!(Maze[i][j] & IS_WEST_WALL)))
						{
							Maze[i-1][j] &= 0xff00;
							Maze[i-1][j] |= ((Maze[i][j] + 1) & 0xff);
							count++;
						}
					}
					if ((i < 6) && ((Maze[i][j] & 0xff) < (Maze[i+1][j] & 0xff)) && ((Maze[i + 1][j] & 0xff) == 0x00ff))
					{
						if ((!(Maze[i][j] & IS_EAST_WALL)))
						{
							Maze[i+1][j] &= 0xff00;
							Maze[i+1][j] |= ((Maze[i][j] + 1) & 0xff);
							count++;
						}
					}
					if ((j > 0) && ((Maze[i][j] & 0xff) < (Maze[i][j - 1] & 0xff)) && ((Maze[i][j - 1] & 0xff) == 0x00ff))
					{
						if ((!(Maze[i][j] & IS_SOUTH_WALL)))
						{
							Maze[i][j - 1] &= 0xff00;
							Maze[i][j - 1] |= ((Maze[i][j] + 1) & 0xff);
							count++;
						}
					}
					if ((j < 12) && ((Maze[i][j] & 0xff) < (Maze[i][j + 1] & 0xff)) && ((Maze[i][j + 1] & 0xff) == 0x00ff))
					{
						if ((!(Maze[i][j] & IS_NORTH_WALL)))
						{
							Maze[i][j + 1] &= 0xff00;
							Maze[i][j + 1] |= ((Maze[i][j] + 1) & 0xff);
							count++;
						}
					}
				}
			}
		}
		if ((!(Maze[0][1] & IS_SOUTH_WALL)))
		{
			Maze[0][0] = Maze[0][1] + 1;
		}
		else if ((!(Maze[1][0] & IS_WEST_WALL)))
		{
			Maze[0][0] = Maze[1][0] + 1;
		}

		if ((!(Maze[0][11] & IS_NORTH_WALL)))
		{
			Maze[0][12] = Maze[0][11] + 1;
		}
		else if ((!(Maze[1][12] & IS_WEST_WALL)))
		{
			Maze[0][12] = Maze[1][12] + 1;
		}

		if ((!(Maze[5][0] & IS_EAST_WALL)))
		{
			Maze[6][0] = Maze[5][0] + 1;
		}
		else if ((!(Maze[6][1] & IS_SOUTH_WALL)))
		{
			Maze[6][0] = Maze[6][1] + 1;
		}

		if ((!(Maze[5][12] & IS_EAST_WALL)))
		{
			Maze[6][12] = Maze[5][12] + 1;
		}
		else if ((!(Maze[6][11] & IS_NORTH_WALL)))
		{
			Maze[6][12] = Maze[6][11] + 1;
		}
		Maze[goal_x][goal_y] = 0;

}

void PrintMaze(void)
{
	uint8_t i, j;
	bluetooth_print("\r\n");

	for (i = 0; i < 7; i++)
	{
		for (j = 0; j < 14; j++)
		{
			bluetooth_print("%d    ",Maze[i][j] & 0xfff);
		}
		bluetooth_print("\r\n");
	}
}

void LockMap(void){
	uint16_t i,j;
	for(i=0;i<7;i++){
		for(j=0;j<14;j++){
			if(Maze[i][j]& HAS_GONE==0){
				Maze[i][j]=0xffff;
			}
		}
	}
}

void InitHome(int8_t x0, int8_t y0, int8_t goal_x, int8_t goal_y)
{
//
	uint8_t i, j;
	uint16_t count = 1;
//	//x = x0;
//	//y = y0;
	for (i = 0; i < 7; i++)
	{
		for (j = 0; j < 14; j++)
		{
			if(callibhome == false)
			{
			if((Maze[i][j] & HAS_GONE) != HAS_GONE ){
				callibhome = true;
				Maze[i][j] |= HAS_GONE2;
			}
			}
			Maze[i][j] |= 0x00ff;
			Maze[i][j] &= ~HAS_GONE;


		}
	}

	Maze[goal_x][goal_y] = 0;
	while (count)
	{
		count = 0;
		for (i = 0; i < 7; i++)
		{
			for (j = 0; j < 14; j++)
			{

				if ((i > 0) && ((Maze[i][j] & 0xff) < (Maze[i-1][j] & 0xff)) && ((Maze[i - 1][j] & 0xff) == 0x00ff))
				{
					if ((!(Maze[i][j] & IS_WEST_WALL)))
					{
						Maze[i-1][j] &= 0xff00;
						Maze[i-1][j] |= ((Maze[i][j] + 1) & 0xff);
						count++;
					}
				}
				if ((i < 6) && ((Maze[i][j] & 0xff) < (Maze[i+1][j] & 0xff)) && ((Maze[i + 1][j] & 0xff) == 0x00ff))
				{
					if ((!(Maze[i][j] & IS_EAST_WALL)))
					{
						Maze[i+1][j] &= 0xff00;
						Maze[i+1][j] |= ((Maze[i][j] + 1) & 0xff);
						count++;
					}
				}
				if ((j > 0) && ((Maze[i][j] & 0xff) < (Maze[i][j - 1] & 0xff)) && ((Maze[i][j - 1] & 0xff) == 0x00ff))
				{
					if ((!(Maze[i][j] & IS_SOUTH_WALL)))
					{
						Maze[i][j - 1] &= 0xff00;
						Maze[i][j - 1] |= ((Maze[i][j] + 1) & 0xff);
						count++;
					}
				}
				if ((j < 12) && ((Maze[i][j] & 0xff) < (Maze[i][j + 1] & 0xff)) && ((Maze[i][j + 1] & 0xff) == 0x00ff))
				{
					if ((!(Maze[i][j] & IS_NORTH_WALL)))
					{
						Maze[i][j + 1] &= 0xff00;
						Maze[i][j + 1] |= ((Maze[i][j] + 1) & 0xff);
						count++;
					}
				}
			}
		}
	}
	if ((!(Maze[0][1] & IS_SOUTH_WALL)))
	{
		Maze[0][0] = Maze[0][1] + 1;
	}
	else if ((!(Maze[1][0] & IS_WEST_WALL)))
	{
		Maze[0][0] = Maze[1][0] + 1;
	}

	if ((!(Maze[0][11] & IS_NORTH_WALL)))
	{
		Maze[0][12] = Maze[0][11] + 1;
	}
	else if ((!(Maze[1][12] & IS_WEST_WALL)))
	{
		Maze[0][12] = Maze[1][12] + 1;
	}

	if ((!(Maze[5][0] & IS_EAST_WALL)))
	{
		Maze[6][0] = Maze[5][0] + 1;
	}
	else if ((!(Maze[6][1] & IS_SOUTH_WALL)))
	{
		Maze[6][0] = Maze[6][1] + 1;
	}

	if ((!(Maze[5][12] & IS_EAST_WALL)))
	{
		Maze[6][12] = Maze[5][12] + 1;
	}
	else if ((!(Maze[6][11] & IS_NORTH_WALL)))
	{
		Maze[6][12] = Maze[6][11] + 1;
	}
	Maze[goal_x][goal_y] = 0;

	for (i = 0; i < 7; i++)
			{
				for (j = 0; j < 14; j++)
				{
					if(Maze[i][j] & HAS_GONE2 == HAS_GONE2){
						Maze[i][j]|=0xff;
					}


				}
			}
	Maze[goal_x][goal_y] = 0;

}

void UpdateHome(uint32_t x, uint32_t y, DIRECTION Direction)
{

	AvailDirection = GetAvailDir();
	Maze[x][y] |= HAS_GONE;
	switch (Direction)
	{
		case UP_DIR:
			if (!(AvailDirection & AVAIL_LEFT))
			{
				Maze[x][y] |= IS_WEST_WALL;
				if (x != 0)
				{
					Maze[x-1][y] |= IS_EAST_WALL;
				}
			}
			if (!(AvailDirection & AVAIL_RIGHT))
			{
				Maze[x][y] |= IS_EAST_WALL;
				if (x != 6)
				{
					Maze[x+1][y] |= IS_WEST_WALL;
				}
			}
			if ((AvailDirection & (AVAIL_FR | AVAIL_FL)) != (AVAIL_FR | AVAIL_FL))
			{
				Maze[x][y] |= IS_NORTH_WALL;
				if (y != 12)
				{
					Maze[x][y+1] |= IS_SOUTH_WALL;
				}
			}
			break;
		case RIGHT_DIR:
			if (!(AvailDirection & AVAIL_LEFT))
			{
				Maze[x][y] |= IS_NORTH_WALL;
				if (y != 12)
				{
					Maze[x][y+1] |= IS_SOUTH_WALL;
				}
			}
			if (!(AvailDirection & AVAIL_RIGHT))
			{
				Maze[x][y] |= IS_SOUTH_WALL;
				if (y != 0)
				{
					Maze[x][y-1] |= IS_NORTH_WALL;
				}
			}
			if ((AvailDirection & (AVAIL_FR | AVAIL_FL)) != (AVAIL_FR | AVAIL_FL))
			{
				Maze[x][y] |= IS_EAST_WALL;
				if (x != 12)
				{
					Maze[x+1][y] |= IS_WEST_WALL;
				}
			}
			break;
		case DOWN_DIR:
			if (!(AvailDirection & AVAIL_LEFT))
			{
				Maze[x][y] |= IS_EAST_WALL;
				if (x != 6)
				{
					Maze[x+1][y] |= IS_WEST_WALL;
				}
			}
			if (!(AvailDirection & AVAIL_RIGHT))
			{
				Maze[x][y] |= IS_WEST_WALL;
				if (x != 0)
				{
					Maze[x-1][y] |= IS_EAST_WALL;
				}
			}
			if ((AvailDirection & (AVAIL_FR | AVAIL_FL)) != (AVAIL_FR | AVAIL_FL))
			{
				Maze[x][y] |= IS_SOUTH_WALL;
				if (y != 0)
				{
					Maze[x][y-1] |= IS_NORTH_WALL;
				}
			}
			break;
		case LEFT_DIR:
			if (!(AvailDirection & AVAIL_LEFT))
			{
				Maze[x][y] |= IS_SOUTH_WALL;
				if (y != 0)
				{
					Maze[x][y-1] |= IS_NORTH_WALL;
				}
			}
			if (!(AvailDirection & AVAIL_RIGHT))
			{
				Maze[x][y] |= IS_NORTH_WALL;
				if (y != 12)
				{
					Maze[x][y+1] |= IS_SOUTH_WALL;
				}
			}
			if ((AvailDirection & (AVAIL_FR | AVAIL_FL)) != (AVAIL_FR | AVAIL_FL))
			{
				Maze[x][y] |= IS_WEST_WALL;
				if (x != 0)
				{
					Maze[x-1][y] |= IS_EAST_WALL;
				}
			}
			break;
	}
}
