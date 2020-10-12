#include <stdio.h>
#include "main.h"
#include "menu.h"
#define MENU_H_

char menu_name [3][3][9]=
{
	"Hello>_<", "Offset", "motorchk",
	"reset   ", "QEPtest", "Sensor",
	"position", "ADCcheck", "IQhandle",
};

void Menu_Show(void)
{
	DELAY_US(50000);
	printf((char*)menu_name[g_int16_menu_y][g_int16_menu_x]);
	printf("\n\r");
}

void Menu_Selection(void)
{
int command=0;

	while(1)
	{

	Menu_Show();
	command=NULL;
	if(command=='d')
	{DELAY_US(100000);
		g_int16_menu_x++;
		if(g_int16_menu_x>2)
		{
			g_int16_menu_x=0;
			g_int16_menu_y++;
			if(g_int16_menu_y>2)
				g_int16_menu_y=0;
		}
	}



	if(command=='a')
		{DELAY_US(100000);
			g_int16_menu_x--;
			if(g_int16_menu_x<0)
			{
				g_int16_menu_x=2;
				g_int16_menu_y--;
				if(g_int16_menu_y<0)
					g_int16_menu_y=2;
			}
		}



	if(command=='s')
	{DELAY_US(100000);
		if((g_int16_menu_x==0)&&(g_int16_menu_y==1))
		{
			//rezero();
		}
		if((g_int16_menu_x==1)&&(g_int16_menu_y==1))
		{
			//qeptest();
		}
		if((g_int16_menu_x==1)&&(g_int16_menu_y==0))
		{
			//zeroset();
		}
		if((g_int16_menu_x==2)&&(g_int16_menu_y==0))
		{

			//motorTEST();
		}
		if((g_int16_menu_x==2)&&(g_int16_menu_y==1))
		{

			//Get_Sensor_MAX_min();
		}
		if((g_int16_menu_x==0)&&(g_int16_menu_y==2))
		{

				//Get_Sensor_Position();

		}
		if((g_int16_menu_x==1)&&(g_int16_menu_y==2))
		{


			//ade_answ
		}
		if((g_int16_menu_x==2)&&(g_int16_menu_y==2))
		{

			//tracking



		}

		g_int16_menu_x=0;
		g_int16_menu_y=0;

		}


	if(command=='w')
		{
		g_int16_menu_x=0;
		g_int16_menu_y=0;
		}
	}

	command = 0;
}
