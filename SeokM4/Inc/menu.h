/*
 * menu.h
 *
 *  Created on: 2020. 3. 12.
 *      Author: cctv1
 */

#include <stdio.h>
#ifdef	MENU_H_
	#ifndef _MENU_GLOBAL_
		#define _MENU_GLOBAL_
	#endif
#else
	#ifndef _MENU_GLOBAL_
		#define _MENU_GLOBAL_	extern
	#endif
#endif



_MENU_GLOBAL_ volatile int16_t		g_int16_menu_x,
									g_int16_menu_y;
