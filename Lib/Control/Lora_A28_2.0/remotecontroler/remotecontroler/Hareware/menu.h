#ifndef __MENU_H_
#define __MENU_H_

#include "stm32f4xx_hal.h"

typedef void (*MenuAction)(void);

typedef struct
{
	const char *name;  //菜单显示文字
	MenuAction action;//执行函数
	
}MenuItem;

void Menu_init(void);
void Menu_Keyup(void);
void Menu_KeyDowm(void);
void Menu_KeyOK(void);
void Menu_Draw(void);

#endif

