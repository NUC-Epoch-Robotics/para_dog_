#ifndef __MENU_LEFT_H_
#define __MENU_LEFT_H_

#include "main.h"

typedef void (*MenuAction_left)(void);

typedef struct
{
	const char *name;  //菜单显示文字
	MenuAction_left action;//执行函数
	
}MenuItem_left;

void Menu_init_left(void);
void Menu_Keyup_left(void);
void Menu_KeyDowm_left(void);
void Menu_KeyOK_left(void);
void Menu_Draw_left(void);


#endif

