#include "menu_left.h"
#include "oled_left.h"
#include "string.h"
#include "key.h"
#include "remote.h"
#include "stdio.h"
#include "remote_tx.h"


#define MENU_FONT_LEFT 				    (&font16x16)
#define SCREEN_WIDTH_LEFT 				128
#define SCREEN_HEIGHT_LEFT 				64

#define MENU_LINE_HEIGHT_LEFT  		(MENU_FONT_LEFT->h)
#define MENU_MAX_LINES_LEFT			(SCREEN_HEIGHT_LEFT/MENU_LINE_HEIGHT_LEFT)

extern  int16_t joy_x;
extern  int16_t joy_y;

extern  int16_t joy_x_left;
extern  int16_t joy_y_left;

//二级菜单的执行函数

static void Menu_mode1_left(void);
static void Menu_mode2_left(void);
static void Menu_mode3_left(void);
static void Menu_mode4_left(void);
static void Menu_mode5_left(void);

static void Menu_updateoffset_left(void);
//一级菜单列表
static const MenuItem_left s_mainMenu_left[] =
{
	{"摇杆" , Menu_mode1_left },
	{"取块" , Menu_mode2_left },
	{"抬升" , Menu_mode3_left },
	{"模式" , Menu_mode4_left },
	{"模式" , Menu_mode5_left },	
};

#define MENU_ITEM_COUNT_LEFT (sizeof(s_mainMenu_left)/sizeof(s_mainMenu_left[0]))
	
static uint8_t s_menuselected_left = 0;
static uint8_t s_menuoffset_left   = 0;

void Menu_init_left(void)
{
	s_menuselected_left = 0;
}

static void Menu_updateoffset_left(void)
{
	if(MENU_ITEM_COUNT_LEFT <= MENU_MAX_LINES_LEFT)
	{
		s_menuoffset_left = 0;
		return;
	}
	if(s_menuselected_left < s_menuoffset_left)
	{
		s_menuoffset_left = s_menuselected_left;
	}
	if(s_menuselected_left >= s_menuoffset_left + MENU_MAX_LINES_LEFT)
	{
		s_menuoffset_left = s_menuselected_left - MENU_MAX_LINES_LEFT + 1;
	}
}

void Menu_Keyup_left(void)
{
	if(s_menuselected_left == 0)
	{
		s_menuselected_left = MENU_ITEM_COUNT_LEFT - 1;
	}
	else
	{
		s_menuselected_left--;
	}
	Menu_updateoffset_left();
}
void Menu_KeyDowm_left(void)
{
	s_menuselected_left++;
	if(s_menuselected_left >= MENU_ITEM_COUNT_LEFT)
	{
		s_menuselected_left = 0;
	}
	Menu_updateoffset_left();
}
void Menu_KeyOK_left(void)
{
	if(s_mainMenu_left[s_menuselected_left].action)
	{
		s_mainMenu_left[s_menuselected_left].action();
	}
	
}

void Menu_Draw_left(void)
{
    OLED_NewFrame_left();

    for (uint8_t line = 0; line < MENU_MAX_LINES_LEFT; line++) {
        uint8_t index = s_menuoffset_left + line;   
        if (index >= MENU_ITEM_COUNT_LEFT) {
            break;                            
        }

        uint8_t y = line * MENU_LINE_HEIGHT_LEFT;
        const char *text = s_mainMenu_left[index].name;

        if (index == s_menuselected_left) {

            OLED_DrawFilledRectangle_left(0, y, SCREEN_WIDTH_LEFT, MENU_FONT_LEFT->h, OLED_COLOR_NORMAL_left);
            OLED_PrintString_left(0, y, (char *)text, MENU_FONT_LEFT, OLED_COLOR_REVERSED_left);
        } else {

            OLED_PrintString_left(0, y, (char *)text, MENU_FONT_LEFT, OLED_COLOR_NORMAL_left);
        }
    }

    OLED_ShowFrame_left();
}


static void Menu_mode1_left(void)
{
//	uint32_t last_time = 0;
	while(1)
	{
		
//	if(HAL_GetTick() - last_time >= 10)
//	{
//		last_time = HAL_GetTick();
//		joy_send(joy_x , joy_y);
//	}
	
	char buf_x[16];
	sprintf(buf_x , "%d" , joy_x_left);
	char buf_y[16];
	sprintf(buf_y , "%d" , joy_y_left);
	OLED_NewFrame_left();
	OLED_DrawFilledRectangle_left(16,0,MENU_FONT_LEFT->w*6 ,MENU_FONT_LEFT->h , OLED_COLOR_REVERSED_left);
	OLED_DrawFilledRectangle_left(16,16,MENU_FONT_LEFT->w*6 ,MENU_FONT_LEFT->h , OLED_COLOR_REVERSED_left);
	OLED_PrintString_left(0,0,"x",MENU_FONT_LEFT ,OLED_COLOR_NORMAL_left);
	OLED_PrintString_left(16,0,buf_x , MENU_FONT_LEFT , OLED_COLOR_NORMAL_left);
	OLED_PrintString_left(0,16,"y",MENU_FONT_LEFT ,OLED_COLOR_NORMAL_left);
	OLED_PrintString_left(16,16,buf_y , MENU_FONT_LEFT , OLED_COLOR_NORMAL_left);
	OLED_ShowFrame_left();
	if (Key_WasPressed(KEY_8)) break;
	}

}
static void Menu_mode2_left(void)
{
	while(1)
	{
	OLED_NewFrame_left();
	OLED_PrintString_left(0,0,"模式",MENU_FONT_LEFT ,OLED_COLOR_NORMAL_left);
	OLED_ShowFrame_left();
	if (Key_WasPressed(KEY_8)) break;
	}

}
static void Menu_mode3_left(void)
{
	while(1)
	{
	OLED_NewFrame_left();
	OLED_PrintString_left(0,0,"模式",MENU_FONT_LEFT ,OLED_COLOR_NORMAL_left);
	OLED_ShowFrame_left();
	if (Key_WasPressed(KEY_8)) break;
	}
}
static void Menu_mode4_left(void)
{
	while(1)
	{
	OLED_NewFrame_left();
	OLED_PrintString_left(0,0,"模式",MENU_FONT_LEFT ,OLED_COLOR_NORMAL_left);
	OLED_ShowFrame_left();
	if (Key_WasPressed(KEY_8)) break;
	}
}
static void Menu_mode5_left(void)
{
	while(1)
	{
	OLED_NewFrame_left();
	OLED_PrintString_left(0,0,"模式",MENU_FONT_LEFT ,OLED_COLOR_NORMAL_left);
	OLED_ShowFrame_left();
	if (Key_WasPressed(KEY_8)) break;
	}
}





