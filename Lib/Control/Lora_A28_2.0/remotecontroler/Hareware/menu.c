#include "menu.h"
#include "oled.h"
#include "string.h"
#include "key.h"
#include "remote.h"
#include "stdio.h"
#include "remote_tx.h"


#define MENU_FONT 				(&font16x16)
#define SCREEN_WIDTH 			128
#define SCREEN_HEIGHT 			64

#define MENU_LINE_HEIGHT  		(MENU_FONT->h)
#define MENU_MAX_LINES			(SCREEN_HEIGHT/MENU_LINE_HEIGHT)

extern  int16_t joy_x;
extern  int16_t joy_y;

extern int16_t joy_x_left;

//二级菜单的执行函数

static void Menu_mode1(void);
static void Menu_mode2(void);
static void Menu_mode3(void);
static void Menu_mode4(void);
static void Menu_mode5(void);

static void Menu_updateoffset(void);
//一级菜单列表
static const MenuItem s_mainMenu[] =
{
	{"平地模式" , Menu_mode1 },
	{"台阶模式" , Menu_mode2 },
	{"自旋模式" , Menu_mode3 },
	{"取块模式" , Menu_mode4 },
	{"放块模式" , Menu_mode5 },	
};

#define MENU_ITEM_COUNT (sizeof(s_mainMenu)/sizeof(s_mainMenu[0]))
	
static uint8_t s_menuselected = 0;
static uint8_t s_menuoffset   = 0;

void Menu_init(void)
{
	s_menuselected = 0;
}

static void Menu_updateoffset(void)
{
	if(MENU_ITEM_COUNT <= MENU_MAX_LINES)
	{
		s_menuoffset = 0;
		return;
	}
	if(s_menuselected < s_menuoffset)
	{
		s_menuoffset = s_menuselected;
	}
	if(s_menuselected >= s_menuoffset + MENU_MAX_LINES)
	{
		s_menuoffset = s_menuselected - MENU_MAX_LINES + 1;
	}
}

void Menu_Keyup(void)
{
	if(s_menuselected == 0)
	{
		s_menuselected = MENU_ITEM_COUNT - 1;
	}
	else
	{
		s_menuselected--;
	}
	Menu_updateoffset();
}
void Menu_KeyDowm(void)
{
	s_menuselected++;
	if(s_menuselected >= MENU_ITEM_COUNT)
	{
		s_menuselected = 0;
	}
	Menu_updateoffset();
}
void Menu_KeyOK(void)
{
	if(s_mainMenu[s_menuselected].action)
	{
		s_mainMenu[s_menuselected].action();
	}
	
}

void Menu_Draw(void)
{
    OLED_NewFrame();

    for (uint8_t line = 0; line < MENU_MAX_LINES; line++) {
        uint8_t index = s_menuoffset + line;   
        if (index >= MENU_ITEM_COUNT) {
            break;                            
        }

        uint8_t y = line * MENU_LINE_HEIGHT;
        const char *text = s_mainMenu[index].name;

        if (index == s_menuselected) {

            OLED_DrawFilledRectangle(0, y, SCREEN_WIDTH, MENU_FONT->h, OLED_COLOR_NORMAL);
            OLED_PrintString(0, y, (char *)text, MENU_FONT, OLED_COLOR_REVERSED);
        } else {

            OLED_PrintString(0, y, (char *)text, MENU_FONT, OLED_COLOR_NORMAL);
        }
    }

    OLED_ShowFrame();
}


static void Menu_mode1(void)
{
	uint32_t last_time = 0;
	while(1)
	{
		
	if(HAL_GetTick() - last_time >= 200)
	{
		last_time = HAL_GetTick();
		joy_send(joy_x , joy_y ,0);
	}
	
	char buf_x[16];
	sprintf(buf_x , "%d" , joy_x);
	char buf_y[16];
	sprintf(buf_y , "%d" , joy_y);
	OLED_NewFrame();
	OLED_DrawFilledRectangle(16,0,MENU_FONT->w*6 ,MENU_FONT->h , OLED_COLOR_REVERSED);
	OLED_DrawFilledRectangle(16,16,MENU_FONT->w*6 ,MENU_FONT->h , OLED_COLOR_REVERSED);
	OLED_PrintString(0,0,"x",MENU_FONT ,OLED_COLOR_NORMAL);
	OLED_PrintString(16,0,buf_x , MENU_FONT , OLED_COLOR_NORMAL);
	OLED_PrintString(0,16,"y",MENU_FONT ,OLED_COLOR_NORMAL);
	OLED_PrintString(16,16,buf_y , MENU_FONT , OLED_COLOR_NORMAL);
	OLED_ShowFrame();
	if (Key_WasPressed(KEY_10)) break;
	}

}
static void Menu_mode2(void)
{
	while(1)
	{
	OLED_NewFrame();
	OLED_PrintString(25 , 0 ,"UP Step",MENU_FONT ,OLED_COLOR_NORMAL);
	OLED_PrintString(0 , 24 , " 前升 ",MENU_FONT ,OLED_COLOR_NORMAL);
	OLED_PrintString(80 , 24 , " 前降 ",MENU_FONT ,OLED_COLOR_NORMAL);
	OLED_PrintString(25, 48, "DOWN Step",MENU_FONT ,OLED_COLOR_NORMAL);
	OLED_ShowFrame();
	if (Key_WasPressed(KEY_1))mode_send(mode_external_rotation);
	if (Key_WasPressed(KEY_2))mode_send(mode_turn_off_air_pump);
	if (Key_WasPressed(KEY_3))mode_send(mode_internal_rotation);
	if (Key_WasPressed(KEY_4))mode_send(mode_turn_on_air_pump);
	if (Key_WasPressed(KEY_10)) break;
	}

}
static void Menu_mode3(void)
{
	uint32_t last_time = 0;
	while(1)
	{
		
	if(HAL_GetTick() - last_time >= 200)
	{
		last_time = HAL_GetTick();
		joy_send(0 , 0 , joy_x_left);
	}
	
	char buf_x[16];
	sprintf(buf_x , "%d" , joy_x_left);
	OLED_NewFrame();
	OLED_DrawFilledRectangle(16,0,MENU_FONT->w*6 ,MENU_FONT->h , OLED_COLOR_REVERSED);
	OLED_PrintString(0,0,"z",MENU_FONT ,OLED_COLOR_NORMAL);
	OLED_PrintString(16,0,buf_x , MENU_FONT , OLED_COLOR_NORMAL);
	OLED_ShowFrame();
	if (Key_WasPressed(KEY_10)) break;
	}
}
static void Menu_mode4(void)
{
	while(1)
	{
	OLED_NewFrame();
	OLED_PrintString(25, 0 , "取块",MENU_FONT ,OLED_COLOR_NORMAL);
	OLED_PrintString(0 , 24 , "装武",MENU_FONT ,OLED_COLOR_NORMAL);
	OLED_PrintString(80 , 24 , "前降",MENU_FONT ,OLED_COLOR_NORMAL);
	OLED_PrintString(25, 48, "抓武",MENU_FONT ,OLED_COLOR_NORMAL);
	OLED_ShowFrame();
	if (Key_WasPressed(KEY_1))mode_send(1u<<4);
	if (Key_WasPressed(KEY_2))mode_send(1u<<5);
	if (Key_WasPressed(KEY_3))mode_send(1u<<6);
	if (Key_WasPressed(KEY_4))mode_send(1u<<7);
	if (Key_WasPressed(KEY_10)) break;
	}
}
static void Menu_mode5(void)
{
	while(1)
	{
	OLED_NewFrame();
	OLED_PrintString(0,0,"模式",MENU_FONT ,OLED_COLOR_NORMAL);
	OLED_ShowFrame();
	if (Key_WasPressed(KEY_10)) break;
	}
}
