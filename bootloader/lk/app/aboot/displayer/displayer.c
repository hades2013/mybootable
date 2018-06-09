#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "displayer.h"
#include "display.h"
#include "sh1106.h"
#include "debug.h"

static Displayer *current_displayer = NULL;
static Displayer displayer_lists[]={
		{
			.name = "sh1106",
			.width = 128,
			.high = 64,
			.init_f = sh1106_init,
			.display_photo_f = sh1106_display_photo,
			.uninit_f = sh1106_uninit,
		},
};

int init_displayer(const char *devname)
{
	unsigned int i;
	current_displayer = NULL;
	for(i = 0; i < sizeof(displayer_lists)/sizeof(displayer_lists[0]); i++){
		if(strcmp(displayer_lists[i].name,devname) == 0){
			current_displayer = &displayer_lists[i];
		}
	}
	if(current_displayer != NULL){
		if(current_displayer->init_f){
			current_displayer->init_f();
		}
		current_displayer->palette = init_palette(current_displayer->width, current_displayer->high, 8);
		if(current_displayer->palette == NULL){
			current_displayer->uninit_f();
			return FAIL;
		}
		dprintf(INFO,"%s\n","init_displayer success");
		return SUCCESS;
	}
	dprintf(INFO,"%s\n","init_displayer FAIL");
	return FAIL;
	
}

int uninit_displayer(void)
{
	if(current_displayer == NULL && current_displayer->uninit_f){
		uninit_palette(current_displayer->palette);
		current_displayer->uninit_f();
	}
	return SUCCESS;
}

int display_photo(int sx, int sy,int ex, int ey, unsigned char *data,int len,int dir, int reverse)
{
	if(current_displayer != NULL && current_displayer->palette != NULL){
		return draw_photo_in_palette(current_displayer->palette,sx, sy, ex, ey, data, len, dir, reverse);
	}
	return SUCCESS;
}

int display_strings(E_LANGUAGE language,FONT_TYPE ftype,int x, int y,const char *data,int len,int reverse)
{
	if(current_displayer != NULL && current_displayer->palette != NULL){
		dprintf(INFO,"%s\n","display_strings##############");
		return draw_string_in_palette(current_displayer->palette, language, ftype, x, y, data, len, reverse);
	}
	return SUCCESS;
}

int display_rectangle(int x, int y,int w,int h,int reverse)
{
  if(current_displayer != NULL && current_displayer->palette != NULL){
	return draw_rectangle_in_palette(current_displayer->palette,x, y, w, h, reverse);
  }
  return FAIL;
}
int refresh_displayer(void)
{
	if(current_displayer != NULL && current_displayer->display_photo_f != NULL){
		return current_displayer->display_photo_f(get_palette_buffer(current_displayer->palette),get_palette_buffer_len(current_displayer->palette));
	}
	return FAIL;
}

int clear_displayer(void)
{
	if(current_displayer != NULL && current_displayer->palette != NULL){
		clear_palette(current_displayer->palette);
	}
	return SUCCESS;
}

int display_battery(int index)
{
	int wigth = 3*index;
	if(index < 0 && index > 15)
	{
		return FAIL;
	}
	display_rectangle(33,17,55,4,0);
	display_rectangle(33,45,55,4,0);
	display_rectangle(33,17,3,32,0);
	display_rectangle(87,17,3,32,0);
	display_rectangle(91,25,3,15,0);
	if(wigth !=0)
		display_rectangle(39,23,wigth,20,0);
	return FAIL;
}
