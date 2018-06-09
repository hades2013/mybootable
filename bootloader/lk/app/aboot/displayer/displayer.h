#ifndef _DISPLAYER_H_
#define _DISPLAYER_H_
#include "font.h"
#include "palette.h"

typedef int (*Init_Function)(void);
typedef int (*Display_Photo_Function)(unsigned char *data, int len);
typedef int (*Uninit_Function)(void);


typedef struct displayer{
	char *name; 
	int width;
	int high;
	Init_Function init_f;
	Display_Photo_Function display_photo_f;
	Uninit_Function uninit_f;
	Palette *palette;
	
	
} Displayer;


int init_displayer(const char *devname);
int uninit_displayer(void);
int display_photo(int sx, int sy,int ex, int ey, unsigned char *data,int len,int dir, int reverse);
int display_strings(E_LANGUAGE language,FONT_TYPE ftype,int x, int y,const char *data,int len,int reverse);
int display_rectangle(int x, int y,int w,int h,int reverse);
int display_battery(int index);
int refresh_displayer(void);
int clear_displayer(void);

#endif