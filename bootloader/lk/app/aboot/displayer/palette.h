#ifndef _PALETTE_H_
#define _PALETTE_H_
#include "font.h"

typedef struct palette{

	int width;
	int high;
	int dwidth;
	int buffer_len;
	unsigned char *buffer;
	
} Palette;

Palette * init_palette(int width,int high,int dwidth);
void  uninit_palette(Palette * palette);
void show_palette_data(Palette *palette);
int draw_point_in_palette(Palette *palette,int x, int y, int reverse);
int draw_line_in_palette(Palette *palette,int sx, int sy,int ex,int ey);
int draw_rectangle_in_palette(Palette *palette,int x, int y,int w,int h,int reverse);
int draw_string_in_palette(Palette *palette,E_LANGUAGE language,FONT_TYPE ftype,int x, int y,const char *data,int len,int reverse);
int draw_photo_in_palette(Palette *palette,int sx, int sy,int ex, int ey, unsigned char *data,int len,int dir, int reverse);
int clear_palette(Palette *palette);
void *get_palette_buffer(Palette *palette);
int get_palette_buffer_len(Palette *palette);

#endif
