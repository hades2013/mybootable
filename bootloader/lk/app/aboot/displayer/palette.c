#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "palette.h"
#include "display.h"




Palette * init_palette(int width,int high,int dwidth)
{
	Palette *palette = NULL;
	palette = (Palette *)malloc(sizeof(Palette) + (int)((width * high)/dwidth));
	if(palette == NULL){
		return NULL;
	}
	palette->width = width;
	palette->high = high;
	palette->dwidth = dwidth;
	palette->buffer = (unsigned char *)(palette + 1);
	palette->buffer_len = (int)((width * high)/dwidth);
	memset(palette->buffer,0,(int)((width * high)/dwidth));
	return palette;
}

void  uninit_palette(Palette * palette)
{
	if(palette != NULL)
			free(palette);
}

void show_palette_data(Palette *palette)
{
	int i,k;
	for(i = 0; i < ((palette->high * palette->width) / palette->dwidth); i++){
				int d_x = i % palette->width;
				int x = i / palette->width;
				for(k = 0; k < 8; k++){
					printf("[x=%d][y=%d]=%d\n",d_x+1, x*8 + k+1 , (palette->buffer[i]>> k)&0x01);
				}
	}
	
}
int draw_point_in_palette(Palette *palette,int x, int y, int reverse)
{
	if(palette == NULL){
		return FAIL;
	}
	if((x <= 0 || x > palette->width) ||( y <= 0|| y > palette->high)){
		return FAIL;
	}
	if(palette->buffer == NULL){
		return FAIL;
	}
	if(reverse){
		palette->buffer[(palette->width*((y-1) / palette->dwidth)) + (x-1)] &= ~(1 << ((y-1) % 8));
	}else{
		palette->buffer[(palette->width*((y-1) / palette->dwidth)) + (x-1)] |= (1 << ((y-1) % 8));
	}
	return SUCCESS;
}

static int draw_hline_in_palette(Palette *palette,int sx, int ex,int y)
{
	int x, i;
	if(ex < sx){
		x = ex;
		ex = sx;
		sx = x;
	}
	for(i = sx; i <= ex; i++){
		if(draw_point_in_palette(palette,i,y,0) == FAIL){
			return FAIL;
		}
	}
	return SUCCESS;
}

static int draw_rline_in_palette(Palette *palette,int x, int sy,int ey)
{
	int y, i;
	if(ey < sy){
		y = ey;
		ey = sy;
		sy = y;
	}
	for(i = sy; i <= ey; i++){
		if(draw_point_in_palette(palette,x,i,0)==FAIL){
			return FAIL;
		}
	}
	return SUCCESS;
}

static int Draw_Bresenhamline_in_palette(Palette *palette,int sx, int sy,int ex,int ey)
{
  #if 0
	int dx = ex-sx;
	int dy = ey-sy;
	int ux = dx > 0 ? 1 : -1;
	int uy = dy > 0 ? 1 : -1;
	int dx2 = dx << 1;
	int dy2 = dy << 1;
	if(abs(dx) > abs(dy)){
		int e = -dx;
		int x = sx;
		int y = sy;
		for(x = sx; x != ex; x += ux){
			if(draw_point_in_palette(palette,x,y,0)==FAIL){
				return FAIL;
			}
			e += dy2;
			if(e > 0){
				y += uy;
				e -= dx2;
			}
		}	
	}else{
		int e = -dy;
		int x = sx;
		int y = sy;
		for(y = sy; y != ey; y+=uy){
			if(draw_point_in_palette(palette,x,y,0)==FAIL){
				return FAIL;
			}
			e += dx2;
			if(e > 0){
				x += ux;
				e -= dy2;
			}
		}
	}
  #endif
	return SUCCESS;
}

int draw_line_in_palette(Palette *palette,int sx, int sy,int ex,int ey)
{
	if(palette == NULL){
		return FAIL;
	}
	if((sx <= 0 || sx > palette->width) ||( sy <= 0|| sy > palette->high)){
		return FAIL;
	}
	
	if((ex <0 || ex > palette->width) ||( ey <= 0|| ey > palette->high)){
		return FAIL;
	}
	
	if(sx == ex){  //Rline
		return draw_rline_in_palette(palette,sx,sy,ey);
	}
	
	if(sy == ey){  //Hline
		return draw_hline_in_palette(palette,sx,ex,sy);
	}
	
	return Draw_Bresenhamline_in_palette(palette,sx,sy,ex,ey);
}

int draw_rectangle_in_palette(Palette *palette,int x, int y,int w,int h,int reverse)
{
	int i;
	if(reverse){
		if(draw_line_in_palette(palette,x,y,x+w,y) == FAIL){
			return FAIL;
		}
		if(draw_line_in_palette(palette,x,y,x,y+h) == FAIL){
			return FAIL;
		}
		if(draw_line_in_palette(palette,x,y+h,x+w,y+h) == FAIL){
			return FAIL;
		}
		if(draw_line_in_palette(palette,x+w,y,x+w,y+h) == FAIL){
			return FAIL;
		}
	}else{
		for(i = 0; i < h; i++){
			if(draw_line_in_palette(palette,x,y+i,x+w,y+i) == FAIL){
				return FAIL;
			}
		}
	}
	return SUCCESS;
}

static int draw_8x8_string_in_palette(Palette *palette,E_LANGUAGE language,int x, int y,const char *data,int len,int reverse)
{
	int i,j,k;
	Font_Data *font = NULL;
	unsigned char tmp = 0;
	for(i = 0; i < len; i++){
			if((font = get_font_data_by_char(language,FONT_TYPE_8X5,data[i])) == NULL){
				return FAIL;
			}
			font->data[font->len] =0;
			font->len += 1;
			for(j = 0; j < font->len; j++){
				for( k = 0; k < 8; k++){
					if(reverse){
						tmp = ~font->data[j];
					}else{
						tmp = font->data[j];
					}
					if(draw_point_in_palette(palette, x, y + k, (((tmp >> k) & 0x01) ? 0 : 1)) == FAIL){
						return FAIL;
					}
				}
				x += 1;
			}
	}
	return SUCCESS;
}

static int draw_16x8_string_in_palette(Palette *palette,E_LANGUAGE language,int x, int y,const char *data,int len, int reverse)
{
	int i, j, k;
	unsigned short tmp = 0;
	Font_Data *font = NULL;
	for(i = 0; i < len; i++){
		if((font = get_font_data_by_char(language,FONT_TYPE_16X8,data[i])) == NULL){
				return FAIL;
		}	
		for(j = 0; j < font-> len; j += 2){
			if(reverse){
					unsigned char rdata;
					rdata = ~font->data[j+1];
					tmp = rdata;
					tmp <<= 8;
					rdata = ~font->data[j];
					tmp |= rdata;
			}else{
					tmp = font->data[j+1];
					tmp <<= 8;
					tmp |= font->data[j];
			}
			for(k = 0; k < 16; k++){
				if(draw_point_in_palette(palette, x, y + k, (((tmp >> k) & 0x01) ? 0 : 1)) == FAIL){
						return FAIL;
				}
			}
			x += 1;
		}
	}
	return 0;
}

int draw_string_in_palette(Palette *palette,E_LANGUAGE language,FONT_TYPE ftype,int x, int y,const char *data,int len,int reverse)
{	
	if(palette == NULL){
		return FAIL;
	}
	if((x < 0 || x >palette->width) || (y < 0 || y > palette->high)){
		return FAIL;
	}
	switch(ftype){
		case FONT_TYPE_8X5:
			return draw_8x8_string_in_palette(palette, language, x, y, data, len, reverse);
		case FONT_TYPE_16X8:
			return draw_16x8_string_in_palette(palette,language, x, y, data, len, reverse);
		default:
			return FAIL;
	}
	return FAIL;
}
int draw_photo_in_palette(Palette *palette,int sx, int sy,int ex, int ey, unsigned char *data,int len,int dir, int reverse)
{
	int dx, dy;
	int i,j;
	int tbits = 0;
	int ibits = 0;
	unsigned char tmp;
	unsigned char move;
	if((sx < 0 || sx > palette->width) ||( sy <= 0|| sy > palette->high)){
		return FAIL;
	}
	
	if((ex <0 || ex > palette->width) ||( ey <= 0|| ey > palette->high)){
		return FAIL;
	}
	
	if(sx > ex || sy > ey){
		return FAIL;
	}
	tbits = 8 * sizeof(data[0]) * len;
	if(dir == 0){
		dx = ex - sx + 1;
		if(tbits % dx != 0){
			return FAIL;
		}
		for(j = 0; j < tbits / dx; j++){
			for(i = 0; i < dx; i++){
				if(reverse){
					tmp = ~data[ibits / 8];
				}else{
					tmp = data[ibits / 8];
				}
				move = ibits % 8;
				if(draw_point_in_palette(palette, sx + i, sy + j, (((tmp >> move) & 0x01) ? 0 : 1)) == FAIL){
					return FAIL;
				}
				ibits++;
			}
		}
		
	}else if(dir == 1){
		dy = ey - sy + 1;
		if(tbits % dy != 0){
			return FAIL;
		}
		for(j = 0; j < tbits / dy; j++){
			for(i = 0; i < dy; i++){
				if(reverse){
					tmp = ~data[ibits / 8];
				}else{
					tmp = data[ibits / 8];
				}
				move = ibits % 8;
				if(draw_point_in_palette(palette, sx + j, sy + i, (((tmp >> move) & 0x01) ? 0 : 1)) == FAIL){
					return FAIL;
				}
				ibits++;
			}
		}

	}
	return SUCCESS;
}

int clear_palette(Palette *palette)
{
	memset(palette->buffer,0,palette->buffer_len);
	return SUCCESS;
}

void *get_palette_buffer(Palette *palette)
{
	return palette->buffer;
}

int get_palette_buffer_len(Palette *palette)
{
	return palette->buffer_len;
}