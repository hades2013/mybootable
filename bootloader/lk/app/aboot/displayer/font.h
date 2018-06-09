#ifndef _FONT_H_
#define _FONT_H_

#define MAX_FONT_DATA_LEN 256

typedef enum{
	
	ENGLISH = 0,
	BAHASA,
	
}E_LANGUAGE;

typedef enum{
	FONT_TYPE_8X5=1,
	FONT_TYPE_16X8,
	
} FONT_TYPE;

typedef struct font_data{
	int len;
	unsigned char data[MAX_FONT_DATA_LEN];
} Font_Data;


Font_Data *get_font_data_by_char(E_LANGUAGE language, FONT_TYPE ftype, char c);

#endif