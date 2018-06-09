#ifndef _SH1106_H_
#define _SH1106_H_

int sh1106_init(void);
int sh1106_display_photo(unsigned char *data,int len);
int sh1106_uninit(void);

#endif
