#include "display.h"
#include <qtimer.h>
#include <dev/gpio.h>

#define SH1106_PAGES_NUM 8
#define SH1106_COLS_NUM  128

typedef unsigned char uint8_t;
typedef unsigned int  uint;

static void spi_write_data_or_cmd(uint8_t cmdordata,uint8_t data)
{
   uint8_t i;
   uint8_t bit;
   bit = 0x80;
   gpio_set(2,0); //cs low
   gpio_set(13,cmdordata ? 2 : 0); //cmd and data select
   for(i=0;i<8;i++)
   {
   	 //udelay(1);
	 gpio_set(3,0); //clk low
     if(data & bit){
		gpio_set(0,2); //miso high
     }else{
     	gpio_set(0,0); // miso low
     }
	 //udelay(1);
	 gpio_set(3,2);//clk high
	 bit >>= 1;
   }
   gpio_set(13,2); //cmd and data select
   gpio_set(2,2); //cs high
}

static void sh1106_power_on_off(int value)
{
	if(value){
		gpio_set(50,2);
	}else{
		gpio_set(50,0);
	}
}

static void sh1106_reset(void)
{
   mdelay(10);
   gpio_set(12,0); //reset pin low
   mdelay(50);
   gpio_set(12,2);//reset pin high
   mdelay(100);
}

static void sh1106_clear(uint pages,uint cols)
{
	uint i,j;
    for(i=0;i<pages;i++){
	  spi_write_data_or_cmd(0,0xb0+i); 
	  spi_write_data_or_cmd(0,0x10); 
	  spi_write_data_or_cmd(0,0x02); 
      for(j=0;j<cols;j++){
      	spi_write_data_or_cmd(1,0x00);
      }
   }
}

int sh1106_init(void)
{
	sh1106_power_on_off(1);
	sh1106_reset();
	spi_write_data_or_cmd(0,0xAE); //Set Display Off
	spi_write_data_or_cmd(0,0xD5); //Display divide ratio/osc. freq. mode
	spi_write_data_or_cmd(0,0x80); //
	spi_write_data_or_cmd(0,0xA8); //Multiplex ration mode:63
	spi_write_data_or_cmd(0,0x3F);
	spi_write_data_or_cmd(0,0xD3); //Set Display Offset
	spi_write_data_or_cmd(0,0x00);
	spi_write_data_or_cmd(0,0x40); //Set Display Start Line
	spi_write_data_or_cmd(0,0xAD); //DC-DC Control Mode Set
	spi_write_data_or_cmd(0,0x8B); //DC-DC ON/OFF Mode Set
	spi_write_data_or_cmd(0,0x32); //Set Pump voltage value
	spi_write_data_or_cmd(0,0xA1); //Segment Remap
	spi_write_data_or_cmd(0,0xC8); //Set COM Output Scan Direction
	spi_write_data_or_cmd(0,0xDA); //Common pads hardware: alternative
	spi_write_data_or_cmd(0,0x12);
	spi_write_data_or_cmd(0,0x81); //Contrast control
	spi_write_data_or_cmd(0,0x50);
	spi_write_data_or_cmd(0,0xD9); //Set pre-charge period
	spi_write_data_or_cmd(0,0x1F);
	spi_write_data_or_cmd(0,0xDB); //VCOM deselect level mode
	spi_write_data_or_cmd(0,0x40);
	spi_write_data_or_cmd(0,0xA4); //Set Entire Display On/Off
	spi_write_data_or_cmd(0,0xA6); //Set Normal Display
	//spi_write_data_or_cmd(0,0xAF); //Set Display On 
	sh1106_clear(8,128);
	spi_write_data_or_cmd(0,0xAF); //Set Display On
	return SUCCESS;
}

int sh1106_uninit(void)
{
	sh1106_power_on_off(0);
	gpio_set(12,0);
	return SUCCESS;
}


int sh1106_display_photo(unsigned char *data,int len)
{
	int i,j;
	if(SH1106_PAGES_NUM * SH1106_COLS_NUM != len){
		return FAIL;
	}
    for(i=0;i<SH1106_PAGES_NUM;i++){
	  spi_write_data_or_cmd(0,0xb0+i); 
	  spi_write_data_or_cmd(0,0x10); 
	  spi_write_data_or_cmd(0,0x02); 
      for(j=0;j<SH1106_COLS_NUM;j++){
      	spi_write_data_or_cmd(1,data[i*SH1106_COLS_NUM + j]);
      }
   }
   return SUCCESS;
}