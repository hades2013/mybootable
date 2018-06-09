/*
 * bq2415x charger driver
 *
 * Copyright (C) 2011-2013  Pali Rohár <pali.rohar@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*
 * Datasheets:
 * http://www.ti.com/product/bq24150
 * http://www.ti.com/product/bq24150a
 * http://www.ti.com/product/bq24152
 * http://www.ti.com/product/bq24153
 * http://www.ti.com/product/bq24153a
 * http://www.ti.com/product/bq24155
 */
#include <blsp_qup.h>
#include <i2c_qup.h>
#include <gsbi.h>
#include <err.h>
#include <debug.h>
#include <malloc.h>
#include <platform/irqs.h>
#include <platform/interrupts.h>
#include <pm8x41.h>
#include <platform/gpio.h>
#include <pm8x41_adc.h>
#include <kernel/thread.h>
#include <target.h>
#include <platform.h>

#include "ww_bq2415x_charger.h"
#include "displayer.h"



#define WW_PRINTK  //printk("%s[%d]--------------->\n",__FUNCTION__,__LINE__)
#define ww_print(format, arg...)		printf(format, ##arg)

extern int ww_hw_type;
#define OTG_ID_GPIO  4 // PMIC GPIO4
/* timeout for resetting chip timer */
#define BQ2415X_TIMER_TIMEOUT		10

#define BQ2415X_REG_STATUS		0x00
#define BQ2415X_REG_CONTROL		0x01
#define BQ2415X_REG_VOLTAGE		0x02
#define BQ2415X_REG_VENDER		0x03
#define BQ2415X_REG_CURRENT		0x04
#define BQ2415X_REG_CONTROL_1   0x05
#define BQ2415X_REG_SAFETY      0x06
#define BQ2415X_REG_MOMITOR     0x10

#define OTG_CHARGE_CLOSE_ST      1
#define OTG_CHARGE_OPEN_ST       2
#define OTG_CHARGE_STOP_ST       3

/* reset state for all registers */
#define BQ2415X_RESET_STATUS		BIT(6)
#define BQ2415X_RESET_CONTROL		(BIT(4)|BIT(5))
#define BQ2415X_RESET_VOLTAGE		(BIT(1)|BIT(3))
#define BQ2415X_RESET_CURRENT		(BIT(0)|BIT(3)|BIT(7))

/* status register */
#define BQ2415X_BIT_TMR_RST		7
#define BQ2415X_BIT_OTG			7
#define BQ2415X_BIT_EN_STAT		6
#define BQ2415X_MASK_STAT		(BIT(4)|BIT(5))
#define BQ2415X_SHIFT_STAT		4
#define BQ2415X_BIT_BOOST		3
#define BQ2415X_MASK_FAULT		(BIT(0)|BIT(1)|BIT(2))
#define BQ2415X_SHIFT_FAULT		0

/* control register */
#define BQ2415X_MASK_LIMIT		(BIT(6)|BIT(7))
#define BQ2415X_SHIFT_LIMIT		6
#define BQ2415X_MASK_VLOWV		(BIT(4)|BIT(5))
#define BQ2415X_SHIFT_VLOWV		4
#define BQ2415X_BIT_TE			3
#define BQ2415X_BIT_CE			2
#define BQ2415X_BIT_HZ_MODE		1
#define BQ2415X_BIT_OPA_MODE		0

/* voltage register */
#define BQ2415X_MASK_VO		(BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7))
#define BQ2415X_SHIFT_VO		2
#define BQ2415X_BIT_OTG_PL		1
#define BQ2415X_BIT_OTG_EN		0

/* vender register */
#define BQ2415X_MASK_VENDER		(BIT(5)|BIT(6)|BIT(7))
#define BQ2415X_SHIFT_VENDER		5
#define BQ2415X_MASK_PN			(BIT(3)|BIT(4))
#define BQ2415X_SHIFT_PN		3
#define BQ2415X_MASK_REVISION		(BIT(0)|BIT(1)|BIT(2))
#define BQ2415X_SHIFT_REVISION		0

/* current register */
#define BQ2415X_MASK_RESET		BIT(7)
#define BQ2415X_MASK_VI_CHRG		(BIT(4)|BIT(5)|BIT(6))
#define BQ2415X_SHIFT_VI_CHRG		4
/* N/A					BIT(3) */
#define BQ2415X_MASK_VI_TERM		(BIT(0)|BIT(1)|BIT(2))
#define BQ2415X_SHIFT_VI_TERM		0
/*set control1*/

/*SAFETY*/
#define BQ2415X_MASK_IOLEVEL		BIT(5)
#define BQ2415X_SHIFT_IOLEVEL		5


/*SAFETY*/
#define BQ2415X_MASK_I_SAFE		(BIT(4)|BIT(5)|BIT(6))
#define BQ2415X_SHIFT_I_SAFE		4

#define BQ2415X_MASK_V_SAFE		(BIT(0)|BIT(1)|BIT(2)|BIT(3))
#define BQ2415X_SHIFT_V_SAFE		0

#define CHG_PWR_KEY_ST                BIT(2)

#define CHG_FULL        2
#define CHG_USB_PRESENT 1


#define BATT_GOOD_ST        0
#define BATT_MISS_ST        1
#define BATT_OVER_TEMP_ST   2


#define CHG_VBUS_VOL_MIN              (1500*1000)
#define BATTERY_DISPRESENT            (1760*1000) //REF 1.8V

#if defined(WW_PRODUCT_MODEL_MF674S)
#define BATT_CHARGE_OVER_TEMP_VOL      (420*1000) // 1.8v 60 `C
#else
#define BATT_CHARGE_OVER_TEMP_VOL      (328*1000) // 1.8v 60 `C
#endif


enum bq2415x_command {
	BQ2415X_TIMER_RESET,
	BQ2415X_OTG_STATUS,
	BQ2415X_STAT_PIN_STATUS,
	BQ2415X_STAT_PIN_ENABLE,
	BQ2415X_STAT_PIN_DISABLE,
	BQ2415X_CHARGE_STATUS,
	BQ2415X_BOOST_STATUS,
	BQ2415X_FAULT_STATUS,

	BQ2415X_CHARGE_TERMINATION_STATUS,
	BQ2415X_CHARGE_TERMINATION_ENABLE,
	BQ2415X_CHARGE_TERMINATION_DISABLE,
	BQ2415X_CHARGER_STATUS,
	BQ2415X_CHARGER_ENABLE,
	BQ2415X_CHARGER_DISABLE,
	BQ2415X_HIGH_IMPEDANCE_STATUS,
	BQ2415X_HIGH_IMPEDANCE_ENABLE,
	BQ2415X_HIGH_IMPEDANCE_DISABLE,
	BQ2415X_BOOST_MODE_STATUS,
	BQ2415X_BOOST_MODE_ENABLE,
	BQ2415X_BOOST_MODE_DISABLE,

	BQ2415X_OTG_LEVEL,
	BQ2415X_OTG_ACTIVATE_HIGH,
	BQ2415X_OTG_ACTIVATE_LOW,
	BQ2415X_OTG_PIN_STATUS,
	BQ2415X_OTG_PIN_ENABLE,
	BQ2415X_OTG_PIN_DISABLE,

	BQ2415X_VENDER_CODE,
	BQ2415X_PART_NUMBER,
	BQ2415X_REVISION,

	BQ2415X_CHARGE_ENABLE_IOLEVE,
	BQ2415X_CHARGE_DISABLE_IOLEVE,	
};

enum bq2415x_chip {
	BQUNKNOWN,
	BQ24150,
	BQ24150A,
	BQ24151,
	BQ24151A,
	BQ24152,
	BQ24153,
	BQ24153A,
	BQ24155,
	BQ24156,
	BQ24156A,
	BQ24158,
};
#if 0 
static char *bq2415x_chip_name[] = {
	"unknown",
	"bq24150",
	"bq24150a",
	"bq24151",
	"bq24151a",
	"bq24152",
	"bq24153",
	"bq24153a",
	"bq24155",
	"bq24156",
	"bq24156a",
	"bq24158",
};
 #endif
 struct bq2415x_platform_data {
	 int current_limit; 	 /* mA */
	 int weak_battery_voltage;	 /* mV */
	 int battery_regulation_voltage; /* mV */
	 int charge_current;	 /* mA */
	 int termination_current;	 /* mA */
	 int resistor_sense;	 /* m ohm */
	 const char *notify_device;  /* name */
 };


struct bq2415x_device {
	struct device *dev;
	struct bq2415x_platform_data init_data;
	enum bq2415x_chip chip;
		int			chg_present;
	int volatile otg_present;
	int			irq_gpio;
	int         otg_irq_gpio;
	int volatile ww_charge_vol;
		/* otg 5V regulator */	
	const char *timer_error;
	char *model;
	char *name;
	int autotimer;	/* 1 - if driver automatically reset timer, 0 - not */
	int automode;	/* 1 - enabled, 0 - disabled; -1 - not supported */
	int id;
};

#define I2C_CLK_FREQ     100000
#define I2C_SRC_CLK_FREQ 19200000
#define I2C_CLIENT_ADDR   0x6A
static struct qup_i2c_dev *i2c_dev;



static int volatile ww_otg_stop_vol = (3480*1000);
static int volatile ww_otg_start_vol = (3750*1000);
/**** i2c read functions ****/
int ww_bq_i2c_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
	if (!buf)
		return ERR_INVALID_ARGS;

	if(!i2c_dev)
		return ERR_NOT_VALID;

	struct i2c_msg rd_buf[] = {
		{addr, I2C_M_WR, 1, &reg},
		{addr, I2C_M_RD, len, buf}
	};

	int err = qup_i2c_xfer(i2c_dev, rd_buf, 2);
	if (err < 0) {
		dprintf(CRITICAL, "Read reg %x failed\n", reg);
		return err;
	}

	return NO_ERROR;
}

int ww_bq_i2c_read_byte(uint8_t addr, uint8_t reg, uint8_t *buf)
{
	if (!buf)
		return ERR_INVALID_ARGS;

	return ww_bq_i2c_read(addr, reg, buf, 1);
}

int ww_bq_i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t val)
{
	if (!i2c_dev)
		return ERR_NOT_VALID;

	unsigned char buf[2] = {reg, val};
	struct i2c_msg msg_buf[] = {
		{addr, I2C_M_WR, 2, buf},
	};

	int err = qup_i2c_xfer(i2c_dev, msg_buf, 1);
	if (err < 0) {
		dprintf(CRITICAL, "Write reg %x failed\n", reg);
		return err;
	}
	return NO_ERROR;
}

static int bq2415x_read_reg(struct bq2415x_device *chip,int reg,uint8_t *val)
{
	int rc;

	rc = ww_bq_i2c_read_byte(I2C_CLIENT_ADDR,reg,val);

	return rc;
}

static int bq2415x_write_reg(struct bq2415x_device *chip,int reg,uint8_t val)
{
	int rc;

	rc = ww_bq_i2c_write_byte(I2C_CLIENT_ADDR, reg, val);

	return rc;
}


/* read value from register, apply mask and right shift it */
static int bq2415x_i2c_read_mask(struct bq2415x_device *bq, uint8_t reg,
				 uint8_t mask, uint8_t shift)
{
	uint8_t ret;
	int rc;
	if (shift > 8)
		return -1;

	 rc= bq2415x_read_reg(bq, reg,&ret);
	if (rc < 0)
		return rc;
	return (ret & mask) >> shift;
}

/* read value from register and return one specified bit */
static int bq2415x_i2c_read_bit(struct bq2415x_device *bq, uint8_t reg, uint8_t bit)
{
	if (bit > 8)
		return -1;
	return bq2415x_i2c_read_mask(bq, reg, BIT(bit), bit);
}

/**** i2c write functions ****/

/* read value from register, change it with mask left shifted and write back */
static int bq2415x_i2c_write_mask(struct bq2415x_device *bq, uint8_t reg, uint8_t val,
				  uint8_t mask, uint8_t shift)
{
	uint8_t ret;
    int rc;
	if (shift > 8)
		return -1;

	rc = bq2415x_read_reg(bq, reg, &ret);
	if (rc < 0)
		return rc;

	ret &= ~mask;
	ret |= val << shift;

	return bq2415x_write_reg(bq, reg, ret);
}

/* change only one bit in register */
static int bq2415x_i2c_write_bit(struct bq2415x_device *bq, uint8_t reg,
				 bool val, uint8_t bit)
{
	if (bit > 8)
		return -1;
	return bq2415x_i2c_write_mask(bq, reg, val, BIT(bit), bit);
}

/**** global functions ****/

/* exec command function */
static int bq2415x_exec_command(struct bq2415x_device *bq,
				enum bq2415x_command command)
{
	int ret;
   printf("bq2415x_exec_command [%d]\n",command);
	switch (command) {
		case BQ2415X_CHARGE_STATUS:
			return bq2415x_i2c_read_mask(bq, BQ2415X_REG_STATUS,
					BQ2415X_MASK_STAT, BQ2415X_SHIFT_STAT);	
		case BQ2415X_FAULT_STATUS:
			return bq2415x_i2c_read_mask(bq, BQ2415X_REG_STATUS,
				BQ2415X_MASK_FAULT, BQ2415X_SHIFT_FAULT);

		case BQ2415X_CHARGE_TERMINATION_STATUS:
			return bq2415x_i2c_read_bit(bq, BQ2415X_REG_CONTROL,
					BQ2415X_BIT_TE);
		case BQ2415X_CHARGE_TERMINATION_ENABLE:
			return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
					1, BQ2415X_BIT_TE);
		case BQ2415X_CHARGE_TERMINATION_DISABLE:
			return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
					0, BQ2415X_BIT_TE);
		case BQ2415X_CHARGER_STATUS:
			ret = bq2415x_i2c_read_bit(bq, BQ2415X_REG_CONTROL,
				BQ2415X_BIT_CE);
			if (ret < 0)
				return ret;
			else
				return ret > 0 ? 0 : 1;
		case BQ2415X_CHARGER_ENABLE:
			return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
					0, BQ2415X_BIT_CE);
		case BQ2415X_CHARGER_DISABLE:
			return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
					1, BQ2415X_BIT_CE);
		case BQ2415X_CHARGE_ENABLE_IOLEVE:
			return bq2415x_i2c_write_mask(bq, BQ2415X_REG_CONTROL_1, 0,
					 BQ2415X_MASK_IOLEVEL ,BQ2415X_SHIFT_IOLEVEL);
		case BQ2415X_CHARGE_DISABLE_IOLEVE:
			return bq2415x_i2c_write_mask(bq, BQ2415X_REG_CONTROL_1, 1,
					 BQ2415X_MASK_IOLEVEL ,BQ2415X_SHIFT_IOLEVEL);
		case BQ2415X_BOOST_MODE_STATUS:
			return bq2415x_i2c_read_bit(bq, BQ2415X_REG_CONTROL,
					BQ2415X_BIT_OPA_MODE);
		case BQ2415X_BOOST_MODE_ENABLE:
			return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
					1, BQ2415X_BIT_OPA_MODE);
		case BQ2415X_BOOST_MODE_DISABLE:
			return bq2415x_i2c_write_bit(bq, BQ2415X_REG_CONTROL,
					0, BQ2415X_BIT_OPA_MODE);

		default:
				break;
	
	}
	return -1;
}

/* print value of chip register, format: 'register=value' */
static void bq2415x_sysfs_print_reg(struct bq2415x_device *bq, uint8_t reg)				      
{
	uint8_t ret;
	int rc = bq2415x_read_reg(bq, reg,&ret);

	if (rc < 0)
		printf("%#.2x=error %d\n", reg, ret);

	printf("%#.2x=%#.2x\n", reg, ret);
}

/* show all raw values of chip register, format per line: 'register=value' */
static void bq2415x_sysfs_show_registers(struct bq2415x_device *bq)
{
	 bq2415x_sysfs_print_reg(bq, BQ2415X_REG_STATUS);
	 bq2415x_sysfs_print_reg(bq, BQ2415X_REG_CONTROL);
	 bq2415x_sysfs_print_reg(bq, BQ2415X_REG_VOLTAGE);
	 bq2415x_sysfs_print_reg(bq, BQ2415X_REG_VENDER);
	 bq2415x_sysfs_print_reg(bq, BQ2415X_REG_CURRENT);
	 bq2415x_sysfs_print_reg(bq, BQ2415X_REG_CONTROL_1);
	 bq2415x_sysfs_print_reg(bq, BQ2415X_REG_SAFETY);
//	 bq2415x_sysfs_print_reg(bq, BQ2415X_REG_MOMITOR);	
}



/* set current limit in mA */
static int bq2415x_set_current_limit(struct bq2415x_device *bq, int mA)
{
	int val;

	if (mA <= 100)
		val = 0;
	else if (mA <= 500)
		val = 1;
	else if (mA <= 800)
		val = 2;
	else
		val = 3;
	return bq2415x_i2c_write_mask(bq, BQ2415X_REG_CONTROL, val,
			BQ2415X_MASK_LIMIT, BQ2415X_SHIFT_LIMIT);
}

/* set weak battery voltage in mV */
static int bq2415x_set_weak_battery_voltage(struct bq2415x_device *bq, int mV)
{
	int val;

	/* round to 100mV */
	if (mV <= 3400 + 50)
		val = 0;
	else if (mV <= 3500 + 50)
		val = 1;
	else if (mV <= 3600 + 50)
		val = 2;
	else
		val = 3;

	return bq2415x_i2c_write_mask(bq, BQ2415X_REG_CONTROL, val,
			BQ2415X_MASK_VLOWV, BQ2415X_SHIFT_VLOWV);
}


/* set battery regulation voltage in mV */
static int bq2415x_set_battery_regulation_voltage(struct bq2415x_device *bq,
						  int mV)
{
	int val = (mV/10 - 350) / 2;

	/*
	 * According to datasheet, maximum battery regulation voltage is
	 * 4440mV which is b101111 = 47.
	 */
	if (val < 0)
		val = 0;
	else if (val > 47)
		return -1;
    ww_print("%s[%d] val[%d]mV[%d]-------->\n",__FUNCTION__,__LINE__,val,mV);
	return bq2415x_i2c_write_mask(bq, BQ2415X_REG_VOLTAGE, val,
			BQ2415X_MASK_VO, BQ2415X_SHIFT_VO);
}

/* set charge current in mA (platform data must provide resistor sense) */
static int bq2415x_set_charge_current(struct bq2415x_device *bq, int mA)
{

	if (bq->init_data.resistor_sense <= 0)
		return -ENOSYS;  
	int val = 0;

	if ( mA < 1000 )
		val = 4; // 950 mA
	else if ( mA <= 1050 )
		val = 5; 
	else if (mA <= 1150)
		val = 6;
	else if ( mA <= 1250 )
		val = 7; 
	
	
	//ww_print("%s[%d] val[%d] mA[%d]-------->\n",__FUNCTION__,__LINE__,val,mA);
	return bq2415x_i2c_write_mask(bq, BQ2415X_REG_CURRENT, val,
			BQ2415X_MASK_VI_CHRG | BQ2415X_MASK_RESET,
			BQ2415X_SHIFT_VI_CHRG);
}


/* set termination current in mA (platform data must provide resistor sense) */
static int bq2415x_set_termination_current(struct bq2415x_device *bq, int mA)
{
	int val;

	if (bq->init_data.resistor_sense <= 0)
		return -ENOSYS;

	val = (mA * bq->init_data.resistor_sense - 3400) / 3400;
	if (val < 0)
		val = 0;
	else if (val > 7)
		val = 7;
	ww_print("%s[%d] val[%d]-------->\n",__FUNCTION__,__LINE__,val);
	#if 1

	return bq2415x_i2c_write_mask(bq, BQ2415X_REG_CURRENT, val,
			BQ2415X_MASK_VI_TERM | BQ2415X_MASK_RESET,
			BQ2415X_SHIFT_VI_TERM);
	#else
	return bq2415x_i2c_write_mask(bq, BQ2415X_REG_CURRENT, val,
			BQ2415X_MASK_VI_TERM,BQ2415X_SHIFT_VI_TERM);
	#endif
}

/* set default values of all properties */
static int bq2415x_set_defaults(struct bq2415x_device *bq,int batt_st)
{
//    if(SY6923D_CHARGE_IC == ww_hw_type){
	bq2415x_write_reg(bq, BQ2415X_REG_CURRENT, BQ2415X_RESET_CURRENT);
	bq2415x_exec_command(bq, BQ2415X_BOOST_MODE_DISABLE);

	if(batt_st != BATT_MISS_ST)
		bq2415x_exec_command(bq, BQ2415X_CHARGER_DISABLE);
	
	bq2415x_exec_command(bq, BQ2415X_CHARGE_TERMINATION_DISABLE);

	
	bq2415x_set_current_limit(bq, bq->init_data.current_limit);
	bq2415x_set_weak_battery_voltage(bq,  bq->init_data.weak_battery_voltage);
	bq2415x_set_battery_regulation_voltage(bq, bq->init_data.battery_regulation_voltage);
	if (bq->init_data.resistor_sense > 0) {
		bq2415x_set_charge_current(bq, bq->init_data.charge_current);
		bq2415x_set_termination_current(bq, bq->init_data.termination_current);
		bq2415x_exec_command(bq, BQ2415X_CHARGE_ENABLE_IOLEVE);
		bq2415x_exec_command(bq, BQ2415X_CHARGE_TERMINATION_ENABLE);
	}
	
	bq2415x_exec_command(bq, BQ2415X_CHARGER_ENABLE);
#if 0	
    }
	else{

		bq2415x_write_reg(bq,0x06,0x49);// BIT [7:4] SAFE CUR:1.1A;  [3:0]  SAFE VOLTAGE :4.35V
		bq2415x_write_reg(bq,0x02,0x94);//[7:2] SET VOREG 4.35
		bq2415x_write_reg(bq,0x01,0xf8);// CURRENT LIMIT[6:7]: no limit;  DISABLE[3] HZ[2] APM[0]    TERM_ENABLE[3]:enable
		bq2415x_write_reg(bq,0x04,0x40);// [6:4]SET CRRENT 1A
		bq2415x_write_reg(bq,0x05,0x84);//bit [2]SET VSP : 4.56V ,BIT[7] VOREG+20mV ,ADD 20mv 
    }
#endif
	return 0;
}

static int bq2415x_charge_safe_init(struct bq2415x_device *bq)
{
  uint8_t val_cur,val_volt,val = 0;
  int ret;
  /*4:1050ma,7:1450*/
  if (bq->init_data.charge_current <= 1050)
  	val_cur = 5; //1050
  else  
  	val_cur = 0xf;//1550mA

  if ( 4200 == bq->init_data.battery_regulation_voltage)
  	val_volt = 0;
  else  if ( 4350 == bq->init_data.battery_regulation_voltage)
  	val_volt = 7; 
  
  val = (val_cur << 4) | val_volt;

  ret= bq2415x_write_reg(bq, BQ2415X_REG_SAFETY, val);
  
  return ret;
}


void ww_bq2415x_parse_dt(struct bq2415x_device *bq){
	bq->init_data.current_limit=1050;
    bq->init_data.weak_battery_voltage=3700;
	bq->init_data.battery_regulation_voltage=4350;
	bq->init_data.charge_current=1050;
	bq->init_data.termination_current=100;
	bq->init_data.resistor_sense=68;
}

static int ww_read_vbus_present_st(struct bq2415x_device *bq){
	int i = 0;
    uint8_t val = 0;
    uint8_t tmp = 0;
	int cnt = 0;
	for(i=0; i<3; i++){
		bq2415x_read_reg(bq, BQ2415X_REG_STATUS, &val);
        //dprintf(CRITICAL, "val: %x\n", val);
        tmp = (val&0x30)>>4;
        if(tmp != 0x00){
			cnt++;
        }
		thread_sleep(20);
	}
	
	dprintf(CRITICAL, "Now vbus voltage is %02x, cnt: %d.\n", tmp, cnt);
	return cnt;

}


static int ww_read_battery_present_st(void)
{
	int therm_vol = 0;
	int i = 0;
	int cnt = 0;
	int cnt_temp = 0;
	while(i < 3){
		therm_vol = pm8x41_get_batt_thermal_id();
		if(therm_vol > BATTERY_DISPRESENT)
			cnt++;
		if(therm_vol <= BATT_CHARGE_OVER_TEMP_VOL)
			cnt_temp++;
		
		i++;
	}

	
	dprintf(CRITICAL, " now thermal id voltage is %d %d uv %d.\n",therm_vol,cnt,cnt_temp);
	
	if(cnt >= 3)
		return BATT_MISS_ST;
	if(cnt_temp >= 3)
		return BATT_OVER_TEMP_ST;
	
	
	return BATT_GOOD_ST;

}

#if 0
#define RESTATRT_BATT_VOL (3450*1000)
#define STATRT_BATT_VOL (3500*1000)
#define BATT_OCV_CNT  3
int ww_det_restart_batt_vot(int val)
{
  int cnt = 0;
  int i = 0;
  int batt_vol = 0;
  int batt_ovl[]={STATRT_BATT_VOL,RESTATRT_BATT_VOL};

  if(val)
  	gpio_set_dir(WW_CHARGE_DIS_EN,2);
//  thread_sleep(30);
  
  while(i < 3){
	thread_sleep(20);
	batt_vol = pm8x41_get_batt_voltage();
	dprintf(CRITICAL, " now battery voltage is %d uv.\n",batt_vol);
	
	if(batt_vol >= batt_ovl[val])
		cnt ++;
	
	i++;
  }
  if(val)
	gpio_set_dir(WW_CHARGE_DIS_EN,0);
   return cnt;
}
#endif

#define BATTERY_MAX_VAL (4400 * 1000)
int ww_det_batt_max_vot(void)
{
  int i = 0;
  int batt_vol = 0;
  int batt_max_vol = 0;;

 
  while(i < 20){
  	
	thread_sleep(20);
	batt_vol = pm8x41_get_batt_voltage();
	
	if (batt_vol > batt_max_vol && batt_vol < BATTERY_MAX_VAL)
		batt_max_vol = batt_vol;
	i++;
  }
  dprintf(CRITICAL, " now battery voltage is %d uv.\n",batt_max_vol);
  return batt_max_vol;
}


static int ww_read_usb_id_state(struct pm8x41_mpp *mpp){

	uint8_t val = 0;
	int i = 0;
	int cnt = 0;
	
	while(i < 3){
		pm8x41_gpio_get(OTG_ID_GPIO,&val);
		if(val)
			cnt++;
		i++;
		thread_sleep(50);
	}
	
	dprintf(CRITICAL, " now USB ID status %d  cnt %d.\n",val,cnt);

	return cnt;
	

}

#define USB_CHARGE_PORT   1
#define USB_OTG_PORT      2
#define USB_STATUS_FAIL   3

static int ww_set_otg_or_charge_st(struct bq2415x_device *bq, struct pm8x41_mpp *mpp){
   int usb_id_st = -1;
   int usb_preset = -1;

	usb_id_st = ww_read_usb_id_state(mpp);
	usb_preset = ww_read_vbus_present_st(bq);
    dprintf(CRITICAL, "usb_preset: %d\n", usb_preset);
	if (usb_id_st == 0 && usb_preset == 0)
		return USB_OTG_PORT;
	else if (usb_preset >= 3)
		return USB_CHARGE_PORT;
	else 
		return  USB_STATUS_FAIL;
}


#define OTG_START_VOL (3700*1000)
#define OTG_STOP_VOL  (3450*1000)
#define PER_TWENTY_VOL   (3680*1000)
#define PER_SIXTY_VOL	 (3900*1000)


int ww_bq_i2c_device_init(int ww_mode)
{
//	uint8_t pon_reason = 0;
	int batt_vol = 0;
	int usb_cnt = 0;
	int ww_pon_key = 0;
	int cnt  = 0;
//	int led_cnt = 0;
	int batt_thermal_cnt = 0;
	uint32_t mpp4_vol = 0;
	int charge_status = 0;
	int charge_again = 0;
	
	uint8_t usb_id = -1;
	int usb_port = -1;
	int usb_id_cnt = 0;
	int usb_stop_otg = 0;
//	int led_st = 0 ;
	int first_full =0;
	struct pm8x41_mpp mpp; 


	batt_thermal_cnt = ww_read_battery_present_st();
#if 0
	if( WW_READ_BATTERY_VOL_MODE == ww_mode){

		dprintf(CRITICAL, "WW_READ_BATTERY_VOL_MODE --->\n");
		
		if(batt_thermal_cnt == BATT_MISS_ST )
			dprintf(CRITICAL, "battery miss warning --->\n");
		else{
			
			if(ww_det_restart_batt_vot(1)>= 3 ){
			//	qup_i2c_deinit(i2c_dev);
			//	free(bq);
				return 0;
			}else
				dprintf(CRITICAL, "battery voltage < 3.5v , go into shutdown charge mode--->\n");
		}
		ww_led_gpio_crtl(WW_ALL_OFF); 

	}
#endif	
	 struct bq2415x_device *bq; 
	bq = malloc(sizeof(struct bq2415x_device));

	if(!bq) {
		dprintf(CRITICAL, "bq2415x_device malloc failed\n");
		return ERR_NOT_VALID;
	}
	
	i2c_dev = qup_blsp_i2c_init(BLSP_ID_1, QUP_ID_3,
				I2C_CLK_FREQ, I2C_SRC_CLK_FREQ);
	
	if(!i2c_dev) {
		dprintf(CRITICAL, "i2c_dev init failed\n");
		free(bq);
		return ERR_NOT_VALID;
	}
//	ww_charge_ctrl_gpio_init();
	ww_bq2415x_parse_dt(bq);//config value
//	if(SY6923D_CHARGE_IC == ww_hw_type)
	bq2415x_charge_safe_init(bq);

	batt_vol = pm8x41_get_batt_voltage();
	//pm8x41_enable_mpp_as_adc(5); // FOR MPP6
	#if 0
	/*mpp1 init*/
	mpp.base = PM8x41_MMP1_BASE; 
	mpp.vin = MPP_VIN3; 
	mpp.mode = MPP_LOW; 
	pm8x41_config_input_mpp(&mpp);
	pm8x41_enable_mpp(&mpp, MPP_ENABLE); 
	#endif
	/* FOR ID PIN PMIC GPIO4*/
	 struct pm8x41_gpio gpio = {
                .direction = PM_GPIO_DIR_IN, 
                .function = PM_GPIO_FUNC_LOW, // DIG INPUT
                .vin_sel = 2,   /* VIN_2  1.8 V  */  
                .output_buffer = PM_GPIO_OUT_CMOS,
                .out_strength = PM_GPIO_OUT_DRIVE_MED,
       };

     pm8x41_gpio_config(OTG_ID_GPIO, &gpio);
	
	//mdelay(500);
	bq2415x_set_defaults(bq,batt_thermal_cnt);
	bq2415x_sysfs_show_registers(bq);

	thread_sleep(3000);
	usb_port = ww_set_otg_or_charge_st(bq, &mpp);
	dprintf(CRITICAL, "WW_BATTERY_CHARGE_MODE --->usb_port[%d]\n",usb_port);


	init_displayer("sh1106");

	if(USB_CHARGE_PORT == usb_port){
	dprintf(CRITICAL, "WW_BATTERY_CHARGE_MODE --->\n");

	while(1){
		clear_displayer();
		if(first_full == 1){
			display_battery(15);
		}
		else{
			display_battery(cnt%16);
		}
		refresh_displayer();
		usb_cnt = ww_read_vbus_present_st(bq);	
		
		if(cnt%3 == 0){
			charge_status = bq2415x_exec_command(bq, BQ2415X_CHARGE_STATUS);
			batt_thermal_cnt = ww_read_battery_present_st();
		}

		
			if( usb_cnt < 3){
				dprintf(CRITICAL,"usb remove and shutdown now,usb_st[%d]\n",usb_cnt);
				thread_sleep(1000);
				//ww_led_gpio_crtl(WW_ALL_OFF);
				shutdown_device();
			}
		
			/*press 1s reboot*/
			if( pm8x41_get_pwrkey_is_pressed()){
				ww_pon_key++;
		
				if(ww_pon_key >= 2){
				//	ww_led_gpio_crtl(WW_EX3P15_OFF); //tangzh for  turnoff led power
				//	ww_led_gpio_crtl(WW_ALL_OFF);
					reboot_device(0);
				}
			}else
				ww_pon_key = 0;
		
			if( cnt%5 == 0){
					batt_vol = pm8x41_get_batt_voltage();
					dprintf(CRITICAL, "usb_cnt[%d] charge_status[%d] batt_vol[%d] batt_thermal_vol[%d] mpp4_vol[%d]--->\n",usb_cnt,charge_status,batt_vol,batt_thermal_cnt,mpp4_vol);
				}
		
		
			if(batt_thermal_cnt == BATT_MISS_ST || batt_thermal_cnt == BATT_OVER_TEMP_ST){
				/*for disable charge*/
				if( batt_thermal_cnt == BATT_OVER_TEMP_ST){
					gpio_set(WW_CHARGE_DIS_EN,2);
					charge_status = 0;
					charge_again = 1;
				}
				
				if(( cnt % 2) == 0){
				//	ww_power_led_gpio_ctrl(PWR_ALL_OFF);
				}
				else{
			#if defined(WW_PRODUCT_SUBMODEL_MF673S)
				//	ww_power_led_gpio_ctrl(PWR_LOW_LED);
			#else
			 //		ww_power_led_gpio_ctrl(PWR_ALL_ON);
			#endif
				}
		
			}
			else{
			
				if(CHG_FULL == charge_status)
					first_full = 1;

				if(first_full)
				//	ww_power_led_gpio_ctrl(PWR_HIGH_LED);
			//	else{
		
				//	ww_power_led_gpio_ctrl(led_cnt);
		
				//	if(led_cnt	== PWR_HIGH_LED)
				//		led_cnt = 0;
				//	else 
				//		led_cnt++;
		//
			//	}
				if(1 == charge_again ){
					 gpio_set(WW_CHARGE_DIS_EN, 0);
					 charge_again = 0;
					 first_full = 0;
				}
		
			}
			
			cnt++;
			thread_sleep(1000);

	}
	}
	else if (USB_OTG_PORT == usb_port){
		
		dprintf(CRITICAL, "WW_BATTERY_OTG_MODE --->\n");
		 batt_vol = ww_det_batt_max_vot();

		if ( batt_vol >= OTG_START_VOL ){

			bq2415x_exec_command(bq,BQ2415X_BOOST_MODE_ENABLE);

			while(1){

				if(cnt%5 == 0){
					 batt_vol = ww_det_batt_max_vot();

					 if ( 0 == usb_stop_otg  && batt_vol <= OTG_STOP_VOL){
						usb_stop_otg = 1;
						bq2415x_exec_command(bq,BQ2415X_BOOST_MODE_DISABLE);
					 }
					 //else{
					//	if(PCS5005_CHARGE_IC == ww_hw_type&&0 == usb_stop_otg)
					//		bq2415x_exec_command(bq,BQ2415X_BOOST_MODE_ENABLE);
					// }
					 
				}
				usb_id = ww_read_usb_id_state(&mpp);

				if(usb_id)
					usb_id_cnt++;
				else
					usb_id_cnt = 0;

				if (2 == usb_id_cnt){
					
					bq2415x_exec_command(bq,BQ2415X_BOOST_MODE_DISABLE);
					thread_sleep(1000);
					dprintf(CRITICAL,"usb_ID remove and shutdown now,usb_st\n");
				//	ww_led_gpio_crtl(WW_ALL_OFF);
					shutdown_device();
					
				}

				/*press 1s reboot*/
				if( pm8x41_get_pwrkey_is_pressed()){
					ww_pon_key++;
			
					if(ww_pon_key >= 2){
					//	ww_led_gpio_crtl(WW_EX3P15_OFF); //tangzh for  turnoff led power
					//	ww_led_gpio_crtl(WW_ALL_OFF);
						reboot_device(0);
					}
				}else
					ww_pon_key = 0;
				#if 0
				if (usb_stop_otg){
					
					if(( cnt % 2) == 0)
				//		ww_power_led_gpio_ctrl(PWR_ALL_OFF);
					else
				//		ww_power_led_gpio_ctrl(PWR_ALL_ON);
				}else{
				#if 0
					if (batt_vol >= PER_SIXTY_VOL)
						led_cnt = PWR_HIGH_LED;
					else if ( batt_vol < PER_SIXTY_VOL && batt_vol >= PER_TWENTY_VOL )
						led_cnt = PWR_MID_LED;
					else 
						led_cnt = PWR_LOW_LED;

					if(( cnt % 2) == 0)
						ww_power_led_gpio_ctrl(PWR_ALL_OFF);
					else
						ww_power_led_gpio_ctrl(led_cnt);
				#endif
				}
				#endif
				cnt++;
				thread_sleep(1000);
			}

		} 
		else {
			
			while (1){
				//if(( cnt % 2) == 0)
				//	ww_power_led_gpio_ctrl(PWR_ALL_OFF);
				//else
			//		ww_power_led_gpio_ctrl(PWR_ALL_ON);

				usb_id = ww_read_usb_id_state(&mpp);

				if(usb_id)
					usb_id_cnt++;
				else
					usb_id_cnt = 0;

				if (2 == usb_id_cnt){
					bq2415x_exec_command(bq,BQ2415X_BOOST_MODE_DISABLE);
					thread_sleep(1000);
					dprintf(CRITICAL,"usb_ID remove and shutdown now,usb_st\n");
					//ww_led_gpio_crtl(WW_ALL_OFF);
					shutdown_device();

				}

					/*press 1s reboot*/
				if( pm8x41_get_pwrkey_is_pressed() ){

					ww_pon_key++;
			
					if(ww_pon_key >= 2){
					//	ww_led_gpio_crtl(WW_EX3P15_OFF); //tangzh for  turnoff led power
					//	ww_led_gpio_crtl(WW_ALL_OFF);
						reboot_device(0);
					}
				}else
					ww_pon_key = 0;

				cnt++;
				thread_sleep(1000);
			}

		}
	
		
		
	}
	
	return NO_ERROR;
}

