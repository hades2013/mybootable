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
//#include <mutex.h>
#include <err.h>
#include <errno-base.h>
#include <i2c_qup.h>
#include <platform.h>
#include <target.h>

#include <ww_bq2415x_charge.h>
#include "displayer.h"
#include <string.h>

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-macros=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/
#define I2C_CLK_FREQ     100000
#define I2C_SRC_CLK_FREQ 19200000
#define I2C_CLIENT_ADDR   0x1C //0x6A

#define WW_PRINTK  //printk("%s[%d]--------------->\n",__FUNCTION__,__LINE__)
#define ww_print(format, arg...)		printk(format, ##arg)


#define  SY6923D_CHARGE_IC       6
#define  PCS5005_CHARGE_IC       5
static int ww_type_hw = SY6923D_CHARGE_IC;
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
	BQ2415X_WW_GET_CHARGE_STATUS,

	
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
 

struct bq2415x_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};


struct bq2415x_device {
	
	
	
	enum bq2415x_mode reported_mode;/* mode reported by hook function */
	enum bq2415x_mode mode;		/* current configured mode */
	enum bq2415x_chip chip;
	
	
	struct qpnp_vadc_chip	*vadc_dev;
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

/* each registered chip must have unique id */
static DEFINE_IDR(bq2415x_id);

static DEFINE_MUTEX(bq2415x_id_mutex);
static DEFINE_MUTEX(bq2415x_timer_mutex);
static DEFINE_MUTEX(bq2415x_i2c_mutex);


static int volatile ww_otg_stop_vol = (3480*1000);
static int volatile ww_otg_start_vol = (3750*1000);
/**** i2c read functions ****/

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-functions=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/
extern void mdelay(unsigned msecs);

int ww_smb_i2c_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
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

int ww_smb_i2c_read_byte(uint8_t addr, uint8_t reg, uint8_t *buf)
{
	if (!buf)
		return ERR_INVALID_ARGS;

	return ww_smb_i2c_read(addr, reg, buf, 1);
}

int ww_smb_i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t val)
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

static int smb135x_read_reg(struct smb135x_chg *chip,int reg,uint8_t *val)
{
	int rc;

	rc = ww_smb_i2c_read_byte(I2C_CLIENT_ADDR, reg, val);

	return rc;
}

static int smb135x_write_reg(struct smb135x_chg *chip, int reg, uint8_t val)
{
	int rc;

	rc = ww_smb_i2c_write_byte(I2C_CLIENT_ADDR, reg, val);

	return rc;
}

static int smb135x_masked_write(struct smb135x_chg *chip, int reg,
							uint8_t mask, uint8_t val)
{
	int rc;
	uint8_t temp;


	rc = ww_smb_i2c_read_byte(I2C_CLIENT_ADDR, reg, &temp);

	if (rc) {
		dprintf(CRITICAL, "smb135x_read_reg Failed: reg=%03X, rc=%d\n", reg, rc);
		goto out;
	}
	temp &= ~mask;
	temp |= val & mask;
	rc = smb135x_write_reg(chip, reg, temp);
	if (rc) {
		dprintf(CRITICAL, "smb135x_write Failed: reg=%03X, rc=%d\n", reg, rc);
	}
out:
	return rc;
}

static int smb135x_enable_volatile_writes(struct smb135x_chg *chip)
{
	int rc;

	rc = smb135x_masked_write(chip, CMD_I2C_REG, ALLOW_VOLATILE_BIT, ALLOW_VOLATILE_BIT);
	if (rc < 0){
		dprintf(CRITICAL, "Couldn't set VOLATILE_W_PERM_BIT rc=%d\n", rc);
	}
	return rc;
}

static int smb135x_float_voltage_set(struct smb135x_chg *chip, int vfloat_mv){
	u8 temp;

	if ((vfloat_mv < MIN_FLOAT_MV) || (vfloat_mv > MAX_FLOAT_MV)) {
		dprintf(CRITICAL, "bad float voltage mv =%d asked to set\n", vfloat_mv);
		return -EINVAL;
	}

	if (vfloat_mv <= HIGH_RANGE_FLOAT_MIN_MV) {
		/* mid range */
		temp = MID_RANGE_FLOAT_MIN_VAL
			+ (vfloat_mv - MID_RANGE_FLOAT_MV_MIN)
				/ MID_RANGE_FLOAT_STEP_MV;
	}
    else if (vfloat_mv < VHIGH_RANGE_FLOAT_MIN_MV) {
		/* high range */
		temp = HIGH_RANGE_FLOAT_MIN_VAL
			+ (vfloat_mv - HIGH_RANGE_FLOAT_MIN_MV)
				/ HIGH_RANGE_FLOAT_STEP_MV;
	}
    else {
		/* very high range */
		temp = VHIGH_RANGE_FLOAT_MIN_VAL
			+ (vfloat_mv - VHIGH_RANGE_FLOAT_MIN_MV)
				/ VHIGH_RANGE_FLOAT_STEP_MV;
	}
	return ww_smb_i2c_write_byte(I2C_CLIENT_ADDR, VFLOAT_REG, temp);
}

static int smb135x_set_fastchg_current(struct smb135x_chg *chip, int current_ma){
	int i, rc, diff, best, best_diff;
	u8 reg;

	/*
	 * if there is no array loaded or if the smallest current limit is
	 * above the requested current, then do nothing
	 */
	if (chip->fastchg_current_arr_size == 0) {
		dprintf(CRITICAL, "no table loaded\n");
		return -EINVAL;
	}
    else if ((current_ma - chip->fastchg_current_table[0]) < 0) {
		dprintf(CRITICAL, "invalid current requested\n");
		return -EINVAL;
	}

	/* use the closest setting under the requested current */
	best = 0;
	best_diff = current_ma - chip->fastchg_current_table[best];

	for (i = 1; i < chip->fastchg_current_arr_size; i++) {
		diff = current_ma - chip->fastchg_current_table[i];
		if (diff >= 0 && diff < best_diff) {
			best_diff = diff;
			best = i;
		}
	}
	i = best;

	reg = i & FCC_MASK;
	rc = smb135x_masked_write(chip, CFG_1C_REG, FCC_MASK, reg);
	if (rc < 0){
		dprintf(CRITICAL, "cannot write to config c rc = %d\n", rc);
    }
	dprintf(CRITICAL, "fastchg current set to %dma\n", chip->fastchg_current_table[i]);
	return rc;
}


static int smb135x_charging_enable(struct smb135x_chg *chip, int enable)
{
	int rc;

	rc = smb135x_masked_write(chip, CMD_CHG_REG, CMD_CHG_EN, enable ? CMD_CHG_EN : 0);
	if (rc < 0) {
		dprintf(CRITICAL, "Couldn't set CHG_ENABLE_BIT enable = %d rc = %d\n", enable, rc);
		return rc;
	}

	return 0;
}

static int __smb135x_usb_suspend(struct smb135x_chg *chip, bool suspend)
{
	int rc;

	rc = smb135x_masked_write(chip, CMD_INPUT_LIMIT, USB_SHUTDOWN_BIT, suspend ? USB_SHUTDOWN_BIT : 0);
	if (rc < 0){
        dprintf(CRITICAL, "Couldn't set cfg 11 rc = %d\n", rc);
    }
	return rc;
}

static int __smb135x_dc_suspend(struct smb135x_chg *chip, bool suspend)
{
	int rc = 0;

	rc = smb135x_masked_write(chip, CMD_INPUT_LIMIT, DC_SHUTDOWN_BIT, suspend ? DC_SHUTDOWN_BIT : 0);
	if (rc < 0){
        dprintf(CRITICAL, "Couldn't set cfg 11 rc = %d\n", rc);
    }
	return rc;
}

static int smb135x_path_suspend(struct smb135x_chg *chip, enum path_type path,
						int reason, bool suspend)
{
	int rc = 0;
	int suspended;
	int *path_suspended;
	int (*func)(struct smb135x_chg *chip, bool suspend);

	//mutex_lock(&chip->path_suspend_lock);
	if (path == USB) {
		suspended = chip->usb_suspended;
		path_suspended = (int *)&chip->usb_suspended;
		func = __smb135x_usb_suspend;
	}
    else {
		suspended = chip->dc_suspended;
		path_suspended = (int *)&chip->dc_suspended;
		func = __smb135x_dc_suspend;
	}

	if (suspend == false)
		suspended &= ~reason;
	else
		suspended |= reason;

	if (*path_suspended && !suspended)
		rc = func(chip, 0);
	if (!(*path_suspended) && suspended)
		rc = func(chip, 1);

	if (rc){
        dprintf(CRITICAL, "Couldn't set/unset suspend for %s path rc = %d\n", path==USB?"usb":"dc", rc);
    }
	else{
		*path_suspended = suspended;
    }

	//mutex_unlock(&chip->path_suspend_lock);
	return rc;
}

static int __smb135x_charging(struct smb135x_chg *chip, int enable){
	int rc = 0;

	if (chip->chg_disabled_permanently) {
		dprintf(CRITICAL, "charging is disabled permanently\n");
		return -EINVAL;
	}

	rc = smb135x_charging_enable(chip, enable);
	if (rc < 0) {
		dprintf(CRITICAL, "Couldn't %s charging, rc = %d\n", enable?"enable":"disable", rc);
		return rc;
	}
	chip->chg_enabled = enable;

	/* set the suspended status */
	rc = smb135x_path_suspend(chip, DC, USER, !enable);
	if (rc < 0) {
		dprintf(CRITICAL, "Couldn't set dc suspend to %d, rc = %d\n", enable, rc);
		return rc;
	}
	rc = smb135x_path_suspend(chip, USB, USER, !enable);
	if (rc < 0) {
		dprintf(CRITICAL, "Couldn't set usb suspend to %d, rc = %d\n", enable, rc);
		return rc;
	}

	dprintf(INFO, "charging %s\n", enable?"enabled":"disabled running from batt");
	return rc;
}

static int smb135x_set_resume_threshold(struct smb135x_chg *chip, int resume_delta_mv){
	int rc;
	u8 reg;

	if (!chip->inhibit_disabled) {
		if (resume_delta_mv < 100)
			reg = CHG_INHIBIT_50MV_VAL;
		else if (resume_delta_mv < 200)
			reg = CHG_INHIBIT_100MV_VAL;
		else if (resume_delta_mv < 300)
			reg = CHG_INHIBIT_200MV_VAL;
		else
			reg = CHG_INHIBIT_300MV_VAL;

		rc = smb135x_masked_write(chip, CFG_4_REG, CHG_INHIBIT_MASK, reg);
		if (rc < 0) {
            dprintf(CRITICAL, "Couldn't set inhibit val rc = %d\n", rc);
			return rc;
		}
	}

	if (resume_delta_mv < 200){
		reg = 0;
    }
	else{
		reg = RECHARGE_200MV_BIT;
	}

	rc = smb135x_masked_write(chip, CFG_5_REG, RECHARGE_200MV_BIT, reg);
	if (rc < 0) {
		dprintf(CRITICAL, "Couldn't set recharge  rc = %d\n", rc);
		return rc;
	}
	return 0;
}

static int smb135x_set_high_usb_chg_current(struct smb135x_chg *chip, int current_ma){
	int i, rc;
	u8 usb_cur_val;

	for (i = chip->usb_current_arr_size - 1; i >= 0; i--) {
		if (current_ma >= chip->usb_current_table[i]){
			break;
        }
	}
	if (i < 0) {
        dprintf(CRITICAL, "Cannot find %dma current_table using %d\n", current_ma, CURRENT_150_MA);
		rc = smb135x_masked_write(chip, CFG_5_REG, USB_2_3_BIT, USB_2_3_BIT);
		rc |= smb135x_masked_write(chip, CMD_INPUT_LIMIT, USB_100_500_AC_MASK, USB_100_VAL);
		if (rc < 0){
			dprintf(CRITICAL, "Couldn't set %dmA rc=%d\n", CURRENT_150_MA, rc);
        }
		else{
			chip->real_usb_psy_ma = CURRENT_150_MA;
        }
		return rc;
	}

	usb_cur_val = i & USBIN_INPUT_MASK;
	rc = smb135x_masked_write(chip, CFG_C_REG, USBIN_INPUT_MASK, usb_cur_val);
	if (rc < 0) {
		dprintf(CRITICAL, "cannot write to config c rc = %d\n", rc);
		return rc;
	}

	rc = smb135x_masked_write(chip, CMD_INPUT_LIMIT, USB_100_500_AC_MASK, USB_AC_VAL);
	if (rc < 0){
		dprintf(CRITICAL, "Couldn't write cfg 5 rc = %d\n", rc);
    }
	else{
		chip->real_usb_psy_ma = chip->usb_current_table[i];
    }
	return rc;
}

/* helper to return the string of USB type */
static char *get_usb_type_name(u8 stat_5){
	int i;

    for(i=0; i<8; i++){
		if(stat_5 == (1 << i)){
			return usb_type_str[i];
        }
    }

	return "NONE";
}

static int smb135x_get_prop_batt_present(struct smb135x_chg *chip)
{
	int rc;
	u8 reg;

	rc = ww_smb_i2c_read_byte(I2C_CLIENT_ADDR, STATUS_4_REG, &reg);
	if (rc < 0)
		return 0;

	/* treat battery gone if less than 2V */
	if (reg & BATT_LESS_THAN_2V)
		return 0;

	return chip->batt_present;
}

static int smb135x_get_prop_charge_type(struct smb135x_chg *chip)
{
	int rc;
	u8 reg;
	u8 chg_type;

	rc = ww_smb_i2c_read_byte(I2C_CLIENT_ADDR, STATUS_4_REG, &reg);
	if (rc < 0)
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	chg_type = (reg & CHG_TYPE_MASK) >> CHG_TYPE_SHIFT;
	if (chg_type == BATT_NOT_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	else if (chg_type == BATT_FAST_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	else if (chg_type == BATT_PRE_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	else if (chg_type == BATT_TAPER_CHG_VAL)
		return POWER_SUPPLY_CHARGE_TYPE_TAPER;

	return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

static int smb135x_set_usb_chg_current(struct smb135x_chg *chip){
	u8 reg;
	int rc;
    int current_ma = 0;
	char *usb_type_name = "null";

	dprintf(INFO, "%s\n", __FUNCTION__);
	/* usb inserted */
	rc = ww_smb_i2c_read_byte(I2C_CLIENT_ADDR, STATUS_5_REG, &reg);
	if (rc < 0) {
		dprintf(CRITICAL, "Couldn't read status 5 rc = %d\n", rc);
		return rc;
	}

	usb_type_name = get_usb_type_name(reg);
	dprintf(INFO, "inserted %s, stat_5 = 0x%02x\n", usb_type_name, reg);

	switch(reg){
		case CDP_BIT: //USB3.0
			current_ma = CURRENT_900_MA;
            break;
		case DCP_BIT: //Adapter
			current_ma = CURRENT_1500_MA;
            break;
		case SDP_BIT: //USB2.0
			current_ma = CURRENT_500_MA;
            break;
		default:
			current_ma = CURRENT_500_MA;
            break;
    }
	dprintf(INFO, "USB current_ma = %d\n", current_ma);

	if (current_ma == 0){
		/* choose the lowest available value of 100mA */
		current_ma = CURRENT_100_MA;
    }

	if (current_ma == SUSPEND_CURRENT_MA) {
		/* force suspend bit */
		rc = smb135x_path_suspend(chip, USB, CURRENT, true);
		chip->real_usb_psy_ma = SUSPEND_CURRENT_MA;
		goto out;
	}
	if (current_ma < CURRENT_150_MA) {
		/* force 100mA */
		rc = smb135x_masked_write(chip, CFG_5_REG, USB_2_3_BIT, 0);
		rc |= smb135x_masked_write(chip, CMD_INPUT_LIMIT, USB_100_500_AC_MASK, USB_100_VAL);
		rc |= smb135x_path_suspend(chip, USB, CURRENT, false);
		chip->real_usb_psy_ma = CURRENT_100_MA;
		goto out;
	}
	/* specific current values */
	if (current_ma == CURRENT_150_MA) {
		rc = smb135x_masked_write(chip, CFG_5_REG, USB_2_3_BIT, USB_2_3_BIT);
		rc |= smb135x_masked_write(chip, CMD_INPUT_LIMIT, USB_100_500_AC_MASK, USB_100_VAL);
		rc |= smb135x_path_suspend(chip, USB, CURRENT, false);
		chip->real_usb_psy_ma = CURRENT_150_MA;
		goto out;
	}
	if (current_ma == CURRENT_500_MA) {
		rc = smb135x_masked_write(chip, CFG_5_REG, USB_2_3_BIT, 0);
		rc |= smb135x_masked_write(chip, CMD_INPUT_LIMIT, USB_100_500_AC_MASK, USB_500_VAL);
		rc |= smb135x_path_suspend(chip, USB, CURRENT, false);
		chip->real_usb_psy_ma = CURRENT_500_MA;
		goto out;
	}
	if (current_ma == CURRENT_900_MA) {
		rc = smb135x_masked_write(chip, CFG_5_REG, USB_2_3_BIT, USB_2_3_BIT);
		rc |= smb135x_masked_write(chip, CMD_INPUT_LIMIT, USB_100_500_AC_MASK, USB_500_VAL);
		rc |= smb135x_path_suspend(chip, USB, CURRENT, false);
		chip->real_usb_psy_ma = CURRENT_900_MA;
		goto out;
	}

	rc = smb135x_set_high_usb_chg_current(chip, current_ma);
	//rc |= smb135x_path_suspend(chip, USB, CURRENT, false);
out:
	if (rc < 0){
		dprintf(CRITICAL, "Couldn't set %dmA rc = %d\n", current_ma, rc);
    }
	return rc;
}

static int smb135x_set_dc_chg_current(struct smb135x_chg *chip,	int current_ma){
	int i, rc;
	u8 dc_cur_val;

	for (i = chip->dc_current_arr_size - 1; i >= 0; i--) {
		if (chip->dc_psy_ma >= chip->dc_current_table[i]){
			break;
        }
	}
	dc_cur_val = i & DCIN_INPUT_MASK;
	rc = smb135x_masked_write(chip, CFG_A_REG, DCIN_INPUT_MASK, dc_cur_val);
	if (rc < 0) {
		dprintf(CRITICAL, "Couldn't set dc charge current rc = %d\n", rc);
		return rc;
	}
	return 0;
}

static int ww_smb_parse_dt(struct bq2415x_device *chip){
	dprintf(INFO, "%s\n", __FUNCTION__);

	chip->chg_enabled = TRUE;
	chip->vfloat_mv = SMB135x_MAX_VFLOAT_MV;
    chip->safety_time = SMB135x_SAFETY_TIME;
	chip->resume_delta_mv = SMB135x_RECHARGE_MV;
	chip->iterm_ma = SMB135x_ITERM_MA;
	chip->iterm_disabled = FALSE;
	chip->inhibit_disabled = FALSE;
	chip->bms_controlled_charging = FALSE;
	chip->soft_vfloat_comp_disabled = FALSE;
	chip->fastchg_ma = SMB135x_FAST_CHG_MAX_MA;
    chip->chg_disabled_permanently = FALSE;
    chip->dc_psy_type = POWER_SUPPLY_TYPE_MAINS;
    chip->dc_psy_ma = 1500;

    //smb135x_set_current_tables 1357, by bc_1
	chip->usb_current_table = usb_current_table_smb1357_smb1358;
	chip->usb_current_arr_size = ARRAY_SIZE(usb_current_table_smb1357_smb1358);
	chip->dc_current_table = dc_current_table;
	chip->dc_current_arr_size = ARRAY_SIZE(dc_current_table);
	chip->fastchg_current_table = fastchg_current_table;
	chip->fastchg_current_arr_size = ARRAY_SIZE(fastchg_current_table);
	return 0;
}

static int smb135x_hw_init(struct smb135x_chg *chip)
{
	int rc;
	int i;
	u8 reg, mask;

	dprintf(INFO, "%s, Line: %d\n", __FUNCTION__, __LINE__);

	/*if (chip->pinctrl_state_name) {
		chip->smb_pinctrl = pinctrl_get_select(chip->dev,
						chip->pinctrl_state_name);
		if (IS_ERR(chip->smb_pinctrl)) {
			pr_err("Could not get/set %s pinctrl state rc = %ld\n",
						chip->pinctrl_state_name,
						PTR_ERR(chip->smb_pinctrl));
			return PTR_ERR(chip->smb_pinctrl);
		}
	}*/

	/*if (chip->therm_bias_vreg) {
		rc = regulator_enable(chip->therm_bias_vreg);
		if (rc) {
			pr_err("Couldn't enable therm-bias rc = %d\n", rc);
			return rc;
		}
	}*/

	/*
	 * Enable USB data line pullup regulator this is needed for the D+
	 * line to be at proper voltage for HVDCP charger detection.
	 */
	/*if (chip->usb_pullup_vreg) {
		rc = regulator_enable(chip->usb_pullup_vreg);
		if (rc) {
			pr_err("Unable to enable data line pull-up regulator rc=%d\n",
					rc);
			if (chip->therm_bias_vreg)
				regulator_disable(chip->therm_bias_vreg);
			return rc;
		}
	}*/

	chip->batt_present = smb135x_get_prop_batt_present(chip);

	rc = smb135x_enable_volatile_writes(chip);
	if (rc < 0) {
		dprintf(CRITICAL, "Couldn't configure for volatile rc = %d\n", rc);
		return rc;
	}

	/*
	 * force using current from the register i.e. ignore auto
	 * power source detect (APSD) mA ratings
	 */
	mask = USE_REGISTER_FOR_CURRENT;
	if (chip->workaround_flags & WRKARND_USB100_BIT){
		reg = 0;
    }
	else{
		/* this ignores APSD results */
		reg = USE_REGISTER_FOR_CURRENT;
    }
	rc = smb135x_masked_write(chip, CMD_INPUT_LIMIT, mask, 0);
	if (rc < 0) {
		dprintf(CRITICAL, "Couldn't set input limit cmd rc=%d\n", rc);
		return rc;
	}

	rc = smb135x_masked_write(chip, CFG_E_REG, POLARITY_100_500_BIT | USB_CTRL_BY_PIN_BIT, POLARITY_100_500_BIT);
	if (rc < 0) {
		dprintf(CRITICAL, "Couldn't set usbin cfg rc=%d\n", rc);
		return rc;
	}

	//Disable HVDCP
	rc = smb135x_masked_write(chip, CFG_E_REG, HVDCP_ENABLE_BIT, 0);

	/*
	 * set chg en by cmd register, set chg en by writing bit 1,
	 * enable auto pre to fast, enable current termination, enable
	 * auto recharge, enable chg inhibition based on the dt flag
	 */
	rc = smb135x_masked_write(chip, CFG_14_REG,	CHG_EN_BY_PIN_BIT | CHG_EN_ACTIVE_LOW_BIT | PRE_TO_FAST_REQ_CMD_BIT
        		| DISABLE_CURRENT_TERM_BIT | DISABLE_AUTO_RECHARGE_BIT | EN_CHG_INHIBIT_BIT, 0);
	if (rc < 0) {
		dprintf(CRITICAL, "Couldn't set cfg 14 rc=%d\n", rc);
		return rc;
	}

	/* control USB suspend via command bits */
	rc = smb135x_masked_write(chip, USBIN_DCIN_CFG_REG,	USBIN_SUSPEND_VIA_COMMAND_BIT, USBIN_SUSPEND_VIA_COMMAND_BIT);

	/* set the float voltage */
	if (chip->vfloat_mv != -EINVAL) {
		rc = smb135x_float_voltage_set(chip, chip->vfloat_mv);
		if (rc < 0) {
			dprintf(CRITICAL, "Couldn't set float voltage rc = %d\n", rc);
			return rc;
		}
	}

	/* set iterm */
	if (chip->iterm_ma != -EINVAL) {
		if (chip->iterm_disabled) {
			dprintf(CRITICAL, "Error: Both iterm_disabled and iterm_ma set\n");
			rc = -EINVAL;
			return rc;
		}
        else {
			if (chip->iterm_ma <= 50)
				reg = CHG_ITERM_50MA;
			else if (chip->iterm_ma <= 100)
				reg = CHG_ITERM_100MA;
			else if (chip->iterm_ma <= 150)
				reg = CHG_ITERM_150MA;
			else if (chip->iterm_ma <= 200)
				reg = CHG_ITERM_200MA;
			else if (chip->iterm_ma <= 250)
				reg = CHG_ITERM_250MA;
			else if (chip->iterm_ma <= 300)
				reg = CHG_ITERM_300MA;
			else if (chip->iterm_ma <= 500)
				reg = CHG_ITERM_500MA;
			else
				reg = CHG_ITERM_600MA;

			rc = smb135x_masked_write(chip, CFG_3_REG, CHG_ITERM_MASK, reg);
			if (rc) {
				dprintf(CRITICAL, "Couldn't set iterm rc = %d\n", rc);
				return rc;
			}

			rc = smb135x_masked_write(chip, CFG_14_REG,	DISABLE_CURRENT_TERM_BIT, 0);
			if (rc) {
				dprintf(CRITICAL, "Couldn't enable iterm rc = %d\n", rc);
				return rc;
			}
		}
	}
    else if(chip->iterm_disabled){
		rc = smb135x_masked_write(chip, CFG_14_REG,	DISABLE_CURRENT_TERM_BIT, DISABLE_CURRENT_TERM_BIT);
		if (rc) {
			dprintf(CRITICAL, "Couldn't set iterm rc = %d\n", rc);
			return rc;
		}
	}

	/* set the safety time voltage */
	if (chip->safety_time != -EINVAL) {
		if (chip->safety_time == 0) {
			/* safety timer disabled */
			reg = 1 << SAFETY_TIME_EN_SHIFT;
			rc = smb135x_masked_write(chip, CFG_16_REG, SAFETY_TIME_EN_BIT, reg);
			if (rc < 0) {
				dprintf(CRITICAL, "Couldn't disable safety timer rc = %d\n", rc);
				return rc;
			}
		}
        else {
			for (i=0; i < (int)ARRAY_SIZE(chg_time); i++) {
				if (chip->safety_time <= chg_time[i]) {
					reg = i << SAFETY_TIME_MINUTES_SHIFT;
					break;
				}
			}
			rc = smb135x_masked_write(chip, CFG_16_REG,	SAFETY_TIME_EN_BIT|SAFETY_TIME_MINUTES_MASK, reg);
			if(rc < 0){
				dprintf(CRITICAL, "Couldn't set safety timer rc = %d\n", rc);
				return rc;
			}
		}
	}

	/* battery missing detection */
	rc = smb135x_masked_write(chip, CFG_19_REG,	BATT_MISSING_ALGO_BIT | BATT_MISSING_THERM_BIT,
						chip->bmd_algo_disabled ? BATT_MISSING_THERM_BIT : BATT_MISSING_ALGO_BIT);
	if (rc < 0) {
		dprintf(CRITICAL, "Couldn't set batt_missing config = %d\n", rc);
		return rc;
	}

	/* set maximum fastchg current */
	if (chip->fastchg_ma != -EINVAL) {
		rc = smb135x_set_fastchg_current(chip, chip->fastchg_ma);
		if (rc < 0) {
			dprintf(CRITICAL, "Couldn't set fastchg current = %d\n", rc);
			return rc;
		}
	}

	//enable 5V HVDCP adapter support
	/*rc = smb135x_masked_write(chip, CFG_E_REG, HVDCP_5_9_BIT, 0);
	if (rc < 0) {
		dprintf(CRITICAL, "Couldn't request for 5 or 9V rc=%d\n", rc);
		return rc;
	}*/

	__smb135x_charging(chip, chip->chg_enabled);

	/* interrupt enabling - active low */
	if (1/*chip->client->irq*/) {
		mask = CHG_STAT_IRQ_ONLY_BIT | CHG_STAT_ACTIVE_HIGH_BIT	| CHG_STAT_DISABLE_BIT;
		reg = CHG_STAT_IRQ_ONLY_BIT;
		rc = smb135x_masked_write(chip, CFG_17_REG, mask, reg);
		if (rc < 0) {
			dprintf(CRITICAL, "Couldn't set irq config rc = %d\n", rc);
			return rc;
		}

		/* enabling only interesting interrupts */
        rc = ww_smb_i2c_write_byte(I2C_CLIENT_ADDR, IRQ_CFG_REG,
			IRQ_BAT_HOT_COLD_HARD_BIT
			| IRQ_BAT_HOT_COLD_SOFT_BIT
			| IRQ_OTG_OVER_CURRENT_BIT
			| IRQ_INTERNAL_TEMPERATURE_BIT
			| IRQ_USBIN_UV_BIT);

        rc |= ww_smb_i2c_write_byte(I2C_CLIENT_ADDR, IRQ2_CFG_REG,
			IRQ2_SAFETY_TIMER_BIT
			| IRQ2_CHG_ERR_BIT
			| IRQ2_CHG_PHASE_CHANGE_BIT
			| IRQ2_POWER_OK_BIT
			| IRQ2_BATT_MISSING_BIT
			| IRQ2_VBAT_LOW_BIT);

		rc |= ww_smb_i2c_write_byte(I2C_CLIENT_ADDR, IRQ3_CFG_REG, IRQ3_SRC_DETECT_BIT
				| IRQ3_DCIN_UV_BIT | IRQ3_RID_DETECT_BIT);
		if (rc < 0) {
			dprintf(CRITICAL, "Couldn't set irq enable rc = %d\n", rc);
			return rc;
		}
	}

	/* resume threshold */ //bc_1 >> if voltage is less than resume_delta_mv, no start charge
	if (chip->resume_delta_mv != -EINVAL) {
		smb135x_set_resume_threshold(chip, chip->resume_delta_mv);
	}

	/* DC path current settings */
	if (chip->dc_psy_type != -EINVAL) {
		rc = smb135x_set_dc_chg_current(chip, chip->dc_psy_ma);
		if (rc < 0) {
			dprintf(CRITICAL, "Couldn't set dc charge current rc = %d\n", rc);
			return -1;
		}
	}

	/*
	 * on some devices the battery is powered via external sources which
	 * could raise its voltage above the float voltage. smb135x chips go
	 * in to reverse boost in such a situation and the workaround is to
	 * disable float voltage compensation (note that the battery will appear
	 * hot/cold when powered via external source).
	 */

	if (chip->soft_vfloat_comp_disabled) {
		mask = HOT_SOFT_VFLOAT_COMP_EN_BIT | COLD_SOFT_VFLOAT_COMP_EN_BIT;
		rc = smb135x_masked_write(chip, CFG_1A_REG, mask, 0);
		if(rc < 0){
			dprintf(CRITICAL, "Couldn't disable soft vfloat rc = %d\n", rc);
			return rc;
		}
	}

	/*
	 * Command mode for OTG control. This gives us RID interrupts but keeps
	 * enabling the 5V OTG via i2c register control
	 */
	rc = smb135x_masked_write(chip, USBIN_OTG_REG, OTG_CNFG_MASK, OTG_CNFG_COMMAND_CTRL);
	if(rc < 0){
		dprintf(CRITICAL, "Couldn't write to otg cfg reg rc = %d\n", rc);
		return rc;
	}
	return 0;
}

static void dump_regs(struct smb135x_chg *chip){
	int rc;
	u8 reg;
	u8 addr;

	for (addr = 0; addr <= LAST_CNFG_REG; addr++) {
        rc = ww_smb_i2c_read_byte(I2C_CLIENT_ADDR, addr, &reg);
		if (rc < 0){
            dprintf(CRITICAL, "Couldn't read 0x%02x rc = %d\n", addr, rc);
        }
		else{
            dprintf(CRITICAL, "0x%02x = 0x%02x\n", addr, reg);
        }
	}

	for (addr = FIRST_STATUS_REG; addr <= LAST_STATUS_REG; addr++) {
		rc = ww_smb_i2c_read_byte(I2C_CLIENT_ADDR, addr, &reg);
		if (rc < 0){
            dprintf(CRITICAL, "Couldn't read 0x%02x rc = %d\n", addr, rc);
        }
		else{
            dprintf(CRITICAL, "0x%02x = 0x%02x\n", addr, reg);
        }
	}

	for (addr = FIRST_CMD_REG; addr <= LAST_CMD_REG; addr++) {
		rc = ww_smb_i2c_read_byte(I2C_CLIENT_ADDR, addr, &reg);
		if (rc < 0){
            dprintf(CRITICAL, "Couldn't read 0x%02x rc = %d\n", addr, rc);
        }
		else{
            dprintf(CRITICAL, "0x%02x = 0x%02x\n", addr, reg);
        }
	}
}

static int ww_smb135x_batt_miss_st(void){
	uint8_t val=0xff;
	int ret = 0;
	int batt_st = 0;

	ret = ww_smb_i2c_read_byte(I2C_CLIENT_ADDR, IRQ_B_REG,&val);
	if(ret){
		dprintf(CRITICAL, "read IRQ_B_REG[%x] ret[%d]--->\n", IRQ_B_REG, ret);
	}
    else{
		 batt_st = val & IRQ_B_BATT_MISSING_BIT;
		 dprintf(CRITICAL, "read IRQ_B_REG[%x] val[%x] batt_miss[%d]--->\n", IRQ_B_REG, val, batt_st);
	}

	return batt_st;
}

static int vol_to_percent(int vol, int charge_st){
    int per = 0;

	if (vol <= VOL_PER_0){
		per = 0;
    }
	else if(vol <= VOL_PER_5){
		per = (vol-VOL_PER_0) * 5 / (VOL_PER_5-VOL_PER_0);
    }
	else if(vol <= VOL_PER_10){
		per = (vol-VOL_PER_5) * 5 / (VOL_PER_10-VOL_PER_5) + 5;
    }
	else if(vol <= VOL_PER_20){
		per = (vol-VOL_PER_10) * 10 / (VOL_PER_20-VOL_PER_10) + 10;
    }
	else if(vol <= VOL_PER_30){
		per = (vol-VOL_PER_20) * 10 / (VOL_PER_30-VOL_PER_20) + 20;
    }
	else if(vol <= VOL_PER_40){
		per = (vol-VOL_PER_30) * 10 / (VOL_PER_40-VOL_PER_30) + 30;
    }
	else if(vol <= VOL_PER_50){
		per = (vol-VOL_PER_40) * 10 / (VOL_PER_50-VOL_PER_40) + 40;
    }
	else if(vol <= VOL_PER_60){
		per = (vol-VOL_PER_50) * 10 / (VOL_PER_60-VOL_PER_50) + 50;
    }
	else if(vol <= VOL_PER_70){
		per = (vol-VOL_PER_60) * 10 / (VOL_PER_70-VOL_PER_60) + 60;
    }
	else if(vol <= VOL_PER_80){
		per = (vol-VOL_PER_70) * 10 / (VOL_PER_80-VOL_PER_70) + 70;
    }
	else if(vol <= VOL_PER_90){
		per = (vol-VOL_PER_80) * 10 / (VOL_PER_90-VOL_PER_80) + 80;
    }
	else if(vol <= VOL_PER_99){
		per = (vol-VOL_PER_90) * 9 / (VOL_PER_99-VOL_PER_90) + 90;
    }
	else{
        if(charge_st == CHG_FULL){
			per = 100;
        }
        else{
			per = 99;
        }
    }

	return per;
}

int ww_bq_i2c_device_init(int ww_mode){
	int i =0;
	uint8_t val=0xff;
	int ret = -1;
	int cnt = 0;
	int hold_off = 0;
	int iterm_ma_st = 0;
	int usb_st = 0;
	int batt_vol = 0;
	int batt_vol_old = 0;
	int charge_st = 0;
	uint8_t ww_pon_key = 0;
	int hot_st = 0;
	//int charge_type = 0;
	int cold_st = 0;
	int batt_miss = 0;
	int hold_cnt = 0;
    uint8_t source_detect = 0;
    int source_change_cnt = 0;
    int batt_percent = 0;

    dprintf(INFO, "\n%s\n", __FUNCTION__);

	struct bq2415x_device *bq2415x_chip;
	bq2415x_chip = malloc(sizeof(struct bq2415x_device);
	if(!bq2415x_chip){
		dprintf(CRITICAL, "bq2415x_chip malloc failed\n");
		return ERR_NOT_VALID;
	}

	i2c_dev = qup_blsp_i2c_init(BLSP_ID_1, QUP_ID_2, I2C_CLK_FREQ, I2C_SRC_CLK_FREQ);
	if(!i2c_dev) {
		dprintf(CRITICAL, "i2c_dev init failed\n");
		free(bq2415x_chip);
		return ERR_NOT_VALID;
	}

	if(ww_mode == WW_BATTERY_CHARGE_MODE){
		dprintf(INFO, "\nshutdown charge starting --->\n");
    }

    ww_smb_i2c_read_byte(I2C_CLIENT_ADDR, BQ2415X_REG_STATUS, &source_detect);
    dprintf(INFO, "BQ2415X_REG_STATUS[%x] %x\n", BQ2415X_REG_STATUS, source_detect);

	ww_smb_parse_dt(bq2415x_device); //config value
	smb135x_hw_init(bq2415x_device);
	smb135x_set_usb_chg_current(smb135x_chip);
	dump_regs(smb135x_chip);

	if((source_detect & CDP_BIT) == CDP_BIT){ //for cdp special treatment
		for(i=0; i<10; i++){
			ret = ww_smb_i2c_read_byte(I2C_CLIENT_ADDR, IRQ_E_REG, &val);
			if(ret){
				dprintf(CRITICAL, "read IRQ_E_REG failed\n");
			}
            else{
				dprintf(INFO, "read IRQ_E_REG[%x] val[%x]--->\n", IRQ_E_REG, val);
				usb_st = !(val & IRQ_E_USB_OV_BIT) && !(val & IRQ_E_USB_UV_BIT);
            }
            if(!usb_st){
				source_change_cnt++;
            }
            if(source_change_cnt >= 4){
                dprintf(INFO, "CDP: usb remove and shutdown now\n");
			//	shutdown_device();
            }
            thread_sleep(1000);
        }
    }

	init_displayer("sh1106");
	while(1){
		clear_displayer();
		display_battery(cnt%16);
		refresh_displayer();
		if((cnt % 3) == 0){
            dprintf(INFO, "\n");
            //STATUS_4_REG
			ret = ww_smb_i2c_read_byte(I2C_CLIENT_ADDR, STATUS_4_REG, &val);
			if(ret){
				dprintf(CRITICAL, "read STATUS_4_REG failed\n");
			}
            else{
				dprintf(INFO, "read STATUS_4_REG[%x] val[%x]--->\n", STATUS_4_REG, val);
				hold_off = val & CHG_HOLD_OFF_BIT;
				//charge_type = val & CHG_TYPE_MASK;
			}

            //IRQ_C_REG
			ret = ww_smb_i2c_read_byte(I2C_CLIENT_ADDR, IRQ_C_REG, &val);
			if(ret){
				dprintf(CRITICAL, "read IRQ_C_REG failed\n");
			}
            else{
				dprintf(INFO, "read IRQ_C_REG[%x] val[%x]--->\n",IRQ_C_REG,val);
				iterm_ma_st = val & IRQ_C_TERM_BIT;
            }

			//IRQ_E_REG
			ret = ww_smb_i2c_read_byte(I2C_CLIENT_ADDR, IRQ_E_REG,&val);
			if(ret){
				dprintf(CRITICAL, "read IRQ_E_REG failed\n");
			}
            else{
				dprintf(INFO, "read IRQ_E_REG[%x] val[%x]--->\n", IRQ_E_REG, val);
				usb_st = !(val & IRQ_E_USB_OV_BIT) && !(val & IRQ_E_USB_UV_BIT);
            }

			batt_vol = pm8x41_get_batt_voltage();
			if(batt_vol > batt_vol_old){
			batt_vol_old = batt_vol;
			}
			else{
			batt_vol = batt_vol_old;
			}

			batt_percent = vol_to_percent((batt_vol+500)/1000, charge_st);
            dprintf(INFO, "batt_percent: %d\n", batt_percent);
		}


		if((cnt % 10) == 0){
			/*monitor THERM */
			ret = ww_smb_i2c_read_byte(I2C_CLIENT_ADDR, IRQ_A_REG, &val);
			if(ret){
				dprintf(CRITICAL, "read IRQ_A_REG[%x] ret[%d]--->\n", IRQ_A_REG, ret);
			}
            else{
				hot_st = (val & IRQ_A_HOT_HARD_BIT) || (val & IRQ_A_HOT_SOFT_BIT);
				cold_st = (val & IRQ_A_COLD_HARD_BIT) || (val & IRQ_A_COLD_SOFT_BIT);
				dprintf(INFO, "read IRQ_A_REG[%x] val[%x] hot_st[%d] cold_st[%d]--->\n", IRQ_A_REG, val, hot_st, cold_st);
			}
			batt_miss = ww_smb135x_batt_miss_st();
		}

		if(charge_st == 0){
			if(iterm_ma_st){
				charge_st = CHG_FULL;
			}
            else if(hold_off && (batt_vol>(SMB135x_MAX_VFLOAT_MV*1000))){
				hold_cnt++;
				if(hold_cnt >= 8){
					charge_st = CHG_FULL;
                }
			}
            else{
			     hold_cnt = 0;
            }
		}

        if(usb_st == 0){
			dprintf(INFO, "usb remove and shutdown now, usb_st[%d]\n", usb_st);
			//ww_led_gpio_crtl(WW_ALL_OFF);
			shutdown_device();
		}

		/*press 1s reboot*/
        ret = pm8x41_get_pwrkey_is_pressed();
		if(ret){
			ww_pon_key++;
			if(ww_pon_key >= 2){
				//ww_led_gpio_crtl(WW_EX3P15_OFF); //tangzh for  turnoff led power
				//ww_led_gpio_crtl(WW_ALL_OFF);
				reboot_device(0);
			}
		}
        else{
			ww_pon_key = 0;
		}

		dprintf(INFO, "usb_st[%d] charge_st[%d] pon_reason[%d] batt_vol[%d] hold_cnt[%d]--->\n",
            	usb_st, charge_st, ww_pon_key, batt_vol, hold_cnt);

		if(hot_st || cold_st || batt_miss){
			if((cnt % 2) == 0){
				//ww_power_led_gpio_ctrl(PWR_ALL_OFF);
			}
			else{
				//ww_power_led_gpio_ctrl(PWR_ALL_ON);
			}

		}
        else{
			if(charge_st == CHG_FULL){
				//ww_power_led_gpio_ctrl(PWR_ALL_ON);
			}
			else{
				//ww_power_led_gpio_ctrl(led_cnt);
				/*if(led_cnt == PWR_HIGH_LED){
					led_cnt = 0;
                }
				else{
					led_cnt++;
                }*/
			}
		}

		cnt++;
		thread_sleep(1000);
	}
	return NO_ERROR;
}

