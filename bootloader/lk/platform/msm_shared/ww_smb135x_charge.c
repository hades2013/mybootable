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

#include <ww_smb135x_charge.h>
#include "displayer.h"
#include <string.h>

/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-macros=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/
#define I2C_CLK_FREQ     100000
#define I2C_SRC_CLK_FREQ 19200000
#define I2C_CLIENT_ADDR   0x1C //0x6A

/* Mask/Bit helpers */
#define _SMB135X_MASK(BITS, POS) ((unsigned char)(((1 << (BITS)) - 1) << (POS)))
#define SMB135X_MASK(LEFT_BIT_POS, RIGHT_BIT_POS) \
		_SMB135X_MASK((LEFT_BIT_POS) - (RIGHT_BIT_POS) + 1, \
				(RIGHT_BIT_POS))

/* Config registers */
#define CFG_3_REG			0x03
#define CHG_ITERM_50MA			0x08
#define CHG_ITERM_100MA			0x10
#define CHG_ITERM_150MA			0x18
#define CHG_ITERM_200MA			0x20
#define CHG_ITERM_250MA			0x28
#define CHG_ITERM_300MA			0x00
#define CHG_ITERM_500MA			0x30
#define CHG_ITERM_600MA			0x38
#define CHG_ITERM_MASK			SMB135X_MASK(5, 3)

#define CFG_4_REG			0x04
#define CHG_INHIBIT_MASK		SMB135X_MASK(7, 6)
#define CHG_INHIBIT_50MV_VAL		0x00
#define CHG_INHIBIT_100MV_VAL		0x40
#define CHG_INHIBIT_200MV_VAL		0x80
#define CHG_INHIBIT_300MV_VAL		0xC0

#define CFG_5_REG			0x05
#define RECHARGE_200MV_BIT		BIT(2)
#define USB_2_3_BIT			BIT(5)

#define CFG_A_REG			0x0A
#define DCIN_INPUT_MASK			SMB135X_MASK(4, 0)

#define CFG_C_REG			0x0C
#define USBIN_INPUT_MASK		SMB135X_MASK(4, 0)
#define USBIN_ADAPTER_ALLOWANCE_MASK	SMB135X_MASK(7, 5)
#define ALLOW_5V_ONLY			0x00
#define ALLOW_5V_OR_9V			0x20
#define ALLOW_5V_TO_9V			0x40
#define ALLOW_9V_ONLY			0x60

#define CFG_D_REG			0x0D

#define CFG_E_REG			0x0E
#define POLARITY_100_500_BIT		BIT(2)
#define USB_CTRL_BY_PIN_BIT		BIT(1)
#define HVDCP_5_9_BIT			BIT(4)
#define HVDCP_ENABLE_BIT		BIT(3)

#define CFG_11_REG			0x11
#define PRIORITY_BIT			BIT(7)
#define AUTO_SRC_DET_EN_BIT			BIT(0)

#define USBIN_DCIN_CFG_REG		0x12
#define USBIN_SUSPEND_VIA_COMMAND_BIT	BIT(6)

#define CFG_14_REG			0x14
#define CHG_EN_BY_PIN_BIT			BIT(7)
#define CHG_EN_ACTIVE_LOW_BIT		BIT(6)
#define PRE_TO_FAST_REQ_CMD_BIT		BIT(5)
#define DISABLE_CURRENT_TERM_BIT	BIT(3)
#define DISABLE_AUTO_RECHARGE_BIT	BIT(2)
#define EN_CHG_INHIBIT_BIT		BIT(0)

#define CFG_16_REG			0x16
#define SAFETY_TIME_EN_BIT		BIT(5)
#define SAFETY_TIME_EN_SHIFT		5
#define SAFETY_TIME_MINUTES_MASK	SMB135X_MASK(3, 2)
#define SAFETY_TIME_MINUTES_SHIFT	2

#define CFG_17_REG			0x17
#define CHG_STAT_DISABLE_BIT		BIT(0)
#define CHG_STAT_ACTIVE_HIGH_BIT	BIT(1)
#define CHG_STAT_IRQ_ONLY_BIT		BIT(4)

#define CFG_19_REG			0x19
#define BATT_MISSING_ALGO_BIT		BIT(2)
#define BATT_MISSING_THERM_BIT		BIT(1)

#define CFG_1A_REG			0x1A
#define HOT_SOFT_VFLOAT_COMP_EN_BIT	BIT(3)
#define COLD_SOFT_VFLOAT_COMP_EN_BIT	BIT(2)

#define VFLOAT_REG			0x1E

#define VERSION1_REG			0x2A
#define VERSION1_MASK			SMB135X_MASK(7,	6)
#define VERSION1_SHIFT			6
#define VERSION2_REG			0x32
#define VERSION2_MASK			SMB135X_MASK(1,	0)
#define VERSION3_REG			0x34

/* Irq Config registers */
#define IRQ_CFG_REG			0x07
#define IRQ_BAT_HOT_COLD_HARD_BIT	BIT(7)
#define IRQ_BAT_HOT_COLD_SOFT_BIT	BIT(6)
#define IRQ_OTG_OVER_CURRENT_BIT	BIT(4)
#define IRQ_USBIN_UV_BIT		BIT(2)
#define IRQ_INTERNAL_TEMPERATURE_BIT	BIT(0)

#define IRQ2_CFG_REG			0x08
#define IRQ2_SAFETY_TIMER_BIT		BIT(7)
#define IRQ2_CHG_ERR_BIT		BIT(6)
#define IRQ2_CHG_PHASE_CHANGE_BIT	BIT(4)
#define IRQ2_CHG_INHIBIT_BIT		BIT(3)
#define IRQ2_POWER_OK_BIT		BIT(2)
#define IRQ2_BATT_MISSING_BIT		BIT(1)
#define IRQ2_VBAT_LOW_BIT		BIT(0)

#define IRQ3_CFG_REG			0x09
#define IRQ3_RID_DETECT_BIT		BIT(4)
#define IRQ3_SRC_DETECT_BIT		BIT(2)
#define IRQ3_DCIN_UV_BIT		BIT(0)

#define USBIN_OTG_REG			0x0F
#define OTG_CNFG_MASK			SMB135X_MASK(3,	2)
#define OTG_CNFG_PIN_CTRL		0x04
#define OTG_CNFG_COMMAND_CTRL		0x08
#define OTG_CNFG_AUTO_CTRL		0x0C

/* Command Registers */
#define CMD_I2C_REG			0x40
#define ALLOW_VOLATILE_BIT		BIT(6)

#define CMD_INPUT_LIMIT			0x41
#define USB_SHUTDOWN_BIT		BIT(6)
#define DC_SHUTDOWN_BIT			BIT(5)
#define USE_REGISTER_FOR_CURRENT	BIT(2)
#define USB_100_500_AC_MASK		SMB135X_MASK(1, 0)
#define USB_100_VAL			0x02
#define USB_500_VAL			0x00
#define USB_AC_VAL			0x01

#define CMD_CHG_REG			0x42
#define CMD_CHG_EN			BIT(1)
#define OTG_EN				BIT(0)

/* Status registers */
#define STATUS_1_REG			0x47
#define USING_USB_BIT			BIT(1)
#define USING_DC_BIT			BIT(0)

#define STATUS_4_REG			0x4A
#define BATT_NET_CHG_CURRENT_BIT	BIT(7)
#define BATT_LESS_THAN_2V		BIT(4)
#define CHG_HOLD_OFF_BIT		BIT(3)
#define CHG_TYPE_MASK			SMB135X_MASK(2, 1)
#define CHG_TYPE_SHIFT			1
#define BATT_NOT_CHG_VAL		0x0
#define BATT_PRE_CHG_VAL		0x1
#define BATT_FAST_CHG_VAL		0x2
#define BATT_TAPER_CHG_VAL		0x3
#define CHG_EN_BIT			BIT(0)

#define STATUS_5_REG			0x4B
#define CDP_BIT				BIT(7)
#define DCP_BIT				BIT(6)
#define OTHER_BIT			BIT(5)
#define SDP_BIT				BIT(4)
#define ACA_A_BIT			BIT(3)
#define ACA_B_BIT			BIT(2)
#define ACA_C_BIT			BIT(1)
#define ACA_DOCK_BIT			BIT(0)

#define STATUS_6_REG			0x4C
#define RID_FLOAT_BIT			BIT(3)
#define RID_A_BIT			BIT(2)
#define RID_B_BIT			BIT(1)
#define RID_C_BIT			BIT(0)

#define STATUS_7_REG			0x4D

#define STATUS_8_REG			0x4E
#define USBIN_9V			BIT(5)
#define USBIN_UNREG			BIT(4)
#define USBIN_LV			BIT(3)
#define DCIN_9V				BIT(2)
#define DCIN_UNREG			BIT(1)
#define DCIN_LV				BIT(0)

#define STATUS_9_REG			0x4F
#define REV_MASK			SMB135X_MASK(3, 0)

/* Irq Status registers */
#define IRQ_A_REG			0x50
#define IRQ_A_HOT_HARD_BIT		BIT(6)
#define IRQ_A_COLD_HARD_BIT		BIT(4)
#define IRQ_A_HOT_SOFT_BIT		BIT(2)
#define IRQ_A_COLD_SOFT_BIT		BIT(0)

#define IRQ_B_REG			0x51
#define IRQ_B_BATT_TERMINAL_BIT		BIT(6)
#define IRQ_B_BATT_MISSING_BIT		BIT(4)
#define IRQ_B_VBAT_LOW_BIT		BIT(2)
#define IRQ_B_TEMPERATURE_BIT		BIT(0)

#define IRQ_C_REG			0x52
#define IRQ_C_TERM_BIT			BIT(0)
#define IRQ_C_FASTCHG_BIT		BIT(6)

#define IRQ_D_REG			0x53
#define IRQ_D_TIMEOUT_BIT		BIT(2)

#define IRQ_E_REG			0x54
#define IRQ_E_DC_OV_BIT			BIT(6)
#define IRQ_E_DC_UV_BIT			BIT(4)
#define IRQ_E_USB_OV_BIT		BIT(2)
#define IRQ_E_USB_UV_BIT		BIT(0)

#define IRQ_F_REG			0x55
#define IRQ_F_POWER_OK_BIT		BIT(0)

#define IRQ_G_REG			0x56
#define IRQ_G_SRC_DETECT_BIT		BIT(6)


/* constants */
#define USB2_MIN_CURRENT_MA		100
#define USB2_MAX_CURRENT_MA		500
#define USB3_MIN_CURRENT_MA		150
#define USB3_MAX_CURRENT_MA		900
#define AC_CHG_CURRENT_MASK		0x70
#define AC_CHG_CURRENT_SHIFT		4
#define SMB135x_IRQ_REG_COUNT		6
#define SMB135x_FAST_CHG_MIN_MA		200
#define SMB135x_FAST_CHG_MAX_MA		1500
#define SMB135x_FAST_CHG_SHIFT		5
#define SMB_FAST_CHG_CURRENT_MASK	0xE0
#define SMB135x_DEFAULT_BATT_CAPACITY	50
#define SMB135x_BATT_GOOD_THRE_2P5	0x1


#define SMB135x_MAX_VFLOAT_MV        4350
#define SMB135x_SAFETY_TIME           1536
#define SMB135x_RECHARGE_MV           200
#define SMB135x_ITERM_MA             100

#define MIN_FLOAT_MV	3600
#define MAX_FLOAT_MV	4500

#define MID_RANGE_FLOAT_MV_MIN		3600
#define MID_RANGE_FLOAT_MIN_VAL		0x05
#define MID_RANGE_FLOAT_STEP_MV		20

#define HIGH_RANGE_FLOAT_MIN_MV		4340
#define HIGH_RANGE_FLOAT_MIN_VAL	0x2A
#define HIGH_RANGE_FLOAT_STEP_MV	10

#define VHIGH_RANGE_FLOAT_MIN_MV	4400
#define VHIGH_RANGE_FLOAT_MIN_VAL	0x2E
#define VHIGH_RANGE_FLOAT_STEP_MV	20

#define FCC_MASK			SMB135X_MASK(5, 0)
#define CFG_1C_REG			0x1C

#define CURRENT_100_MA		100
#define CURRENT_150_MA		150
#define CURRENT_500_MA		500
#define CURRENT_900_MA		900
#define CURRENT_1500_MA		1500
#define SUSPEND_CURRENT_MA	2

#define LAST_CNFG_REG	0x1F
#define FIRST_STATUS_REG	0x46
#define LAST_STATUS_REG		0x56
#define FIRST_CMD_REG	0x40
#define LAST_CMD_REG	0x42

#define CHG_FULL        2
#define CHG_USB_PRESENT 1

#ifndef u8
#define u8 unsigned char
#endif

#ifndef u32
#define u32 unsigned int
#endif

#define VOL_PER_0	3000
#define VOL_PER_5	3450
#define VOL_PER_10	3600
#define VOL_PER_20	3680
#define VOL_PER_30	3710
#define VOL_PER_40	3760
#define VOL_PER_50	3830
#define VOL_PER_60	3900
#define VOL_PER_70	3980
#define VOL_PER_80	4080
#define VOL_PER_90	4150
#define VOL_PER_99	4290
#define VOL_PER_100	4350


/*-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-variables=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-*/
static struct qup_i2c_dev *i2c_dev;

enum {
	WRKARND_USB100_BIT = BIT(0),
	WRKARND_APSD_FAIL = BIT(1),
};

enum {
	REV_1 = 1,	/* Rev 1.0 */
	REV_1_1 = 2,	/* Rev 1.1 */
	REV_2 = 3,		/* Rev 2 */
	REV_2_1 = 5,	/* Rev 2.1 */
	REV_MAX,
};

enum {
	V_SMB1356,
	V_SMB1357,
	V_SMB1358,
	V_SMB1359,
	V_MAX,
};

enum {
	USER = BIT(0),
	THERMAL = BIT(1),
	CURRENT = BIT(2),
};

enum path_type {
	USB,
	DC,
};

static int chg_time[] = {
	192,
	384,
	768,
	1536,
};

enum power_supply_type {
	POWER_SUPPLY_TYPE_UNKNOWN = 0,
	POWER_SUPPLY_TYPE_BATTERY,
	POWER_SUPPLY_TYPE_UPS,
	POWER_SUPPLY_TYPE_MAINS,
	POWER_SUPPLY_TYPE_USB,		/* Standard Downstream Port */
	POWER_SUPPLY_TYPE_USB_DCP,	/* Dedicated Charging Port */
	POWER_SUPPLY_TYPE_USB_CDP,	/* Charging Downstream Port */
	POWER_SUPPLY_TYPE_USB_ACA,	/* Accessory Charger Adapters */
	POWER_SUPPLY_TYPE_USB_HVDCP,	/* High Voltage DCP */
	POWER_SUPPLY_TYPE_USB_HVDCP_3,	/* Efficient High Voltage DCP */
	POWER_SUPPLY_TYPE_WIRELESS,	/* Accessory Charger Adapters */
	POWER_SUPPLY_TYPE_BMS,		/* Battery Monitor System */
	POWER_SUPPLY_TYPE_USB_PARALLEL,		/* USB Parallel Path */
	POWER_SUPPLY_TYPE_WIPOWER,		/* Wipower */
};

enum {
	POWER_SUPPLY_CHARGE_TYPE_UNKNOWN = 0,
	POWER_SUPPLY_CHARGE_TYPE_NONE,
	POWER_SUPPLY_CHARGE_TYPE_TRICKLE,
	POWER_SUPPLY_CHARGE_TYPE_FAST,
	POWER_SUPPLY_CHARGE_TYPE_TAPER,
};

static int usb_current_table_smb1357_smb1358[] = {
	300,
	400,
	450,
	475,
	500,
	550,
	600,
	650,
	700,
	900,
	950,
	1000,
	1100,
	1200,
	1400,
	1450,
	1500,
	1600,
	1800,
	1850,
	1880,
	1910,
	1930,
	1950,
	1970,
	2000,
	2050,
	2100,
	2300,
	2400,
	2500,
	3000
};

static int fastchg_current_table[] = {
	300,
	400,
	450,
	475,
	500,
	550,
	600,
	650,
	700,
	900,
	950,
	1000,
	1100,
	1200,
	1400,
	2700,
	1500,
	1600,
	1800,
	1850,
	1880,
	1910,
	2800,
	1950,
	1970,
	2000,
	2050,
	2100,
	2300,
	2400,
	2500,
	3000
};

static int dc_current_table[] = {
	300,
	400,
	450,
	475,
	500,
	550,
	600,
	650,
	700,
	900,
	950,
	1000,
	1100,
	1200,
	1400,
	1450,
	1500,
	1600,
	1800,
	1850,
	1880,
	1910,
	1930,
	1950,
	1970,
	2000,
};

static char *usb_type_str[] = {
	"ACA_DOCK",	/* bit 0 */
	"ACA_C",	/* bit 1 */
	"ACA_B",	/* bit 2 */
	"ACA_A",	/* bit 3 */
	"SDP",		/* bit 4 */
	"OTHER",	/* bit 5 */
	"DCP",		/* bit 6 */
	"CDP",		/* bit 7 */
	"NONE",		/* bit 8  error case */
};

struct smb135x_chg {
	//struct i2c_client		*client;
	//struct device			*dev;
	//struct mutex			read_write_lock;

	u8				revision;
	int				version;

	bool				chg_enabled;
	bool				chg_disabled_permanently;

	bool				usb_present;
	bool				dc_present;
	bool				usb_slave_present;
	bool				dc_ov;

	bool				bmd_algo_disabled;
	bool				iterm_disabled;
	int				iterm_ma;
	int				vfloat_mv;
	int				safety_time;
	int				resume_delta_mv;
	int				fake_battery_soc;
	//struct dentry			*debug_root;
	int				usb_current_arr_size;
	int				*usb_current_table;
	int				dc_current_arr_size;
	int				*dc_current_table;
	bool				inhibit_disabled;
	int				fastchg_current_arr_size;
	int				*fastchg_current_table;
	int				fastchg_ma;
	u8				irq_cfg_mask[3];
	int				otg_oc_count;
	//struct delayed_work		reset_otg_oc_count_work;
	//struct mutex			otg_oc_count_lock;
	//struct delayed_work		hvdcp_det_work;

	bool				parallel_charger;
	bool				parallel_charger_present;
	bool				bms_controlled_charging;

	/* psy */
	//struct power_supply		*usb_psy;
	int				usb_psy_ma;
	int				real_usb_psy_ma;
	//struct power_supply		batt_psy;
	//struct power_supply		dc_psy;
	//struct power_supply		parallel_psy;
	//struct power_supply		*bms_psy;
	int				dc_psy_type;
	int				dc_psy_ma;
	const char			*bms_psy_name;

	/* status tracking */
	bool				chg_done_batt_full;
	bool				batt_present;
	bool				batt_hot;
	bool				batt_cold;
	bool				batt_warm;
	bool				batt_cool;

	bool				resume_completed;
	bool				irq_waiting;
	u32				usb_suspended;
	u32				dc_suspended;
	//struct mutex			path_suspend_lock;

	u32				peek_poke_address;
	//struct smb135x_regulator	otg_vreg;
	int				skip_writes;
	int				skip_reads;
	u32				workaround_flags;
	bool				soft_vfloat_comp_disabled;
	//struct mutex			irq_complete;
	//struct regulator		*therm_bias_vreg;
	//struct regulator		*usb_pullup_vreg;
	//struct delayed_work		wireless_insertion_work;

	unsigned int			thermal_levels;
	unsigned int			therm_lvl_sel;
	unsigned int			*thermal_mitigation;
	//struct mutex			current_change_lock;

	//const char			*pinctrl_state_name;
	//struct pinctrl			*smb_pinctrl;

	bool				apsd_rerun;
	bool				id_line_not_connected;
};


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

static int ww_smb_parse_dt(struct smb135x_chg *chip){
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

int ww_smb_i2c_device_init(int ww_mode){
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
	
	struct smb135x_chg *smb135x_chip;
	smb135x_chip = malloc(sizeof(struct smb135x_chg));
	if(!smb135x_chip){
		dprintf(CRITICAL, "smb135x_chip malloc failed\n");
		return ERR_NOT_VALID;
	}
    
	i2c_dev = qup_blsp_i2c_init(BLSP_ID_1, QUP_ID_2, I2C_CLK_FREQ, I2C_SRC_CLK_FREQ);
	if(!i2c_dev) {
		dprintf(CRITICAL, "i2c_dev init failed\n");
		free(smb135x_chip);
		return ERR_NOT_VALID;
	}

	if(ww_mode == WW_BATTERY_CHARGE_MODE){
		dprintf(INFO, "\nshutdown charge starting --->\n");
    }

    ww_smb_i2c_read_byte(I2C_CLIENT_ADDR, STATUS_5_REG, &source_detect);
    dprintf(INFO, "STATUS_5_REG[%x] %x\n", STATUS_5_REG, source_detect);
    
	ww_smb_parse_dt(smb135x_chip); //config value
	smb135x_hw_init(smb135x_chip);
	smb135x_set_usb_chg_current(smb135x_chip);
	//dump_regs(smb135x_chip);

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
				shutdown_device();
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

