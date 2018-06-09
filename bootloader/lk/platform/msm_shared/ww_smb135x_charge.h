#ifndef __WW_SMB135X_CHARGE_H__
#define __WW_SMB135X_CHARGE_H__

#include <stdint.h>


enum{
	WW_READ_BATTERY_VOL_MODE,
	WW_BATTERY_CHARGE_MODE,
};


int ww_smb_i2c_read_byte(uint8_t addr, uint8_t reg, uint8_t *buf);
int ww_smb_i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t val);
int ww_smb_i2c_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len);
int ww_smb_i2c_device_init(int ww_mode);


#endif /* __WW_SMB135X_CHARGE_H__ */

