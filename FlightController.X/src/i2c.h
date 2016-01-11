/* 
 * File:   i2c.h
 * Author: tbriggs, Kevin Dederer
 * Comments: Header file for the i2c bus
 * Revision history: 
 */

#ifndef I2C_H
#define	I2C_H

#ifdef	__cplusplus
extern "C" {
#endif

int lsm330_read_reg(uint8_t dev, uint8_t reg, uint8_t *data);
int lsm330_write_reg(uint8_t dev, uint8_t reg, uint8_t data);
int lsm330_read_multiple_reg(uint8_t dev, uint8_t reg, uint8_t *data);

#ifdef	__cplusplus
}
#endif

#endif	/* I2C_H */

