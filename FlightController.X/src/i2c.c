/* 
 * File:   i2c.c
 * Author: tbriggs, Kevin Dederer
 * Comments: main logic file for the i2c bus operation
 * Revision history: 
 */

#include "config.h"

#define I2C_I2C_BUS (I2C2)
#define I2C_DELAY (32)

int i2c_wait_timeout = 0;
#define I2C_WAIT_USDELAY(x,y) \
{ i2c_wait_timeout = 0; \
  unsigned int __end = ReadCoreTimer() + (GetInstructionClock() / 4000000L) * y; \
  while((x(I2C_I2C_BUS) == 0) && (ReadCoreTimer() < __end)); \
  i2c_wait_timeout = (ReadCoreTimer() >= __end)  ? 1 : 0; } \

#define I2C_WAIT(x) \
{ while((x(I2C_I2C_BUS) == 0)); } \

#define I2C_TRY(x) \
{ int _rc; rc = x; if (rc != 0) goto error; } \

#ifndef EOK
#define EOK 0
#endif

#ifndef EDATA
#define EDATA 13
#endif

/**
 * I2C DELAY - delay for given number of microseconds.
 * @param usecs
 */
void i2c_delay(int usecs) 
{
    unsigned int start = ReadCoreTimer();
    unsigned int dtime = (GetSystemClock() / 4000000L) * usecs;
    while ((ReadCoreTimer() - start) < dtime);
}

/**
 * I2C START - Start an I2C bus transaction (required at the start of every I2C transaction
 * @param restart - whether this is a second start without a stop in between
 * @return EOK if no error, -EBADF if start was not successfully sent
 */
int i2c_start(int restart) 
{
    I2C_STATUS status;

    int rc;
    if (restart) 
    {
        rc =I2CRepeatStart(I2C_I2C_BUS);
    }
    else 
    {
        I2C_WAIT(I2CBusIsIdle);

        rc = I2CStart(I2C_I2C_BUS);
    }
    if (rc != I2C_SUCCESS)
        return -EBADF;

    I2C_WAIT(I2CTransmissionHasCompleted);

    do 
    {
        status = I2CGetStatus(I2C_I2C_BUS);
    } while (!(status & I2C_START));

    i2c_delay(10);
    return EOK;
}

/**
 * I2C STOP - Stop transmission.
 */
void i2c_stop() 
{
    I2C_STATUS status;

    I2C_WAIT(I2CTransmitterIsReady);

    I2CStop(I2C_I2C_BUS);

    do 
    {
        status = I2CGetStatus(I2C_I2C_BUS);
    } while (!(status & (I2C_STOP)));
}

/**
 * I2C XMIT BYTE - Transmit one byte (internal use)
 * @param data Byte to transfer
 * @return EOK or -EDATA
 */
int i2c_xmit_byte(UINT8 data) 
{
    I2C_STATUS status;

    I2C_WAIT_USDELAY(I2CTransmitterIsReady, I2C_DELAY);
    if (i2c_wait_timeout) 
    {
        printf("ERROR: XMIT timed out waiting for transmitter\n");
        return -EDATA;
    }

    if (I2CSendByte(I2C_I2C_BUS, data) != I2C_SUCCESS) 
    {
        printf("ERROR: XMIT master bus collision (%x)\n", I2CGetStatus(I2C_I2C_BUS));
        return -EDATA;
    }

    I2C_WAIT(I2CTransmissionHasCompleted);

    if (I2CByteWasAcknowledged(I2C_I2C_BUS))
        return EOK;

    status = I2CGetStatus(I2C_I2C_BUS);
    if (status & I2C_ARBITRATION_LOSS)
    {
        printf("ERROR: ARBITRATION LOSS\n");
        return -EDATA;
    }
    else if (status & I2C_TRANSMITTER_OVERFLOW)
    {
        printf("ERROR: XMIT Overflow ");
        return -EDATA;
    }
    else if (status & I2C_BYTE_ACKNOWLEDGED)
    {
        return EOK;
    }
    else 
    {
        printf("ERROR: XMIT Unspecified error %x\n", status);
        return -EDATA;
    }
}

/**
 * I2C RCV BYTE Receive one byte from I2C bus - internal transaction
 * @param byte byte that was received
 * @param ack whether to acknowledge
 * @return EOK or -EDATA
 */
int i2c_rcv_byte(UINT8 ack, UINT8 *byte)
{
    UINT8 inp;
    int rc;

    rc = I2CReceiverEnable(I2C_I2C_BUS, TRUE);
    if (rc == I2C_RECEIVE_OVERFLOW)
        return -EDATA;

    while (!I2CReceivedDataIsAvailable(I2C_I2C_BUS));

    inp = I2CGetByte(I2C_I2C_BUS);

    I2CAcknowledgeByte(I2C_I2C_BUS, ack);
    I2C_WAIT(I2CAcknowledgeHasCompleted);

    *byte = inp;
    
    return EOK;
}

/**
 * I2C OPEN - Open I2C controller.
 * @return EOK if controller opened, or -EBADF if bus clock cannot be achieved.
 */
int i2c_open() 
{
    unsigned int actualClock;

    I2CConfigure(I2C_I2C_BUS, I2C_ENABLE_SLAVE_CLOCK_STRETCHING);
    actualClock = I2CSetFrequency(I2C_I2C_BUS, GetSystemClock(), I2C_CLOCK_FREQ);
    if (abs(actualClock - I2C_CLOCK_FREQ) > I2C_CLOCK_FREQ / 10) 
    {
        printf("Error: I2C Bus Clock Frequency error exceeds 10%%\n");
        return -EBADF;
    }

    I2CEnable(I2C_I2C_BUS, TRUE);
    
    return EOK;
}

/**
 * I2C CLOSE - Close I2C controller
 */
void i2c_close() {
    I2CEnable(I2C_I2C_BUS, FALSE);
}

/*
 * I2C WRITE DEV ADDRESS - sends a write signal to the slave
 * @param dev_address the 7 bit address of the slave
 * @return EOK or -EDATA 
 */
int i2c_write_dev_address(uint8_t dev_address)
{
    I2C_7_BIT_ADDRESS i2c_ctrl;
    
    I2C_FORMAT_7_BIT_ADDRESS(i2c_ctrl, dev_address, I2C_WRITE);

    I2C_WAIT(I2CTransmissionHasCompleted);

    // next, we send another "write" command, and wait for it to ack
    // that we know the command is done - this write ack polling
    int count = 100;
    while (--count > 0) {
        I2C_WAIT(I2CTransmitterIsReady);
        i2c_start(0);

        I2C_WAIT(I2CTransmitterIsReady);
        I2CSendByte(I2C_I2C_BUS,i2c_ctrl.byte);

        I2C_WAIT(I2CTransmissionHasCompleted);

        if (I2CByteWasAcknowledged(I2C_I2C_BUS))
            break;

        i2c_stop();
        i2c_delay(I2C_DELAY);
    }
    
    if (count == 0) return -1;
    return 0;
}

/*
 * I2C READ DEV ADDRESS - sends a read command to the slave
 * @param dev_address - the 7 bit address of the slave
 * @return EOK or -EDATA
 */
int i2c_read_dev_address(uint8_t dev_address)
{
    I2C_7_BIT_ADDRESS i2c_ctrl;
    
    I2C_FORMAT_7_BIT_ADDRESS(i2c_ctrl, dev_address, I2C_READ);

    I2C_WAIT(I2CTransmissionHasCompleted);

    i2c_start(1);
    I2C_WAIT(I2CTransmitterIsReady);
    
    I2CSendByte(I2C_I2C_BUS,i2c_ctrl.byte);

    I2C_WAIT(I2CTransmissionHasCompleted);

    if (I2CByteWasAcknowledged(I2C_I2C_BUS))
        return 0;
    else
        return -1;
}
    
/*
 * LSM330 READ REG - reads the byte transmitted by the slave
 * @param dev - the device address of the slave
 * @param reg - the register address in the slave to be accessed
 * @param data - the byte to be read
 * @return EOK or -EDATA
 */
int lsm330_read_reg(uint8_t dev, uint8_t reg, uint8_t *data)
{
    if (i2c_open() <0) return -1;
    
    if (i2c_write_dev_address(dev) < 0) return -1;
    
    if (i2c_xmit_byte(reg) < 0) return -1;
    
    if (i2c_read_dev_address(dev)< 0) return -1;
    
    if(i2c_rcv_byte(0, data) < 0) return -1;

    i2c_stop();

    return 0;
}

/*
 * LSM330 WRITE REG - writes a byte to the slave
 * @param dev - the device address of the slave
 * @param reg - the register address in the slave to be written to
 * @param data - the byte to be written
 * @return EOK or -EDATA
 */
int lsm330_write_reg(uint8_t dev, uint8_t reg, uint8_t data)
{
    if (i2c_open() <0) return -1;
    
    if (i2c_write_dev_address(dev) < 0) return -1;
    
    if (i2c_xmit_byte(reg) < 0) return -1;
    
    if (i2c_xmit_byte(data) < 0) return -1;
    
    i2c_stop();
    
    return 0;
}

/*
 * LSM330 READ MULTIPLE REG - completes a multi-register read from the slave
 * @param dev - the 7 bit address of the slave
 * @param reg - one byte of data including the 7 bit address of the register in the
 *              slave to be read from and 1 bit indicating that it is a multiple 
 *              register read
 * @param data - pointer to an array to store the read bytes
 * @return EOK or -EDATA
 */
int lsm330_read_multiple_reg(uint8_t dev, uint8_t reg, UINT8 data[6])
{
    int ack, i, err;
    
    if (i2c_open() <0) return -1;

    if (i2c_write_dev_address(dev) < 0) return -1;

    if (i2c_xmit_byte(reg) < 0) return -1;

    if (i2c_read_dev_address(dev) < 0) return -1;

    for(i = 0; i < 6; i++)
    {
        ack = (i == 5) ? 0 : 1;
        if(i2c_rcv_byte(ack,&data[i]) < 0) return -1;
    }
       
    i2c_stop();

    return 0;
}