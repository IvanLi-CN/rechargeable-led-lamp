#ifndef _APP_I2C_H
#define _APP_I2C_H

#include <stdio.h>

#include "ch32v003fun.h"

#define I2C_CLKRATE 100000
#define I2C_PRERATE 2000000

#define I2C_TIMEOUT_MAX 100000

#define I2C_ERROR_TIMEOUT 1
#define I2C_ERROR_WAIT_FOR_RX_TIMEOUT 2
#define I2C_ERROR_BUSY 3

void init_i2c() {
  // Enable GPIOC and I2C
  RCC->APB2PCENR |= RCC_APB2Periph_GPIOC;
  RCC->APB1PCENR |= RCC_APB1Periph_I2C1;

  // PC1 is SDA, 10MHz Output, alt func, open-drain
  GPIOC->CFGLR &= ~(0xf << (4 * 1));
  GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF) << (4 * 1);

  // PC2 is SCL, 10MHz Output, alt func, open-drain
  GPIOC->CFGLR &= ~(0xf << (4 * 2));
  GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF) << (4 * 2);

  // set freq
  I2C1->CTLR2 &= ~I2C_CTLR2_FREQ;
  I2C1->CTLR2 |= 8 & I2C_CTLR2_FREQ;

  // set prescaler for 100KHz
  I2C1->CKCFGR |= (FUNCONF_SYSTEM_CORE_CLOCK / (2 * 100000)) & I2C_CKCFGR_CCR;

  // // Enable interrupt
  // I2C1->CTLR1 |= I2C_CTLR2_ITEVTEN;

  // Enable I2C
  I2C1->CTLR1 |= I2C_CTLR1_PE;
}

u8 i2c_error(u8 err) {
  switch (err) {
    case I2C_ERROR_TIMEOUT:
      printf("I2C timeout\n");
      return 1;
    case I2C_ERROR_WAIT_FOR_RX_TIMEOUT:
      printf("I2C wait for RX timeout\n");
      return 2;
    case I2C_ERROR_BUSY:
      printf("I2C busy\n");
      return 3;
    default:
      printf("I2C error %d\n", err);
      return 3;
  }

  return 1;
}

/*
 * check for 32-bit event codes
 */
u8 i2c_check_event(u32 event_mask) {
  /* read order matters here! STAR1 before STAR2!! */
  u32 status = I2C1->STAR1 | (I2C1->STAR2 << 16);
  return (status & event_mask) == event_mask;
}

u8 i2c_send(u8 addr, u8 *data, u8 size) {
  int32_t timeout;

  // Waiting for not busy
  timeout = I2C_TIMEOUT_MAX;
  while ((I2C1->STAR2 & I2C_STAR2_BUSY) && (timeout--));
  if (timeout == -1) {
    return i2c_error(I2C_ERROR_BUSY);
  }

  // Send start
  I2C1->CTLR1 |= I2C_CTLR1_START;

  // Waiting for master mode select
  timeout = I2C_TIMEOUT_MAX;
  while ((!i2c_check_event(I2C_EVENT_MASTER_MODE_SELECT)) && (timeout--));
  if (timeout == -1) {
    printf("I2C_EVENT_MASTER_MODE_SELECT, %d, %d \n", I2C1->STAR1, I2C1->STAR2);
    return i2c_error(I2C_ERROR_TIMEOUT);
  }

  // Send 7-bit slave address and WRITE flag
  I2C1->DATAR = addr << 1;

  // Waiting for address sent
  timeout = I2C_TIMEOUT_MAX;
  while ((!i2c_check_event(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) &&
         (timeout--));
  if (timeout == -1) {
    printf("I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, %d, %d \n", I2C1->STAR1,
           I2C1->STAR2);
    return i2c_error(I2C_ERROR_TIMEOUT);
  }

  // Send one byte of data at a time
  while (size--) {
    timeout = I2C_TIMEOUT_MAX;
    while (!(I2C1->STAR1 & I2C_STAR1_TXE) && (timeout--));
    if (timeout == -1) {
      printf("I2C_STAR1_TXE, %d, %d \n", I2C1->STAR1, I2C1->STAR2);
      return i2c_error(I2C_ERROR_TIMEOUT);
    }

    I2C1->DATAR = *data++;
  }

  // Waiting for end of transmission
  timeout = I2C_TIMEOUT_MAX;
  while ((!i2c_check_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) && (timeout--));
  if (timeout == -1) {
    printf("I2C_EVENT_MASTER_BYTE_TRANSMITTED, %d, %d \n", I2C1->STAR1,
           I2C1->STAR2);
    return i2c_error(I2C_ERROR_TIMEOUT);
  }

  // Send stop
  I2C1->CTLR1 |= I2C_CTLR1_STOP;

  return 0;
}

u8 i2c_recv(u8 addr, u8 reg, u8 *data, u8 size) {
  int32_t timeout;

  // Waiting for not busy
  timeout = I2C_TIMEOUT_MAX;
  while ((I2C1->STAR2 & I2C_STAR2_BUSY) && (timeout--));
  if (timeout == -1) {
    printf("I2C_STAR2_BUSY, %d, %d \n", I2C1->STAR1, I2C1->STAR2);
    return i2c_error(I2C_ERROR_TIMEOUT);
  }

  // Send start
  I2C1->CTLR1 |= I2C_CTLR1_START;

  // Waiting for master mode select
  timeout = I2C_TIMEOUT_MAX;
  while ((!i2c_check_event(I2C_EVENT_MASTER_MODE_SELECT)) && (timeout--));
  if (timeout == -1) {
    printf("I2C_EVENT_MASTER_MODE_SELECT, %d, %d \n", I2C1->STAR1, I2C1->STAR2);
    return i2c_error(I2C_ERROR_TIMEOUT);
  }

  // Send 7-bit slave address and WRITE flag
  I2C1->DATAR = addr << 1;

  // Waiting for address sent
  timeout = I2C_TIMEOUT_MAX;
  while ((!i2c_check_event(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) &&
         (timeout--));
  if (timeout == -1) {
    printf("I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, %d, %d \n", I2C1->STAR1,
           I2C1->STAR2);
    return i2c_error(I2C_ERROR_TIMEOUT);
  }

  // Send one byte of data at a time
  timeout = I2C_TIMEOUT_MAX;
  while (!(I2C1->STAR1 & I2C_STAR1_TXE) && (timeout--));
  if (timeout == -1) {
    printf("I2C_STAR1_TXE, %d, %d \n", I2C1->STAR1, I2C1->STAR2);
    return i2c_error(I2C_ERROR_TIMEOUT);
  }
  I2C1->DATAR = reg;

  // Waiting for end of transmission
  timeout = I2C_TIMEOUT_MAX;
  while ((!i2c_check_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) && (timeout--));
  if (timeout == -1) {
    printf("I2C_EVENT_MASTER_BYTE_TRANSMITTED, %d, %d \n", I2C1->STAR1,
           I2C1->STAR2);
    return i2c_error(I2C_ERROR_TIMEOUT);
  }

  // Send Restart
  I2C1->CTLR1 |= I2C_CTLR1_START;

  // Waiting for master mode select
  timeout = I2C_TIMEOUT_MAX;
  while ((!i2c_check_event(I2C_EVENT_MASTER_MODE_SELECT)) && (timeout--));
  if (timeout == -1) {
    return i2c_error(I2C_ERROR_BUSY);
  }

  // Set ACK flag
  I2C1->CTLR1 |= I2C_CTLR1_ACK;

  // Send 7-bit slave address and READ flag
  I2C1->DATAR = (addr << 1) | 1;

  // Waiting for address sent
  timeout = I2C_TIMEOUT_MAX;
  while ((!i2c_check_event(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) &&
         (timeout--));
  if (timeout == -1) return i2c_error(I2C_ERROR_TIMEOUT);

  while (size--) {
    timeout = I2C_TIMEOUT_MAX * 3;

    while (!(I2C1->STAR1 & I2C_STAR1_RXNE) && (timeout--));
    if (timeout == -1) {
      I2C1->CTLR1 |= I2C_CTLR1_STOP;
      return i2c_error(I2C_ERROR_WAIT_FOR_RX_TIMEOUT);
    }

    // set ACK flag for next read
    if (size > 1) {
      I2C1->CTLR1 |= I2C_CTLR1_ACK;
    } else {
      I2C1->CTLR1 &= ~I2C_CTLR1_ACK;
    }

    *data++ = I2C1->DATAR;
    printf("data: %d, p: %d\n", I2C1->DATAR, size);
  }

  // Send stop
  I2C1->CTLR1 |= I2C_CTLR1_STOP;

  return 0;
}

#endif