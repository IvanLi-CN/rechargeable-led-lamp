#ifndef GX21M15_H
#define GX21M15_H

#include "i2c.h"

#define GX21M15_ADDR 0x48       // 0b1001000x
#define GX21M15_REG_CONF 0x01   // Config
#define GX21M15_REG_TEMP 0x00   // Temperature
#define GX21M15_REG_TOS 0x03    // Over-temperature shutdown
#define GX21M15_REG_THYST 0x02  // Hysteresis

void bq25890h_init() {
  u8 buf[3];
  u8 err;

  // Over-temperature shutdown at 60 ℃
  buf[0] = GX21M15_REG_TOS;
  buf[1] = 0x3C;    // 60 x 1
  buf[2] = 0 << 7;  // 0.5 x 0

  err = i2c_send(GX21M15_ADDR, buf, 3);
  if (err) {
    printf("Set over-temperature shutdown failed\n\r");
  }

  // Hysteresis at 50 ℃
  buf[0] = GX21M15_REG_THYST;
  buf[1] = 0x32;    // 50 x 1
  buf[2] = 0 << 7;  // 0.5 x 0
  i2c_send(GX21M15_ADDR, buf, 3);
  if (err) {
    printf("Set hysteresis failed\n\r");
  }

  // Fault Queue 4, Interrupt Output Low,
  // Over Temperature Interrupt Mode, Chip  Wake-Up
  buf[0] = GX21M15_REG_CONF;
  buf[1] = 0b00010010;
  i2c_send(GX21M15_ADDR, buf, 2);
  if (err) {
    printf("Set config failed\n");
  }

  printf("GX21M15 initialized\n");
}

u8 gx21m15_read(u16 *temp) {
  u8 buf[2] = {0, 0};
  if (i2c_recv(GX21M15_ADDR, GX21M15_REG_TEMP, buf, 2)) {
    printf("Read temperature failed\n");
    return 1;
  }

  *temp = (buf[0] << 3) | buf[1] >> 5;
  *temp *= 125;
  return 0;
}

#endif
