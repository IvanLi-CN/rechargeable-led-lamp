#ifndef BQ25890H
#define BQ25890H

#include "i2c.h"

#define BQ25890H_ADDR 0x6a  // 0b1001000x

// REG00-REG14
#define BQ25890H_REG00 0x00
#define BQ25890H_REG01 0x01
#define BQ25890H_REG02 0x02
#define BQ25890H_REG03 0x03
#define BQ25890H_REG04 0x04
#define BQ25890H_REG05 0x05
#define BQ25890H_REG06 0x06
#define BQ25890H_REG07 0x07
#define BQ25890H_REG08 0x08
#define BQ25890H_REG09 0x09
#define BQ25890H_REG0A 0x0a
#define BQ25890H_REG0B 0x0b
#define BQ25890H_REG0C 0x0c
#define BQ25890H_REG0D 0x0d
#define BQ25890H_REG0E 0x0e
#define BQ25890H_REG0F 0x0f
#define BQ25890H_REG10 0x10
#define BQ25890H_REG11 0x11
#define BQ25890H_REG12 0x12
#define BQ25890H_REG13 0x13
#define BQ25890H_REG14 0x14

// Fast charge Timer
#define BQ25890H_REG07_FAST_CHARGE_TIMER_ENABLE (1 << 3)
#define BQ25890H_REG07_FAST_CHARGE_TIMER_5HOURS (0b00 << 1)
#define BQ25890H_REG07_FAST_CHARGE_TIMER_8HOURS (0b01 << 1)
#define BQ25890H_REG07_FAST_CHARGE_TIMER_12HOURS (0b10 << 1)
#define BQ25890H_REG07_FAST_CHARGE_TIMER_20HOURS (0b11 << 1)

void gx21m15_init() {
  u8 buf[3];
  u8 err;

  // Enable 12V input
  buf[0] = BQ25890H_REG01;
  buf[1] = 0b00000011;
  err = i2c_send(BQ25890H_ADDR, buf, 2);
  if (err) {
    printf("Set REG01 error: %d\n", err);
  }

  // ADC Continuous Conversion
  buf[0] = BQ25890H_REG02;
  buf[1] = 0b11010001;
  err = i2c_send(BQ25890H_ADDR, buf, 2);
  if (err) {
    printf("Set REG02 error: %d\n", err);
  }

  // Set Fast charge Timer to 5 hours
  buf[0] = BQ25890H_REG07;
  buf[1] = 0b10011001;
  err = i2c_send(BQ25890H_ADDR, buf, 2);
  if (err) {
    printf("Set REG07 error: %d\n", err);
  }

  // Set Thermal  Regulation Threshold to 60℃
  buf[0] = BQ25890H_REG08;
  buf[1] = 0x00;
  err = i2c_send(BQ25890H_ADDR, buf, 2);
  if (err) {
    printf("Set REG08 error: %d\n", err);
  }

  printf("BQ25890H Initialized\n");
}

#define BQ25890H_NOT_CHARGING 0b00
#define BQ25890H_PRE_CHARGE 0b01
#define BQ25890H_FAST_CHARGING 0b10
#define BQ25890H_CHARGE_TERMINATION_DONE 0b11
#define BQ25890H_CHARGE_STATUS_MASK (0b11 << 3)

/**
 * Get charge status
 *
 * @param status pointer to u8
 * @return execution status, non-zero on error
 */
u8 bq25890h_get_charge_status(u8* status) {
  u8 buf[1];
  u8 err;
  err = i2c_recv(BQ25890H_ADDR, BQ25890H_REG0B, buf, 1);
  if (err) return err;
  *status = (buf[0] & BQ25890H_CHARGE_STATUS_MASK) >> 3;

  return 0;
}

#define BQ25890H_CHARGE_FAULT_MASK (0b11 << 4)
#define BQ25890H_CHARGE_FAULT_NONE 0b00
#define BQ25890H_CHARGE_FAULT_INPUT_FAULT \
  0b01  //  (VBUS > VACOV or VBAT < VBUS < VVBUSMIN(typical 3.8V)
#define BQ25890H_CHARGE_FAULT_THERMAL_SHUTDOWN 0b10
#define BQ25890H_CHARGE_FAULT_CHARGE_SAFELY_TIMER_EXPIRATION 0b11

/**
 * Get charge fault status
 *
 * @param status pointer to u8
 * @return execution status, non-zero on error
 */
u8 bq25890h_get_charge_fault(u8* status) {
  u8 buf[1];
  u8 err;
  err = i2c_recv(BQ25890H_ADDR, BQ25890H_REG0C, buf, 1);
  if (err) return err;
  *status = (buf[0] & BQ25890H_CHARGE_FAULT_MASK) >> 4;

  return 0;
}

#define BQ25890H_BATTERY_VOLTAGE_MASK (0b01111111 << 0)

/**
 * Get battery voltage
 *
 * @param voltage pointer to u8
 * @return execution status, non-zero on error
 */
u8 bq25890h_get_battery_voltage(u8* voltage) {
  u8 buf[1];
  u8 err;
  err = i2c_recv(BQ25890H_ADDR, BQ25890H_REG0E, buf, 1);
  if (err) return err;
  *voltage = (buf[0] & BQ25890H_BATTERY_VOLTAGE_MASK) >> 0;

  return 0;
}

/**
 * Get SYS voltage
 *
 * @param voltage pointer to u8
 * @return execution status, non-zero on error
 */
u8 bq25890h_get_sys_voltage(u8* voltage) {
  return i2c_recv(BQ25890H_ADDR, BQ25890H_REG0F, voltage, 1);
}

/**
 * Get TS Percent
 *
 * @param status pointer to u8
 * tspct[6] 29.76%
 * tspct[5] 14.88%
 * tspct[4] 7.44%
 * tspct[3] 3.72%
 * tspct[2] 1.86%
 * tspct[1] 0.93%
 * tspct[0] 0.465%
 * @return execution status, non-zero on error
 */
u8 bq25890h_get_thermal_shutdown(u8* status) {
  return i2c_recv(BQ25890H_ADDR, BQ25890H_REG10, status, 1);
}

/**
 * Get Charge Current
 *
 * @param voltage pointer to u8
 * @return execution status, non-zero on error
 */
u8 bq25890h_get_charge_current(u8* current) {
  return i2c_recv(BQ25890H_ADDR, BQ25890H_REG12, current, 1);
}

#endif
