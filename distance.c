/*
 *  TU/e 5EID0::LIBPYNQ Example for VL53L0X TOF Sensor
 *
 *  This version follows the same working method as:
 *  - examples/vl_main.c
 *  - examples/single_vl.c
 *  - examples/vl.c
 *
 *  Flow:
 *  - route IIC0 to the Arduino I2C pins
 *  - ping the sensor at 0x29
 *  - initialize it with tofInit()
 *  - read model/revision
 *  - continuously print tofReadDistance()
 */

#include <libpynq.h>
#include <iic.h>
#include "vl53l0x.h"

#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define VL53L0X_ADDR 0x29
#define RANGE_MODE_LONG 0
#define LOOP_DELAY_MS 100

/*
 * Many VL53L0X setups report a small non-zero value even when the target is
 * effectively at the sensor face. Tune this to your measured bench offset.
 */
#define TOF_ZERO_OFFSET_MM 23U

static void i2c_scan_bus(void) {
  bool found_any = false;

  printf("Scanning I2C bus...\n");
  for (uint8_t addr = 0x08; addr < 0x78; ++addr) {
    uint8_t value = 0;
    if (iic_read_register(IIC0, addr, 0xC0, &value, 1) == 0) {
      printf("  Found device at 0x%02X\n", addr);
      found_any = true;
    }
  }

  if (!found_any) {
    printf("  No I2C devices responded.\n");
  }
}

static uint32_t apply_distance_offset(uint32_t raw_distance_mm) {
  if (raw_distance_mm == UINT32_MAX) {
    return UINT32_MAX;
  }

  if (raw_distance_mm <= TOF_ZERO_OFFSET_MM) {
    return 0;
  }

  return raw_distance_mm - TOF_ZERO_OFFSET_MM;
}

int main(void) {
  int status = 0;
  uint8_t model = 0;
  uint8_t revision = 0;
  vl53x sensor;

  printf("VL53L0X ToF test starting.\n");
  printf("Method: vl_main.c + single_vl.c + vl.c\n");
  printf("I2C address: 0x%02X (7-bit)\n", VL53L0X_ADDR);
  printf("Pins: SDA=%d SCL=%d\n", IO_AR_SDA, IO_AR_SCL);
  printf("Zero offset correction: %u mm\n", TOF_ZERO_OFFSET_MM);

  pynq_init();

  switchbox_set_pin(IO_AR_SCL, SWB_IIC0_SCL);
  switchbox_set_pin(IO_AR_SDA, SWB_IIC0_SDA);
  iic_init(IIC0);
  iic_reset(IIC0);

  sleep_msec(50);
  i2c_scan_bus();

  status = tofPing(IIC0, VL53L0X_ADDR);
  printf("Sensor Ping: ");
  if (status != 0) {
    printf("Fail\n");
    iic_destroy(IIC0);
    pynq_destroy();
    return EXIT_FAILURE;
  }
  printf("Success\n");

  status = tofInit(&sensor, IIC0, VL53L0X_ADDR, RANGE_MODE_LONG);
  if (status != 0) {
    printf("tofInit failed with status %d\n", status);
    iic_destroy(IIC0);
    pynq_destroy();
    return EXIT_FAILURE;
  }

  printf("VL53L0X device successfully opened.\n");
  if (tofGetModel(&sensor, &model, &revision) == 0) {
    printf("Model ID - %u\n", model);
    printf("Revision ID - %u\n", revision);
  } else {
    printf("Failed to read model and revision.\n");
  }
  fflush(stdout);

  while (true) {
    const uint32_t raw_distance_mm = tofReadDistance(&sensor);
    if (raw_distance_mm == UINT32_MAX) {
      printf("Distance read failed (timeout)\n");
    } else {
      const uint32_t corrected_distance_mm =
          apply_distance_offset(raw_distance_mm);
      printf("Distance = %u mm (raw %u mm)\n", corrected_distance_mm,
             raw_distance_mm);
    }
    fflush(stdout);
    sleep_msec(LOOP_DELAY_MS);
  }

  iic_destroy(IIC0);
  pynq_destroy();
  return EXIT_SUCCESS;
}
