#pragma once

#include <stdint.h>

enum cc_debugger_error {
  CC_OK,
  CC_WRONG_DIRECTION
};

enum cc_debugger_cmd {
  CC_CMD_CHIP_ERASE = 0x14,
  CC_CMD_WR_CONFIG = 0x1D,
  CC_CMD_RD_CONFIG = 0x24,
  CC_CMD_GET_PC = 0x28,
  CC_CMD_READ_STATUS = 0x34,
  CC_CMD_SET_HW_BRKPNT = 0x3b,
  CC_CMD_HALT = 0x44,
  CC_CMD_RESUME = 0x4C,
  CC_CMD_DEBUG_INSTR = 0x54,
  CC_CMD_STEP_INSTR = 0x5C,
  CC_CMD_STEP_REPLACE = 0x64,
  CC_CMD_GET_CHIP_ID = 0x68,
};

class cc_debugger {
public:
  cc_debugger(int pin_rst, int pin_dc, int pin_dd, int pin_led);

  cc_debugger_error enter();
  cc_debugger_error exit();

  cc_debugger_error write(uint8_t data);
  uint8_t read();

  uint16_t get_chip_id();
  uint16_t get_pc();
  uint8_t get_status();

  uint8_t exec(uint8_t *istr, uint8_t count);

  uint8_t step();
  uint8_t resume();
  uint8_t halt();

  uint8_t chip_erase();

  uint8_t get_config();
  uint8_t set_config(uint8_t cfg);

  uint8_t set_breakpoint(uint8_t cfg, uint8_t addr_hi, uint8_t addr_lo);

  void switch_to_read();
  void switch_to_write();

private:
  int direction;

  int pin_rst;
  int pin_dc;
  int pin_dd;
  int pin_led;

  void set_direction(int dir);

  cc_debugger_error send_cmd(cc_debugger_cmd cmd);
};