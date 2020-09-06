#pragma once

#include <stdint.h>

enum serial_cmd {
  CMD_ENTER = 0x01,
  CMD_EXIT = 0x02,
  CMD_CHIP_ID = 0x03,
  CMD_STATUS = 0x04,
  CMD_PC = 0x05,
  CMD_STEP = 0x06,
  CMD_EXEC_1 = 0x07,
  CMD_EXEC_2 = 0x08,
  CMD_EXEC_3 = 0x09,
  CMD_BRUSTWR = 0x0A,
  CMD_RD_CFG = 0x0B,
  CMD_WR_CFG = 0x0C,
  CMD_CHPERASE = 0x0D,
  CMD_RESUME = 0x0E,
  CMD_HALT = 0x0F,
  CMD_SET_BREAKPOINT = 0x10,
  CMD_PING = 0xF0,
};

enum serial_answer {
  ANS_OK = 0x01,
  ANS_ERROR = 0x02,
  ANS_READY = 0x03,
};

struct serial_request {
  serial_cmd cmd;
  uint8_t data[3];
};

struct serial_response {

  serial_response(serial_answer ans, uint8_t low = 0, uint8_t high = 0)
      : ans(ans)
      , low(low)
      , high(high) {}

  serial_answer ans;
  uint8_t low;
  uint8_t high;
};