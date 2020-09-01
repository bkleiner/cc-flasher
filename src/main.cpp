#include <Arduino.h>

#include "cc_debugger.h"

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

cc_debugger *dbg;

void setup() {
  dbg = new cc_debugger(PA2, PA3, PB0, LED_BUILTIN);

  Serial.begin(115200);

  delay(500);
  dbg->enter();
}

void send_frame(const serial_answer ans, const uint8_t low = 0, const uint8_t high = 0) {
  Serial.write(ans);
  Serial.write(high);
  Serial.write(low);
  Serial.flush();
}

void loop() {
  static uint8_t cmd;
  static uint8_t data[3];

  if (Serial.available() < 4)
    return;

  cmd = Serial.read();
  data[0] = (uint8_t)Serial.read();
  data[1] = (uint8_t)Serial.read();
  data[2] = (uint8_t)Serial.read();

  switch (cmd) {

  case CMD_ENTER: {
    dbg->enter();
    send_frame(ANS_OK);
    break;
  }
  case CMD_EXIT: {
    dbg->exit();
    send_frame(ANS_OK);
    break;
  }
  case CMD_CHIP_ID: {
    const uint16_t id = dbg->get_chip_id();
    send_frame(ANS_OK, id & 0xFF, (id >> 8) & 0xFF);
    break;
  }
  case CMD_STATUS: {
    const uint8_t status = dbg->get_status();
    send_frame(ANS_OK, status);
    break;
  }
  case CMD_PC: {
    const uint16_t pc = dbg->get_pc();
    send_frame(ANS_OK, pc & 0xFF, (pc >> 8) & 0xFF);
    break;
  }
  case CMD_STEP: {
    const uint8_t acc = dbg->step();
    send_frame(ANS_OK, acc);
    break;
  }
  case CMD_EXEC_1: {
    const uint8_t acc = dbg->exec(data, 1);
    send_frame(ANS_OK, acc);
    break;
  }
  case CMD_EXEC_2: {
    const uint8_t acc = dbg->exec(data, 2);
    send_frame(ANS_OK, acc);
    break;
  }
  case CMD_EXEC_3: {
    const uint8_t acc = dbg->exec(data, 3);
    send_frame(ANS_OK, acc);
    break;
  }
  case CMD_BRUSTWR: {
    const uint16_t len = (data[0] << 8) | data[1];
    if (len > 2048) {
      send_frame(ANS_ERROR, 3);
      return;
    }

    dbg->switch_to_write();
    send_frame(ANS_READY);
    dbg->write(0x80 | (data[0] & 0x07));
    dbg->write(data[1]);

    uint16_t to_write = len;

    while (to_write > 0) {
      if (Serial.available() >= 1) {
        const uint8_t data = Serial.read();
        dbg->write(data);
        to_write--;
        continue;
      }

      delay(10);
    }

    dbg->switch_to_read();
    send_frame(ANS_OK, dbg->read());
    break;
  }
  case CMD_RD_CFG: {
    const uint8_t cfg = dbg->get_config();
    send_frame(ANS_OK, cfg);
    break;
  }
  case CMD_WR_CFG: {
    const uint8_t cfg = dbg->set_config(data[0]);
    send_frame(ANS_OK, cfg);
    break;
  }
  case CMD_CHPERASE: {
    const uint8_t res = dbg->chip_erase();
    send_frame(ANS_OK, res);
    break;
  }
  case CMD_RESUME: {
    const uint8_t res = dbg->resume();
    send_frame(ANS_OK, res);
    break;
  }
  case CMD_HALT: {
    const uint8_t res = dbg->halt();
    send_frame(ANS_OK, res);
    break;
  }
  case CMD_SET_BREAKPOINT: {
    const uint8_t res = dbg->set_breakpoint(data[0], data[1], data[2]);
    send_frame(ANS_OK, res);
    break;
  }
  case CMD_PING: {
    send_frame(ANS_OK);
    break;
  }
  default:
    send_frame(ANS_ERROR, 0xFF);
    break;
  }
}