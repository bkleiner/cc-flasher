#include "cc_debugger.h"

#include <Arduino.h>

#define CC_CLOCK_DELAY 2

cc_debugger::cc_debugger(int pin_rst, int pin_dc, int pin_dd, int pin_led)
    : direction(-1)
    , pin_rst(pin_rst)
    , pin_dc(pin_dc)
    , pin_dd(pin_dd)
    , pin_led(pin_led) {

  pinMode(pin_rst, OUTPUT);
  pinMode(pin_dc, OUTPUT);
  pinMode(pin_dd, INPUT);
  pinMode(pin_led, OUTPUT);

  digitalWrite(pin_rst, HIGH);
  digitalWrite(pin_dc, LOW);
  digitalWrite(pin_led, LOW);

  set_direction(INPUT);
}

void clock_delay(uint32_t count) {
  while (count--)
    __NOP();
}

void cc_debugger::set_direction(int dir) {
  if (direction == dir) {
    return;
  }

  if (dir == INPUT) {
    digitalWrite(pin_dd, LOW);
    pinMode(pin_dd, INPUT);
    digitalWrite(pin_dd, LOW);
  } else {
    digitalWrite(pin_dd, LOW);
    pinMode(pin_dd, OUTPUT);
    digitalWrite(pin_dd, LOW);
  }

  direction = dir;
}

cc_debugger_error cc_debugger::switch_to_read() {
  set_direction(INPUT);
  clock_delay(100);

  bool did_wait = false;
  uint16_t timeout = 0x400;
  while (digitalRead(pin_dd) == HIGH && timeout--) {
    for (uint8_t i = 0; i < 8; i++) {
      digitalWrite(pin_dc, HIGH);
      clock_delay(CC_CLOCK_DELAY);
      digitalWrite(pin_dc, LOW);
      clock_delay(CC_CLOCK_DELAY);
    }
    did_wait = true;
  }

  if (!did_wait) {
    clock_delay(CC_CLOCK_DELAY);
  }

  if (timeout == 0) {
    return CC_TIMEOUT;
  }
  return CC_OK;
}

void cc_debugger::switch_to_write() {
  set_direction(OUTPUT);
}

void cc_debugger::enter() {
  digitalWrite(pin_rst, LOW);
  delay(200);

  digitalWrite(pin_dc, HIGH);
  delay(3);
  digitalWrite(pin_dc, LOW);
  delay(3);
  digitalWrite(pin_dc, HIGH);
  delay(3);
  digitalWrite(pin_dc, LOW);
  delay(200);

  digitalWrite(pin_rst, HIGH);
  delay(200);
}

void cc_debugger::write(uint8_t data) {
  set_direction(OUTPUT);

  for (uint8_t i = 0; i < 8; i++) {
    if (data & 0x80)
      digitalWrite(pin_dd, HIGH);
    else
      digitalWrite(pin_dd, LOW);

    digitalWrite(pin_dc, HIGH);

    data <<= 1;
    clock_delay(CC_CLOCK_DELAY);

    digitalWrite(pin_dc, LOW);
    clock_delay(CC_CLOCK_DELAY);
  }
}

uint8_t cc_debugger::read() {
  set_direction(INPUT);

  uint8_t data = 0;
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(pin_dc, HIGH);
    clock_delay(CC_CLOCK_DELAY);

    data <<= 1;
    if (digitalRead(pin_dd) == HIGH)
      data |= 0x01;

    digitalWrite(pin_dc, LOW);
    clock_delay(CC_CLOCK_DELAY);
  }

  return data;
}

cc_debugger_error cc_debugger::send_cmd(cc_debugger_cmd cmd) {
  switch_to_write();
  write(cmd);

  return switch_to_read();
}

serial_response cc_debugger::exec(uint8_t *istr, uint8_t count) {
  switch_to_write();
  write(CC_CMD_DEBUG_INSTR + count);
  for (uint8_t i = 0; i < count; i++) {
    write(istr[i]);
  }

  cc_debugger_error err = switch_to_read();
  if (err != CC_OK) {
    return {ANS_ERROR, err};
  }

  return {ANS_OK, read()};
}

serial_response cc_debugger::handle(serial_request &req) {
  switch (req.cmd) {
  case CMD_ENTER: {
    enter();
    return {ANS_OK};
  }
  case CMD_EXIT:
  case CMD_RESUME: {
    cc_debugger_error err = send_cmd(CC_CMD_RESUME);
    if (err != CC_OK) {
      return {ANS_ERROR, err};
    }
    return {ANS_OK, read()};
  }
  case CMD_CHIP_ID: {
    cc_debugger_error err = send_cmd(CC_CMD_GET_CHIP_ID);
    if (err != CC_OK) {
      return {ANS_ERROR, err};
    }

    uint8_t high = read();
    uint8_t low = read();
    return {ANS_OK, low, high};
  }
  case CMD_STATUS: {
    cc_debugger_error err = send_cmd(CC_CMD_READ_STATUS);
    if (err != CC_OK) {
      return {ANS_ERROR, err};
    }
    return {ANS_OK, read()};
  }
  case CMD_PC: {
    cc_debugger_error err = send_cmd(CC_CMD_GET_PC);
    if (err != CC_OK) {
      return {ANS_ERROR, err};
    }

    uint8_t high = read();
    uint8_t low = read();
    return {ANS_OK, low, high};
  }
  case CMD_STEP: {
    cc_debugger_error err = send_cmd(CC_CMD_STEP_INSTR);
    if (err != CC_OK) {
      return {ANS_ERROR, err};
    }
    return {ANS_OK, read()};
  }
  case CMD_EXEC_1: {
    return exec(req.data, 1);
  }
  case CMD_EXEC_2: {
    return exec(req.data, 2);
  }
  case CMD_EXEC_3: {
    return exec(req.data, 3);
  }
  /*
  case CMD_BRUSTWR: {
    const uint16_t len = (req.data[0] << 8) | req.data[1];
    if (len > 2048) {
      return {ANS_ERROR, 3};
    }

    switch_to_write();
    return {ANS_READY};0x80 | (req.data[0] & 0x07));
    write(req.data[1]);

    uint16_t to_write = len;

    while (to_write > 0) {
      if (Serial.available() >= 1) {
        const uint8_t req.data = Serial.read();
        write(req.data);
        to_write--;
        continue;
      }

      delay(10);
    }

    switch_to_read();
    return {ANS_OK, read()};
  }
  */
  case CMD_RD_CFG: {
    cc_debugger_error err = send_cmd(CC_CMD_RD_CONFIG);
    if (err != CC_OK) {
      return {ANS_ERROR, err};
    }
    return {ANS_OK, read()};
  }
  case CMD_WR_CFG: {
    switch_to_write();
    write(CC_CMD_WR_CONFIG);
    write(req.data[0]);

    cc_debugger_error err = switch_to_read();
    if (err != CC_OK) {
      return {ANS_ERROR, err};
    }
    return {ANS_OK, read()};
  }
  case CMD_CHPERASE: {
    cc_debugger_error err = send_cmd(CC_CMD_CHIP_ERASE);
    if (err != CC_OK) {
      return {ANS_ERROR, err};
    }
    return {ANS_OK, read()};
  }

    // CMD_RESUME

  case CMD_HALT: {
    cc_debugger_error err = send_cmd(CC_CMD_HALT);
    if (err != CC_OK) {
      return {ANS_ERROR, err};
    }
    return {ANS_OK, read()};
  }
  case CMD_SET_BREAKPOINT: {
    switch_to_write();
    write(CC_CMD_SET_HW_BRKPNT);
    write(req.data[0]);
    write(req.data[1]);
    write(req.data[2]);

    return {ANS_OK};
  }
  case CMD_PING: {
    return {ANS_OK};
  }
  default:
    return {ANS_ERROR, 0xFF};
  }
}