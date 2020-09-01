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

inline void clock_delay(uint32_t count) {
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

void cc_debugger::switch_to_read() {
  set_direction(INPUT);
  clock_delay(100);

  bool did_wait = false;
  while (digitalRead(pin_dd) == HIGH) {
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
}

void cc_debugger::switch_to_write() {
  set_direction(OUTPUT);
}

cc_debugger_error cc_debugger::enter() {
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

  return CC_OK;
}

cc_debugger_error cc_debugger::exit() {
  resume();
  return CC_OK;
}

cc_debugger_error cc_debugger::write(uint8_t data) {
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

  return CC_OK;
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

  switch_to_read();
  return CC_OK;
}

uint16_t cc_debugger::get_chip_id() {
  uint16_t result = 0;

  send_cmd(CC_CMD_GET_CHIP_ID);
  result |= read() << 8;
  result |= read() << 0;

  return result;
}

uint16_t cc_debugger::get_pc() {
  uint16_t result = 0;

  send_cmd(CC_CMD_GET_PC);
  result |= read() << 8;
  result |= read() << 0;

  return result;
}

uint8_t cc_debugger::get_status() {
  send_cmd(CC_CMD_READ_STATUS);
  return read();
}

uint8_t cc_debugger::step() {
  send_cmd(CC_CMD_STEP_INSTR);
  return read(); // Accumulator
}

uint8_t cc_debugger::resume() {
  send_cmd(CC_CMD_RESUME);
  return read();
}

uint8_t cc_debugger::halt() {
  send_cmd(CC_CMD_HALT);
  return read();
}

uint8_t cc_debugger::get_config() {
  send_cmd(CC_CMD_RD_CONFIG);
  return read();
}

uint8_t cc_debugger::set_config(uint8_t cfg) {
  switch_to_write();
  write(CC_CMD_WR_CONFIG);
  write(cfg);

  switch_to_read();
  return read();
}

uint8_t cc_debugger::exec(uint8_t *istr, uint8_t count) {
  switch_to_write();
  write(CC_CMD_DEBUG_INSTR + count);
  for (uint8_t i = 0; i < count; i++) {
    write(istr[i]);
  }

  switch_to_read();
  return read();
}

uint8_t cc_debugger::chip_erase() {
  send_cmd(CC_CMD_CHIP_ERASE);
  return read();
}

uint8_t cc_debugger::set_breakpoint(uint8_t cfg, uint8_t addr_hi, uint8_t addr_lo) {
  switch_to_write();
  write(CC_CMD_SET_HW_BRKPNT);
  write(cfg);
  write(addr_hi);
  write(addr_lo);

  return cfg;
}