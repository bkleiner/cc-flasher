#include <Arduino.h>

#include "cc_debugger.h"

cc_debugger *dbg;

void setup() {
  dbg = new cc_debugger(PA2, PA3, PB0, LED_BUILTIN);

  Serial.begin(115200);

  delay(500);
  dbg->enter();
}

void send_frame(const serial_response res) {
  Serial.write(res.ans);
  Serial.write(res.high);
  Serial.write(res.low);
  Serial.flush();
}

void loop() {
  static serial_request req;

  if (Serial.available() < 4)
    return;

  req.cmd = serial_cmd(Serial.read());
  req.data[0] = (uint8_t)Serial.read();
  req.data[1] = (uint8_t)Serial.read();
  req.data[2] = (uint8_t)Serial.read();

  const serial_response res = dbg->handle(req);
  send_frame(res);
}