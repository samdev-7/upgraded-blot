#include "config.h"
#include "protocol.h"
#include "planner.h"
#include "stepper.h"
#include "gcode.h"
#include "settings.h"

void setup() {
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    // Load persisted settings BEFORE anything else reads planner_* caps.
    settings_load();

    plan_init();
    stepper_init();
    gcode_reset();
    protocol_init();

    // Three short blinks so a human watching knows the board came up.
    for (int i = 0; i < 3; i++) {
        digitalWrite(PIN_LED, LOW);  delay(120);
        digitalWrite(PIN_LED, HIGH); delay(120);
    }
}

void loop() {
    protocol_service();
}
