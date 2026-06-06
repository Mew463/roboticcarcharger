// #include <Arduino.h>
#include <AccelStepper.h>
#include <pins.h>
#include <string>
#include <CmdTokenizer.h>
#include <algorithm>
#include <ESP32Servo.h>
#include <FastLED.h>

AccelStepper stepper(AccelStepper::DRIVER, ELEV_STEP, ELEV_DIR);
const int USEDPINS[] = {ELEV_STEP, ELEV_DIR, ELEV_EN, ELEV_SOLENOID, ELEV_SWITCH_0, ELEV_SWITCH_1};

int READONLYPINS[] = {2, 4};
int WRITEANALOGPINS[] = {10, 8};
int WRITESERVOPINS[] = {0, 1, 10};

const int SIZEOFREADPINS = sizeof(READONLYPINS) / sizeof(READONLYPINS[0]);
const int SIZEOFWRITEANALOGPINS = sizeof(WRITEANALOGPINS) / sizeof(WRITEANALOGPINS[0]);
const int SIZEOFWRITESERVOPINS = sizeof(WRITESERVOPINS) / sizeof(WRITESERVOPINS[0]);
Servo servos[SIZEOFWRITESERVOPINS];

CRGB leds[NUM_LEDS];
uint8_t hue = 0;
ANIMATIONS curAnimation = ANIMATIONS::STATIC;
CRGB curColor = CRGB::Black;
int whiteBrightness = 100;

void setup() {
    Serial.begin(115200);  

    stepper.setAcceleration(5000);
    stepper.setMaxSpeed(7000);

    pinMode(ELEV_EN, OUTPUT); digitalWrite(ELEV_EN, HIGH);
    pinMode(ELEV_SOLENOID, OUTPUT); digitalWrite(ELEV_SOLENOID, LOW);
    pinMode(ELEV_SWITCH_0, INPUT_PULLUP);
    pinMode(ELEV_SWITCH_1, INPUT_PULLUP);

    for (int i = 0; i < SIZEOFWRITEANALOGPINS; i++) {
        pinMode(WRITEANALOGPINS[i], OUTPUT);
    } 

    for (int i = 0; i < SIZEOFWRITESERVOPINS; i++) {
        servos[i].attach(WRITESERVOPINS[i]);
    }

    for (int i = 0; i < SIZEOFREADPINS; i++) {
        pinMode(READONLYPINS[i], INPUT);
    } 

    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
}

void runStepperInBounds() {
    while (!(digitalRead(ELEV_SWITCH_0))) {
        stepper.runSpeed();
    }
}

void processCommand(CmdTokenizer cmd) {
    if (cmd.action == ACTIONS::WRITEANALOG) {
        auto end = WRITEANALOGPINS + SIZEOFWRITEANALOGPINS;
        int *result = std::find(WRITEANALOGPINS, end, cmd.pin);
        if (result != end) {
            analogWrite(cmd.pin, cmd.value);
            Serial.println("ACK"); // Response back to PI
        } else {
            LogReporter(LOGLEVEL::ERROR, "Unknown pin").report();
        }
    } else if (cmd.action == ACTIONS::WRITESERVO) {
        for (int i = 0; i < SIZEOFWRITESERVOPINS; i++) {
            if (WRITESERVOPINS[i] == cmd.pin) {
                servos[i].write(cmd.value);
                Serial.println("ACK");
            }
        }
    } else if (cmd.action == ACTIONS::READPIN) {
        auto end = READONLYPINS + SIZEOFREADPINS;
        int *result = std::find(READONLYPINS, end, cmd.pin);
        if (result != end) {
            Serial.println(analogRead(cmd.pin)); // Reponse back to PI
        } else {
            LogReporter(LOGLEVEL::ERROR, "Unknown pin").report();
        }
    } else if (cmd.action == ACTIONS::STEP) {
        Serial.println("ACK");
        digitalWrite(ELEV_EN, LOW);
        delay(50);
        digitalWrite(ELEV_SOLENOID, HIGH);

        if (cmd.pin == 1) {// Interpret as "home" command
            stepper.setSpeed(cmd.value);
            runStepperInBounds();
            stepper.setCurrentPosition(0);
        } else if (cmd.pin == 0) { // Move relative command
            stepper.moveTo(-cmd.value); // Invert motor so that positive is up
            while (stepper.distanceToGo() != 0) {
                stepper.run();
            }
        }

        digitalWrite(ELEV_SOLENOID, LOW);
        delay(50);
        digitalWrite(ELEV_EN, HIGH);
    } else if (cmd.action == ACTIONS::LED) {
        Serial.println("ACK");
        if (cmd.pin == 0) {
            curAnimation = ANIMATIONS::BREATHING;
        } else if (cmd.pin == 1) {
            curAnimation = ANIMATIONS::CIRCLE;
        } else if (cmd.pin == 2) {
            curAnimation = ANIMATIONS::STATIC;
        }

        if (cmd.value == 0) {
            curColor = CRGB::Black;
        } else if (cmd.value == 1) {
            curColor = CRGB::Red;
        } else if (cmd.value == 2) {
            curColor = CRGB::Green;
        } else if (cmd.value == 3) {
            curColor = CRGB::Blue;
        } else if (cmd.value == 4) {
            curColor = CRGB::Yellow;
        } else if (cmd.value == 5) {
            curColor = CRGB::Purple;
        } else if (cmd.value >= 500 && cmd.value <= 600) {
            curColor = CRGB::White;
            whiteBrightness = cmd.value - 500;
        }

    }
}

String input = "";

void loop() {
    while (Serial.available()) {
      char c = Serial.read();

      if (c == '\n') {
        CmdTokenizer cmdTokens(input);
        if (cmdTokens.validateChecksum().tokenize().isValid()) {
            processCommand(cmdTokens);
        } else {
            // Serial.println(cmdTokens.errMessage);
            LogReporter(LOGLEVEL::WARNING, cmdTokens.errMessage).report();
        }
        input = "";
      } else {
        input += c;
      }
    }

    
    if (curAnimation == ANIMATIONS::BREATHING) {
        EVERY_N_MILLIS(50) {
            fill_solid(leds, NUM_LEDS, curColor);
            static int curBrightness = 0;
            static bool increase = true;
            {
                curBrightness += increase ? 5 : -5;
                curBrightness = constrain(curBrightness, 0, 100);
                if (curBrightness == 100) {
                    increase = false;
                } else if (curBrightness == 0) {
                    increase = true;
                }
            }
            FastLED.setBrightness(curBrightness);
        }
    } else if (curAnimation == ANIMATIONS::CIRCLE) {
        EVERY_N_MILLIS(100) {
            FastLED.setBrightness(100);
            static int curIndex = 0;
            static int lastIndex = 0;
            curIndex++;
            curIndex = (curIndex>= 8) ? 0 : curIndex;
            leds[curIndex] = curColor;
            leds[lastIndex] = CRGB::Black;
            lastIndex = curIndex;
        }
        static bool toggleState = false;

        EVERY_N_MILLIS(500) {
            if (toggleState) {
                leds[NUM_LEDS - 1] = curColor;
                leds[NUM_LEDS - 2] = curColor;
            } else {
                leds[NUM_LEDS - 1] = CRGB::Black;
                leds[NUM_LEDS - 2] = CRGB::Black;
            }
            toggleState = !toggleState;
        }

    } else if (curAnimation == ANIMATIONS::STATIC) {
        FastLED.setBrightness ( (curColor == CRGB::White ? whiteBrightness : 100));
        fill_solid(leds, NUM_LEDS, curColor);
    }
    FastLED.show();
    delay(10); // Do I need this??
    // String testCmd = "R:2 :0|0C";
    // String testCmd = "W:08:255|11";
    // String testCmd = "W:08:0|13";

}


