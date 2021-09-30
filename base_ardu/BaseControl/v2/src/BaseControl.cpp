/*! @mainpage
/******************************************************************
 * soft.vers:             rover base control
 * arduino IDE version:   platformio  linux-64
 *                        
 * arduino board version: Arduino due
 * bootloader:            standard
 * libraries:             
 * interface:             serial: 115200,8,N,1
 *                        
 * last revision:         11.09.2018 coding
 *                        10.10.2018 testing 1
 *                        14.12.2018 camera actuators
 *                        24.12.2018 camera relay test 2
 *                        19.06.2021 eliminato ros1 / ros2 interface
 *                        27.06.2021 new 2021 interface
 * TESTS COMMAND LINE:    
 * 
 * CAM COMMANDS:
 * 
 * MATEK LEDS:
 ******************************************************************/

#pragma GCC push_options
#pragma GCC optimize ("O3")
#include <stdint.h>

//------------------------------------------------------------------------------
#define BANNER1  "I:BASE-LINK"
#define VERSION  "I:20210623"

//-- DEBUG ---------------------------------------------------------------------
#define UART_ECHO           (0)
#define UART_BAUDRATE       (115200)

//-- WATCHDOG ------------------------------------------------------------------
// http://forum.arduino.cc/index.php?topic=132986.0
//
// commentare nella funzione init di:
// /home/antonio/.arduino15/packages/arduino/hardware/sam/1.6.4/variants/arduino_due_x/variant.cpp
// Disable watchdog
// antonio s - 30.6.2015
// WDT_Disable(WDT);        <-- commentare
//
// Variable wdp_ms hold the periode in 256 th of seconds of watchdog
// It must be greater than 3 et less or equal to 4096
// The following value set a periode of 4,5 seconds (256 x 4,5 = 1152)
uint32_t wdp_ms = 1152;

#define  WATCHDOG_RESET       WDT_Restart( WDT );

//-- LEDS Matek WS2812B --------------------------------------------------------
#include <FastLED.h>
#define NUM_LEDS          6
#define FAST_LED_PIN      (42)
#define FAST_BUZZER       (4)

CRGB fastleds[NUM_LEDS];

//-- BUZZER --------------------------------------------------------------------

//-- SERVO + PID CONTROLLER ----------------------------------------------------
#include <Servo.h>
#include <PID_v1.h>

#define PWM_WHEEL_L       (3)
#define PWM_WHEEL_R       (2)

#define aggKp             4.00
#define aggKi             0.20
#define aggKd             1.00
#define consKp            1.00
#define consKi            0.05
#define consKd            0.25

double vwXset;          // linear axis X velocity V
double vwZset;          // angular axis Z velocity V
Servo servoL;
double vwLin;           // left wheel velocity V  RPM
double vwLset;          // left wheel velocity V
double vwLout;          // left wheel velocity V
PID wlPID(&vwLin, &vwLout, &vwLset, consKp, consKi, consKd, DIRECT);

Servo servoR;
double vwRin;           // right wheel velocity V  RPM
double vwRset;          // right wheel velocity V
double vwRout;          // right wheel velocity V
PID wrPID(&vwRin, &vwRout, &vwRset, consKp, consKi, consKd, DIRECT);

//-- CAMERA SERVOS -------------------------------------------------------------
#include <Derivs_Limiter.h>

Derivs_Limiter camLimiter = Derivs_Limiter(100, 75); // velocityLimit, accelerationLimit
Servo camPan;
Servo camTilt;

//-- SONAR ---------------------------------------------------------------------
// richiede il convertitore 5V 3V3
#define SONAR_TRIG_N_PIN       (35)
#define SONAR_ECHO_N_PIN       (36)
#define SONAR_TRIG_S_PIN       (37)
#define SONAR_ECHO_S_PIN       (38)

//-- KEYBOARD ------------------------------------------------------------------
#include <Bounce2.h>

// utility buttons CSI Cable
#define BTN_SHUTDOWN_PIN       (22)
#define BTN_EMERGENCY_PIN      (23)
// bumpers buttons
#define BTN_BUMPER_N_PIN       (24)
#define BTN_BUMPER_NE_PIN      (25)
#define BTN_BUMPER_E_PIN       (26)
#define BTN_BUMPER_SE_PIN      (27)
#define BTN_BUMPER_S_PIN       (28)
#define BTN_BUMPER_SW_PIN      (29)
#define BTN_BUMPER_W_PIN       (30)
#define BTN_BUMPER_NW_PIN      (31)
// PIR motion sensors, uniform bounced interface
#define BTN_PIR_N_PIN          (32)
#define BTN_PIR_SE_PIN         (33)
#define BTN_PIR_SW_PIN         (34)
// LED status
#define LED_READY_PIN          (41)
// LED lights
#define LED_ON                 (HIGH)
#define LED_OFF                (LOW)
#define LED_BLINK_LONG_MS      (1000)
#define LED_BLINK_SHORT_MS     (500)
#define LED_BLINK_PAUSE_MS     (2000)
char          ledFunct;
unsigned long ledLastMillis;
byte          ledState;
// INTEGRATED CAMERA MOUNT
#define CAMERA_MOUNT_TILT_PIN  (49)
#define CAMERA_MOUNT_PAN_PIN   (48)
#define CAMERA_MOUNT_LIGHT_PIN (46)
#define CAMERA_MOUNT_IR_PIN    (47)

// i bumpers meccanici sono sostituiti dall accelerometro
byte BTNshutValue;
byte BTNemerValue;
#ifdef HAVE_BUMPERS
byte BTNbumpNValue;
byte BTNbumpNEValue;
byte BTNbumpEValue;
byte BTNbumpSEValue;
byte BTNbumpSValue;
byte BTNbumpSWValue;
byte BTNbumpWValue;
byte BTNbumpNWValue;
#endif
byte BTNpirNValue;
byte BTNpirSEValue;
byte BTNpirSWValue;
Bounce BTNshut   = Bounce(BTN_SHUTDOWN_PIN, 5);
Bounce BTNemer   = Bounce(BTN_EMERGENCY_PIN, 5);
#ifdef HAVE_BUMPERS
Bounce BTNbumpN  = Bounce(BTN_BUMPER_N_PIN, 5);
Bounce BTNbumpNE = Bounce(BTN_BUMPER_NE_PIN, 5);
Bounce BTNbumpE  = Bounce(BTN_BUMPER_E_PIN, 5);
Bounce BTNbumpSE = Bounce(BTN_BUMPER_SE_PIN, 5);
Bounce BTNbumpS  = Bounce(BTN_BUMPER_S_PIN, 5);
Bounce BTNbumpSW = Bounce(BTN_BUMPER_SW_PIN, 5);
Bounce BTNbumpW  = Bounce(BTN_BUMPER_W_PIN, 5);
Bounce BTNbumpNW = Bounce(BTN_BUMPER_NW_PIN, 5);
#endif
Bounce BTNpirN   = Bounce(BTN_PIR_N_PIN, 5);
Bounce BTNpirSE  = Bounce(BTN_PIR_SE_PIN, 5);
Bounce BTNpirSW  = Bounce(BTN_PIR_SW_PIN, 5);

//-- FUTURES -------------------------------------------------------------------
void ledSet(char fun);
void onLeftPulse();
void onRightPulse();

float mapf(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
  return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
} 

//-- INTERFACE -------------------------------------------------------------
#define AXIS_WHEEL_MIN  (-1.0)
#define AXIS_WHEEL_MAX  (+1.0)
#define WHEEL_R_MM      (79.0 / 2.0)
#define WHEEL_L_MM      (280.0)

// set camera pan [0, 180] + --> left (ref. on base frame, face forward)
int camPanValue;
// set camera tilt [0, 180] + --> down(ref. on base frame)
int camTiltValue;

//-- SINGLE PULSE MOTOR FEEDBACK  --------------------------------------------
#define WHEEL_MAX_RPM        (60)
#define PULSE_REVOLUTION     (3.0)
#define PULSE_LEFT_PIN       (39)
#define PULSE_RIGHT_PIN      (40)

int32_t xwLrel;         // left wheel position X relative
int32_t xwLtim;         // left wheel timing X relative
int32_t xwRrel;         // right wheel position X relative
int32_t xwRtim;         // right wheel timing X relative
int32_t xwLabs;         // left wheel position X absolute
int32_t xwRabs;         // right wheel position X absolute

void pulserInit() {
  attachInterrupt(PULSE_LEFT_PIN, onLeftPulse, FALLING);
  attachInterrupt(PULSE_RIGHT_PIN, onRightPulse, FALLING);
}

void onLeftPulse() {
  xwLrel++;
  xwLabs++;
  vwLin = ((millis() - xwLtim) * 60 * 1000) / PULSE_REVOLUTION; // RPM dx = v
  xwLtim = millis();
}

void onRightPulse() {
  xwRrel++;
  xwRabs++;
  vwRin = ((millis() - xwRtim) * 60 * 1000) / PULSE_REVOLUTION; // RPM dx = v
  xwRtim = millis();
}

void pulserProcess() {
  // single pulser --> no operations
}

//-- CONSOLE -------------------------------------------------------------------
#include <SimpleCLI.h>

SimpleCLI cli;
Command cmdSTP;
Command cmdSTA;
Command cmdCAM;
Command cmdIR;
Command cmdLIGHT;
Command cmdSTRIP_LED;
Command cmdSTATUS_LED;
Command cmdAP;
Command cmdRP;
Command cmdZP;
Command cmdBUZ;

void cbSTP(cmd* cmdPtr) {
    Command cmd(cmdPtr);
}

void cbSTA(cmd* cmdPtr) {
    Command cmd(cmdPtr);
    Argument xArg = cmd.getArgument("X");
    Argument zArg = cmd.getArgument("Z");
    Argument lArg = cmd.getArgument("L"); // unused
    float x = xArg.getValue().toFloat();
    float z = zArg.getValue().toFloat();
    float l = lArg.getValue().toFloat();
    x = constrain(x, AXIS_WHEEL_MIN, AXIS_WHEEL_MAX);
    vwZset = constrain(z, AXIS_WHEEL_MIN, AXIS_WHEEL_MAX);
    vwXset = mapf(x, AXIS_WHEEL_MIN, AXIS_WHEEL_MAX, -WHEEL_MAX_RPM, WHEEL_MAX_RPM);
    vwLset = vwXset + vwXset * vwZset;
    vwRset = vwXset - vwXset * vwZset;
}

void cbCAM(cmd* cmdPtr) {
    Command cmd(cmdPtr);
    Argument xArg = cmd.getArgument("X");
    Argument yArg = cmd.getArgument("Y");
    int x = xArg.getValue().toInt();
    int y = yArg.getValue().toInt();
    x = constrain(x, AXIS_WHEEL_MIN, AXIS_WHEEL_MAX);
    y = constrain(y, AXIS_WHEEL_MIN, AXIS_WHEEL_MAX);
    camPanValue = mapf(x, AXIS_WHEEL_MIN, AXIS_WHEEL_MAX, 0, 180);
    camTiltValue = mapf(y, AXIS_WHEEL_MIN, AXIS_WHEEL_MAX, 0, 180);

}

void cbIR(cmd* cmdPtr) {
    Command cmd(cmdPtr);
    Argument argS = cmd.getArgument("S");
    bool S = argS.isSet();
    if (S) {
      digitalWrite(CAMERA_MOUNT_IR_PIN, LOW);
    } else {
      digitalWrite(CAMERA_MOUNT_IR_PIN, HIGH);
    }
}

void cbLIGHT(cmd* cmdPtr) {
    Command cmd(cmdPtr);
    Argument argS = cmd.getArgument("S");
    bool S = argS.isSet();
    if (S) {
      digitalWrite(CAMERA_MOUNT_LIGHT_PIN, HIGH);
    } else {
      digitalWrite(CAMERA_MOUNT_LIGHT_PIN, LOW);
    }
}

//
// matek led strip RGB
// limit: 15.12.2018 only 1 LED a is the index
// TODO: extend to 6 leds
//    custom messages:
//    http://wiki.ros.org/rosserial/Tutorials/Adding%20Other%20Messages
//
void cbSTRIP_LED(cmd* cmdPtr) {
    Command cmd(cmdPtr);
    Argument rArg = cmd.getArgument("R");
    Argument gArg = cmd.getArgument("G");
    Argument bArg = cmd.getArgument("B");
    Argument iArg = cmd.getArgument("I");
    int r = rArg.getValue().toInt();
    int g = gArg.getValue().toInt();
    int b = bArg.getValue().toInt();
    int i = iArg.getValue().toInt();
    byte led_index = constrain(i, 0, NUM_LEDS - 1);
    fastleds[led_index].setRGB(r, g, b);
    FastLED.show();
}

void cbSTATUS_LED(cmd* cmdPtr) {
    Command cmd(cmdPtr);
    Argument argS = cmd.getArgument("S");
    bool S = argS.isSet();
    if (S) {
      ledSet('H');
    } else {
      ledSet('Z');
    }
}

void cbAP(cmd* cmdPtr) {
    Serial.print("L" + String(xwLabs) + "R" + String(xwRabs));
}

void cbRP(cmd* cmdPtr) {
    Serial.print("L" + String(xwLrel) + "R" + String(xwRrel));
}

void cbZP(cmd* cmdPtr) {
    xwLabs = 0;
    xwRabs = 0;
    Serial.print("OK");
}

// buzzer tone
void cbBUZ(cmd* cmdPtr) {
  Command cmd(cmdPtr);
  Argument argS = cmd.getArgument("S");
  bool S = argS.isSet();
  if (S) {
    digitalWrite(FAST_BUZZER, LED_ON);
  } else {
    digitalWrite(FAST_BUZZER, LED_OFF);
  }
}

void consoleInit() {
  cmdSTP = cli.addCommand("STP", cbSTP);
  cmdSTA = cli.addCommand("STA", cbSTA);
  cmdSTA.addArgument("X");
  cmdSTA.addArgument("Z");
  cmdSTA.addArgument("L");
  cmdSTA.addArgument("T");
  cmdCAM = cli.addCommand("CAM", cbCAM);
  cmdCAM.addArgument("X");
  cmdCAM.addArgument("Y");
  cmdIR = cli.addCommand("IR", cbIR);
  cmdIR.addFlagArgument("S");
  cmdLIGHT = cli.addCommand("LIGHT", cbLIGHT);
  cmdLIGHT.addFlagArgument("S");
  cmdSTRIP_LED = cli.addCommand("STRIP_LED", cbSTRIP_LED);
  cmdSTRIP_LED.addArgument("R");
  cmdSTRIP_LED.addArgument("G");
  cmdSTRIP_LED.addArgument("B");
  cmdSTRIP_LED.addArgument("I");
  cmdSTATUS_LED = cli.addCommand("STATUS_LED", cbSTATUS_LED);
  cmdSTATUS_LED.addFlagArgument("S");
  cmdAP = cli.addCommand("AP", cbAP);
  cmdRP = cli.addCommand("RP", cbRP);
  cmdZP = cli.addCommand("ZP", cbZP);
  cmdBUZ = cli.addCommand("BUZ", cbBUZ);
  cmdBUZ.addFlagArgument("S");
}

void consoleProcess() {
  if (Serial.available()) {
      String input = Serial.readStringUntil('\n');

      if (input.length() > 0) {
        Serial.print("# ");
        Serial.println(input);

        cli.parse(input);
      }
  }

  if (cli.errored()) {
      CommandError cmdError = cli.getError();

      Serial.print("E: ");
      Serial.println(cmdError.toString());
  }
}

//------------------------------------------------------------------------------

void buttonInit() {
  pinMode(BTN_SHUTDOWN_PIN, INPUT_PULLUP);
  pinMode(BTN_EMERGENCY_PIN, INPUT_PULLUP);
#ifdef HAVE_BUMPERS
  pinMode(BTN_BUMPER_N_PIN, INPUT_PULLUP);
  pinMode(BTN_BUMPER_NE_PIN, INPUT_PULLUP);
  pinMode(BTN_BUMPER_E_PIN, INPUT_PULLUP);
  pinMode(BTN_BUMPER_SE_PIN, INPUT_PULLUP);
  pinMode(BTN_BUMPER_S_PIN, INPUT_PULLUP);
  pinMode(BTN_BUMPER_SW_PIN, INPUT_PULLUP);
  pinMode(BTN_BUMPER_W_PIN, INPUT_PULLUP);
  pinMode(BTN_BUMPER_NW_PIN, INPUT_PULLUP);
#endif
  // logic input
  pinMode(BTN_PIR_N_PIN, INPUT);
  pinMode(BTN_PIR_SE_PIN, INPUT);
  pinMode(BTN_PIR_SW_PIN, INPUT);
  // 
  pinMode(LED_READY_PIN, OUTPUT);
  digitalWrite(LED_READY_PIN, LED_OFF);
  ledSet('B');
    
  BTNshutValue = HIGH;    // unpressed
  BTNemerValue = HIGH;
#ifdef HAVE_BUMPERS
  BTNbumpNEValue = HIGH;
  BTNbumpNEValue = HIGH;
  BTNbumpEValue = HIGH;
  BTNbumpSEValue = HIGH;
  BTNbumpSValue = HIGH;
  BTNbumpSWValue = HIGH;
  BTNbumpWValue = HIGH;
  BTNbumpNWValue = HIGH;
#endif
  BTNpirNValue = HIGH;
  BTNpirSEValue = HIGH;
  BTNpirSWValue = HIGH;
}

void buttonProcess() {
  BTNshut.update();
  BTNemer.update();
#ifdef HAVE_BUMPERS
  BTNbumpN.update();
  BTNbumpNE.update();
  BTNbumpE.update();
  BTNbumpSE.update();
  BTNbumpS.update();
  BTNbumpSW.update();
  BTNbumpW.update();
  BTNbumpNW.update();
#endif
  BTNpirN.update();
  BTNpirSE.update();
  BTNpirSW.update();
  
  if (BTNshutValue != BTNshut.read()) {
    BTNshutValue = BTNshut.read();
    if (BTNshutValue == LOW) {
        ledSet('B');
        Serial.print("BTN_SHUT 0");
    } else {
        Serial.print("BTN_SHUT 1");
    }
  }

  if (BTNemerValue != BTNemer.read()) {
    BTNemerValue = BTNemer.read();
    if (BTNemerValue == LOW) {
      Serial.print("BTN_EME 0");
    } else {
      Serial.print("BTN_EME 1");
    }
  }

#ifdef HAVE_BUMPERS
  if (BTNbumpNValue != BTNbumpN.read()) {
    BTNbumpNValue = BTNbumpN.read();
    if (BTNbumpNValue == LOW) {

    } else {

    }
  }

  if (BTNbumpNEValue != BTNbumpNE.read()) {
    BTNbumpNEValue = BTNbumpNE.read();
    if (BTNbumpNEValue == LOW) {

    } else {

    }
  }

  if (BTNbumpEValue != BTNbumpE.read()) {
    BTNbumpEValue = BTNbumpE.read();
    if (BTNbumpEValue == LOW) {

    } else {

    }
  }

  if (BTNbumpSEValue != BTNbumpSE.read()) {
    BTNbumpSEValue = BTNbumpSE.read();
    if (BTNbumpSEValue == LOW) {

    } else {

    }
  }

  if (BTNbumpSValue != BTNbumpS.read()) {
    BTNbumpSValue = BTNbumpS.read();
    if (BTNbumpSValue == LOW) {

    } else {

    }
  }

  if (BTNbumpSWValue != BTNbumpSW.read()) {
    BTNbumpSWValue = BTNbumpSW.read();
    if (BTNbumpSWValue == LOW) {

    } else {

    }
  }

  if (BTNbumpWValue != BTNbumpW.read()) {
    BTNbumpWValue = BTNbumpW.read();
    if (BTNbumpWValue == LOW) {

    } else {

    }
  }

  if (BTNbumpNWValue != BTNbumpNW.read()) {
    BTNbumpNWValue = BTNbumpNW.read();
    if (BTNbumpNWValue == LOW) {

    } else {

    }
  }
#endif

  if (BTNpirNValue != BTNpirN.read()) {
    BTNpirNValue = BTNpirN.read();
    if (BTNpirNValue == LOW) {
      Serial.print("PIR_N 0");
    } else {
      Serial.print("PIR_N 1");
    }
  }

  if (BTNpirSEValue != BTNpirSE.read()) {
    BTNpirSEValue = BTNpirSE.read();
    if (BTNpirSEValue == LOW) {
      Serial.print("PIR_SE 0");
    } else {
      Serial.print("PIR_SE 1");
    }
  }

  if (BTNpirSWValue != BTNpirSW.read()) {
    BTNpirSWValue = BTNpirSW.read();
    if (BTNpirSWValue == LOW) {
      Serial.print("PIR_SW 0");
    } else {
      Serial.print("PIR_SW 1");
    }
  }
}

void ledSet(char fun) {
  ledFunct = fun;
  ledLastMillis = millis();
  ledState = 0;
}

void ledGotoState(byte n, unsigned int delayms) {
  if (millis() - ledLastMillis > delayms) {
    ledState = n;
    ledLastMillis = millis();
  }
}

void ledNextState(unsigned int delayms) {
  if (millis() - ledLastMillis > delayms) {
    ledState++;
    ledLastMillis = millis();
  }
}

void ledProcess(void) {
  switch (ledFunct) {
    case 'B':
       switch (ledState) {
         case 0:
           digitalWrite(LED_READY_PIN, LED_ON);
           ledNextState(LED_BLINK_SHORT_MS);
           break;
         case 1:
           digitalWrite(LED_READY_PIN, LED_OFF);
           ledGotoState(0, LED_BLINK_SHORT_MS);
           break;
       }
       break;

    case 'Z':
       // Z -> led off
       digitalWrite(LED_READY_PIN, LED_OFF);
       break;

    case 'H':
       // H -> led on
       digitalWrite(LED_READY_PIN, LED_ON);
       break;

    default: break;
  }
}

//------------------------------------------------------------------------------

void camMountInit() {
  camPan.attach(CAMERA_MOUNT_PAN_PIN);
  camTilt.attach(CAMERA_MOUNT_TILT_PIN);
  
  pinMode(CAMERA_MOUNT_LIGHT_PIN, OUTPUT);
  digitalWrite(CAMERA_MOUNT_LIGHT_PIN, LED_OFF);
  // IR LIGHT ENABLED. AUTO ON LOW LIGHT
  pinMode(CAMERA_MOUNT_IR_PIN, OUTPUT);
  digitalWrite(CAMERA_MOUNT_IR_PIN, LOW);
  // initial values: center axis
  camPanValue = 90;
  camTiltValue = 90;
}

void camMountprocess() {
  camPan.write(camLimiter.calc(camPanValue));
  camTilt.write(camLimiter.calc(camTiltValue));
}

//------------------------------------------------------------------------------

void stripLedInit() {
  FastLED.addLeds<NEOPIXEL, FAST_LED_PIN>(fastleds, NUM_LEDS);
  
  // test sequence
  for (int i = 0; i < NUM_LEDS / 2; i++) { 
      fastleds[i] = CRGB::Red;
      fastleds[NUM_LEDS - i - 1] = CRGB::Green;
      FastLED.show();
      FastLED.delay(500); 
      fastleds[i] = CRGB::Black;
      fastleds[NUM_LEDS - i - 1] = CRGB::Black;
      FastLED.show();
  }
  
  // initial led setup as boat lighting
  // red = left, green = right
  fastleds[0] = CRGB::Red;
  fastleds[NUM_LEDS - 1] = CRGB::Green;
  FastLED.show();
}

void buzzerInit() {
  pinMode(FAST_BUZZER, OUTPUT);
  digitalWrite(FAST_BUZZER, HIGH);
  delay(500);
  digitalWrite(FAST_BUZZER, LOW);
}

//------------------------------------------------------------------------------

void setup(){
  // WDT_Disable(WDT);
  
  // Variable wdp_ms hold the periode in 256 th of seconds of watchdog
  // It must be greater than 3 et less or equal to 4096
  // The following value set a periode of 4,5 seconds (256 x 4,5 = 1152)
  WDT_Enable( WDT, 0x2000 | wdp_ms | ( wdp_ms << 16 ));
  //
  buttonInit();
  pulserInit();
  consoleInit();
  // drive
  servoL.attach(PWM_WHEEL_L);
  servoR.attach(PWM_WHEEL_R);
  // camera mount
  camMountInit();
  // Matek Board
  stripLedInit();
  buzzerInit();
  //
  Serial.print(BANNER1);
  Serial.print(VERSION);
  Serial.print("I: *** PLEASE, ARM MOTORS ***");
}

void loop() {
  consoleProcess();
  ledProcess();
  buttonProcess();
  pulserProcess();
  camMountprocess();
  
  WATCHDOG_RESET;
}