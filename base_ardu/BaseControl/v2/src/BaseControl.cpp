/*! @mainpage
/******************************************************************
 * soft.vers:             ros base link control
 * arduino IDE version:   1.8.5  linux-64
 *                        
 * arduino board version: Arduino due
 * bootloader:            standard
 * libraries:             ros
 * interface:             serial: 115200,8,N,1
 *                        
 * last revision:         11.09.2018 coding
 *                        10.10.2018 testing 1
 *                        14.12.2018 camera actuators
 *                        24.12.2018 camera relay test 2
 *                        19.06.2021 eliminato ros1 / ros2 interface
 *                        
 * TESTS COMMAND LINE:    rostopic pub base/status_led std_msgs/Bool True --once   ---> LED ON
 *                        rostopic pub base/status_led std_msgs/Bool False --once  ---> LED OFF
 *                        rostopic pub base/cam_light_led std_msgs/Bool False --once  ---> LED OFF
 *                        rostopic pub base/cam_light_led std_msgs/Bool True --once   ---> LED ON
 * 
 *                        rostopic echo base/btn_shutdown_cb    ---> True = pressed
 *                        rostopic echo base/btn_emergency   ---> True = pressed
 *                        rostopic echo base/pir_n
 *                        rostopic echo base/pir_se
 *                        rostopic echo base/pir_sw
 * 
 * CAM COMMANDS:          rostopic pub base/cam_pan std_msgs/Int16 90 --once
 *                        rostopic pub base/cam_tilt std_msgs/Int16 90 --once
 *                        rostopic pub base/cam_ir_led std_msgs/Bool True --once
 *                        rostopic pub base/cam_light_led std_msgs/Bool True --once
 * 
 * MATEK LEDS:            rostopic pub base/strip_led std_msgs/ColorRGBA 0 255 0 0 --once
 *                                                                       R  G  B  index [0-5]
 ******************************************************************/

#pragma GCC push_options
#pragma GCC optimize ("O3")
#include <stdint.h>

//------------------------------------------------------------------------------
#define BANNER1  "*-BASE LINK-*"
#define VERSION  "*-20210623--*"

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

#define PWM_WHEEL_L       (3)
#define PWM_WHEEL_R       (2)

#define aggKp             4.00
#define aggKi             0.20
#define aggKd             1.00
#define consKp            1.00
#define consKi            0.05
#define consKd            0.25

Servo servoL;
PID wlPID(&vwLin, &vwLout, &vwLset, consKp, consKi, consKd, DIRECT);
float xwLin[2];        // left wheel position X
float vwLin;           // left wheel velocity V
float vwLset;          // left wheel velocity V
float vwLout;          // left wheel velocity V
float awL;             // left wheel acceleration A

Servo servoR;
PID wrPID(&vwRin, &vwRout, &vwRset, consKp, consKi, consKd, DIRECT);
float xwRin[2];        // right wheel position X
float vwRin;           // right wheel velocity V
float vwRset;          // right wheel velocity V
float vwRout;          // right wheel velocity V
float awR;             // right wheel acceleration A

#include <Derivs_Limiter.h>
Derivs_Limiter limiterCamPan = Derivs_Limiter(100, 75);
Servo camPan;
Derivs_Limiter limiterCamTilt = Derivs_Limiter(100, 75);
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

//-- ROS INTERFACE -------------------------------------------------------------
// sites: https://answers.ros.org/question/164191/rosserial-arduino-cant-connect-arduino-micro/
//#define  USE_USBCON
// 
//#include <ros.h>
//#include <std_msgs/Int16.h>
//#include <std_msgs/Int32.h>
//#include <std_msgs/Bool.h>
//#include <std_msgs/Int16MultiArray.h>
//#include <std_msgs/ColorRGBA.h>

#define ROS_WHEEL_MIN   (-1.0)
#define ROS_WHEEL_MAX   (+1.0)
//ros::NodeHandle  nh;

//int valueL = 0;
//int valueR = 0;

// set left wheel velocity + --> forward
void wheelLvCb(const std_msgs::Int16& cmd_msg) {
  vwLset = constrain(cmd_msg.data, ROS_WHEEL_MIN, ROS_WHEEL_MAX);
  servoL.write(mapf(vwLset, ROS_WHEEL_MIN, ROS_WHEEL_MAX, 0, 180));  
}

ros::Subscriber<std_msgs::Int16> subL("base/wheel_lv", wheelLvCb);

// set right wheel velocity + --> forward
void wheelRvCb(const std_msgs::Int16& cmd_msg) {
  vwRset = constrain(cmd_msg.data, ROS_WHEEL_MIN, ROS_WHEEL_MAX);
  servoR.write(mapf(vwRset, ROS_WHEEL_MIN, ROS_WHEEL_MAX, 180, 0));   
}

ros::Subscriber<std_msgs::Int16> subR("base/wheel_rv", wheelRvCb);

// set status LED
void statusLEDCb(const std_msgs::Bool& cmd_msg) {
  if (cmd_msg.data) {
      ledSet('H');
  } else {
      ledSet('Z');
  }
}

ros::Subscriber<std_msgs::Bool> subStatusLED("base/status_led", statusLEDCb);

// camera mount LED
void camLightLEDCb(const std_msgs::Bool& cmd_msg) {
  if (cmd_msg.data) {
      digitalWrite(CAMERA_MOUNT_LIGHT_PIN, HIGH);
  } else {
      digitalWrite(CAMERA_MOUNT_LIGHT_PIN, LOW);
  }
}

ros::Subscriber<std_msgs::Bool> subLightLED("base/cam_light_led", camLightLEDCb);

// camera mount IR LED
void camIrLEDCb(const std_msgs::Bool& cmd_msg) {
  if (cmd_msg.data) {
      digitalWrite(CAMERA_MOUNT_IR_PIN, LOW);
  } else {
      digitalWrite(CAMERA_MOUNT_IR_PIN, HIGH);
  }
}

ros::Subscriber<std_msgs::Bool> subIrLED("base/cam_ir_led", camIrLEDCb);

// set camera pan [0, 180] + --> left (ref. on base frame, face forward)
int camPanValue = 90;

void camPanCb(const std_msgs::Int16& cmd_msg) {
  camPanValue = constrain(cmd_msg.data, 0, 180);
  camPan.write(camPanValue);  
}

ros::Subscriber<std_msgs::Int16> camPanMount("base/cam_pan", camPanCb);

// set camera tilt [0, 180] + --> down(ref. on base frame)
int camTiltValue = 90;

void camTiltCb(const std_msgs::Int16& cmd_msg) {
  camTiltValue = constrain(cmd_msg.data, 0, 180);
  camTilt.write(camTiltValue);  
}

ros::Subscriber<std_msgs::Int16> camTiltMount("base/cam_tilt", camTiltCb);

// buzzer tone
void buzzerCb(const std_msgs::Bool& cmd_msg) {
  if (cmd_msg.data) {
      digitalWrite(FAST_BUZZER, LED_ON);
  } else {
      digitalWrite(FAST_BUZZER, LED_OFF);
  }
}

ros::Subscriber<std_msgs::Bool> buzzer("base/buzzer", buzzerCb);

//
// matek led strip RGB
// limit: 15.12.2018 only 1 LED a is the index
// TODO: extend to 6 leds
//    custom messages:
//    http://wiki.ros.org/rosserial/Tutorials/Adding%20Other%20Messages
//
void stripLedCb(const std_msgs::ColorRGBA& cmd_msg) {
  byte led_index = constrain(cmd_msg.a, 0, NUM_LEDS - 1);
  fastleds[led_index].setRGB(cmd_msg.r, cmd_msg.g, cmd_msg.b);
  FastLED.show();
}

ros::Subscriber<std_msgs::ColorRGBA> stripLed("base/strip_led", stripLedCb);


std_msgs::Bool bshut;
std_msgs::Bool bemer;
#ifdef HAVE_BUMPERS
std_msgs::Bool bbumpN;
std_msgs::Bool bbumpNE; 
std_msgs::Bool bbumpE;
std_msgs::Bool bbumpSE;
std_msgs::Bool bbumpS;
std_msgs::Bool bbumpSW;
std_msgs::Bool bbumpW;
std_msgs::Bool bbumpNW;
#endif
std_msgs::Bool bpirN;
std_msgs::Bool bpirSE;
std_msgs::Bool bpirSW;

ros::Publisher bshutPub("base/btn_shutdown_cb", &bshut);
ros::Publisher bemerPub("base/btn_emergency", &bemer);
#ifdef HAVE_BUMPERS
ros::Publisher bbumpNPub("base/bump_n", &bbumpN);
ros::Publisher bbumpNEPub("base/bump_ne", &bbumpNE);
ros::Publisher bbumpEPub("base/bump_e", &bbumpE);
ros::Publisher bbumpSEPub("base/bump_se", &bbumpSE);
ros::Publisher bbumpSPub("base/bump_s", &bbumpS);
ros::Publisher bbumpSWPub("base/bump_sw", &bbumpSW);
ros::Publisher bbumpWPub("base/bump_w", &bbumpW);
ros::Publisher bbumpNWPub("base/bump_nw", &bbumpNW);
#endif
ros::Publisher bpirNPub("base/pir_n", &bpirN);
ros::Publisher bpirSEPub("base/pir_se", &bpirSE);
ros::Publisher bpirSWPub("base/pir_sw", &bpirSW);

//-- ENCODER DRIVER  ------------------------------------------------------------
#include <Encoder.h>

// quadrature encoder
//Encoder leftEnc(A0, A1);
//Encoder rightEnc(A2, A3);
// single pulse counter

#define PULSE_LEFT_PIN       (39)
#define PULSE_RIGHT_PIN      (40)

int16_t ln;
int16_t rn;
int16_t leftPosition;
int16_t rightPosition;

std_msgs::Int16 lCount;
std_msgs::Int16 rCount;

ros::Publisher lCountPub("base/lwheel", &lCount);
ros::Publisher rCountPub("base/rwheel", &rCount);

void encoderInit() {
  // quadrature
  //leftEnc.write(0);
  //rightEnc.write(0);
  // single counter
  attachInterrupt(PULSE_LEFT_PIN, onLeftPulse, FALLING);
  attachInterrupt(PULSE_RIGHT_PIN, onRightPulse, FALLING);
}

void onLeftPulse() {
  if (valueL > 0) {
    ln++;
  } else {
    ln--;
  }
}

void onRightPulse() {
  if (valueR > 0) {
    rn++;
  } else {
    rn--;
  }
}

void encoderProcess() {
  // quadrature
  //int16_t ln = leftEnc.read();
  //int16_t rn = rightEnc.read();
  // single. no op.
  
  if (ln != leftPosition) {
    leftPosition = ln;
    lCount.data = leftPosition;
    lCountPub.publish(&lCount);
  }
  
  if (rn != rightPosition) {
    rightPosition = rn;
    rCount.data = rightPosition;
    rCountPub.publish(&rCount);
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
    bshut.data = !BTNshutValue;
    bshutPub.publish(&bshut);
    if (BTNshutValue == LOW) {
        ledSet('B');
    } else {

    }
  }

  if (BTNemerValue != BTNemer.read()) {
    BTNemerValue = BTNemer.read();
    bemer.data = BTNemerValue;
    bemerPub.publish(&bemer);
    if (BTNemerValue == LOW) {

    } else {

    }
  }

#ifdef HAVE_BUMPERS
  if (BTNbumpNValue != BTNbumpN.read()) {
    BTNbumpNValue = BTNbumpN.read();
    bbumpN.data = BTNbumpNValue;
    bbumpNPub.publish(&bbumpN);
    if (BTNbumpNValue == LOW) {

    } else {

    }
  }

  if (BTNbumpNEValue != BTNbumpNE.read()) {
    BTNbumpNEValue = BTNbumpNE.read();
    bbumpNE.data = BTNbumpNEValue;
    bbumpNEPub.publish(&bbumpNE);
    if (BTNbumpNEValue == LOW) {

    } else {

    }
  }

  if (BTNbumpEValue != BTNbumpE.read()) {
    BTNbumpEValue = BTNbumpE.read();
    bbumpE.data = BTNbumpEValue;
    bbumpEPub.publish(&bbumpE);
    if (BTNbumpEValue == LOW) {

    } else {

    }
  }

  if (BTNbumpSEValue != BTNbumpSE.read()) {
    BTNbumpSEValue = BTNbumpSE.read();
    bbumpSE.data = BTNbumpSEValue;
    bbumpSEPub.publish(&bbumpSE);
    if (BTNbumpSEValue == LOW) {

    } else {

    }
  }

  if (BTNbumpSValue != BTNbumpS.read()) {
    BTNbumpSValue = BTNbumpS.read();
    bbumpS.data = BTNbumpSValue;
    bbumpSPub.publish(&bbumpS);
    if (BTNbumpSValue == LOW) {

    } else {

    }
  }

  if (BTNbumpSWValue != BTNbumpSW.read()) {
    BTNbumpSWValue = BTNbumpSW.read();
    bbumpSW.data = BTNbumpSWValue;
    bbumpSWPub.publish(&bbumpSW);
    if (BTNbumpSWValue == LOW) {

    } else {

    }
  }

  if (BTNbumpWValue != BTNbumpW.read()) {
    bbumpW.data = BTNbumpWValue;
    bbumpWPub.publish(&bbumpW);
    BTNbumpWValue = BTNbumpW.read();
    if (BTNbumpWValue == LOW) {

    } else {

    }
  }

  if (BTNbumpNWValue != BTNbumpNW.read()) {
    BTNbumpNWValue = BTNbumpNW.read();
    bbumpNW.data = BTNbumpNWValue;
    bbumpNWPub.publish(&bbumpNW);
    if (BTNbumpNWValue == LOW) {

    } else {

    }
  }
#endif

  if (BTNpirNValue != BTNpirN.read()) {
    BTNpirNValue = BTNpirN.read();
    bpirN.data = BTNpirNValue;
    bpirNPub.publish(&bpirN);
    if (BTNpirNValue == LOW) {

    } else {

    }
  }

  if (BTNpirSEValue != BTNpirSE.read()) {
    BTNpirSEValue = BTNpirSE.read();
    bpirSE.data = BTNpirSEValue;
    bpirSEPub.publish(&bpirSE);
    if (BTNpirSEValue == LOW) {

    } else {

    }
  }

  if (BTNpirSWValue != BTNpirSW.read()) {
    BTNpirSWValue = BTNpirSW.read();
    bpirSW.data = BTNpirSWValue;
    bpirSWPub.publish(&bpirSW);
    if (BTNpirSWValue == LOW) {

    } else {

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
       //nh.loginfo("B");
       switch (ledState) {
         case 0:
           //nh.loginfo("0");
           digitalWrite(LED_READY_PIN, LED_ON);
           ledNextState(LED_BLINK_SHORT_MS);
           break;
         case 1:
           //nh.loginfo("1");
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
  //
  nh.subscribe(subLightLED);
  nh.subscribe(subIrLED);
  nh.subscribe(camPanMount);
  nh.subscribe(camTiltMount);
  // initial values: center axis
  camPan.write(camPanValue);
  camTilt.write(camTiltValue);
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
  
  nh.subscribe(buzzer);
  nh.subscribe(stripLed);
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
  encoderInit();
  
  nh.getHardware()->setBaud(UART_BAUDRATE);
  nh.initNode();
  //
  nh.subscribe(subL);
  nh.subscribe(subR);
  nh.subscribe(subStatusLED);
  //
  nh.advertise(bshutPub);
  nh.advertise(bemerPub);
#ifdef HAVE_BUMPERS
  nh.advertise(bbumpNPub);
  nh.advertise(bbumpNEPub);
  nh.advertise(bbumpEPub);
  nh.advertise(bbumpSEPub);
  nh.advertise(bbumpSPub);
  nh.advertise(bbumpSWPub);
  nh.advertise(bbumpWPub);
  nh.advertise(bbumpNWPub);
#endif
  nh.advertise(bpirNPub);
  nh.advertise(bpirSEPub);
  nh.advertise(bpirSWPub);
  //
  nh.advertise(lCountPub);
  nh.advertise(rCountPub);
  // drive
  servoL.attach(PWM_WHEEL_L);
  servoR.attach(PWM_WHEEL_R);
  // camera mount
  camMountInit();
  // Matek Board
  stripLedInit();
  buzzerInit();
  
  // wait until you are actually connected
  while (!nh.connected()) {
    WATCHDOG_RESET;
    nh.spinOnce();
    ledProcess();
  }
  nh.loginfo(BANNER1);
  nh.loginfo(VERSION);
  nh.loginfo("*** PLEASE, ARM MOTORS ***");
}

void loop() {
  ledProcess();
  nh.spinOnce();
  buttonProcess();
  encoderProcess();
  
  WATCHDOG_RESET;
}

//-----------------------------------------------------------------------------------------------------------------------
/*
   Copyright (c) 2019 Stefan Kremser
   This software is licensed under the MIT License. See the license file for details.
   Source: github.com/spacehuhn/SimpleCLI
 */

// Watch the tutorial here: https://youtu.be/UyW-wICdnKo

 #include <SimpleCLI.h>

SimpleCLI cli;

Command cmdPing;
Command cmdPong;

void errorCallback(cmd_error* errorPtr) {
    CommandError e(errorPtr);

    Serial.println("ERROR: " + e.toString());

    if (e.hasCommand()) {
        Serial.println("Did you mean? " + e.getCommand().toString());
    } else {
        Serial.println(cli.toString());
    }
}

void pongCallback(cmd* cmdPtr) {
    Command cmd(cmdPtr);

    int argNum = cmd.countArgs();

    for (int i = 0; i < argNum; i++) {
        Serial.println(cmd.getArgument(i).getValue());
    }
}

void pingCallback(cmd* cmdPtr) {
    Command cmd(cmdPtr);

    Argument argN   = cmd.getArgument("num");
    String   argVal = argN.getValue();
    int n           = argVal.toInt();

    Argument argStr = cmd.getArgument("str");
    String   strVal = argStr.getValue();

    Argument argC = cmd.getArgument("c");
    bool     c    = argC.isSet();

    if (c) strVal.toUpperCase();

    for (int i = 0; i < n; i++) {
        Serial.println(strVal);
    }
}

void setup_2() {
    Serial.begin(9600);
    Serial.println("Hello World");

    cmdPing = cli.addCommand("ping", pingCallback);
    cmdPing.addPositionalArgument("str", "pong");
    cmdPing.addArgument("n/um/ber,anzahl", "1");
    cmdPing.addFlagArgument("c");

    cmdPong = cli.addBoundlessCommand("pong,hello", pongCallback);

    cli.setOnError(errorCallback);
}

void loop_2() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        Serial.println("# " + input);

        cli.parse(input);
    }
}
