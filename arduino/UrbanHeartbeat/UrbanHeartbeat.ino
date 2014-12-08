// AUTHOR: Seongtaek Lim
//
// START WIRING
// Ultrasonic
// Echo -> 7
// Trig -> 8
//
// SD card slot
// CS -> 4
// MOSI-> ~11
// CSK -> 13
// MISO -> 12
//
// NeoPixel LEDs (DAISY CHAINED)
// ~6
//
// Reset Wire
// A0
//
// Mode Changer Wire
// A1
// END WIRING

#include <Adafruit_NeoPixel.h>
#include <SD.h>
#include <EEPROM.h>

// START CONSTANTS
// START CONTROL CONSTANTS
#define RESET_PIN 0
#define RESET_THRESHOLD 20
#define RESET_EEPROM_ADDRESS 0
#define RESET_ENABLED 1
#define RESET_DISABLED 0
#define SERIAL_PORT 9600
#define LOOP_DELAY 100
#define LV1_TICK_INC PI/30
#define LV2_TICK_INC PI/15
#define LV3_TICK_INC PI/7
#define FAST_TICK_INC PI/5
#define FASTER_TICK_INC PI/3
#define SECOND 1000
#define MINUTE 60000
// END CONTROL CONSTANTS

// START STATE CONSTANTS
#define ON 1
#define OFF -1
// END STATE CONSTANTS

// START MODE CHANGER MODULE CONSTANTS
#define MODE_PIN 1
#define MODE_THRESHOLD 1
#define NO_LIGHT_MODE 10
#define LIGHT_MODE 20
// END MODE CHANGER MODULE CONSTANTS

// START SENSOR MODULE CONSTANTS
#define NUM_SAMPLES 10
#define ULTRASONIC_THRESHOLD 145 // FOR RELEASE
//#define ULTRASONIC_THRESHOLD 15 // FOR DEBUGGING
#define FASTER_THRESHOLD_LOWER 15
#define FASTER_THRESHOLD_UPPER 25
#define FAST_THRESHOLD_UPPER 35
#define TRIG_PIN 8
#define ECHO_PIN 7
#define TRIG_DELAY_LOW_HIGH 2
#define TRIG_DELAY_HIGH_LOW 5
// END SENSOR MODULE CONSTANTS

// START LIGHT MODULE CONSTANTS
#define LED_PIN 6 // Which pin on the Arduino is connected to the NeoPixels?
#define NUM_LEDS 6 // How many NeoPixels are attached to the Arduino?
// END LIGHT MODULE CONSTANTS

// START DATA MODULE CONSTANTS
#define SD_CHIP_SELECT 4
#define SD_DEFAULT_CHIP_SELECT 10
// END DATA MODULE CONSTANTS
// END CONSTANTS

// START GLOBAL VARIABLES
// START CONTROL GLOBAL VARIABLES
int easterEggVal = 0;
int resetVal = 0;
int eepromVal = RESET_DISABLED;
float lv1Tick = 0.0f;
float lv2Tick = 0.0f;
float lv3Tick = 0.0f;
int state = OFF;
long started = -1;
unsigned long startElapsed = 0;
long left = -1;
unsigned long leaveElapsed = 0;
// END CONTROL GLOBAL VARIABLES

// START MODE CHANGER GLOBAL VARIABLES
int modeVal = 0;
int mode = NO_LIGHT_MODE;
// END MODE CHANGER GLOBAL VARIABLES

// START SENSOR MODULE GLOBAL VARIABLES
int distance = 300;
int duration = 0;
int sensorInputs[NUM_SAMPLES];
int sensorInputIndex = 0;
int sensorInputTotal = 0;
int sensorInputAvg = 0;
// END SENSOR MODULE GLOBAL VARIABLES

// START DATA MODULE GLOBAL VARIABLES
const char FILE_NAME[] = "data.txt";
// END DATA MODULE GLOBAL VARIABLES

// START ACTIVITY VARIABLES
boolean lv1Lasting = false;
boolean lv2Lasting = false;
boolean lv3Lasting = false;
const int level2OnDelay = 10 * SECOND;
const int level2OffDelay = 10 * SECOND;
const int level3OnDelay = 30 * SECOND;
const int level3OffDelay = 30 * SECOND;
// END ACTIVITY VARIABLES

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
// END GLOBAL VARIABLES

void setup() {
  // SET SERIAL CONNECTION
  Serial.begin(SERIAL_PORT);
  
  // START RESET HANDLER SETUP
//  pinMode(RESET_PIN, INPUT);
  // END RESET HANDLER SETUP
  
  // START MODE CHANGER SETUP
//  pinMode(MODE_PIN, INPUT);
  // END MODE CHANGER SETUP
  
  // START STATE HANDLER SETUP
  /*lv1Tick = 0.0f;
  lv2Tick = 0.0f;
  lv3Tick = 0.0f;
  state = OFF;
  started = -1;
  startElapsed = 0;
  left = -1;
  leaveElapsed = 0;
  lv1Lasting = false;
  lv2Lasting = false;
  lv3Lasting = false;*/
  // END STATE HANDLER SETUP
  
  // START SENSOR MODULE SETUP
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  distance = 300;
  duration = 0;
  for(int i=0;i<NUM_SAMPLES;i++) sensorInputs[i] = 0;
  // END SENSOR MODULE SETUP
  
  // START DATA MODULE SETUP
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(SD_DEFAULT_CHIP_SELECT, OUTPUT);
  // see if the card is present and can be initialized:
  if (!SD.begin(SD_CHIP_SELECT)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  // END DATA MODULE SETUP
  
  // START LIGHT MODULE SETUP
  pinMode(LED_PIN, OUTPUT);

  // INITIALIZE NeoPixel LIBRARY
  pixels.begin();
}
//void(* resetFunc) (void) = 0;

void loop() {
  
  // START RESET HANDLER
  /*resetVal = analogRead(RESET_PIN);
  eepromVal = EEPROM.read(RESET_EEPROM_ADDRESS);
  if(resetVal < RESET_THRESHOLD) { // WHEN CONNECTED
    // START PREVENTING RESET
    EEPROM.write(RESET_EEPROM_ADDRESS, RESET_DISABLED);
    // END PREVENTING RESET
  }
  else if(resetVal >= RESET_THRESHOLD) { // WHEN DISCONNECTED
    if(eepromVal == RESET_DISABLED) {
      Serial.println("RESET");
      EEPROM.write(RESET_EEPROM_ADDRESS, RESET_ENABLED);
      resetFunc();
    }
    else if(eepromVal == RESET_ENABLED) { // AFTER REBOOT
      EEPROM.write(RESET_EEPROM_ADDRESS, RESET_DISABLED);
    }
  }*/
  // END RESET HANDLER
  
  // START SENSOR MODULE
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(TRIG_DELAY_LOW_HIGH);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(TRIG_DELAY_HIGH_LOW);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = (duration / 2) / 29.1;
//  Serial.println(distance);
  // END SENSOR MODULE
  
  // START MODE HANDLER
  /*modeVal = analogRead(MODE_PIN);
  if(modeVal < RESET_THRESHOLD) { // WHEN CONNECTED
    mode = NO_LIGHT_MODE;
  }
  else if(modeVal >= RESET_THRESHOLD) { // WHEN DISCONNECTED
    mode = LIGHT_MODE;
  }*/
  // END MODE HANDLER

  // START STATE HANDLER
  // START SMOOTHING
  sensorInputTotal = sensorInputTotal - sensorInputs[sensorInputIndex];
  sensorInputs[sensorInputIndex] = distance;
  sensorInputTotal = sensorInputTotal + sensorInputs[sensorInputIndex];
  sensorInputIndex++;
  if(sensorInputIndex >= NUM_SAMPLES) sensorInputIndex = 0;
  sensorInputAvg = sensorInputTotal / NUM_SAMPLES;
  Serial.println(sensorInputAvg);
  // END SMOOTHING
  
  // START EASTER EGG HANDLER
  if(sensorInputAvg >= FASTER_THRESHOLD_LOWER
  && sensorInputAvg <= FASTER_THRESHOLD_UPPER) easterEggVal = 2;
  else if(sensorInputAvg <= FAST_THRESHOLD_UPPER) easterEggVal = 1;
  else easterEggVal = 0;
  // END EASTER EGG HANDLER
  
  if(sensorInputAvg <= ULTRASONIC_THRESHOLD) {
    // SOMEONE IS HERE
    
    if(state == OFF) { // WHEN IT WAS OFF
      // START RECORDING TIMESTAMP
      String timestamp = String(millis());
      String distanceStr = "\t" + String(sensorInputAvg);
      /*String modeStr;
      if(mode == NO_LIGHT_MODE) modeStr = "\tNOLIGHT";
      else modeStr = "\tLIGHT";*/
      String levelStr = "\tLEVEL1";
      String stateStr = "\tON";
      writeLineOnSD(FILE_NAME, timestamp + distanceStr /*+ modeStr*/ + levelStr + stateStr);
      // END RECORDING TIMESTAMP
    }
    
    state = ON;
    left = -1;
    leaveElapsed = 0;

    if(started < 0) {
      started = millis();
    }

    startElapsed = millis() - started;
  }
  else {
    // NOONE IS HERE
    
    if(state == ON) { // WHEN IT WAS ON
      // START RECORDING TIMESTAMP
      String timestamp = String(millis());
      String distanceStr = "\t" + String(sensorInputAvg);
      /*String modeStr;
      if(mode == NO_LIGHT_MODE) modeStr = "\tNOLIGHT";
      else if(mode == LIGHT_MODE) modeStr = "\tLIGHT";*/
      String levelStr = "\tLEVEL1";
      String stateStr = "\tOFF";
      writeLineOnSD(FILE_NAME, timestamp + distanceStr /*+ modeStr*/ + levelStr + stateStr);
      // END RECORDING TIMESTAMP
    }
    
    state = OFF;
    started = -1;
    startElapsed = 0;

    if(left < 0) {
      left = millis();
    }

    leaveElapsed = millis() - left;
  }

//  Serial.print("startElapsed: ");
//  Serial.print(startElapsed);
//  Serial.print(" leaveElapsed: ");
//  Serial.println(leaveElapsed);
  // END STATE HANDLER

  // START LIGHT MODULE
  // START LIGHT LEVEL 1
  if(state == ON) { // WHEN SONEONE IS HERE
    // START TURNING ON LIGHT LEVEL 1
    double lv1red1 = (sin(lv1Tick) + 1) * 127;
    double lv1green2 = (cos(lv1Tick) + 1) * 127;
//    if(mode == LIGHT_MODE) {
      pixels.setPixelColor(0, pixels.Color(0, lv1red1, 40));
      pixels.setPixelColor(1, pixels.Color(lv1green2, 50, lv1green2));
//    }
    // END TURNING ON LIGHT LEVEL 1
  }
  else { // WHEN NOONE IS HERE
    // START TURNING OFF LIGHT LEVEL 1
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1, pixels.Color(0, 0, 0));
    // END TURNING OFF LIGHT LEVEL 1
  }
  // END LIGHT LEVEL 1
  
  // START LIGHT LEVEL 2
  if(startElapsed > level2OnDelay // WHEN SOMEONE STAYED FOR LONG ENOUGH
  || (lv2Lasting && leaveElapsed < level2OffDelay)) { // WHEN THE LIGHT NEEDS TO STAY AFTER HE LEFT
    
    if(!lv2Lasting) { // WHEN LEVEL 2 IS OFF
      // START RECORDING TIMESTAMP
      String timestamp = String(millis());
      String distanceStr = "\t" + String(sensorInputAvg);
      /*String modeStr;
      if(mode == NO_LIGHT_MODE) modeStr = "\tNOLIGHT";
      else if(mode == LIGHT_MODE) modeStr = "\tLIGHT";*/
      String levelStr = "\tLEVEL2";
      String stateStr = "\tON";
      writeLineOnSD(FILE_NAME, timestamp + distanceStr /* + modeStr*/ + levelStr + stateStr);
      // END RECORDING TIMESTAMP
    }
    
    // START TURNING ON LIGHT LEVEL 2
    lv2Lasting = true;
    double lv2red1 = (sin(lv2Tick) + 1) * 127;
    double lv2green2 = (cos(lv2Tick) + 1) * 127;
//    if(mode == LIGHT_MODE) {
      pixels.setPixelColor(2, pixels.Color(0, lv2red1, 40));
      pixels.setPixelColor(3, pixels.Color(lv2green2, 0, 0));
//    }
    // END TURNING ON LIGHT LEVEL 2
  }
  else { // WHEN NOONE HAS BEEN HERE FOR LONG ENOUGH
    
    if(lv2Lasting) { // WHEN LEVEL 2 IS ON
      // START RECORDING TIMESTAMP
      String timestamp = String(millis());
      String distanceStr = "\t" + String(sensorInputAvg);
      /*String modeStr;
      if(mode == NO_LIGHT_MODE) modeStr = "\tNOLIGHT";
      else if(mode == LIGHT_MODE) modeStr = "\tLIGHT";*/
      String levelStr = "\tLEVEL2";
      String stateStr = "\tOFF";
      writeLineOnSD(FILE_NAME, timestamp + distanceStr /* + modeStr*/ + levelStr + stateStr);
      // END RECORDING TIMESTAMP
    }
    
    // START TURNING OFF LIGHT LEVEL 2
    lv2Lasting = false;
    pixels.setPixelColor(2, pixels.Color(0, 0, 0));
    pixels.setPixelColor(3, pixels.Color(0, 0, 0));
    // END TURNING OFF LIGHT LEVEL 2
  }
  // END LIGHT LEVEL 2
  
  // START LIGHT LEVEL 3
  if(startElapsed > level3OnDelay // WHEN SOMEONE STAYED FOR LONG ENOUGH
  || (lv3Lasting && leaveElapsed < level3OffDelay)) { // WHEN THE LIGHT NEEDS TO STAY AFTER HE LEFT
    
    if(!lv3Lasting) { // WHEN LEVEL 3 IS OFF
      // START RECORDING TIMESTAMP
      String timestamp = String(millis());
      String distanceStr = "\t" + String(sensorInputAvg);
      /*String modeStr;
      if(mode == NO_LIGHT_MODE) modeStr = "\tNOLIGHT";
      else if(mode == LIGHT_MODE) modeStr = "\tLIGHT";*/
      String levelStr = "\tLEVEL3";
      String stateStr = "\tON";
      writeLineOnSD(FILE_NAME, timestamp + distanceStr /* + modeStr*/ + levelStr + stateStr);
      // END RECORDING TIMESTAMP
    }
    
    // START TURNING ON LIGHT LEVEL 3
    lv3Lasting = true;
    double lv3blue1 = (sin(lv3Tick) + 1) * 127;
    double lv3red2 = (cos(lv3Tick) + 1) * 127;
//    if(mode == LIGHT_MODE) {
      pixels.setPixelColor(4, pixels.Color(0, 0, lv3blue1));
      pixels.setPixelColor(5, pixels.Color(60, lv3red2, 0));
//    }
    // END TURNING OFF LIGHT LEVEL 3
  }
  else { // WHEN NOONE HAS BEEN HERE FOR LONG ENOUGH
    
    if(lv3Lasting) { // WHEN LEVEL 3 IS ON
      // START RECORDING TIMESTAMP
      String timestamp = String(millis());
      String distanceStr = "\t" + String(sensorInputAvg);
      /*String modeStr;
      if(mode == NO_LIGHT_MODE) modeStr = "\tNOLIGHT";
      else if(mode == LIGHT_MODE) modeStr = "\tLIGHT";*/
      String levelStr = "\tLEVEL3";
      String stateStr = "\tOFF";
      writeLineOnSD(FILE_NAME, timestamp + distanceStr /* + modeStr*/ + levelStr + stateStr);
      // END RECORDING TIMESTAMP
    }
    
    // START TURNING OFF LIGHT LEVEL 3
    lv3Lasting = false;
    pixels.setPixelColor(4, pixels.Color(0, 0, 0));
    pixels.setPixelColor(5, pixels.Color(0, 0, 0));
    // END TURNING OFF LIGHT LEVEL 3
  }
  // END LIGHT LEVEL 3
  
  pixels.show(); // APPLY THE PIXELS
  // END LIGHT MODULE

  // START TICKING
  if(easterEggVal==0) {
    lv1Tick += LV1_TICK_INC;
    lv2Tick += LV2_TICK_INC;
    lv3Tick += LV3_TICK_INC;
  }
  else if(easterEggVal==1) {
    lv1Tick += FAST_TICK_INC;
    lv2Tick += FAST_TICK_INC;
    lv3Tick += FAST_TICK_INC;
    Serial.println(">>> fast");
  }
  else if(easterEggVal==2) {
    lv1Tick += FASTER_TICK_INC;
    lv2Tick += FASTER_TICK_INC;
    lv3Tick += FASTER_TICK_INC;
    Serial.println(">>>>>> faster");
  }
  
  if(lv1Tick > 2 * PI) lv1Tick = 0;
  if(lv2Tick > 2 * PI) lv2Tick = 0;
  if(lv3Tick > 2 * PI) lv3Tick = 0;
  // END TICKING

  delay(LOOP_DELAY);
}

void writeLineOnSD(const char* fileName, String whatToWrite) {
  File dataFile = SD.open(FILE_NAME, FILE_WRITE);
  if(dataFile) {
    dataFile.println(whatToWrite);
    dataFile.close();
    // print to the serial port too:
//    Serial.println(whatToWrite);
  }
  else {
    Serial.println("Error writing " + String(fileName));
  }
}

