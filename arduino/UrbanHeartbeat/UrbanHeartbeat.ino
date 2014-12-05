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
// NeoPixel LEDs
// ~6
// END WIRING

#include <Adafruit_NeoPixel.h>
#include <SD.h>

// START CONSTANTS
// START CONTROL CONSTANTS
#define SERIAL_PORT 9600
#define LOOP_DELAY 85
#define LV1_TICK_INC PI/30
#define LV2_TICK_INC PI/15
#define LV3_TICK_INC PI/7
#define SECOND 1000
#define MINUTE 60000
// END CONTROL CONSTANTS

// START STATE CONSTANTS
#define ON 1
#define OFF -1
// END STATE CONSTANTS

// START SENSOR MODULE CONSTANTS
#define ULTRASONIC_THRESHOLD 185 // FOR RELEASE
//#define ULTRASONIC_THRESHOLD 15 // FOR DEBUGGING
#define TRIG_PIN 8
#define ECHO_PIN 7
#define TRIG_DELAY_LOW_HIGH 2
#define TRIG_DELAY_HIGH_LOW 10
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
float lv1Tick = 0.0f;
float lv2Tick = 0.0f;
float lv3Tick = 0.0f;
int state = OFF;
long started = -1;
long startElapsed = 0;
long left = -1;
long leaveElapsed = 0;
// END CONTROL GLOBAL VARIABLES

// START SENSOR MODULE GLOBAL VARIABLES
long distance = 300;
long duration = 0;
// END SENSOR MODULE GLOBAL VARIABLES

// START DATA MODULE GLOBAL VARIABLES
const char FILE_NAME[] = "data.txt";
// END DATA MODULE GLOBAL VARIABLES

// START ACTIVITY VARIABLES
boolean lv1Lasting = false;
boolean lv2Lasting = false;
boolean lv3Lasting = false;
// END ACTIVITY VARIABLES

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// END GLOBAL VARIABLES

void setup() {
  // SET SERIAL CONNECTION
  Serial.begin(SERIAL_PORT);

  // START SENSOR MODULE SETUP
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
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

void loop() {
  // START SENSOR MODULE
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(TRIG_DELAY_LOW_HIGH);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(TRIG_DELAY_HIGH_LOW);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = (duration/2) / 29.1;
  //  Serial.println(distance);
  // END SENSOR MODULE

  // START STATE HANDLER
  if(distance <= ULTRASONIC_THRESHOLD) {
    
    if(state==OFF) {
      // START RECORDING TIMESTAMP
      String timestamp = String(millis());
      File dataFile = SD.open(FILE_NAME, FILE_WRITE);
      if(dataFile) {
        dataFile.print(timestamp);
        dataFile.println(" LEVEL1ON");
        dataFile.close();
        // print to the serial port too:
        Serial.println(timestamp);
      }
      else {
        Serial.println("error opening " + String(FILE_NAME));
      }
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
    
    if(state==ON) {
      // START RECORDING TIMESTAMP
      String timestamp = String(millis());
      File dataFile = SD.open(FILE_NAME, FILE_WRITE);
      if(dataFile) {
        dataFile.print(timestamp);
        dataFile.println(" LEVEL1OFF");
        dataFile.close();
        // print to the serial port too:
        Serial.println(timestamp);
      }
      else {
        Serial.println("error opening " + String(FILE_NAME));
      }
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

  Serial.print("startElapsed: ");
  Serial.print(startElapsed);
  Serial.print(" leaveElapsed: ");
  Serial.println(leaveElapsed);
  // END STATE HANDLER

  // START LIGHT MODULE
  // START LIGHT LEVEL 1
  if(state == ON) {    
    double lv1red1 = (sin(lv1Tick) + 1) * 127;
    double lv1green2 = (cos(lv1Tick) + 1) * 127;
    pixels.setPixelColor(0, pixels.Color(0, lv1red1, 40));
    pixels.setPixelColor(1, pixels.Color(lv1green2, 50, lv1green2));
  }
  else {
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1, pixels.Color(0, 0, 0));
  }
  // END LIGHT LEVEL 1
  
  // START LIGHT LEVEL 2
  if(startElapsed > 10 * SECOND
  || (lv2Lasting && leaveElapsed < 15 * SECOND)) {
    
    if(!lv2Lasting) {
      // START RECORDING TIMESTAMP
      String timestamp = String(millis());
      File dataFile = SD.open(FILE_NAME, FILE_WRITE);
      if(dataFile) {
        dataFile.print(timestamp);
        dataFile.println(" LEVEL2ON");
        dataFile.close();
        // print to the serial port too:
        Serial.println(timestamp);
      }
      else {
        Serial.println("error opening " + String(FILE_NAME));
      }
      // END RECORDING TIMESTAMP
    }
    
    lv2Lasting = true;
    double lv2red1 = (sin(lv2Tick) + 1) * 127;
    double lv2green2 = (cos(lv2Tick) + 1) * 127;
    pixels.setPixelColor(2, pixels.Color(0, lv2red1, 40));
    pixels.setPixelColor(3, pixels.Color(lv2green2, 0, 0));
  }
  else {
    
    if(lv2Lasting) {
      // START RECORDING TIMESTAMP
      String timestamp = String(millis());
      File dataFile = SD.open(FILE_NAME, FILE_WRITE);
      if(dataFile) {
        dataFile.print(timestamp);
        dataFile.println(" LEVEL2OFF");
        dataFile.close();
        // print to the serial port too:
        Serial.println(timestamp);
      }
      else {
        Serial.println("error opening " + String(FILE_NAME));
      }
      // END RECORDING TIMESTAMP
    }
    
    lv2Lasting = false;
    pixels.setPixelColor(2, pixels.Color(0, 0, 0));
    pixels.setPixelColor(3, pixels.Color(0, 0, 0));
  }
  // END LIGHT LEVEL 2
  
  // START LIGHT LEVEL 3
  if(startElapsed > MINUTE
  || (lv3Lasting && leaveElapsed < 90 * SECOND)) {
    
    if(!lv3Lasting) {
      // START RECORDING TIMESTAMP
      String timestamp = String(millis());
      File dataFile = SD.open(FILE_NAME, FILE_WRITE);
      if(dataFile) {
        dataFile.print(timestamp);
        dataFile.println(" LEVEL2ON");
        dataFile.close();
        // print to the serial port too:
        Serial.println(timestamp);
      }
      else {
        Serial.println("error opening " + String(FILE_NAME));
      }
      // END RECORDING TIMESTAMP
    }
    
    lv3Lasting = true;
    double lv3blue1 = (sin(lv3Tick) + 1) * 127;
    double lv3red2 = (cos(lv3Tick) + 1) * 127;
    pixels.setPixelColor(4, pixels.Color(0, 0, lv3blue1));
    pixels.setPixelColor(5, pixels.Color(60, lv3red2, 0));
  }
  else {
    
    if(lv3Lasting) {
      // START RECORDING TIMESTAMP
      String timestamp = String(millis());
      File dataFile = SD.open(FILE_NAME, FILE_WRITE);
      if(dataFile) {
        dataFile.print(timestamp);
        dataFile.println(" LEVEL2OFF");
        dataFile.close();
        // print to the serial port too:
        Serial.println(timestamp);
      }
      else {
        Serial.println("error opening " + String(FILE_NAME));
      }
      // END RECORDING TIMESTAMP
    }
    
    lv3Lasting = false;
    pixels.setPixelColor(4, pixels.Color(0, 0, 0));
    pixels.setPixelColor(5, pixels.Color(0, 0, 0));
  }
  // END LIGHT LEVEL 3
  
  pixels.show();
  // END LIGHT MODULE

  lv1Tick += LV1_TICK_INC;
  if(lv1Tick > 2 * PI) lv1Tick = 0;

  lv2Tick += LV2_TICK_INC;
  if(lv2Tick > 2 * PI) lv2Tick = 0;

  lv3Tick += LV3_TICK_INC;
  if(lv3Tick > 2 * PI) lv3Tick = 0;

  delay(LOOP_DELAY);
}

