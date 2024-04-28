#include <ArduinoBLE.h>
#include <Arduino.h>
#include <U8x8lib.h>

// Display stuff
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

U8X8_SSD1306_128X32_UNIVISION_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);   // Adafruit ESP8266/32u4/ARM Boards + FeatherWing OLED

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

//BLE stuff
#define MS_1024_SCALING_FACTOR 0.977
#define CPF_WHEEL_REVOLUTION_DATA_SUPPORTED ((uint32_t) 1 << 2)
#define CPF_CRANK_REVOLUTION_DATA_SUPPORTED ((uint32_t) 1 << 3)

#define CPM_WHEEL_REV_DATA_PRESENT ((uint16_t)1 << 4)
#define CPM_CRANK_REV_DATA_PRESENT ((uint16_t) 1 << 5)

#define RPM_PWR_LUT_LEN 13

#define POWER_FEATURE_CHAR_LEN 4      // 4 bytes = 32 bits
#define POWER_MEASUREMENT_CHAR_LEN 8 // 2-flags, 2-pwr, 2-crank revs, 2-crank time

#define CSC_FEATURE_CHAR_LEN 2
#define CSC_MEASUREMENT_CHAR_LEN 11  // 1 for flags, 4 for wheel revs, 2 for wheel event time, 2 for crank revs, 2 for crank event time

#define PIN_IN 3
#define DEBOUNCE_TIME 5


const uint32_t cpFeatureValue = CPF_CRANK_REVOLUTION_DATA_SUPPORTED;

const unsigned char sensorLocation = 4;  //front wheel

const uint16_t cpMeasurementFlags = CPM_CRANK_REV_DATA_PRESENT;

unsigned long startTime = 0;
uint32_t revsCount = 0;
uint16_t crankCount = 0;
uint16_t crankTime = 0;
uint16_t lastCrankTime = 0;
int16_t instantaneousPower = 0;
bool shouldUpdate = false;
uint8_t cpValue[POWER_MEASUREMENT_CHAR_LEN];

//Spec: https://www.bluetooth.com/specifications/specs/cycling-power-service-1-1/
BLEService bikePowerService("1818");  //cycling power service
BLECharacteristic cyclingPowerFeatureChar("2A65", BLERead, POWER_FEATURE_CHAR_LEN);
BLECharacteristic cyclingPowerMeasurementChar("2A63", BLENotify, POWER_MEASUREMENT_CHAR_LEN);

//dependency shared by both CP and CSC specs
BLEUnsignedCharCharacteristic cyclingSensorLocationChar("2A5D", BLERead);

// BLEUnsignedCharCharacteristic cyclingSCControlPointChar("2A55", BLERead); needed for speed on Garmin - fuck speed actually!

uint16_t rpm2kph(uint16_t rpm){
  //speed is approximately half of cadence - see https://ergarcade.com/articles/airbike-rpm-speed-power.html
  return rpm >> 1;
}

//https://ergarcade.com/articles/airbike-rpm-speed-power.html
int16_t rpm2watts(uint16_t rpm) {
  // const uint16_t rpms[RPM_PWR_LUT_LEN] =   { 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100 }
  // const uint16_t watts[RPM_PWR_LUT_LEN] =  { 35,64,107,136,170,209,255,308,368,436,512,597,691 }
  int16_t power = 0;
  if (rpm < 30) {
    power = rpm;  //approximately directly linear in first region (based on x,y = 0,0 ; 30,35)
  } else if (rpm < 60) {
    power = (rpm << 2) - 85;  // y= 4x-85
  } else if (rpm < 80) {
    power = (rpm << 3) - 310;  //y= 8x -310
  } else {
    power = (rpm << 4) - 928; //y = 16x - 928
  }
  return power;
}

int16_t wheelspin2watts( uint16_t wheelPeriod ) { 
  uint16_t crankFreq = 5581 / wheelPeriod; // 5581 is conversion from ms to min (60000) * gear ratio factor (4/43)
  return rpm2watts(crankFreq);
}

void testISR() {
  char loopstr[64];
  sprintf(loopstr, "Millis: %d", millis());
  u8x8.clearDisplay();
  u8x8.drawString(0,0,"ISR Occurred");
  u8x8.drawString(0,2,loopstr);
}

void readDataISR() {
  unsigned long stopTime = millis();

  if (startTime == 0) {  //first run
    startTime = stopTime;
    return;
  }
  else if (startTime > stopTime) { //overflow occurred - reset state and continue
    crankCount = 0;
    crankTime = 0;
    lastCrankTime = 0;
    revsCount = 0;
    startTime = stopTime;
    return;
  }

  uint16_t delta = stopTime - startTime;

  if (delta <= DEBOUNCE_TIME) {  //false positive, disregard
    return;
  }
 
  revsCount++;
  instantaneousPower = wheelspin2watts( delta );

  // handle crank cadence
  uint8_t cycle = revsCount % 43;

  if (cycle == 0) {  //last rev
    lastCrankTime = crankTime;
    crankTime = stopTime;
    crankCount++;
  } else if (cycle % 11 == 0) {
    uint16_t r = cycle / 11;
    uint16_t crankDelta = r * (delta >> 2) * 43 - (cycle - 1) * delta;
    lastCrankTime = crankTime;
    crankTime = startTime + crankDelta;
    crankCount++;
  }
  //set state variables
  startTime = stopTime;

  //update BLE
  uint16_t crankTimeValue = lastCrankTime * MS_1024_SCALING_FACTOR;
  cpValue[0] = (uint8_t) ( ( cpMeasurementFlags ) & 0xFF);
  cpValue[1] = (uint8_t) ( ( cpMeasurementFlags >> 8 ) & 0xFF);
  cpValue[2] = (uint8_t) ( ( instantaneousPower ) & 0xFF);
  cpValue[3] = (uint8_t) ( ( instantaneousPower >> 8 ) & 0xFF);
  cpValue[4] = (uint8_t) ( ( crankCount ) & 0xFF);
  cpValue[5] = (uint8_t) ( ( crankCount >> 8 ) & 0xFF);
  cpValue[6] = (uint8_t) ( ( crankTimeValue ) & 0xFF);
  cpValue[7] = (uint8_t) ( ( crankTimeValue ) >> 8 & 0xFF);
  // uint8_t cpValue[POWER_MEASUREMENT_CHAR_LEN] = {
  //   (uint8_t) ( ( cpMeasurementFlags ) & 0xFF),
  //   (uint8_t) ( ( cpMeasurementFlags >> 8 ) & 0xFF),
  //   (uint8_t) ( ( instantaneousPower ) & 0xFF),
  //   (uint8_t) ( ( instantaneousPower >> 8 ) & 0xFF),
  //   (uint8_t) ( ( crankCount ) & 0xFF),
  //   (uint8_t) ( ( crankCount >> 8 ) & 0xFF),
  //   (uint8_t) ( ( crankTimeValue ) & 0xFF),
  //   (uint8_t) ( ( crankTimeValue ) >> 8 & 0xFF),
  // };
  // cyclingPowerMeasurementChar.writeValue(cpValue, POWER_MEASUREMENT_CHAR_LEN, true);
  shouldUpdate = true;
}

void setup() {
  // put your setup code here, to run once:
  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.setFlipMode(1);
  u8x8.setFont(u8x8_font_7x14B_1x2_f );

  // begin initialization
  if (!BLE.begin()) {
    u8x8.clearDisplay();
    u8x8.drawString(0,0,"BLE error:");
    u8x8.drawString(0,2,"Can't start");
    while (1);
  }

  /* Set a local name for the Bluetooth® Low Energy device
     This name will appear in advertising packets
     and can be used by remote devices to identify this Bluetooth® Low Energy device
     The name can be changed but maybe be truncated based on space left in advertisement packet
  */
  BLE.setLocalName("AirBike 2AC6");

  BLE.setAdvertisedService(bikePowerService);  // add the service UUID

  bikePowerService.addCharacteristic(cyclingPowerFeatureChar);
  bikePowerService.addCharacteristic(cyclingPowerMeasurementChar);
  bikePowerService.addCharacteristic(cyclingSensorLocationChar);
  BLE.addService(bikePowerService);

  cyclingPowerFeatureChar.writeValue(cpFeatureValue);
  cyclingSensorLocationChar.writeValue(sensorLocation);

  // start advertising
  BLE.advertise();

  u8x8.clearDisplay();
  u8x8.drawString(0,0,"Ready to connect");
  u8x8.drawString(0,2,"Bluetooth device");

  // start taking measurements & updating BLE char values:
  pinMode(PIN_IN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_IN), readDataISR, FALLING);
}

void loop() {
  // wait for a Bluetooth® Low Energy central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central) {
      u8x8.clearDisplay();
      u8x8.drawString(0,0,"Connected!");
      u8x8.drawString(0,2,"Pedal to start");
      delay(500);

    while (central.connected()) {
      if (shouldUpdate) {
        u8x8.clearDisplay();
        char loopstr[64];
        static uint16_t lastCadence = 0;
        uint16_t cadence = 60000/(crankTime - lastCrankTime);
        uint16_t avgCadence = (lastCadence + cadence) >> 1; //smooth out any craziness a little
        lastCadence = cadence;
        sprintf(loopstr, "Power: %d W", instantaneousPower);
        u8x8.drawString(0,0,loopstr);
        sprintf(loopstr, "Cadence: %d RPM", avgCadence);
        u8x8.drawString(0,2,loopstr);
        cyclingPowerMeasurementChar.writeValue(cpValue, POWER_MEASUREMENT_CHAR_LEN, true);
        shouldUpdate = false;
      }
      delay(100);
    }
    // when the central disconnects, turn off the LED:
      u8x8.clearDisplay();
      u8x8.drawString(0,0,"Device");
      u8x8.drawString(0,2,"disconnected");
  }
}
