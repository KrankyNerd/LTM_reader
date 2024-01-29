// FIXME:MAKE SURE SWITCH SB IS DOWN OTHERWISE NO DATA!!!!
//use read int to read uint16
#include <Arduino.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//-----------------start display stuff------------------
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };
//------------------end OLED stuff----------------

//----------start BLE stuff
#include <ArduinoBLE.h>

BLEService customService("19B10000-E8F2-537E-4F6C-D104768A1214"); // create a custom service
BLEStringCharacteristic GPS("19B10001-E8F2-537E-4F6C-D104768A1215", BLERead | BLENotify, 20); // create the first custom characteristic with a unique UUID
BLEStringCharacteristic Battery("19B10002-E8F2-537E-4F6C-D104768A1216", BLERead | BLENotify, 20); // create the second custom characteristic with a unique UUID

String GPSString = "Default";       //string that will be sent via bt
String batteryString = "Default";   //string that will be sent via bt

//----------end ble stuff

/* #################################################################################################################
 * LightTelemetry protocol (LTM)
 *
 * Ghettostation one way telemetry protocol for really low bitrates (1200/2400 bauds).
 *
 * Protocol details: 3 different frames, little endian.
 *   G Frame (GPS position) (2hz @ 1200 bauds , 5hz >= 2400 bauds): 18BYTES
 *    0x24 0x54 0x47 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF  0xFF   0xC0
 *     $     T    G  --------LAT-------- -------LON---------  SPD --------ALT-------- SAT/FIX  CRC
 *   A Frame (Attitude) (5hz @ 1200bauds , 10hz >= 2400bauds): 10BYTES
 *     0x24 0x54 0x41 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xC0
 *      $     T   A   --PITCH-- --ROLL--- -HEADING-  CRC
 *   S Frame (Sensors) (2hz @ 1200bauds, 5hz >= 2400bauds): 11BYTES
 *     0x24 0x54 0x53 0xFF 0xFF  0xFF 0xFF    0xFF    0xFF      0xFF       0xC0
 *      $     T   S   VBAT(mv)  Current(ma)   RSSI  AIRSPEED  ARM/FS/FMOD   CRC
 * ################################################################################################################# */
/*
 LTM based on https://github.com/KipK/Ghettostation/blob/master/GhettoStation/LightTelemetry.cpp implementation
 https://github.com/iNavFlight/inav/wiki/Lightweight-Telemetry-(LTM)
*/

enum LTMStates
{
  IDLE,
  HEADER_START1,
  HEADER_START2,
  HEADER_MSGTYPE,
  HEADER_DATA
};

const int LONGEST_FRAME_LENGTH = 18;

// define the length of each frame type
const int G_FRAME_LENGTH = 18;
const int A_FRAME_LENGTH = 10; // 1 byte function, 6 byte payload, 3 byte CRC
const int S_FRAME_LENGTH = 11;
const int O_FRAME_LENGTH = 18;
const int N_FRAME_LENGTH = 10;
const int X_FRAME_LENGTH = 10;

// define the data type of each variable, this should match up with the LTM docs
typedef struct RemoteData_s
{
  // A frame (attitude) 6 bytes total
  int16_t pitch;
  int16_t roll;
  int16_t heading;

  // S frame (status) 7 bytes
  uint16_t voltage; // 2 bytes
  uint16_t batteryConsumption; //2 bytes
  byte rssi;        // 1 byte
  bool armed;
  bool failsafe;
  byte flightmode;

  // G frame (GPS) 14 bytes //something in here is wrong me thinks
  int32_t latitude;    // 4 bytes
  int32_t longitude;   // 4 bytes
  int32_t altitude;    // 4 bytes
  uint8_t groundSpeed; // 1 byte

  int16_t hdop; // part of X frame
  uint8_t gpsFix;
  uint8_t gpsSats;

  // O frame (origin) 14 bytes
  int32_t homeLatitude;  // 4 bytes
  int32_t homeLongitude; // 4 bytes
  uint32_t homeAltitude; // 4 bytes
  uint8_t OSD;           // 1 byte //these are the same as uchar
  uint8_t homeFix;       // 1 byte

  uint8_t sensorStatus;
} RemoteData_t;

RemoteData_t remoteData;

uint8_t serialBuffer[LONGEST_FRAME_LENGTH];
uint8_t state = IDLE;
char frameType;
byte frameLength;
byte receiverIndex;

String fixTypes[3] = {
    "NO",
    "2D",
    "3D"};

byte readByte(uint8_t offset)
{
  return serialBuffer[offset];
}

int readInt(uint8_t offset)
{
  return (int)serialBuffer[offset] + ((int)serialBuffer[offset + 1] << 8);
}

int16_t readInt16(uint8_t offset)
{
  return (int16_t)serialBuffer[offset] + ((int16_t)serialBuffer[offset + 1] << 8);
}

int32_t readInt32(uint8_t offset)
{
  return (int32_t)serialBuffer[offset] + ((int32_t)serialBuffer[offset + 1] << 8) + ((int32_t)serialBuffer[offset + 2] << 16) + ((int32_t)serialBuffer[offset + 3] << 24);
}

void parseFrame()
{
  if (frameType == 'A') // attitude //FIXME:
  {
    remoteData.pitch = readInt16(0);
    remoteData.roll = readInt16(2);
    remoteData.heading = readInt16(4);
  }

  if (frameType == 'S') // status
  {
    remoteData.voltage = readInt(0); //this part works
    remoteData.batteryConsumption = readInt(2); //this works
    remoteData.rssi = readByte(4);

    byte raw = readByte(6);
    remoteData.flightmode = raw >> 2;
  }

  if (frameType == 'G') // gps this works
  {
    remoteData.latitude = readInt32(0);
    remoteData.longitude = readInt32(4);
    remoteData.groundSpeed = readByte(8);
    remoteData.altitude = readInt32(9);

    uint8_t raw = readByte(13);
    remoteData.gpsSats = raw >> 2;
    remoteData.gpsFix = raw & 0x03;
  }

  if (frameType == 'X')
  {
    remoteData.hdop = readInt(0); //its a uint therefore read int rather than int16
    remoteData.sensorStatus = readByte(2);
  }

  if (frameType == 'O')
    ;

  if (frameType == 'N')
    ;
}

void readData()
{
  if (Serial1.available())
  {
    char data = Serial1.read();
    // Serial.println(data);

    if (state == IDLE)
    {
      if (data == '$')
      {
        state = HEADER_START1;
      }
    }
    else if (state == HEADER_START1)
    {
      if (data == 'T')
      {
        state = HEADER_START2;
      }
      else
      {
        state = IDLE;
      }
    }
    else if (state == HEADER_START2)
    {
      frameType = data;
      state = HEADER_MSGTYPE;
      receiverIndex = 0;

      switch (data) // based on the data in the header_start 2, figure out the length of the frame
      {
      case 'G':
        frameLength = G_FRAME_LENGTH;
        break;
      case 'A':
        frameLength = A_FRAME_LENGTH;
        break;
      case 'S':
        frameLength = S_FRAME_LENGTH;
        break;
      case 'O':
        frameLength = O_FRAME_LENGTH;
        break;
      case 'N':
        frameLength = N_FRAME_LENGTH;
        break;
      case 'X':
        frameLength = X_FRAME_LENGTH;
        break;
      default:
        state = IDLE; // Reset to IDLE on invalid frame type
      }
    }
    else if (state == HEADER_MSGTYPE)
    {
      if (receiverIndex == frameLength - 4)
      {
        parseFrame();
        state = IDLE;
        memset(serialBuffer, 0, LONGEST_FRAME_LENGTH);
      }
      else if (receiverIndex >= LONGEST_FRAME_LENGTH)
      {
        // Buffer overflow, reset state to IDLE
        state = IDLE;
      }
      else
      {
        serialBuffer[receiverIndex++] = data;
      }
    }
  }
}

void serialPrintData()
{
  // Serial prints
  Serial.print("Lat:");
  Serial.println(remoteData.latitude);

  Serial.print("Lon:");
  Serial.println(remoteData.longitude);

  Serial.print("HDOP:");
  Serial.println(remoteData.hdop);

  Serial.print("Fix:");
  Serial.println(fixTypes[remoteData.gpsFix]);

  Serial.print("Sat:");
  Serial.println(remoteData.gpsSats);

  Serial.print("Spd:");
  Serial.println(remoteData.groundSpeed);

  Serial.print("Alt:");
  Serial.println(remoteData.altitude);

  Serial.print("Roll:");
  Serial.println(remoteData.roll);

  Serial.print("Pitch:");
  Serial.println(remoteData.pitch);

  Serial.print("Voltage:");
  Serial.println(remoteData.voltage/1000.);

  Serial.print("Battery Consumption:");
  Serial.println(remoteData.batteryConsumption/1000.);

  Serial.print("RSSI:"); //is correct
  Serial.println(remoteData.rssi);
}

void displayTelem()
{
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  //display.cp437(true);         // Use full 256 char 'Code Page 437' font
  
  //display GPS
  display.print(F("Lat: "));
  display.println(remoteData.latitude);
  display.print(F("Long: "));
  display.println(remoteData.longitude);
  
  //display Battery info
  display.print(F("Voltage:"));
  display.print(remoteData.voltage/1000.);
  display.println(F("V"));

  display.print(F("Consumed: "));
  display.print(remoteData.batteryConsumption);
  display.println(F("mah"));

  display.display();
}

void sendBLE()
{
  BLEDevice central = BLE.central(); // Wait for a BLE central

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    while (central.connected()) {
      //FIXME: the strings are too long and are being truncated
      //you can increase the ATT_MTU but realistally you would send integers or smthn and then have an app that knows the UUID of each characteristic read the data
      // Send strings of data
      GPSString = "Lat: " + String(remoteData.latitude) + " Long: " + String(remoteData.longitude);
      batteryString = "Voltage: " + String(remoteData.voltage) + "V "+ "Battery Consumption: " + String(remoteData.batteryConsumption) + "mAh";
      GPS.writeValue(GPSString);
      Battery.writeValue(batteryString);

      //delay(1000); // Adjust the delay according to your needs
    }
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

void setup()
{
  Serial.begin(2400);    // begin serial with pc
  Serial1.begin(115200); // begin serial with tx16s
  //while(!Serial);

  //BLE stuff--------------------------
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("TX16s Telem");  // Set the local name advertised by the BLE peripheral
  BLE.setAdvertisedService(customService); // Advertise the custom service
  customService.addCharacteristic(GPS); // Add the first custom characteristic
  customService.addCharacteristic(Battery); // Add the second custom characteristic
  BLE.addService(customService); // Add the service

  BLE.advertise(); // Start advertising

  Serial.println("BLE Peripheral - Waiting for connections...");

//-----------end BLE stuff

//-------------OLED stuff--------------
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  // Draw a single pixel in white
  display.drawPixel(10, 10, SSD1306_WHITE);
  Serial.println("Init display");

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();
  delay(2000);
  //-------------------end oled stuff---------
}

unsigned long nextDisplay = 0;

void loop()
{
  readData();
  if (millis() >= nextDisplay)
  {
    serialPrintData();
    displayTelem();
    sendBLE();
    nextDisplay = millis() + 500;
  }
}
