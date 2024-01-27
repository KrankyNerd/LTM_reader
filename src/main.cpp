//FIXME:MAKE SURE SWUTCH SB IS DOWN OTHERWISE NO DATA!!!!

#include <Arduino.h>
// #include <Adafruit_SSD1306.h>
// #include <SoftwareSerial.h>
// #include "printf.h"

// #define OLED_RESET 4
// Adafruit_SSD1306 display(OLED_RESET);

// SoftwareSerial Serial1(8, 9);

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

String fixTypes[3] = {
    "NO",
    "2D",
    "3D"};

void setup()
{
  Serial.begin(2400); //begin serial with pc
  Serial1.begin(115200); //begin serial with tx16s
}

enum ltmStates 
{
  IDLE,
  HEADER_START1,
  HEADER_START2,
  HEADER_MSGTYPE,
  HEADER_DATA
};

#define LONGEST_FRAME_LENGTH 18

/*
 * LTM based on https://github.com/KipK/Ghettostation/blob/master/GhettoStation/LightTelemetry.cpp implementation
 */

//define the length of each function type
#define GFRAMELENGTH 18
#define AFRAMELENGTH 10 //1 byte function, 6 byte payload, 3 byte CRC
#define SFRAMELENGTH 11
#define OFRAMELENGTH 18
#define NFRAMELENGTH 10
#define XFRAMELENGTH 10

const char *flightModes[] = {
    "Manual",
    "Rate",
    "Angle",
    "Horizon",
    "Acro",
    "Stabilized1",
    "Stabilized2",
    "Stabilized3",
    "Altitude Hold",
    "GPS Hold",
    "Waypoints",
    "Head free",
    "Circle",
    "RTH",
    "Follow me",
    "Land",
    "Fly by wire A",
    "Fly by wire B",
    "Cruise",
    "Unknown"};

typedef struct remoteData_s
{
  //these should not be ints, they should be int16
  int16_t pitch;
  int16_t roll;
  int16_t heading;
  uint16_t voltage;
  byte rssi;
  bool armed;
  bool failsafe;
  byte flightmode;

  int32_t latitude;
  int32_t longitude;
  int32_t altitude;
  uint8_t groundSpeed;
  int16_t hdop;
  uint8_t gpsFix;
  uint8_t gpsSats;

  int32_t homeLatitude;
  int32_t homeLongitude;

  uint8_t sensorStatus;
} remoteData_t;

remoteData_t remoteData;

uint8_t serialBuffer[LONGEST_FRAME_LENGTH];
uint8_t state = IDLE;
char frameType;
byte frameLength;
byte receiverIndex;

//there might be something wrong with readByte() or readInt() 
byte readByte(uint8_t offset)
{
  return serialBuffer[offset];
}

int readInt(uint8_t offset)
{
  return (int)serialBuffer[offset] + ((int)serialBuffer[offset + 1] << 8);
}

//added this because i think it is needed but wasn't used before
int16_t readInt16(uint8_t offset)
{
  return (int16_t)serialBuffer[offset] + ((int16_t)serialBuffer[offset + 1] << 8);
}


int32_t readInt32(uint8_t offset)
{
  return (int32_t)serialBuffer[offset] + ((int32_t)serialBuffer[offset + 1] << 8) + ((int32_t)serialBuffer[offset + 2] << 16) + ((int32_t)serialBuffer[offset + 3] << 24);
}

uint32_t nextDisplay = 0;

/*************************************************************************
 * //Function to calculate the distance between two waypoints
 *************************************************************************/
float calc_dist(float flat1, float flon1, float flat2, float flon2)
{
  float dist_calc = 0;
  float dist_calc2 = 0;
  float diflat = 0;
  float diflon = 0;

  // I've to spplit all the calculation in several steps. If i try to do it in a single line the arduino will explode.
  diflat = radians(flat2 - flat1);
  flat1 = radians(flat1);
  flat2 = radians(flat2);
  diflon = radians((flon2) - (flon1));

  dist_calc = (sin(diflat / 2.0) * sin(diflat / 2.0));
  dist_calc2 = cos(flat1);
  dist_calc2 *= cos(flat2);
  dist_calc2 *= sin(diflon / 2.0);
  dist_calc2 *= sin(diflon / 2.0);
  dist_calc += dist_calc2;

  dist_calc = (2 * atan2(sqrt(dist_calc), sqrt(1.0 - dist_calc)));

  dist_calc *= 6371000.0; // Converting to meters
  // Serial.println(dist_calc);
  return dist_calc;
}

void loop()
{

  if (millis() >= nextDisplay)
  {
    // Serial prints
    Serial.print("Lat:");
    Serial.println(remoteData.latitude);

    Serial.print("Lon:");
    Serial.println(remoteData.longitude);

    Serial.print("HDOP:");
    Serial.println(remoteData.hdop);

    // Serial.print("Fix:");
    // Serial.println(fixTypes[remoteData.gpsFix]);

    Serial.print("Fix:");
    if (remoteData.gpsFix >= 0 && remoteData.gpsFix < sizeof(fixTypes) / sizeof(fixTypes[0]))
    {
      Serial.println(fixTypes[remoteData.gpsFix]);
    }
    else
    {
      Serial.println("Unknown");
    }

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
    nextDisplay = millis() + 500;
  }


  if (Serial1.available())
  {

    char data = Serial1.read();
    //Serial.println(data);

    if (state == IDLE)
    {
      if (data == '$') //if $then start of new info
      {
        state = HEADER_START1;
      }
    }
    else if (state == HEADER_START1) //T means next char is payload type
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

      switch (data) //data is payload type
      {

      case 'G':
        frameLength = GFRAMELENGTH;
        break;
      case 'A':
        frameLength = AFRAMELENGTH;
        break;
      case 'S':
        frameLength = SFRAMELENGTH;
        break;
      case 'O':
        frameLength = OFRAMELENGTH;
        break;
      case 'N':
        frameLength = NFRAMELENGTH;
        break;
      case 'X':
        frameLength = XFRAMELENGTH;
        break;
      default:
        state = IDLE;
      }
    }
    else if (state == HEADER_MSGTYPE)
    {

      /*
       * Check if last payload byte has been received.
       */
      if (receiverIndex == frameLength - 4)
      {
        /*
         * If YES, check checksum and execute data processing
         */

        if (frameType == 'A') //this does not make sense these read uint8_t but it should read int16 as the payload is 6 bytes
        {//TODO:find me
          remoteData.pitch = readInt16(0);
          remoteData.roll = readInt16(2);
          remoteData.heading = readInt16(4);

          // remoteData.pitch = readInt(0);
          // remoteData.roll = readInt(2);
          // remoteData.heading = readInt(4);
        }

        if (frameType == 'S')
        {
          remoteData.voltage = readInt(0);
          remoteData.rssi = readByte(4);

          byte raw = readByte(6);
          remoteData.flightmode = raw >> 2;
        }

        if (frameType == 'G')
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
          remoteData.hdop = readInt(0);
          remoteData.sensorStatus = readByte(2);
        }

        state = IDLE;
        memset(serialBuffer, 0, LONGEST_FRAME_LENGTH);
      }
      else
      {
        /*
         * If no, put data into buffer
         */
        serialBuffer[receiverIndex++] = data;
      }
    }
  }
}
