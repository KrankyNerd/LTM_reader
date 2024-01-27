//FIXME:MAKE SURE SWUTCH SB IS DOWN OTHERWISE NO DATA!!!!

#include <Arduino.h>

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

//define the length of each frame type
const int G_FRAME_LENGTH = 18;
const int A_FRAME_LENGTH = 10; //1 byte function, 6 byte payload, 3 byte CRC
const int S_FRAME_LENGTH = 11;
const int O_FRAME_LENGTH = 18;
const int N_FRAME_LENGTH = 10;
const int X_FRAME_LENGTH = 10;

//define the data type of each variable, this should match up with the LTM docs
typedef struct RemoteData_s
{
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
  if (frameType == 'A')
  {
    remoteData.pitch = readInt16(0);
    remoteData.roll = readInt16(2);
    remoteData.heading = readInt16(4);
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
}

void readData()
{
  if (Serial1.available())
  {
    char data = Serial1.read();
    //Serial.println(data);

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

      switch (data)
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
        state = IDLE;
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
}

void setup()
{
  Serial.begin(2400); //begin serial with pc
  Serial1.begin(115200); //begin serial with tx16s
}

void loop()
{
  unsigned long nextDisplay = 0;
  
  if (millis() >= nextDisplay)
  {
    readData();
    serialPrintData();
    nextDisplay = millis() + 5000;
  }
}
