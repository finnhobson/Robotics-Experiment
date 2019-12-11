#ifndef _Mapping_h
#define _Mapping_h

#include <EEPROM.h>
#include <USBCore.h>    // To fix serial print behaviour bug.
u8 USB_SendSpace(u8 ep);
#define SERIAL_ACTIVE (USB_SendSpace(CDC_TX) >= 50)

const byte MAP_RESOLUTION = 25;
const byte MAP_DEFAULT_FEATURE = '#';
const int MAP_X = 1000;
const int MAP_Y = 1000;

class Mapper {
  public:
    void start();
    void resetMap();
    void printMap();
    void updateMapFeature(byte feature, int y, int x);
    void updateMapFeature(byte feature, float y, float x);
    float percentCoverage();

    int  indexToPose(int i, int map_size, int resolution);
    int  poseToIndex(int x, int map_size, int resolution);

  private:
    int X_size;
    int Y_size;
    int repeatCount;
    int lastFeatAddress;
};


//This function will write a 2 byte integer to the eeprom at the specified address and address + 1
void EEPROMWriteInt(int p_address, int p_value)
{
  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF);

  EEPROM.write(p_address, lowByte);
  EEPROM.write(p_address + 1, highByte);
}

//This function will read a 2 byte integer from the eeprom at the specified address and address + 1
unsigned int EEPROMReadInt(int p_address)
{
  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);

  return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}

void Mapper::start() {
  repeatCount = EEPROMReadInt(1021);
}

void Mapper::resetMap() {
  lastFeatAddress = 0;
  repeatCount = 0;
  EEPROMWriteInt(1021, repeatCount);

  for (int i = 0; i < MAP_RESOLUTION; i++) {
    for (int j = 0; j < MAP_RESOLUTION; j++) {

      int eeprom_address = (i * MAP_RESOLUTION) + j;

      if (eeprom_address > 1023 - 2) { // taken off 2 bytes for repeat
        if ( SERIAL_ACTIVE ) Serial.println(F("Error1: EEPROM Address greater than 1023-2"));
      } else {
        EEPROM.update(eeprom_address, MAP_DEFAULT_FEATURE );
      }
    }
  }
}

void Mapper::printMap() {

  if ( SERIAL_ACTIVE ) {

    int counter1 = 0;
    int counter2 = 0;
    Serial.println("Map");
    for (int i = 0; i < MAP_RESOLUTION; i++) {
      for (int j = 0; j < MAP_RESOLUTION; j++) {
        int eeprom_address = (i * MAP_RESOLUTION) + j;
        byte value;
        value = EEPROM.read(eeprom_address);//, value);
        if ( (char)value == '#') {
          counter1 = counter1 + 1;
        }
        else {
          counter2 = counter2 + 1;
        }
        Serial.print( (char)value );
        Serial.print(" ");
      }
      Serial.println("");
    }
    Serial.println((String)"Map Areas Repeated: " + repeatCount);
    float percentageCoverage = ((float)counter2 / ((float)counter1 + (float)counter2)) * 100.0;
    Serial.println((String)"Percent of Map covered: " + percentageCoverage + "%");
  }
}

float Mapper::percentCoverage() {
  int counter1 = 0;
  int counter2 = 0;
  for (int i = 0; i < MAP_RESOLUTION; i++) {
    for (int j = 0; j < MAP_RESOLUTION; j++) {

      int eeprom_address = (i * MAP_RESOLUTION) + j;
      byte value;
      value = EEPROM.read(eeprom_address);//, value);
      if ( (char)value == '#') {
        counter1 = counter1 + 1;
      }
      else {
        counter2 = counter2 + 1;
      }

    }
  }
  float percentageCoverage = ((float)counter2 / ((float)counter1 + (float)counter2)) * 100.0;
  return percentageCoverage;
}

int Mapper::poseToIndex(int x, int map_size, int resolution)
{
  return x / (map_size / resolution);
}

int Mapper::indexToPose(int i, int map_size, int resolution)
{
  return i * (map_size / resolution);
}


void Mapper::updateMapFeature(byte feature, float y, float x) {
  updateMapFeature( feature, (int)y, (int)x );
}

void Mapper::updateMapFeature(byte feature, int y, int x) {

  if (x > MAP_X || x < 0 || y > MAP_Y || y < 0) {
    //if (SERIAL_ACTIVE) Serial.println(F("Error:Invalid co-ordinate"));
    return;
  }

  int x_index = poseToIndex(x, MAP_X, MAP_RESOLUTION);
  int y_index = poseToIndex(y, MAP_Y, MAP_RESOLUTION);

  int eeprom_address = (x_index * MAP_RESOLUTION) + y_index;

  //New Location
  if (eeprom_address != lastFeatAddress) {
    lastFeatAddress = eeprom_address;
    byte value;
    value = EEPROM.read(eeprom_address);//, value);
    if (value != '#') {
      repeatCount = repeatCount + 1;
      //Serial.println((String)"Repeated: "+repeatCount);
      EEPROMWriteInt(1021, repeatCount);
    }
  }

  if (eeprom_address > 1023 - 2) {
    if (SERIAL_ACTIVE)Serial.println(F("Error: EEPROM Address greater than 1023-2"));
  } else {
    EEPROM.update(eeprom_address, feature);
  }
}

#endif
