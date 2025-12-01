#include "DLBus.h"
#include "esphome/core/log.h"

static const char *TAG = "DLBus";
DLBus *DLBus::instance = nullptr;

DLBus::DLBus() {
  // Setze den statischen Zeiger auf diese Instanz
  instance = this;
  // Initialisiere den Buffer
  for (int i = 0; i < DL_Bus_PacketLength; i++) {
    DL_Bus_Buffer[i] = 0xFF;
  }
  edgeBufferWritePos = 0;
  edgeBufferReadPos = 0;
  edgeBufferCount = 0;
}

void DLBus::init() {
  #ifdef DEBUGPIN_ENABLE
    pinMode(DEBUGPIN_NO, OUTPUT);
    digitalWrite(DEBUGPIN_NO, LOW);
  #endif
  timeSincelastEdge = 0;
}

#ifdef DEBUGPIN_ENABLE
void DLBus::DebugPulse() {
  digitalWrite(DEBUGPIN_NO, HIGH);
  delayMicroseconds(10);
  digitalWrite(DEBUGPIN_NO, LOW);
}
#endif

// Statische ISR-Funktion, die vom Interrupt-Controller aufgerufen wird.
void IRAM_ATTR DLBus::isr() {
  if (instance) {
    instance->handleInterrupt();
  }
}

void DLBus::handleInterrupt() {
  timeOfActEdge = micros();
  // Berechne die Pulsdauer:
  actData.edgetime = (uint32_t)(timeOfActEdge - timeSincelastEdge);
  actData.pinState = !digitalRead(DL_Input_Pin);
  if (edgeBufferCount < EdgeBufferSize) {
    edgeTimeBuffer[edgeBufferWritePos] = actData;
    edgeBufferWritePos = (edgeBufferWritePos + 1) % EdgeBufferSize;
    edgeBufferCount++;
  }
  timeSincelastEdge = timeOfActEdge;
}

bool DLBus::loadBitFromEdgeTimeBuffer() {
  int i = 0;
  while (edgeBufferCount == 0) {
    delay(2);
    i++;
    if (i == 3) {
      return false;
    }
  }
  newData = edgeTimeBuffer[edgeBufferReadPos];
  edgeBufferReadPos = (edgeBufferReadPos + 1) % EdgeBufferSize;
  edgeBufferCount--;
  return true;
}

int DLBus::captureBit() {
  if (loadBitFromEdgeTimeBuffer()) {
    edgetime = newData.edgetime;
    if (edgetime > Tmin && edgetime < Tmax) {
      if (loadBitFromEdgeTimeBuffer()) {
        edgetime = newData.edgetime;
        if (edgetime > Tmin && edgetime < Tmax) {
          nextBit = curBit;
        } else {
          return 2;
        }
      } else {
        return 2;
      }
    } else if (edgetime > 2 * Tmin && edgetime < 2 * Tmax) {
      nextBit = !curBit;
    } else {
      return 2;
    }
    curBit = nextBit;
    return nextBit;
  } else {
    return 2;
  }
}

unsigned char DLBus::recieveByte() {
  char rxByte = 0;
  //StartBit detected?
  if (captureBit() == 0) {
    //read 8 Bits
    for (int i = 0; i < 8; i++) {
      int bit = captureBit();
      if (bit == 2) {
        return 0x00;
      } else {
        if (bit == 1) {
          SET_BIT(rxByte, i);
        }
      }
    }
    //StopBit
    if (captureBit() == 1) {
      return rxByte;
    }
  }
  return 0x00;
}

bool DLBus::testChecksum() {
  unsigned char checksum = 1;
  for (int i = 0; i < (DL_Bus_PacketLength - 1); i++) {
    checksum = checksum + DL_Bus_Buffer[i];
  }
  return (checksum == DL_Bus_Buffer[DL_Bus_PacketLength - 1]);
}

bool DLBus::testChecksumSensorSlave() {
  unsigned char checksum = 1;
  for (int i = 0; i < (DL_Bus_PacketLength - 1); i++) {
    checksum = checksum + DL_Bus_Buffer[i];
  }
  return (checksum == DL_Bus_Buffer[DL_Bus_PacketLength - 1]);
}

int16_t DLBus::processTemperature(char lowByte, char highbyte) {
  int16_t temp;
  if ((highbyte & 0b10000000) == 0) {  // Positive Temperatur
    highbyte = highbyte & 0b10001111;
  } else {  // Negative Temperatur
    SET_BIT(highbyte, 6);
    SET_BIT(highbyte, 5);
    SET_BIT(highbyte, 4);
  }
  temp = (int16_t)lowByte + (256 * (int16_t)highbyte);
  return temp;
}

void DLBus::processData() {
  lastFrame.DeviceID = DL_Bus_Buffer[1];
  lastFrame.Sec = DL_Bus_Buffer[3];
  lastFrame.Min = DL_Bus_Buffer[4];
  lastFrame.Hour = DL_Bus_Buffer[5];
  lastFrame.Day = DL_Bus_Buffer[6];
  lastFrame.Month = DL_Bus_Buffer[7];
  lastFrame.Year = DL_Bus_Buffer[8] + 2000;
  lastFrame.Sensor1 = processTemperature(DL_Bus_Buffer[9], DL_Bus_Buffer[10]);
  lastFrame.Sensor2 = processTemperature(DL_Bus_Buffer[11], DL_Bus_Buffer[12]);
  lastFrame.Sensor3 = processTemperature(DL_Bus_Buffer[13], DL_Bus_Buffer[14]);
  lastFrame.Sensor4 = processTemperature(DL_Bus_Buffer[15], DL_Bus_Buffer[16]);
  lastFrame.Sensor5 = processTemperature(DL_Bus_Buffer[17], DL_Bus_Buffer[18]);
  lastFrame.Sensor6 = processTemperature(DL_Bus_Buffer[19], DL_Bus_Buffer[20]);
  lastFrame.Outputs = (uint16_t)DL_Bus_Buffer[41] + 256 * (uint16_t)(DL_Bus_Buffer[42]);
}

bool DLBus::captureSinglePacket() {
  ESP_LOGI(TAG, "captureSinglePacket started");
  DL_Bus_Buffer[0] = 0xFF;
  DL_Bus_Buffer[1] = recieveByte();
  //check DeviceType
  if (DL_Bus_Buffer[1] != 0x80){
    // error exit
    ESP_LOGI(TAG, "captureSinglePacket error exit");
    detachInterrupt(digitalPinToInterrupt(DL_Input_Pin));
    return false;
  }
  for (int i = 2; i < DL_Bus_PacketLength; i++) {
    DL_Bus_Buffer[i] = recieveByte();
  }
  if (testChecksum() == true) {
    // clean exit
    processData();
    ESP_LOGI(TAG, "Dataframe processed");
    detachInterrupt(digitalPinToInterrupt(DL_Input_Pin));
    return true;
  }
  else {
    // error exit
    detachInterrupt(digitalPinToInterrupt(DL_Input_Pin));
    return false;
  }
}

bool DLBus::sensorSlave(){
    
    DL_Bus_Buffer[0] = recieveByte();
    DL_Bus_Buffer[1] = recieveByte();
    DL_Bus_Buffer[2] = recieveByte();
    DL_Bus_Buffer[3] = recieveByte();

    if (testChecksumSensorSlave() == true) {
        // clean exit
        //processData();
        detachInterrupt(digitalPinToInterrupt(DL_Input_Pin));
        return true;
    }
    else {
        // error exit
        detachInterrupt(digitalPinToInterrupt(DL_Input_Pin));
        return false;
    }
}

bool DLBus::capture(){
  // Reset Buffer
  edgeBufferWritePos = 0;
  edgeBufferReadPos = 0;
  edgeBufferCount = 0;
  // Registriere den Interrupt mit der statischen ISR
  attachInterrupt(digitalPinToInterrupt(DL_Input_Pin), DLBus::isr, CHANGE);
  T_Start = millis();
  curBit = true;

  //Sync
  while (true) {
      // detect if 0x55FFFF or 0xFFFF
      if (captureBit() == 1) {
          if (captureBit() == 1) {
              // check for dataframe from UVR.....
              bool sync = true;
              for (int i=0; i<14; i++){
                if (captureBit() != 1) {
                  bool sync = false;
                  break;
                }
              }
              if (sync == true) {
                  //run captureFrame
                  ESP_LOGI(TAG, "Sync for Dataframe detected");
                  return DLBus::captureSinglePacket();
              }
          }
          else if (captureBit() == 0){
              //check for sensor-Read-frame from Master
              bool sync = true;
              byte syncByte = 0b00000010;
              byte newBit = captureBit();
              for (int i=0; i<6; i++){
                  if (newBit = 2) {
                    bool sync = false;
                    break;
                  }
                  syncByte = (syncByte << 1) | newBit; // shift in valid newBit
              }
              if (sync == true) {
                  //run sensorSlaveFrame
                  ESP_LOGI(TAG, "Sync 0x55 for SensorSlaveFrame detected");
                  return DLBus::sensorSlave();
              }

          }
      }
      
      edgeBufferWritePos = 0;
      edgeBufferReadPos = 0;
      edgeBufferCount = 0;
      yield();
      
      if ((millis() - T_Start) > timeout) {
        detachInterrupt(digitalPinToInterrupt(DL_Input_Pin));
        return false;
      }
    }
}
