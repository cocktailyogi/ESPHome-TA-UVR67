#include "DLBus.h"
#include "esphome/core/log.h"

static const char *TAG = "DLBus";
DLBus *DLBus::instance = nullptr;

DLBus::DLBus() {
  
  instance = this;
  // Initialisiere den Buffer
  for (int i = 0; i < DL_Bus_PacketLength; i++) {
    DL_Bus_Buffer[i] = 0xFF;
  }
  edgeBufferWritePos = 0;
  edgeBufferReadPos = 0;
  edgeBufferCount = 0;
  timeSincelastEdge = 0;
  currentHeatingMode = HeatingMode::NORMAL;
  roomTemperatureRASPT = 19.8;
  // set TX-Pin to inactive state
  pinMode(DL_Output_Pin, OUTPUT);
  digitalWrite(DL_Output_Pin, LOW);
  curBit = true;
}

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
  edgeTimeBuffer[edgeBufferWritePos] = actData;
  edgeBufferWritePos = (edgeBufferWritePos + 1) % EdgeBufferSize;
  edgeBufferCount++;
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
  byte checksum = 0;
  //DL_Bus_Buffer[0] = 0xFF; //bugfix, device sends 0x00
  for (int i = 0; i < (DL_Bus_PacketLength - 1); i++) {
    checksum = checksum + DL_Bus_Buffer[i];
  }
  //ESP_LOGI(TAG, "checksum=0x%02X", checksum);
  //ESP_LOGI(TAG, "Buffer[0]=0x%02X, Buffer[1]=0x%02X", DL_Bus_Buffer[0], DL_Bus_Buffer[1]);
  //ESP_LOGI(TAG, "Buffer[61]=0x%02X, Buffer[62]=0x%02X", DL_Bus_Buffer[61], DL_Bus_Buffer[62]);
  //ESP_LOGI(TAG, "Buffer[63]=0x%02X, Buffer[64]=0x%02X", DL_Bus_Buffer[63], DL_Bus_Buffer[64]);
  return (checksum == DL_Bus_Buffer[DL_Bus_PacketLength - 1]);
}

bool DLBus::testChecksumSensorSlave() {
  byte checksum = 0;
  for (int i = 0; i < 3; i++) {
    checksum = checksum + DL_Bus_Buffer[i];
  }
  return (checksum == DL_Bus_Buffer[3]);
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
  return;
}

bool DLBus::captureSinglePacket() {
  //ESP_LOGI(TAG, "captureSinglePacket started");
  for (int i = 2; i < DL_Bus_PacketLength; i++) {
    DL_Bus_Buffer[i] = recieveByte();
  }
  detachInterrupt(digitalPinToInterrupt(DL_Input_Pin));
  if (testChecksum() == true) {
    // clean exit
    processData();
    ESP_LOGI(TAG, "Dataframe recieved and processed");
    return true;
  }
  else {
    // error exit
    ESP_LOGI(TAG, "Dataframe Checksum Error");
    //ESP_LOGI(TAG, "Buffer[0]=0x%02X, Buffer[1]=0x%02X", DL_Bus_Buffer[0], DL_Bus_Buffer[1]);
    return false;
  }
}

void DLBus::sendManchesterBit(bool bit) {
    //DL_Output_Pin is inverted!
    if (bit) {
        // Bit 1: Low → High (steigende Flanke)
        digitalWrite(DL_Output_Pin, HIGH);
        delayMicroseconds(T);
        digitalWrite(DL_Output_Pin, LOW);
        delayMicroseconds(T);
    } else {
        // Bit 0: High → Low (fallende Flanke)
        digitalWrite(DL_Output_Pin, LOW);
        delayMicroseconds(T);
        digitalWrite(DL_Output_Pin, HIGH);
        delayMicroseconds(T);
    }
    return;
}

void DLBus::sendManchesterByte(uint8_t byte) {
    // Start Bit
    sendManchesterBit(0);
    // 8 Databits
    for (int i = 0; i < 8; i++) {
        bool bit = (byte >> i) & 1;
        sendManchesterBit(bit);
    }
    // Stop Bit
    sendManchesterBit(1);
    return;
}

void DLBus::sensorSlaveRespond(byte sensorAddress){
    /*
    2 ms Pause nach Empfang einer Anfrage
    Slaveadresse (0xVU, V = 4 Bit Slaveadresse, U = 4 Bit Subadresse)
    Datenkennzeichnung (1Byte)
    Daten (normalerweise 2 Byte kann aber durch die Datenkennzeichnung anders
    spezifiziert werden)
    Prüfsumme (8 Bit)
    
    Der Temperaturwert eines Raumsensors enthält auch dessen Betriebsmodus. In den Datenbytes eines 
    Temperaturwertes ist ein Raumsensor gekennzeichnet, dass Bit 6 des High-Bytes
    das Vorzeichenbit invertiert darstellt. 
    */
    if (sensorAddress == 0x1B) {
        // RAS-PT
        delay(4); // min. 20ms specified in datasheet
        byte Datenkennzeichnung = 0x01; // TempSensor
        //encode room temperature
        int16_t dataWord = (int16_t)(roomTemperatureRASPT * 10.0f);
        
        byte DatenbyteLow = dataWord & 0xFF;
        byte DatenbyteHigh = (dataWord >> 8) & 0xFF;
        // in RAS-PT Bit 6 of DatenbyteHigh needs to be inverted of Bit7
        if (DatenbyteHigh & 0x80) {
            // clear Bit 6
            DatenbyteHigh = DatenbyteHigh & 0b10111111;
        } else {
            // set Bit 6
            DatenbyteHigh = DatenbyteHigh | 0b01000000;
        }
        //encode currentHeatingMode
        DatenbyteHigh = (DatenbyteHigh & 0b11111001) | (byte)currentHeatingMode;
        //make checksum
        byte checksum = 0;
        checksum = sensorAddress + Datenkennzeichnung + DatenbyteLow + DatenbyteHigh;
        //send
        DLBus::sendManchesterByte(sensorAddress);
        DLBus::sendManchesterByte(Datenkennzeichnung);
        DLBus::sendManchesterByte(DatenbyteLow);
        DLBus::sendManchesterByte(DatenbyteHigh);
        DLBus::sendManchesterByte(checksum);
        //disable TX-Output
        digitalWrite(DL_Output_Pin, LOW);
        //ESP_LOGI(TAG, "sensorAddress=0x%02X", sensorAddress);
        //ESP_LOGI(TAG, "Datenkennzeichnung=0x%02X", Datenkennzeichnung);
        //ESP_LOGI(TAG, "DatenbyteLow=0x%02X", DatenbyteLow);
        //ESP_LOGI(TAG, "DatenbyteHigh=0x%02X", DatenbyteHigh);
        //ESP_LOGI(TAG, "checksum=0x%02X", checksum);
        //send slave-Response
        ESP_LOGI(TAG, "RAS-PT request processed");
    }
    else {
        ESP_LOGI(TAG, "unknown Sensor-Address: 0x%02X", sensorAddress);
    }
    return;
}

bool DLBus::sensorSlave(){
    
    DL_Bus_Buffer[2] = recieveByte();
    DL_Bus_Buffer[3] = recieveByte();
    detachInterrupt(digitalPinToInterrupt(DL_Input_Pin));
    if (testChecksumSensorSlave() == true) {
        // clean exit
        byte sensorAddress = DL_Bus_Buffer[2];
        DLBus::sensorSlaveRespond(sensorAddress);
        // here needs SlaveResponse to be implemented
        //ESP_LOGI(TAG, "MasterSlaveframe for sensorAddress=0x%02X processed", sensorAddress);
        return true;
    }
    else {
        // error exit
        ESP_LOGI(TAG, "MasterSlaveframe Checksum Error");
        return false;
    }
}

bool DLBus::waitForBusIdle(unsigned long idleTimeMs) {
  unsigned long startWait = millis();
  
  while ((millis() - startWait) < timeout) {
    // Warte bis Pin HIGH ist
    while (digitalRead(DL_Input_Pin) == LOW) {
      if ((millis() - startWait) > timeout) {
        return false;  // Timeout
      }
      yield();
    }
    
    // Pin ist HIGH - prüfe ob er stabil bleibt
    unsigned long highStart = millis();
    int glitchCount = 0;
    bool stable = true;
    
    while ((millis() - highStart) < idleTimeMs) {
      if (digitalRead(DL_Input_Pin) == LOW) {
          stable = false;
          break;
      }
      yield();
    }
    
    if (stable) {
      //ESP_LOGD(TAG, "Bus idle for %lu ms", idleTimeMs);
      return true;  // Erfolg!
    }
  }
  
  //ESP_LOGW(TAG, "Timeout waiting for bus idle");
  return false;  // Timeout
}

bool DLBus::capture(){
  // Reset Buffer
  bool sync = false;
  edgeBufferWritePos = 0;
  edgeBufferReadPos = 0;
  edgeBufferCount = 0;
  // Registriere den Interrupt mit der statischen ISR
  
  T_Start = millis();
  
  //Sync
  while (true) {
      
      //wait for BusIdle or high sequence
      if (!waitForBusIdle(1)) {
          //ESP_LOGE(TAG, "Bus never became idle for timeouttime");
          return false;
      }
      
      attachInterrupt(digitalPinToInterrupt(DL_Input_Pin), DLBus::isr, CHANGE);
      
      //preload Buffer
      while(edgeBufferCount < 3) {
          yield();
          //delay(1);
          if ((millis() - T_Start) > timeout) {
            detachInterrupt(digitalPinToInterrupt(DL_Input_Pin));
            return false;
          }
      }
      captureBit(); // dummy to get into the rythm
      if (captureBit() == 0) {
          //check for SYNC-Sequence from Master ... 0x55FFFF 
          // detect 0x55
          sync = true;
          byte syncByte = 0; 
          byte newBit = false;
          for (int i=0; i<7; i++){
              newBit = captureBit();
              if (newBit == 2) {
                sync = false;
                break;
              }
              syncByte = (syncByte << 1) | newBit; // shift in valid newBit
          }
          //ESP_LOGI(TAG, "Syncbyte=0x%02X", syncByte);
      
          if ((sync == true) && (syncByte == 0x55)) {
              
              // check for sync 0xFFFF
              
              curBit = false; // correction needed
              byte bit = false;
              for (int i=0; i < 16; i++) {
                bit = (byte)captureBit(); 
                //ESP_LOGI(TAG, "bit=0x%02X", bit);
                if (bit != 1) {
                  sync = false;
                  break;
                }
              }
              if (sync == true) {
                  //ESP_LOGI(TAG, "Sync 0x55FFFF detected");
                  DL_Bus_Buffer[0] = recieveByte();
                  DL_Bus_Buffer[1] = recieveByte(); 
                  byte deviceType = DL_Bus_Buffer[1];
                  //check DeviceType
                  if (deviceType == 0x00){
                    return DLBus::sensorSlave();
                    // Warning: after end of response, master does not wait 2ms as specified! It only waits 100µs!
                    // This must be kept in mind when making Sync-Algo!
                  }
                  if (deviceType == 0x80){
                    //ESP_LOGI(TAG, "captureSinglePacket");
                    return DLBus::captureSinglePacket();
                  }
                  else {
                    // error exit
                    ESP_LOGI(TAG, "unknown deviceType");
                    ESP_LOGI(TAG, "deviceType=0x%02X", DL_Bus_Buffer[1]);
                    detachInterrupt(digitalPinToInterrupt(DL_Input_Pin));
                    return false;
                  }
              }
              //ESP_LOGE(TAG, "sync failed after recieved 0x55");
          }
      }
      detachInterrupt(digitalPinToInterrupt(DL_Input_Pin));
      edgeBufferWritePos = 0;
      edgeBufferReadPos = 0;
      edgeBufferCount = 0;
      yield();
      //timeoutcheck
      if ((millis() - T_Start) > timeout) {
        //detachInterrupt(digitalPinToInterrupt(DL_Input_Pin));
        return false;
      }
    }
}
