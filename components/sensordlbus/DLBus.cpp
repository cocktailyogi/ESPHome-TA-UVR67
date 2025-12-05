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
  currentHeatingMode = HeatingMode::NORMAL;
  roomTemperatureRASPT = 19.8;
  // set TX-Pin to inactive state
  pinMode(DL_Input_Pin, INPUT);
  pinMode(DL_Output_Pin, OUTPUT);
  digitalWrite(DL_Output_Pin, LOW);
  attachInterrupt(digitalPinToInterrupt(DL_Input_Pin), DLBus::isr, CHANGE);
  currentCaptureState = captureState::UNSYNC;
}

// Statische ISR-Funktion, die vom Interrupt-Controller aufgerufen wird.
void IRAM_ATTR DLBus::isr() {
  if (instance) {
    instance->handleInterrupt();
  }
}

void DLBus::resetManchesterBuffers() {
    currentCaptureState = captureState::UNSYNC;
    bitCount = 0;
    bitBuffer = 0;
    needSecondT = false;
}

byte DLBus::getByteFromBuffer() {
    // check for start/stop bits
    bool stopBit = bitBuffer & 0x0001;        // Bit 0
    bool startBit = (bitBuffer >> 9) & 0x0001; // Bit 9
    byte receivedByte = 0xFF;
    if (startBit == 0 && stopBit == 1) {
        receivedByte = (bitBuffer >> 1) & 0xFF; 
    }
    bitCount = 0;
    bitBuffer = 0;
    needSecondT = false;
    return receivedByte;
}

byte DLBus::getByteFromBuffer_WithoutStartStop() {
    byte receivedByte = bitBuffer & 0xFF;
    bitCount = 0;
    bitBuffer = 0;
    needSecondT = false;
    return receivedByte;
}

bool DLBus::aquireByte(unsigned long duration){
    // returns true, when byte is ready
    byte result = captureBit(duration);
    if (result == 2){
        //error -> reset
        resetManchesterBuffers();
    }
    if (result == 0){
        bitBuffer = (bitBuffer << 1) | currentBit;
        bitCount++;
    }
    if (bitCount == 10) {
        return true;
    }
    if (bitCount > 10) {
        //error -> reset
        resetManchesterBuffers();
        ESP_LOGI(TAG, "DLBus::aquireByte(): bitCount > 10");
    }
    return false;
}

void DLBus::handleInterrupt() {
    unsigned long now = micros();
    unsigned long duration = now - lastEdgeTime;
    bool level = digitalRead(DL_Input_Pin);
    byte result = 0xFF;
    lastEdgeTime = now;

    switch (currentCaptureState) {
        case captureState::UNSYNC: 
            level = digitalRead(DL_Input_Pin);
            if(level == HIGH) {
                if (duration > 2 * Tmin && duration < 2 * Tmax) {
                    lastBit = level;
                    currentCaptureState = captureState::PREAMBLE_0x55;
                    ESP_LOGI(TAG, "DLBus::handleInterrupt(): switched to captureState::PREAMBLE_0x55");
                }
            }
            break;
        
        case captureState::PREAMBLE_0x55:
            // 0x55 without start/stop Bits
            result = captureBit(duration);
            if (result == 2){
                //error -> reset
                resetManchesterBuffers();
            }
            if (result == 0){
                bitBuffer = (bitBuffer << 1) | currentBit;
                bitCount++;
            }
            if (bitCount == 8) {
                byte receivedByte = getByteFromBuffer_WithoutStartStop();
                if (receivedByte == 0x55) {
                    currentCaptureState = captureState::PREAMBLE_0xFFFF;
                    ESP_LOGI(TAG, "DLBus::handleInterrupt(): switched to captureState::PREAMBLE_0xFFFF");
                }
                else {
                    resetManchesterBuffers();
                }
            }
            break;

        case captureState::PREAMBLE_0xFFFF:
            // 0xFFFF without start/stop Bits
            result = captureBit(duration);
            if (result == 2){
                //error -> reset
                resetManchesterBuffers();
            }
            if (result == 0){
                bitBuffer = (bitBuffer << 1) | currentBit;
                bitCount++;
            }
            if (bitCount == 8) {
                byte receivedByte = bitBuffer & 0xFF; // leave Bitbuffer untouched!
                if (receivedByte != 0xFF) {
                    resetManchesterBuffers();
                }
            }
            if (bitCount == 16) {
                if (bitBuffer == 0xFFFF) {
                    currentCaptureState = captureState::RECIEVE_BYTE0;
                    bitCount = 0;
                    bitBuffer = 0;
                }
                else {
                    resetManchesterBuffers();
                }
            }
            break;
          
        case captureState::RECIEVE_BYTE0:
            // expecting 0xFF
            if (aquireByte(duration)) {
                DL_Bus_Buffer[0] = getByteFromBuffer();
                currentCaptureState = captureState::RECIEVE_BYTE1_MSGTYPE;
            }
            break;
        
        case captureState::RECIEVE_BYTE1_MSGTYPE:
            if (aquireByte(duration)) {
                DL_Bus_Buffer[1] = getByteFromBuffer();
                currentCaptureState = captureState::MSGFRAME_SWITCH;
            }
            break;
        
        case captureState::MSGFRAME_SWITCH:
            if (DL_Bus_Buffer[1] == 0x00) {
                currentCaptureState = captureState::PREPARE_RECIEVE_SLAVEREQUEST;
            }
            else if (DL_Bus_Buffer[1] == 0x80) {
                currentCaptureState = captureState::PREPARE_RECIEVE_DATAFRAME;
            }
            else {
                resetManchesterBuffers();
                ESP_LOGI(TAG, "DLBus::handleInterrupt(): Unknown MSGFRAME_SWITCH=0x%02X", DL_Bus_Buffer[1]);
            }
            break;
        
        case captureState::PREPARE_RECIEVE_DATAFRAME:
            current_DL_Bus_Buffer_Index = 1;
            if(FLAG_NEW_DATAFRAME_PENDING == false) {
                currentCaptureState = captureState::RECIEVE_DATAFRAME;
            }
            else {
                ESP_LOGI(TAG, "DLBus::handleInterrupt(): FLAG_NEW_DATAFRAME_PENDING still set");
            }
            break;
          
        case captureState::RECIEVE_DATAFRAME:
            if (aquireByte(duration)) {
                current_DL_Bus_Buffer_Index++;
                DL_Bus_Buffer[current_DL_Bus_Buffer_Index] = getByteFromBuffer();
            }
            if(current_DL_Bus_Buffer_Index == (DL_Bus_PacketLength - 1) ) {
                detachInterrupt(digitalPinToInterrupt(DL_Input_Pin));
                if (testChecksum() == true) {
                  // clean exit
                  FLAG_NEW_DATAFRAME_PENDING = true;
                }
                else {
                  // error exit
                  ESP_LOGI(TAG, "Dataframe Checksum Error");
                  resetManchesterBuffers();
                  //ESP_LOGI(TAG, "Buffer[0]=0x%02X, Buffer[1]=0x%02X", DL_Bus_Buffer[0], DL_Bus_Buffer[1]);
                }
                attachInterrupt(digitalPinToInterrupt(DL_Input_Pin), DLBus::isr, CHANGE);
            }
            break;
        
        case captureState::PREPARE_RECIEVE_SLAVEREQUEST:
            current_DL_Bus_Buffer_Index = 1;
            if(FLAG_NEW_SLAVEREQUEST_PENDING == false) {
                currentCaptureState = captureState::RECIEVE_SLAVEREQUEST;
            }
            else {
                ESP_LOGI(TAG, "DLBus::handleInterrupt(): FLAG_NEW_SLAVEREQUEST_PENDING still set");
            }
            break;

        case captureState::RECIEVE_SLAVEREQUEST:
            if (aquireByte(duration)) {
                current_DL_Bus_Buffer_Index++;
                DL_Bus_Buffer[current_DL_Bus_Buffer_Index] = DLBus::getByteFromBuffer();
            }
            if(current_DL_Bus_Buffer_Index == 3) {
                detachInterrupt(digitalPinToInterrupt(DL_Input_Pin));
                if (testChecksumSensorSlave() == true) {
                    // clean exit
                    FLAG_NEW_SLAVEREQUEST_PENDING = true;
                }
                else {
                  // error exit
                  ESP_LOGI(TAG, "MasterSlaveframe Checksum Error");
                  resetManchesterBuffers();
                  //ESP_LOGI(TAG, "Buffer[0]=0x%02X, Buffer[1]=0x%02X", DL_Bus_Buffer[0], DL_Bus_Buffer[1]);
                }
                attachInterrupt(digitalPinToInterrupt(DL_Input_Pin), DLBus::isr, CHANGE);
            }
            break;
          
        default:
            ESP_LOGI(TAG, "DLBus::handleInterrupt(): Unknown currentCaptureState=0x%02X", currentCaptureState);
            break;

    }
    return;

}

byte DLBus::captureBit(unsigned long duration) {
    if (duration > Tmin && duration < Tmax) {
        if(needSecondT == false) {
            needSecondT = true;
            return 1;
        }
        else {
            needSecondT = false;
            currentBit = lastBit;
            //lastBit = currentBit;
            return 0;
        }
    }
    else if (duration > 2 * Tmin && duration < 2 * Tmax) {
        currentBit = !lastBit;
        lastBit = currentBit;
        return 0;
    }
    else {
        return 2;
    }
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


void DLBus::capture(){
    if(FLAG_NEW_DATAFRAME_PENDING == true) {
        processData();
        last_valid_data_timestamp = millis();
        has_valid_data = true;
        FLAG_NEW_DATAFRAME_PENDING = false;
        ESP_LOGI(TAG, "Dataframe recieved and processed");
    }

    if ( (last_valid_data_timestamp + timeout) < millis() ) {
        //Timeout
        has_valid_data = false;
    }

    if(FLAG_NEW_SLAVEREQUEST_PENDING == true) {
        detachInterrupt(digitalPinToInterrupt(DL_Input_Pin));
        byte sensorAddress = DL_Bus_Buffer[2];
        DLBus::sensorSlaveRespond(sensorAddress);
        FLAG_NEW_SLAVEREQUEST_PENDING = false;
        attachInterrupt(digitalPinToInterrupt(DL_Input_Pin), DLBus::isr, CHANGE);
        ESP_LOGI(TAG, "SLAVEREQUEST recieved and processed");
    }
    return;
}
