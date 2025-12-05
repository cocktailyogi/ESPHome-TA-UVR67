
#ifndef DLBus_h
#define DLBus_h

#define SET_BIT(byte, bit) ((byte) |= (1UL << (bit)))

#include <Arduino.h>

class DLBus {
public:
  // Konstanten (statt #define)
  static const int DL_Input_Pin = 27;           // Arduino-Pin
  static const int DL_Output_Pin = 17;          // Arduino-Pin

  // Konstruktor
  DLBus();

  void capture();
  
  struct DL_Bus_Frame {
    uint8_t DeviceID;
    uint8_t Sec, Min, Hour, Day, Month;
    uint16_t Year;
    int16_t Sensor1, Sensor2, Sensor3, Sensor4, Sensor5, Sensor6;
    uint16_t Outputs;
  };

  enum HeatingMode {
    AUTOMATIC = 0b000,
    NORMAL = 0b010,
    ENERGYSAVING = 0b100,
    STANDBY = 0b110,
  };

  HeatingMode currentHeatingMode;
  float roomTemperatureRASPT;

  // Das zuletzt empfangene Frame
  DL_Bus_Frame lastFrame;

  // Interrupt-Service-Routine (ISR) als statische Methode, die intern die Instanz-Methode aufruft
  static void IRAM_ATTR isr();

  // Zeiger auf die Instanz (wird im Konstruktor gesetzt)
  static DLBus *instance;

  unsigned long last_valid_data_timestamp;
  bool has_valid_data = false;

private:

  enum captureState {
    UNSYNC = 0,
    PREAMBLE_0x55 = 1,
    PREAMBLE_0xFFFF = 2,
    RECIEVE_BYTE0 = 3,
    RECIEVE_BYTE1_MSGTYPE = 4,
    MSGFRAME_SWITCH = 5,
    PREPARE_RECIEVE_DATAFRAME = 6,
    RECIEVE_DATAFRAME = 7,
    PREPARE_RECIEVE_SLAVEREQUEST = 8,
    RECIEVE_SLAVEREQUEST = 9,
  };

  static const int DL_Bus_PacketLength = 65;      // for UVR67 and UVR1611
  volatile unsigned char DL_Bus_Buffer[DL_Bus_PacketLength];
  volatile byte current_DL_Bus_Buffer_Index = 0;

  static constexpr int bittime = 2048; // Mikrosekunden
  static constexpr int T = bittime / 2;
  static constexpr int margin = 200;
  static constexpr int Tmin = T - margin;                    // Mikrosekunden
  static constexpr int Tmax = T + margin;                   // Mikrosekunden
  static constexpr int timeout = 10000;                // ms

  volatile unsigned long lastEdgeTime = 0;
  volatile uint16_t bitBuffer = 0;
  volatile int bitCount = 0;
  volatile bool lastBit = HIGH;
  volatile bool currentBit = HIGH;
  volatile bool needSecondT = false;
  volatile bool FLAG_NEW_DATAFRAME_PENDING = false;
  volatile bool FLAG_NEW_SLAVEREQUEST_PENDING = false;

  // Private Hilfsfunktionen

  void handleInterrupt();

  byte captureBit(unsigned long duration);
  bool aquireByte(unsigned long duration);
  bool testChecksum();
  bool testChecksumSensorSlave();
  int16_t processTemperature(char lowByte, char highbyte);
  void processData();
  void sensorSlaveRespond(byte sensorAddress);
  void sendManchesterByte(uint8_t byte);
  void sendManchesterBit(bool bit);
  void resetManchesterBuffers();
  byte getByteFromBuffer_WithoutStartStop();
  

};

#endif  // DLBus_h
