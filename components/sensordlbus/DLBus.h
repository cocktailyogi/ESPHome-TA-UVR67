
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

  bool capture();
  
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

private:

  // Strukturen
  struct InterruptData {
    uint32_t edgetime;
    bool pinState;
  };

  // Membervariablen, die vorher globale Variablen waren:
  InterruptData actData;
  unsigned long timeSincelastEdge;
  unsigned long timeOfActEdge;
  bool nextBit;
  bool curBit;
  uint32_t edgetime;
  volatile uint16_t edgeBufferWritePos;  // Schreibposition (ISR)
  volatile uint16_t edgeBufferReadPos;   // Leseposition (Main)
  volatile uint16_t edgeBufferCount;     // Anzahl Elemente
  static const int DL_Bus_PacketLength = 65;      // for UVR67 and UVR1611
  unsigned char DL_Bus_Buffer[DL_Bus_PacketLength];
  unsigned long T_Start;
  static const int EdgeBufferSize = 2000;
  static constexpr int bittime = 2048; // Mikrosekunden
  static constexpr int T = bittime / 2;
  static constexpr int margin = T / 4;
  static constexpr int Tmin = T - margin;                    // Mikrosekunden
  static constexpr int Tmax = T + margin;                   // Mikrosekunden
  static constexpr int timeout = 4000;                // ms
  InterruptData newData;
  InterruptData edgeTimeBuffer[EdgeBufferSize];

  // Private Hilfsfunktionen

  void handleInterrupt();

  bool loadBitFromEdgeTimeBuffer();
  int captureBit();
  unsigned char recieveByte();
  bool testChecksum();
  bool testChecksumSensorSlave();
  int16_t processTemperature(char lowByte, char highbyte);
  void processData();
  bool captureSinglePacket();
  bool sensorSlave();
  bool waitForBusIdle(unsigned long idleTimeMs);
  void sensorSlaveRespond(byte sensorAddress);
  void sendManchesterByte(uint8_t byte);
  void sendManchesterBit(bool bit);

};

#endif  // DLBus_h
