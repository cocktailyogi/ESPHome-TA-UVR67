#pragma once

#ifndef DLBus_h
#define DLBus_h

#define SET_BIT(byte, bit) ((byte) |= (1UL << (bit)))

#include <Arduino.h>
#include "esphome/core/log.h"

class DLBus {
public:
  // Konstanten (statt #define)
  static const int DL_Input_Pin = 27;           // Arduino-Pin
  static const int EdgeBufferSize = 2000;
  static const int DL_Bus_PacketLength = 65;      // for UVR67 and UVR1611
  static constexpr int bittime = 2048; //ms
  static constexpr int T = bittime / 2;
  static constexpr int margin = T / 5;
  static constexpr int Tmin = T - margin;                    // Mikrosekunden
  static constexpr int Tmax = T + margin;                   // Mikrosekunden
  static constexpr int timeout = 4000;                // Mikrosekunden


  // Strukturen
  struct InterruptData {
    uint32_t edgetime;
    bool pinState;
  };

  struct DL_Bus_Frame {
    uint8_t DeviceID;
    uint8_t Sec, Min, Hour, Day, Month;
    uint16_t Year;
    int16_t Sensor1, Sensor2, Sensor3, Sensor4, Sensor5, Sensor6;
    uint16_t Outputs;
  };

  // Konstruktor
  DLBus();

  // Initialisierung (sollte vor dem Start der Kommunikation aufgerufen werden)
  void init();

  // Diese Funktion versucht, ein Paket vom DL-Bus zu erfassen.
  
  bool capture();
  

  // Das zuletzt empfangene Frame
  DL_Bus_Frame lastFrame;

  // Interrupt-Service-Routine (ISR) als statische Methode, die intern die Instanz-Methode aufruft
  static void IRAM_ATTR isr();

  // Zeiger auf die Instanz (wird im Konstruktor gesetzt)
  static DLBus *instance;

private:
  // Membervariablen, die vorher globale Variablen waren:
  InterruptData actData;
  unsigned long timeSincelastEdge;
  unsigned long timeOfActEdge;
  bool nextBit;
  bool curBit;
  uint32_t edgetime;
  InterruptData newData;
  InterruptData edgeTimeBuffer[EdgeBufferSize];
  volatile uint16_t edgeBufferWritePos;  // Schreibposition (ISR)
  volatile uint16_t edgeBufferReadPos;   // Leseposition (Main)
  volatile uint16_t edgeBufferCount;     // Anzahl Elemente
  unsigned char DL_Bus_Buffer[DL_Bus_PacketLength];
  unsigned long T_Start;

  // Private Hilfsfunktionen
  #ifdef DEBUGPIN_ENABLE
  void DebugPulse();
  #endif

  // Nicht-statische Interruptbehandlung (wird von isr() aufgerufen)
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

};

#endif  // DLBus_h
