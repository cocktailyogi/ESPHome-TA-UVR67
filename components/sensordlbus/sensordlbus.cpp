#include "sensordlbus.h"
#include "esphome/core/log.h"
#include "DLBus.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace sensordlbus {

static const char *TAG = "sensordlbus";
DLBus dlBus;  // Erzeugt eine Instanz der Klasse DLBus

void SensorDLBus::setup() {
  ESP_LOGD(TAG, "Setting up SensorDLBus");
  has_valid_data_ = false;
  last_valid_data_timestamp_ = 0;
  ESP_LOGI(TAG, "Timeout for stale data: %lu ms (%.1f minutes)", TIMEOUT_MS, TIMEOUT_MS / 60000.0);
}

void SensorDLBus::loop() {
  // Diese Methode wird kontinuierlich aufgerufen
  // und versucht, DL-Bus Pakete zu empfangen
  
  // Versuche ein Paket zu empfangen (nicht-blockierend)
  if (dlBus.capture()) {
    // Paket erfolgreich empfangen!
    // Daten bleiben in dlBus.lastFrame
    // Wir merken uns nur den Zeitstempel
    last_valid_data_timestamp_ = millis();
    has_valid_data_ = true;
  }
  
  // Kleine Pause, um anderen Tasks CPU-Zeit zu geben
  yield();
}

void SensorDLBus::update() {
  // Diese Methode wird periodisch aufgerufen (alle 10 Sekunden)
  // und publiziert die Sensor-Werte direkt aus dlBus.lastFrame
  
  if (!has_valid_data_) {
    ESP_LOGW(TAG, "No valid data received yet");
    return;
  }
  
  // Prüfe, ob Daten zu alt sind 
  if (is_data_stale_()) {
    unsigned long age_seconds = (millis() - last_valid_data_timestamp_) / 1000;
    ESP_LOGE(TAG, "Data is stale! Last update: %lu seconds ago (%.1f minutes)", 
             age_seconds, age_seconds / 60.0);
    
    // Markiere Daten als ungültig
    has_valid_data_ = false;
    
    // Publiziere NaN für alle Sensoren um Fehler zu signalisieren
    if (this->tempSensor1_ != nullptr) 
        this->tempSensor1_->publish_state(NAN);
    if (this->tempSensor2_ != nullptr) 
        this->tempSensor2_->publish_state(NAN);
    if (this->tempSensor3_ != nullptr) 
        this->tempSensor3_->publish_state(NAN);
    if (this->tempSensor4_ != nullptr) 
        this->tempSensor4_->publish_state(NAN);
    if (this->tempSensor5_ != nullptr) 
        this->tempSensor5_->publish_state(NAN);
    if (this->tempSensor6_ != nullptr) 
        this->tempSensor6_->publish_state(NAN);
    
    if (this->outputA1Sensor_ != nullptr) 
        this->outputA1Sensor_->publish_state(NAN);
    if (this->outputA2Sensor_ != nullptr) 
        this->outputA2Sensor_->publish_state(NAN);
    if (this->outputA3Sensor_ != nullptr) 
        this->outputA3Sensor_->publish_state(NAN);
    if (this->outputA4Sensor_ != nullptr) 
        this->outputA4Sensor_->publish_state(NAN);
    if (this->outputA5Sensor_ != nullptr) 
        this->outputA5Sensor_->publish_state(NAN);
    if (this->outputA6Sensor_ != nullptr) 
        this->outputA6Sensor_->publish_state(NAN);
    if (this->outputA7Sensor_ != nullptr) 
        this->outputA7Sensor_->publish_state(NAN);
    
    return;
  }
  
  // Daten sind gültig und aktuell - publiziere sie direkt aus lastFrame
  publish_sensors_();
}

void SensorDLBus::publish_sensors_() {
  unsigned long age_ms = millis() - last_valid_data_timestamp_;
  ESP_LOGD(TAG, "Publishing sensor values (data age: %lu ms)", age_ms);
  
  // Lese direkt aus dlBus.lastFrame
  if (this->deviceTypeSensor_ != nullptr) 
      this->deviceTypeSensor_->publish_state(dlBus.lastFrame.DeviceID);

  if (this->tempSensor1_ != nullptr) 
      this->tempSensor1_->publish_state(dlBus.lastFrame.Sensor1 / 10.0);
  if (this->tempSensor2_ != nullptr) 
      this->tempSensor2_->publish_state(dlBus.lastFrame.Sensor2 / 10.0);
  if (this->tempSensor3_ != nullptr) 
      this->tempSensor3_->publish_state(dlBus.lastFrame.Sensor3 / 10.0);
  if (this->tempSensor4_ != nullptr) 
      this->tempSensor4_->publish_state(dlBus.lastFrame.Sensor4 / 10.0);
  if (this->tempSensor5_ != nullptr) 
      this->tempSensor5_->publish_state(dlBus.lastFrame.Sensor5 / 10.0);
  if (this->tempSensor6_ != nullptr) 
      this->tempSensor6_->publish_state(dlBus.lastFrame.Sensor6 / 10.0);

  if (this->outputA1Sensor_ != nullptr) 
      this->outputA1Sensor_->publish_state((dlBus.lastFrame.Outputs >> 0) & 0b1);
  if (this->outputA2Sensor_ != nullptr) 
      this->outputA2Sensor_->publish_state((dlBus.lastFrame.Outputs >> 1) & 0b1);
  if (this->outputA3Sensor_ != nullptr) 
      this->outputA3Sensor_->publish_state((dlBus.lastFrame.Outputs >> 2) & 0b1);
  if (this->outputA4Sensor_ != nullptr) 
      this->outputA4Sensor_->publish_state((dlBus.lastFrame.Outputs >> 3) & 0b1);
  if (this->outputA5Sensor_ != nullptr) 
      this->outputA5Sensor_->publish_state((dlBus.lastFrame.Outputs >> 4) & 0b1);
  if (this->outputA6Sensor_ != nullptr) 
      this->outputA6Sensor_->publish_state((dlBus.lastFrame.Outputs >> 5) & 0b1);
  if (this->outputA7Sensor_ != nullptr) 
      this->outputA7Sensor_->publish_state((dlBus.lastFrame.Outputs >> 6) & 0b1);
}

bool SensorDLBus::is_data_stale_() {
  unsigned long age = millis() - last_valid_data_timestamp_;
  return age > TIMEOUT_MS;
}

}  // namespace sensordlbus
}  // namespace esphome
