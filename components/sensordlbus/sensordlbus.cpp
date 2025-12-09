#include "sensordlbus.h"
#include "esphome/core/log.h"
#include "DLBus.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace sensordlbus {

static const char *TAG = "sensordlbus";

void SensorDLBus::setup() {
    this->dlBus_ = new DLBus(this->input_pin_, this->output_pin_);
    ESP_LOGD(TAG, "Setting up SensorDLBus");
    return;
}

void SensorDLBus::loop() {
  // Diese Methode wird kontinuierlich aufgerufen
  // und versucht, DL-Bus Pakete zu empfangen
  if (this->dlBus_ == nullptr) {
    ESP_LOGE(TAG, "dlBus_ is NULL in loop()!");
    return;
  }

  this->dlBus_->capture(); 
  return;
}

void SensorDLBus::update() {
  // Diese Methode wird periodisch aufgerufen (alle 10 Sekunden)
  // und publiziert die Sensor-Werte direkt aus dlBus.lastFrame
  if (this->dlBus_ == nullptr) {
    ESP_LOGW(TAG, "DLBus not initialized!");
    return;
  }

  if (!this->dlBus_->has_valid_data) {
    ESP_LOGW(TAG, "No valid data received yet");
    
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
  }
  else {
        // Daten sind gültig und aktuell - publiziere sie direkt aus lastFrame
        publish_sensors_();
  }  
  
  return;
}

void SensorDLBus::publish_sensors_() {
  ESP_LOGD(TAG, "Publishing sensor values");
  
  // Lese direkt aus dlBus.lastFrame
  if (this->deviceTypeSensor_ != nullptr) 
      this->deviceTypeSensor_->publish_state(this->dlBus_->lastFrame.DeviceID);

  if (this->tempSensor1_ != nullptr) 
      this->tempSensor1_->publish_state(this->dlBus_->lastFrame.Sensor1 / 10.0);
  if (this->tempSensor2_ != nullptr) 
      this->tempSensor2_->publish_state(this->dlBus_->lastFrame.Sensor2 / 10.0);
  if (this->tempSensor3_ != nullptr) 
      this->tempSensor3_->publish_state(this->dlBus_->lastFrame.Sensor3 / 10.0);
  if (this->tempSensor4_ != nullptr) 
      this->tempSensor4_->publish_state(this->dlBus_->lastFrame.Sensor4 / 10.0);
  if (this->tempSensor5_ != nullptr) 
      this->tempSensor5_->publish_state(this->dlBus_->lastFrame.Sensor5 / 10.0);
  if (this->tempSensor6_ != nullptr) 
      this->tempSensor6_->publish_state(this->dlBus_->lastFrame.Sensor6 / 10.0);

  if (this->outputA1Sensor_ != nullptr) 
      this->outputA1Sensor_->publish_state((this->dlBus_->lastFrame.Outputs >> 0) & 0b1);
  if (this->outputA2Sensor_ != nullptr) 
      this->outputA2Sensor_->publish_state((this->dlBus_->lastFrame.Outputs >> 1) & 0b1);
  if (this->outputA3Sensor_ != nullptr) 
      this->outputA3Sensor_->publish_state((this->dlBus_->lastFrame.Outputs >> 2) & 0b1);
  if (this->outputA4Sensor_ != nullptr) 
      this->outputA4Sensor_->publish_state((this->dlBus_->lastFrame.Outputs >> 3) & 0b1);
  if (this->outputA5Sensor_ != nullptr) 
      this->outputA5Sensor_->publish_state((this->dlBus_->lastFrame.Outputs >> 4) & 0b1);
  if (this->outputA6Sensor_ != nullptr) 
      this->outputA6Sensor_->publish_state((this->dlBus_->lastFrame.Outputs >> 5) & 0b1);
  if (this->outputA7Sensor_ != nullptr) 
      this->outputA7Sensor_->publish_state((this->dlBus_->lastFrame.Outputs >> 6) & 0b1);
}

void RoomTemperatureNumber::setup() {
  float default_value = 19.0f;
  this->publish_state(default_value);
  if (this->parent_ != nullptr && this->parent_->get_dlbus() != nullptr) {
    this->parent_->get_dlbus()->roomTemperatureRASDL = default_value;
  }
}

void RoomTemperatureNumber::control(float value) {
  if (this->parent_ != nullptr && this->parent_->get_dlbus() != nullptr) {
    this->parent_->get_dlbus()->roomTemperatureRASDL = value;
  }
  this->publish_state(value);
}

void HeatingModeSelect::setup() {
  this->publish_state("NORMAL");
  if (this->parent_ != nullptr && this->parent_->get_dlbus() != nullptr) {
    this->parent_->get_dlbus()->currentHeatingMode = DLBus::NORMAL;
  }
}

void HeatingModeSelect::control(const std::string &value) {
  if (this->parent_ != nullptr && this->parent_->get_dlbus() != nullptr) {
    if (value == "AUTOMATIC") {
      this->parent_->get_dlbus()->currentHeatingMode = DLBus::AUTOMATIC;
    } else if (value == "NORMAL") {
      this->parent_->get_dlbus()->currentHeatingMode = DLBus::NORMAL;
    } else if (value == "ENERGYSAVING") {
      this->parent_->get_dlbus()->currentHeatingMode = DLBus::ENERGYSAVING;
    } else if (value == "STANDBY") {
      this->parent_->get_dlbus()->currentHeatingMode = DLBus::STANDBY;
    }
  }
  this->publish_state(value);
}

}  // namespace sensordlbus
}  // namespace esphome
