#include "sensordlbus.h"
#include "esphome/core/log.h"
#include "DLBus.h"
#include "esphome/core/helpers.h"
//#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace sensordlbus {

static const char *TAG = "sensordlbus";
DLBus dlBus;  // Erzeugt eine Instanz der Klasse DLBus

void SensorDLBus::setup() {
  ESP_LOGD(TAG, "Setting up SensorDLBus");
  dlBus.init(); // Initialisiert das Objekt, z. B. setzt interne Zustände
}

void SensorDLBus::update() {
    int i = 0;
    while (i < 3) {
        if (dlBus.captureSinglePacket()) {
            ESP_LOGD("DL-Bus", "Received Packet");

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
            
            break;
        } else {
            ESP_LOGD("DL-Bus", "Signal-Error");
            i++;
            delay(50);
        }
    }
}



}  // namespace sensordlbus
}  // namespace esphome
