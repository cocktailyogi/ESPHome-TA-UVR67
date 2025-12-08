#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace sensordlbus {

class SensorDLBus : public PollingComponent {
 public:
  
  // Setter-Methoden für die Sensoren
  void set_device_type_sensor(sensor::Sensor *deviceTypeSensor){deviceTypeSensor_ = deviceTypeSensor; }
  void set_temp_sensor1(sensor::Sensor *tempSensor1){tempSensor1_ = tempSensor1; }
  void set_temp_sensor2(sensor::Sensor *tempSensor2){tempSensor2_ = tempSensor2; }
  void set_temp_sensor3(sensor::Sensor *tempSensor3){tempSensor3_ = tempSensor3; }
  void set_temp_sensor4(sensor::Sensor *tempSensor4){tempSensor4_ = tempSensor4; }
  void set_temp_sensor5(sensor::Sensor *tempSensor5){tempSensor5_ = tempSensor5; }
  void set_temp_sensor6(sensor::Sensor *tempSensor6){tempSensor6_ = tempSensor6; }
  void set_output_a1_sensor(sensor::Sensor *outputA1Sensor){outputA1Sensor_ = outputA1Sensor; }
  void set_output_a2_sensor(sensor::Sensor *outputA2Sensor){outputA2Sensor_ = outputA2Sensor; }
  void set_output_a3_sensor(sensor::Sensor *outputA3Sensor){outputA3Sensor_ = outputA3Sensor; }
  void set_output_a4_sensor(sensor::Sensor *outputA4Sensor){outputA4Sensor_ = outputA4Sensor; }
  void set_output_a5_sensor(sensor::Sensor *outputA5Sensor){outputA5Sensor_ = outputA5Sensor; }
  void set_output_a6_sensor(sensor::Sensor *outputA6Sensor){outputA6Sensor_ = outputA6Sensor; }
  void set_output_a7_sensor(sensor::Sensor *outputA7Sensor){outputA7Sensor_ = outputA7Sensor; }
  
  SensorDLBus() : PollingComponent(15000) {} // 15 Seconds  
  float get_setup_priority() const override { return esphome::setup_priority::DATA; }
  

  void setup() override;
  void loop() override;
  void update() override;
  
 protected:
  sensor::Sensor *deviceTypeSensor_{nullptr};
  sensor::Sensor *tempSensor1_{nullptr};
  sensor::Sensor *tempSensor2_{nullptr};
  sensor::Sensor *tempSensor3_{nullptr};
  sensor::Sensor *tempSensor4_{nullptr};
  sensor::Sensor *tempSensor5_{nullptr};
  sensor::Sensor *tempSensor6_{nullptr};
  sensor::Sensor *outputA1Sensor_{nullptr};
  sensor::Sensor *outputA2Sensor_{nullptr};
  sensor::Sensor *outputA3Sensor_{nullptr};
  sensor::Sensor *outputA4Sensor_{nullptr};
  sensor::Sensor *outputA5Sensor_{nullptr};
  sensor::Sensor *outputA6Sensor_{nullptr};
  sensor::Sensor *outputA7Sensor_{nullptr};
  
  // Nur Timestamp-Tracking, keine Daten-Kopie nötig!
  unsigned long last_valid_data_timestamp_{0};
  bool has_valid_data_{false};
  
  // Konstanten
  static const unsigned long TIMEOUT_MS = 180000; // 3 Minuten

  // Private Hilfsmethoden
  void publish_sensors_();
  bool is_data_stale_();
};

}  // namespace sensordlbus
}  // namespace esphome