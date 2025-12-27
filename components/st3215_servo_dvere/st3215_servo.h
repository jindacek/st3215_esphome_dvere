#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/core/helpers.h"
#include <vector>

// 👉 forward declaration binary_sensor (BEZ include)
namespace esphome {
namespace binary_sensor {
class BinarySensor;
}
}

namespace esphome {
namespace st3215_servo {

static const float RAW_PER_TURN = 4096.0f;

// ======================== Torque Switch ========================
class St3215Servo;

class St3215TorqueSwitch : public switch_::Switch, public Component {
 public:
  void set_parent(St3215Servo *p) { parent_ = p; }
  void write_state(bool state) override;
 protected:
  St3215Servo *parent_{nullptr};
};

// =================== Auto Unlock Switch ===================
class St3215AutoUnlockSwitch : public switch_::Switch, public Component {
 public:
  void set_parent(St3215Servo *p) { parent_ = p; }
  void write_state(bool state) override;
 protected:
  St3215Servo *parent_{nullptr};
};

// ====================== Kalibrační stav ========================
enum CalibState {
  CALIB_IDLE = 0,
  CALIB_WAIT_TOP = 1,
  CALIB_WAIT_BOTTOM = 2,
  CALIB_DONE = 3,
  CALIB_ERROR = 4
};


// =========================== Servo =============================
class St3215Servo : public PollingComponent, public uart::UARTDevice {
 public:
  St3215Servo()
    : PollingComponent(100), uart::UARTDevice(nullptr) {}

  St3215Servo(uart::UARTComponent *parent, uint8_t id)
    : PollingComponent(100), uart::UARTDevice(parent), servo_id_(id) {}

  void setup() override;
  void dump_config() override;
  void update() override;

  // Binary sensor – stav torque (LOCKED / UNLOCKED)
  void set_torque_state_sensor(binary_sensor::BinarySensor *s) {
    torque_state_sensor_ = s;
  }

  // YAML settery
  void set_servo_id(uint8_t id) { servo_id_ = id; }
  void set_turns_full_open(float t) { max_turns_ = t; has_max_ = true; }
  void set_max_angle(float) {}

  // Inverze směru – pro fyzicky otočená serva
  void set_invert_direction(bool inv) { invert_direction_ = inv; }

  // Ovládání
  void rotate(bool cw, int speed);
  void stop();
  void set_torque(bool on);

  // voláno z torque_switch (ruční override)
  void set_torque_from_switch(bool on);

  // voláno z auto_unlock_switch
  void set_auto_unlock_from_switch(bool on);

  // Rampa Factor
  void set_ramp_factor(float f) { ramp_factor_ = f; pending_ramp_save_ = true; }

  // Kalibrace
  void set_zero();
  void set_max();
  void start_calibration();
  void confirm_calibration_step();

  // Sensory
  void set_angle_sensor(sensor::Sensor *s) { angle_sensor_ = s; }
  void set_turns_sensor(sensor::Sensor *s) { turns_sensor_ = s; }
  void set_percent_sensor(sensor::Sensor *s) { percent_sensor_ = s; }
  void set_calib_state_sensor(sensor::Sensor *s) { calib_state_sensor_ = s; }

  // Switch
  void set_torque_switch(St3215TorqueSwitch *s);
  void set_open_switch(switch_::Switch *s) { open_switch_ = s; }
  void set_close_switch(switch_::Switch *s) { close_switch_ = s; }
  void set_auto_unlock_switch(St3215AutoUnlockSwitch *s);


  // ===== POSITION MODE (AUTOPILOT) =====
  void move_to_percent(float pct);

 protected:
  uint8_t servo_id_{1};
  bool torque_on_{false};
  bool manual_torque_override_{false};   // true pokud uživatel drží torque přes torque_switch
  bool auto_unlock_{true};               // pokud true, torque se po stop() automaticky vypne

  // Binární senzor TORQUE
  binary_sensor::BinarySensor *torque_state_sensor_{nullptr};

  // Inverze směru (true = prohodit CW/CCW na drátu, logika zůstává stejná)
  bool invert_direction_{false};

  uint16_t last_raw_{0};
  bool have_last_{false};
  float turns_unwrapped_{0};
  int32_t turns_base_{0};

  float zero_offset_{0};
  float max_turns_{0};
  bool has_zero_{false};
  bool has_max_{false};

  // Uložená absolutní poloha z flash
  float stored_turns_{0.0f};
  bool has_stored_turns_{false};

  // Persistentní kalibrace + poloha (NVS)
  bool load_calibration_();
  void save_calibration_();

  // Rampa Factor
  float ramp_factor_ = 1.0f;
  bool pending_ramp_save_ = false;

  // Kalibrace
  bool calibration_active_{false};
  CalibState calib_state_{CALIB_IDLE};

  // Switche
  switch_::Switch *open_switch_{nullptr};
  switch_::Switch *close_switch_{nullptr};

  // Sensory
  sensor::Sensor *angle_sensor_{nullptr};
  sensor::Sensor *turns_sensor_{nullptr};
  sensor::Sensor *percent_sensor_{nullptr};
  sensor::Sensor *calib_state_sensor_{nullptr};

  St3215TorqueSwitch *torque_switch_{nullptr};
  St3215AutoUnlockSwitch *auto_unlock_switch_{nullptr};


  // Stav pohybu pro SW koncáky (logický směr: CW = DOLŮ, CCW = NAHORU)
  bool moving_{false};
  bool moving_cw_{false};

  // ===== RAMP ENGINE =====
  int target_speed_{0};
  int current_speed_{0};
  uint32_t last_ramp_update_{0};

  // ===== POSITION MODE =====
  float target_percent_{-1.0f};   // -1 = žádný cíl
  bool position_mode_{false};     // aktivní řízení na cíl

  // ===== ENCODER FAULT =====
  uint8_t encoder_fail_count_{0};
  bool encoder_fault_{false};
  static constexpr uint8_t ENCODER_FAIL_LIMIT = 3;

  // ===== MULTI-SERVO STATE (oddělené pro každou instanci) =====
  uint32_t last_uart_recovery_{0};

  float ramp_last_dist_{0.0f};
  int   ramp_last_sent_speed_{-1};
  bool  ramp_last_sent_cw_{true};

  // Low level comm
  uint8_t checksum_(const uint8_t *data, size_t len);
  void send_packet_(uint8_t id, uint8_t cmd, const std::vector<uint8_t> &params);
  bool read_registers_(uint8_t id, uint8_t addr, uint8_t len, std::vector<uint8_t> &out);

  void update_calib_state_(CalibState s);
};

}  // namespace st3215_servo
}  // namespace esphome
