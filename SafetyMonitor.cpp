/**
 * @file SafetyMonitor.cpp
 */
#include "control/SafetyMonitor.hpp"
#include "utils/Logger.hpp"
#include <algorithm>

namespace control {

std::string to_string(SafetyEvent e) {
    switch (e) {
        case SafetyEvent::ADHESION_LOSS:   return "ADHESION_LOSS";
        case SafetyEvent::ATTITUDE_LIMIT:  return "ATTITUDE_LIMIT";
        case SafetyEvent::SERVO_OVERLOAD:  return "SERVO_OVERLOAD";
        case SafetyEvent::BATTERY_LOW:     return "BATTERY_LOW";
        case SafetyEvent::BATTERY_CRITICAL:return "BATTERY_CRITICAL";
        case SafetyEvent::FOOTHOLD_SLIP:   return "FOOTHOLD_SLIP";
        case SafetyEvent::CONTROL_TIMEOUT: return "CONTROL_TIMEOUT";
        case SafetyEvent::LOOP_OVERRUN:    return "LOOP_OVERRUN";
        case SafetyEvent::HARDWARE_FAULT:  return "HARDWARE_FAULT";
        case SafetyEvent::COMMS_LOST:      return "COMMS_LOST";
        default: return "UNKNOWN";
    }
}

SafetyMonitor::SafetyMonitor(const Config& cfg,
                              hexapod::Hexapod& hexapod,
                              gecko::GeckoAdhesion& gecko,
                              sensors::SensorFusion& sensors)
    : cfg_(cfg), hexapod_(hexapod), gecko_(gecko), sensors_(sensors)
{
    thresholds_.max_pitch_wall_deg  = cfg_.get<double>("safety.max_pitch_wall_deg",   15.0);
    thresholds_.max_roll_wall_deg   = cfg_.get<double>("safety.max_roll_wall_deg",    10.0);
    thresholds_.battery_low_v       = cfg_.get<double>("safety.battery_low_v",        10.5);
    thresholds_.battery_critical_v  = cfg_.get<double>("safety.battery_critical_v",    9.9);

    last_comm_ack_ = last_loop_ack_ = std::chrono::steady_clock::now();
}

void SafetyMonitor::update() {
    if (e_stop_) return;

    check_adhesion();
    check_attitude();
    check_battery();
    check_comms_timeout();
}

void SafetyMonitor::check_adhesion() {
    if (!wall_mode_) return;

    auto status = gecko_.status();
    if (!status.minimum_met) {
        trigger(SafetyEvent::ADHESION_LOSS,
                "Only " + std::to_string(status.adhered_count) +
                " pads adhered (min=" +
                std::to_string(gecko::GeckoAdhesion::MIN_ADHERED_FLOOR) + ")");
        e_stop_ = true;
    }
}

void SafetyMonitor::check_attitude() {
    const auto& imu = sensors_.imu();
    double max_pitch = wall_mode_ ? thresholds_.max_pitch_wall_deg
                                  : thresholds_.max_pitch_floor_deg;
    double max_roll  = wall_mode_ ? thresholds_.max_roll_wall_deg
                                  : thresholds_.max_roll_floor_deg;

    double pitch_deg = math::rad2deg(imu.rpy_fused.y);
    double roll_deg  = math::rad2deg(imu.rpy_fused.x);

    if (std::abs(pitch_deg) > max_pitch || std::abs(roll_deg) > max_roll) {
        trigger(SafetyEvent::ATTITUDE_LIMIT,
                "pitch=" + std::to_string(pitch_deg) +
                " roll="  + std::to_string(roll_deg));
        if (wall_mode_) e_stop_ = true;
    }
}

void SafetyMonitor::check_battery() {
    double v = sensors_.battery_voltage();
    if (v < thresholds_.battery_critical_v) {
        trigger(SafetyEvent::BATTERY_CRITICAL,
                std::to_string(v) + "V");
        e_stop_ = true;
    } else if (v < thresholds_.battery_low_v) {
        trigger(SafetyEvent::BATTERY_LOW, std::to_string(v) + "V");
    }
}

void SafetyMonitor::check_comms_timeout() {
    auto now  = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_comm_ack_).count();
    if (dt > thresholds_.control_timeout_s) {
        trigger(SafetyEvent::CONTROL_TIMEOUT,
                "no comm for " + std::to_string(dt) + "s");
    }
}

void SafetyMonitor::trigger(SafetyEvent e, const std::string& detail) {
    auto it = std::find(active_events_.begin(), active_events_.end(), e);
    if (it != active_events_.end()) return; // already active

    active_events_.push_back(e);
    LOG_ERROR("SAFETY: " + to_string(e) + (detail.empty() ? "" : " — " + detail));
    if (event_cb_) event_cb_(e, detail);
}

void SafetyMonitor::clear_event(SafetyEvent e) {
    active_events_.erase(
        std::remove(active_events_.begin(), active_events_.end(), e),
        active_events_.end());
}

bool SafetyMonitor::clear_e_stop() {
    if (active_events_.empty()) {
        e_stop_ = false;
        LOG_INFO("Safety: e-stop cleared");
        return true;
    }
    LOG_WARN("Safety: cannot clear e-stop — active events remain");
    return false;
}

void SafetyMonitor::ack_comms() { last_comm_ack_ = std::chrono::steady_clock::now(); }
void SafetyMonitor::ack_loop()  { last_loop_ack_ = std::chrono::steady_clock::now(); }

// Stub checks
void SafetyMonitor::check_servos()    {}
void SafetyMonitor::check_footholds() {}
void SafetyMonitor::check_loop_timing() {}

} // namespace control
