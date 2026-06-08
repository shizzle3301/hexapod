/**
 * @file Leg.cpp
 * @brief Leg kinematics, servo control, and gecko foot management
 */

#include "locomotion/Leg.hpp"
#include "utils/Logger.hpp"
#include "utils/MathUtils.hpp"
#include <cmath>
#include <stdexcept>

namespace hexapod {

// ─────────────────────────────────────────────────────────────────────────────
Leg::Leg(LegID id, const LegConfig& cfg, bool simulation)
    : id_(id), cfg_(cfg), simulation_(simulation)
{
    // Initialise tip at home position via FK
    tip_pos_ = forward_kinematics(angles_);
}

// ─────────────────────────────────────────────────────────────────────────────
Vec3 Leg::forward_kinematics(const JointAngles& a) const {
    using namespace math;

    double coxa_r  = deg2rad(a.coxa);
    double femur_r = deg2rad(a.femur);
    double tibia_r = deg2rad(a.tibia);

    // Coxa tip (XY plane rotation)
    double cx = cfg_.coxa_length * std::cos(coxa_r);
    double cy = cfg_.coxa_length * std::sin(coxa_r);

    // Femur reaches forward-down
    double femur_x = cfg_.femur_length * std::cos(coxa_r) * std::cos(femur_r);
    double femur_y = cfg_.femur_length * std::sin(coxa_r) * std::cos(femur_r);
    double femur_z = cfg_.femur_length * std::sin(femur_r);

    // Tibia (knee joint adds to femur direction)
    double tibia_angle = femur_r + tibia_r;
    double tibia_x = cfg_.tibia_length * std::cos(coxa_r) * std::cos(tibia_angle);
    double tibia_y = cfg_.tibia_length * std::sin(coxa_r) * std::cos(tibia_angle);
    double tibia_z = cfg_.tibia_length * std::sin(tibia_angle);

    // Sum all segments — position in leg-local frame, then add mount offset
    Vec3 tip;
    tip.x = cx + femur_x + tibia_x + cfg_.mount_offset.x;
    tip.y = cy + femur_y + tibia_y + cfg_.mount_offset.y;
    tip.z =      femur_z + tibia_z + cfg_.mount_offset.z;

    return tip;
}

// ─────────────────────────────────────────────────────────────────────────────
bool Leg::inverse_kinematics(const Vec3& target_body, JointAngles& out) const {
    using namespace math;

    // Transform body-frame target to leg-local frame
    Vec3 t = body_to_leg_frame(target_body);

    // ── Coxa (rotation in XY plane) ──────────────────
    double coxa_angle = std::atan2(t.y, t.x);

    // Horizontal reach from coxa pivot to target
    double reach_xy = std::sqrt(t.x * t.x + t.y * t.y) - cfg_.coxa_length;

    // Vertical offset (z component)
    double dz = t.z;

    // Distance from femur pivot to foot in the sagittal plane
    double d = std::sqrt(reach_xy * reach_xy + dz * dz);

    // Check reachability
    double max_reach = cfg_.femur_length + cfg_.tibia_length;
    double min_reach = std::abs(cfg_.femur_length - cfg_.tibia_length);
    if (d > max_reach || d < min_reach) {
        LOG_WARN("Leg " + std::to_string(static_cast<int>(id_)) +
                 " IK out of reach: d=" + std::to_string(d));
        return false;
    }

    // ── Tibia (by cosine rule) ────────────────────────
    double cos_tibia = (cfg_.femur_length * cfg_.femur_length
                      + cfg_.tibia_length * cfg_.tibia_length
                      - d * d)
                     / (2.0 * cfg_.femur_length * cfg_.tibia_length);
    cos_tibia = clamp(cos_tibia, -1.0, 1.0);
    double tibia_angle = std::acos(cos_tibia) - PI; // negative = bent down

    // ── Femur ─────────────────────────────────────────
    double alpha = std::atan2(dz, reach_xy);
    double cos_beta = (cfg_.femur_length * cfg_.femur_length + d * d
                     - cfg_.tibia_length * cfg_.tibia_length)
                    / (2.0 * cfg_.femur_length * d);
    cos_beta = clamp(cos_beta, -1.0, 1.0);
    double beta = std::acos(cos_beta);
    double femur_angle = alpha - beta;

    // ── Build result ──────────────────────────────────
    out.coxa  = rad2deg(coxa_angle);
    out.femur = rad2deg(femur_angle);
    out.tibia = rad2deg(tibia_angle);

    return clamp_angles(out);
}

// ─────────────────────────────────────────────────────────────────────────────
Vec3 Leg::body_to_leg_frame(const Vec3& body_pos) const {
    // Translate by mount offset, then rotate by -mount_angle
    Vec3 shifted = body_pos - cfg_.mount_offset;
    double c = std::cos(-cfg_.mount_angle_rad);
    double s = std::sin(-cfg_.mount_angle_rad);
    return { shifted.x * c - shifted.y * s,
             shifted.x * s + shifted.y * c,
             shifted.z };
}

// ─────────────────────────────────────────────────────────────────────────────
bool Leg::clamp_angles(JointAngles& a) const {
    using namespace math;

    bool clamped = false;
    auto chk = [&](double& v, double lo, double hi, const char* name) {
        double clamped_v = clamp(v, lo, hi);
        if (std::abs(clamped_v - v) > 0.1) {
            LOG_DEBUG(std::string(name) + " clamped from " +
                      std::to_string(v) + " to " + std::to_string(clamped_v));
            clamped = true;
        }
        v = clamped_v;
    };

    chk(a.coxa,  cfg_.coxa_min,  cfg_.coxa_max,  "coxa");
    chk(a.femur, cfg_.femur_min, cfg_.femur_max,  "femur");
    chk(a.tibia, cfg_.tibia_min, cfg_.tibia_max,  "tibia");

    return !clamped;  // return true if angles were within limits
}

// ─────────────────────────────────────────────────────────────────────────────
bool Leg::move_to(const Vec3& target) {
    JointAngles sol;
    if (!inverse_kinematics(target, sol)) {
        LOG_WARN("Leg " + std::to_string(static_cast<int>(id_)) +
                 ": IK failed for target " + target.to_string());
        return false;
    }
    set_joint_angles(sol);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
void Leg::set_joint_angles(const JointAngles& angles) {
    angles_ = angles;
    clamp_angles(angles_);
    tip_pos_ = forward_kinematics(angles_);
    write_servos(angles_);
}

// ─────────────────────────────────────────────────────────────────────────────
void Leg::write_servos(const JointAngles& a) {
    if (simulation_) {
        // In simulation, just log at debug level
        LOG_DEBUG("Leg " + std::to_string(static_cast<int>(id_)) +
                  " servos: coxa=" + std::to_string(a.coxa) +
                  " femur=" + std::to_string(a.femur) +
                  " tibia=" + std::to_string(a.tibia));
        return;
    }

#ifdef PLATFORM_RASPI
    // TODO: Write via PCA9685 I2C servo driver
    // ServoDriver::set_angle(cfg_.coxa_servo_ch,  a.coxa);
    // ServoDriver::set_angle(cfg_.femur_servo_ch, a.femur);
    // ServoDriver::set_angle(cfg_.tibia_servo_ch, a.tibia);
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
void Leg::home() {
    JointAngles home_angles{ 0.0, -30.0, 60.0 };
    set_joint_angles(home_angles);
}

// ─────────────────────────────────────────────────────────────────────────────
void Leg::retract() {
    JointAngles retract{ 0.0, 70.0, -140.0 };
    set_joint_angles(retract);
}

// ─────────────────────────────────────────────────────────────────────────────
void Leg::engage_gecko() {
    if (simulation_) {
        LOG_INFO("Leg " + std::to_string(static_cast<int>(id_)) +
                 ": ENGAGE gecko pad (sim)");
        adhered_ = true;
        return;
    }
    // TODO: Hardware — pulse peel servo to 0°, wait press_time_ms, read FSR
    adhered_ = true;
}

// ─────────────────────────────────────────────────────────────────────────────
void Leg::disengage_gecko() {
    if (simulation_) {
        LOG_INFO("Leg " + std::to_string(static_cast<int>(id_)) +
                 ": DISENGAGE gecko pad (sim)");
        adhered_ = false;
        return;
    }
    // TODO: Hardware — rotate peel servo to 15° to break contact
    adhered_ = false;
}

// ─────────────────────────────────────────────────────────────────────────────
void Leg::swing_step(const Vec3& next_foothold, double lift_height, double progress) {
    if (progress <= 0.0) {
        start_pos_    = tip_pos_;
        swing_target_ = next_foothold;
    }
    swing_progress_ = progress;

    Vec3 pos = math::swing_trajectory(start_pos_, swing_target_, lift_height, progress);
    move_to(pos);
}

// ─────────────────────────────────────────────────────────────────────────────
void Leg::stance_step(const Vec3& body_delta) {
    // In stance, foot stays in world frame — body moves, so tip moves opposite
    Vec3 new_tip = tip_pos_ - body_delta;
    move_to(new_tip);
}

// ─────────────────────────────────────────────────────────────────────────────
void Leg::update() {
    // Nothing needed this tick unless we add async servo feedback
}

} // namespace hexapod
