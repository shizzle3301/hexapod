/**
 * @file Hexapod.cpp
 * @brief Hexapod top-level: leg management, body pose, state machine
 */

#include "locomotion/Hexapod.hpp"
#include "utils/Logger.hpp"
#include <stdexcept>
#include <sstream>

namespace hexapod {

// im made this cool ass ascii diagram to show u guys whatsup n allat
//
//          FR   FL
//         /       \
//       MR         ML
//         \       /
//          RR   RL
//
//angles from body forward (+X):
//   FL =  60°  FR = -60°
//   ML =  90°  MR = -90°
//   RL = 120°  RR = -120°
//
//body (rn) is 120mm wide × 200mm long


static LegConfig make_default_leg_config(LegID id, int servo_base_ch) {
    LegConfig c;
    c.coxa_length  = 52.0;
    c.femur_length = 66.0;
    c.tibia_length = 130.0;

    c.coxa_servo_ch  = servo_base_ch;
    c.femur_servo_ch = servo_base_ch + 1;
    c.tibia_servo_ch = servo_base_ch + 2;

    const double R = 100.0;//mount rad [must be one coooool ass hill bro]

    switch (id) {
        case LegID::FRONT_LEFT:
            c.mount_angle_rad = math::deg2rad(60.0);
            c.mount_offset = {R * std::cos(c.mount_angle_rad),
                              R * std::sin(c.mount_angle_rad), 0};
            break;
        case LegID::MIDDLE_LEFT:
            c.mount_angle_rad = math::deg2rad(90.0);
            c.mount_offset = {0, R, 0};
            break;
        case LegID::REAR_LEFT:
            c.mount_angle_rad = math::deg2rad(120.0);
            c.mount_offset = {R * std::cos(c.mount_angle_rad),
                              R * std::sin(c.mount_angle_rad), 0};
            break;
        case LegID::REAR_RIGHT:
            c.mount_angle_rad = math::deg2rad(-120.0);
            c.mount_offset = {R * std::cos(c.mount_angle_rad),
                              R * std::sin(c.mount_angle_rad), 0};
            break;
        case LegID::MIDDLE_RIGHT:
            c.mount_angle_rad = math::deg2rad(-90.0);
            c.mount_offset = {0, -R, 0};
            break;
        case LegID::FRONT_RIGHT:
            c.mount_angle_rad = math::deg2rad(-60.0);
            c.mount_offset = {R * std::cos(c.mount_angle_rad),
                              R * std::sin(c.mount_angle_rad), 0};
            break;
    }
    return c;
}


Hexapod::Hexapod(const Config& cfg, bool simulation)
    : cfg_(cfg), simulation_(simulation)
{}


void Hexapod::init() {
    build_legs();
    LOG_INFO("Hexapod: " + std::to_string(NUM_LEGS) + " legs initialised");
    set_state(RobotState::IDLE);
}


void Hexapod::build_legs() {
    const std::array<LegID, NUM_LEGS> ids = {
        LegID::FRONT_LEFT, LegID::MIDDLE_LEFT, LegID::REAR_LEFT,
        LegID::REAR_RIGHT, LegID::MIDDLE_RIGHT, LegID::FRONT_RIGHT
    };

    for (int i = 0; i < NUM_LEGS; ++i) {
        auto lcfg = make_default_leg_config(ids[i], i * 3);
        // Allow config override
        // (In a real build, read from cfg_)
        legs_[i] = std::make_unique<Leg>(ids[i], lcfg, simulation_);
    }
}


void Hexapod::update() {
    if (state_ == RobotState::EMERGENCY_STOP) return;

    for (int i = 0; i < NUM_LEGS; ++i) {
        legs_[i]->update();
    }
}


void Hexapod::park() {
    LOG_INFO("Hexapod: parking to safe pose");
    for (int i = 0; i < NUM_LEGS; ++i) {
        legs_[i]->disengage_gecko();
        legs_[i]->retract();
    }
    set_state(RobotState::PARKING);
}


void Hexapod::emergency_stop() {
    LOG_ERROR("Hexapod: EMERGENCY STOP");
    set_state(RobotState::EMERGENCY_STOP);
    //freeze all legs, maintain adhesion
    
}


void Hexapod::set_state(RobotState s) {
    if (s == state_) return;
    RobotState prev = state_;
    state_ = s;
    LOG_INFO("Hexapod state: " + to_string(prev) + " → " + to_string(s));
    if (state_cb_) state_cb_(prev, s);
}


void Hexapod::set_body_pose(const BodyPose& pose) {
    body_pose_ = pose;
}

void Hexapod::translate_body(const Vec3& delta) {
    body_pose_.position += delta;
}

void Hexapod::rotate_body(const Vec3& drpy) {
    body_pose_.rpy += drpy;
}


std::array<Vec3, Hexapod::NUM_LEGS> Hexapod::foot_positions() const {
    std::array<Vec3, NUM_LEGS> fps;
    for (int i = 0; i < NUM_LEGS; ++i)
        fps[i] = legs_[i]->tip_position();
    return fps;
}


int Hexapod::planted_count() const {
    int n = 0;
    for (int i = 0; i < NUM_LEGS; ++i)
        if (legs_[i]->is_planted()) ++n;
    return n;
}

int Hexapod::adhered_count() const {
    int n = 0;
    for (int i = 0; i < NUM_LEGS; ++i)
        if (legs_[i]->is_adhered()) ++n;
    return n;
}


Vec3 Hexapod::support_polygon_cog() const {
    Vec3 sum{0, 0, 0};
    int  count = 0;
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (legs_[i]->is_planted()) {
            sum += legs_[i]->tip_position();
            ++count;
        }
    }
    return count > 0 ? sum / count : body_pose_.position;
}

bool Hexapod::is_stable() const {
    // Collect planted footholds in XY
    std::vector<Vec3> poly;
    for (int i = 0; i < NUM_LEGS; ++i)
        if (legs_[i]->is_planted())
            poly.push_back(legs_[i]->tip_position());

    if (poly.size() < 3) return false;

    Vec3 cog = body_pose_.position;
    cog.z = 0;  // project to ground plane

    return math::point_in_convex_polygon(cog, poly);
}


void Hexapod::set_velocity(const Vec3& linear_mms, double yaw_rads) {
    cmd_linear_ = linear_mms;
    cmd_yaw_    = yaw_rads;
}

void Hexapod::set_surface_normal(const Vec3& normal) {
    surface_normal_ = normal.normalised();
}


std::array<Vec3, Hexapod::NUM_LEGS> Hexapod::compute_default_footholds(
    const BodyPose& pose, const Vec3& /*surface_normal*/) const
{
    std::array<Vec3, NUM_LEGS> footholds;
    for (int i = 0; i < NUM_LEGS; ++i) {
        //default foothold=coxa_length + femur_length * 0.7 radially outward
        const auto& lcfg = legs_[i]->config();
        double r = lcfg.coxa_length + lcfg.femur_length * 0.7;
        footholds[i] = {
            pose.position.x + lcfg.mount_offset.x +
                r * std::cos(lcfg.mount_angle_rad),
            pose.position.y + lcfg.mount_offset.y +
                r * std::sin(lcfg.mount_angle_rad),
            0.0   //ground 
        };
    }
    return footholds;
}


std::string to_string(RobotState s) {
    switch (s) {
        case RobotState::IDLE:                   return "IDLE";
        case RobotState::HOMING:                 return "HOMING";
        case RobotState::STANDING:               return "STANDING";
        case RobotState::WALKING_FLAT:           return "WALKING_FLAT";
        case RobotState::TRANSITIONING_TO_WALL:  return "TRANSITIONING_TO_WALL";
        case RobotState::CLIMBING_WALL:          return "CLIMBING_WALL";
        case RobotState::TRAVERSING_EDGE:        return "TRAVERSING_EDGE";
        case RobotState::EMERGENCY_STOP:         return "EMERGENCY_STOP";
        case RobotState::PARKING:                return "PARKING";
        default:                                 return "UNKNOWN";
    }
}

}
