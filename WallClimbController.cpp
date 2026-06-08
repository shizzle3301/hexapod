/**
 * @file WallClimbController.cpp
 * @brief Wall-climbing state machine implementation
 */

#include "control/WallClimbController.hpp"
#include "utils/Logger.hpp"
#include <stdexcept>

namespace control {

// ─────────────────────────────────────────────────────────────────────────────
std::string to_string(ClimbPhase p) {
    switch (p) {
        case ClimbPhase::IDLE:                  return "IDLE";
        case ClimbPhase::DETECTING_WALL:        return "DETECTING_WALL";
        case ClimbPhase::APPROACHING:           return "APPROACHING";
        case ClimbPhase::MOUNTING_FIRST_PAIR:   return "MOUNTING_FIRST_PAIR";
        case ClimbPhase::MOUNTING_SECOND_PAIR:  return "MOUNTING_SECOND_PAIR";
        case ClimbPhase::MOUNTING_REAR_PAIR:    return "MOUNTING_REAR_PAIR";
        case ClimbPhase::CLIMBING:              return "CLIMBING";
        case ClimbPhase::PAUSING:               return "PAUSING";
        case ClimbPhase::DETECTING_EDGE:        return "DETECTING_EDGE";
        case ClimbPhase::TRAVERSING_EDGE:       return "TRAVERSING_EDGE";
        case ClimbPhase::DESCENDING:            return "DESCENDING";
        case ClimbPhase::DISMOUNTING:           return "DISMOUNTING";
        case ClimbPhase::ABORTED:               return "ABORTED";
        default:                                return "UNKNOWN";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
WallClimbController::WallClimbController(
    const Config& cfg,
    hexapod::Hexapod& hexapod,
    gecko::GeckoAdhesion& gecko,
    sensors::SensorFusion& sensors,
    SafetyMonitor& safety)
    : cfg_(cfg), hexapod_(hexapod), gecko_(gecko),
      sensors_(sensors), safety_(safety)
{
    // Load climb config from file
    climb_cfg_.wall_detect_dist_mm  = cfg_.get<double>("climb.wall_detect_dist_mm",  150.0);
    climb_cfg_.approach_speed_mms   = cfg_.get<double>("climb.approach_speed_mms",    20.0);
    climb_cfg_.climb_speed_mms      = cfg_.get<double>("climb.climb_speed_mms",       15.0);
    climb_cfg_.min_adhesion_n       = cfg_.get<double>("climb.min_adhesion_n",         10.0);
    climb_cfg_.max_tilt_error_deg   = cfg_.get<double>("climb.max_tilt_error_deg",      5.0);

    // Create gait controller for wall climbing
    gait_ = std::make_unique<hexapod::GaitController>(cfg_);
    gait_->set_gait(hexapod::GaitType::GECKO_WALL);
}

// ─────────────────────────────────────────────────────────────────────────────
void WallClimbController::update(uint64_t /*tick*/) {
    // Abort on safety events
    if (safety_.e_stop() && phase_ != ClimbPhase::ABORTED) {
        LOG_ERROR("WallClimbController: safety e-stop triggered — aborting climb");
        abort();
        return;
    }

    switch (phase_) {
        case ClimbPhase::IDLE:                 handle_idle();                 break;
        case ClimbPhase::DETECTING_WALL:       handle_detecting_wall();       break;
        case ClimbPhase::APPROACHING:          handle_approaching();          break;
        case ClimbPhase::MOUNTING_FIRST_PAIR:  handle_mounting_first_pair();  break;
        case ClimbPhase::MOUNTING_SECOND_PAIR: handle_mounting_second_pair(); break;
        case ClimbPhase::MOUNTING_REAR_PAIR:   handle_mounting_rear_pair();   break;
        case ClimbPhase::CLIMBING:             handle_climbing();             break;
        case ClimbPhase::PAUSING:              handle_pausing();              break;
        case ClimbPhase::DETECTING_EDGE:       handle_detecting_edge();       break;
        case ClimbPhase::TRAVERSING_EDGE:      handle_traversing_edge();      break;
        case ClimbPhase::DESCENDING:           handle_descending();           break;
        case ClimbPhase::DISMOUNTING:          handle_dismounting();          break;
        case ClimbPhase::ABORTED:              handle_aborted();              break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
void WallClimbController::start_climb() {
    if (phase_ != ClimbPhase::IDLE) {
        LOG_WARN("WallClimbController: start_climb called in phase " +
                 to_string(phase_));
        return;
    }
    LOG_INFO("WallClimbController: starting climb sequence");
    transition_to(ClimbPhase::DETECTING_WALL);
}

void WallClimbController::start_descent() {
    if (phase_ != ClimbPhase::CLIMBING && phase_ != ClimbPhase::PAUSING) {
        LOG_WARN("WallClimbController: start_descent called in phase " +
                 to_string(phase_));
        return;
    }
    transition_to(ClimbPhase::DESCENDING);
}

void WallClimbController::abort() {
    LOG_WARN("WallClimbController: ABORT — attempting safe recovery");
    gecko_.disengage_all();
    hexapod_.set_velocity({0, 0, 0}, 0);
    transition_to(ClimbPhase::ABORTED);
}

// ─────────────────────────────────────────────────────────────────────────────
void WallClimbController::handle_idle() {
    // Nothing — wait for start_climb()
}

// ─────────────────────────────────────────────────────────────────────────────
void WallClimbController::handle_detecting_wall() {
    const auto& prox = sensors_.proximity();
    if (prox.front_mm < climb_cfg_.wall_detect_dist_mm) {
        LOG_INFO("WallClimbController: wall detected at " +
                 std::to_string(prox.front_mm) + " mm");
        transition_to(ClimbPhase::APPROACHING);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
void WallClimbController::handle_approaching() {
    const auto& prox = sensors_.proximity();

    if (prox.front_mm < 50.0) {
        // Close enough — stop and begin mounting
        hexapod_.set_velocity({0, 0, 0}, 0);
        LOG_INFO("WallClimbController: at wall — beginning mount sequence");
        transition_to(ClimbPhase::MOUNTING_FIRST_PAIR);
        return;
    }

    // Slow down as we approach
    double speed = (prox.front_mm < climb_cfg_.edge_approach_slow_mm)
                 ? climb_cfg_.approach_speed_mms * 0.5
                 : climb_cfg_.approach_speed_mms;

    hexapod_.set_velocity({speed, 0, 0}, 0);
}

// ─────────────────────────────────────────────────────────────────────────────
void WallClimbController::handle_mounting_first_pair() {
    // Place front legs (FL=0, FR=5) onto wall
    static bool started = false;
    if (!started) {
        LOG_INFO("WallClimbController: mounting front legs...");
        started = true;
    }

    if (mount_leg_pair(0, 5)) {  // FL, FR
        LOG_INFO("WallClimbController: front legs adhered ✓");
        started = false;
        transition_to(ClimbPhase::MOUNTING_SECOND_PAIR);
    }
}

void WallClimbController::handle_mounting_second_pair() {
    static bool started = false;
    if (!started) {
        LOG_INFO("WallClimbController: mounting middle legs...");
        started = true;
    }

    if (mount_leg_pair(1, 4)) {  // ML, MR
        LOG_INFO("WallClimbController: middle legs adhered ✓");
        started = false;
        transition_to(ClimbPhase::MOUNTING_REAR_PAIR);
    }
}

void WallClimbController::handle_mounting_rear_pair() {
    static bool started = false;
    if (!started) {
        LOG_INFO("WallClimbController: mounting rear legs...");
        started = true;
    }

    if (mount_leg_pair(2, 3)) {  // RL, RR
        LOG_INFO("WallClimbController: ALL legs adhered ✓ — wall climbing active!");
        started = false;

        // Switch to wall mode
        safety_.set_wall_mode(true);
        hexapod_.set_state(hexapod::RobotState::CLIMBING_WALL);
        align_body_to_wall();
        transition_to(ClimbPhase::CLIMBING);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
void WallClimbController::handle_climbing() {
    // Check adhesion safety
    auto status = gecko_.status();
    if (!status.minimum_met) {
        LOG_ERROR("WallClimbController: adhesion failure! Aborting.");
        abort();
        return;
    }

    // Pause if too low battery
    if (sensors_.battery_voltage() < 10.2) {
        LOG_WARN("WallClimbController: low battery — pausing on wall");
        hexapod_.set_velocity({0, 0, 0}, 0);
        transition_to(ClimbPhase::PAUSING);
        return;
    }

    // Check for edge ahead
    if (sensors_.surface().edge_detected &&
        sensors_.surface().edge_distance_mm < 80.0) {
        LOG_INFO("WallClimbController: edge detected — entering edge traversal");
        transition_to(ClimbPhase::DETECTING_EDGE);
        return;
    }

    // Normal climb: drive upward using gecko wall gait
    // Surface normal defines "up" direction
    Vec3 surface_normal = sensors_.estimated_surface_normal();
    hexapod_.set_surface_normal(surface_normal);

    // Velocity in surface normal direction = climbing upward
    Vec3 climb_vel = surface_normal * climb_cfg_.climb_speed_mms;
    hexapod_.set_velocity(climb_vel, 0.0);

    // Maintain body alignment with wall
    align_body_to_wall();
}

// ─────────────────────────────────────────────────────────────────────────────
void WallClimbController::handle_pausing() {
    hexapod_.set_velocity({0, 0, 0}, 0);
    // Wait for battery to be acceptable or manual resume
    if (sensors_.battery_voltage() > 10.8) {
        LOG_INFO("WallClimbController: resuming climb");
        transition_to(ClimbPhase::CLIMBING);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
void WallClimbController::handle_detecting_edge() {
    // Slow down before edge
    hexapod_.set_velocity(sensors_.surface().normal * 5.0, 0);
    if (sensors_.surface().edge_distance_mm < 30.0) {
        LOG_INFO("WallClimbController: at edge — beginning traversal");
        transition_to(ClimbPhase::TRAVERSING_EDGE);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
void WallClimbController::handle_traversing_edge() {
    // Edge traversal: top pair crosses over, body follows
    // This is the most complex phase — simplified here
    // TODO: Full edge traversal algorithm

    LOG_INFO("WallClimbController: traversing edge (simplified)");

    // 1. Place front legs on new surface
    // 2. Transfer body weight
    // 3. Move remaining legs across
    // 4. Update surface normal

    // For now: just continue and update the surface normal
    const auto& prox = sensors_.proximity();
    if (prox.front_mm > 200.0) {
        // We've crossed the edge
        LOG_INFO("WallClimbController: edge traversal complete");
        transition_to(ClimbPhase::CLIMBING);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
void WallClimbController::handle_descending() {
    hexapod_.set_velocity(sensors_.surface().normal * -climb_cfg_.climb_speed_mms, 0);

    // Detect approach to floor
    const auto& prox = sensors_.proximity();
    if (prox.front_mm > 500.0) {
        // We're low — probably approaching floor transition
        transition_to(ClimbPhase::DISMOUNTING);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
void WallClimbController::handle_dismounting() {
    LOG_INFO("WallClimbController: dismounting...");

    // Sequential disengage — rear → middle → front
    for (int i : {2, 3, 1, 4, 0, 5}) {
        gecko_.disengage(i);
        hexapod_.leg(i).set_planted(false);
    }

    hexapod_.set_velocity({0, 0, 0}, 0);
    safety_.set_wall_mode(false);
    hexapod_.set_state(hexapod::RobotState::WALKING_FLAT);
    transition_to(ClimbPhase::IDLE);
    LOG_INFO("WallClimbController: dismount complete — back on floor");
}

// ─────────────────────────────────────────────────────────────────────────────
void WallClimbController::handle_aborted() {
    static bool logged = false;
    if (!logged) {
        LOG_ERROR("WallClimbController: in ABORTED state — awaiting reset");
        logged = true;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
bool WallClimbController::mount_leg_pair(int leg_a, int leg_b) {
    // Returns true when both legs of the pair are confirmed adhered
    bool a_ok = hexapod_.leg(leg_a).is_adhered();
    bool b_ok = hexapod_.leg(leg_b).is_adhered();

    if (!a_ok) {
        // Position leg against wall and begin engage
        // (simplified — in reality we'd interpolate the leg toward surface)
        gecko_.engage(leg_a);
        hexapod_.leg(leg_a).engage_gecko();
    }
    if (!b_ok) {
        gecko_.engage(leg_b);
        hexapod_.leg(leg_b).engage_gecko();
    }

    return a_ok && b_ok;
}

// ─────────────────────────────────────────────────────────────────────────────
bool WallClimbController::wall_step(int leg_idx) {
    if (!check_adhesion_for_swing(leg_idx)) {
        LOG_DEBUG("Wall step refused for leg " + std::to_string(leg_idx) +
                  " — insufficient adhesion");
        return false;
    }

    // Disengage this pad, lift, move forward, re-engage
    gecko_.disengage(leg_idx);
    hexapod_.leg(leg_idx).disengage_gecko();
    hexapod_.leg(leg_idx).set_planted(false);

    // TODO: Actually move leg using GaitController

    gecko_.engage(leg_idx);
    hexapod_.leg(leg_idx).engage_gecko();
    hexapod_.leg(leg_idx).set_planted(true);

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
bool WallClimbController::check_adhesion_for_swing(int leg_idx) const {
    return gecko_.can_disengage(leg_idx, true);
}

// ─────────────────────────────────────────────────────────────────────────────
void WallClimbController::align_body_to_wall() {
    // Adjust body roll/pitch to stay parallel to wall surface
    const auto& imu     = sensors_.imu();
    Vec3 wall_normal    = sensors_.estimated_surface_normal();

    // Desired body Z axis should be anti-parallel to wall normal
    // (body Z points away from wall when clinging to it)
    // Compute required RPY correction
    double pitch_error = imu.rpy_fused.y;
    double roll_error  = imu.rpy_fused.x;

    // Small corrections to reduce tilt error
    hexapod::BodyPose adjusted = hexapod_.body_pose();
    adjusted.rpy.x -= roll_error  * 0.1;
    adjusted.rpy.y -= pitch_error * 0.1;
    hexapod_.set_body_pose(adjusted);
}

// ─────────────────────────────────────────────────────────────────────────────
void WallClimbController::transition_to(ClimbPhase next) {
    ClimbPhase prev = phase_;
    phase_ = next;
    LOG_INFO("Climb phase: " + to_string(prev) + " → " + to_string(next));
    if (phase_cb_) phase_cb_(prev, next);
}

// ─────────────────────────────────────────────────────────────────────────────
bool WallClimbController::on_wall() const {
    return (phase_ == ClimbPhase::CLIMBING ||
            phase_ == ClimbPhase::PAUSING  ||
            phase_ == ClimbPhase::DETECTING_EDGE ||
            phase_ == ClimbPhase::TRAVERSING_EDGE||
            phase_ == ClimbPhase::DESCENDING);
}

bool WallClimbController::can_start_climb() const {
    return (phase_ == ClimbPhase::IDLE) &&
           !safety_.e_stop()            &&
           sensors_.battery_voltage() > 11.0;
}

} // namespace control
