
#include "locomotion/GaitController.hpp"
#include "utils/Logger.hpp"
#include <cmath>
#include <sstream>

namespace hexapod {

std::string to_string(GaitType g) {
    switch (g) {
        case GaitType::TRIPOD:      return "TRIPOD";
        case GaitType::WAVE:        return "WAVE";
        case GaitType::RIPPLE:      return "RIPPLE";
        case GaitType::GECKO_WALL:  return "GECKO_WALL";
        case GaitType::GECKO_EDGE:  return "GECKO_EDGE";
        default: return "UNKNOWN";
    }
}

GaitController::GaitController(const Config& cfg) : cfg_(cfg) {
    set_gait(GaitType::WAVE);
}

void GaitController::set_gait(GaitType type) {
    params_.type = type;
    phases_      = compute_phases(type);

    switch (type) {
        case GaitType::TRIPOD:
            params_.duty_factor    = 0.5;
            params_.cycle_time_s   = 0.8;
            params_.step_length_mm = 50.0;
            break;
        case GaitType::WAVE:
            params_.duty_factor    = 5.0/6.0;
            params_.cycle_time_s   = 1.5;
            params_.step_length_mm = 40.0;
            break;
        case GaitType::GECKO_WALL:
            params_.duty_factor     = 5.0/6.0;
            params_.cycle_time_s    = 2.0;
            params_.step_length_mm  = 25.0;
            params_.step_height_wall= 12.0;
            params_.wall_mode       = true;
            break;
        default:
            break;
    }

    LOG_INFO("Gait: " + to_string(type) +
             " (duty=" + std::to_string(params_.duty_factor) +
             ", step=" + std::to_string(params_.step_length_mm) + "mm)");
}

void GaitController::set_params(const GaitParams& p) { params_ = p; }
void GaitController::reset()                          { cycle_phase_ = 0.0; }

GaitController::LegPhases GaitController::compute_phases(GaitType type) const {
    switch (type) {
        case GaitType::TRIPOD:     return TRIPOD_PHASES;
        case GaitType::WAVE:       return WAVE_PHASES;
        case GaitType::RIPPLE:     return RIPPLE_PHASES;
        case GaitType::GECKO_WALL: return GECKO_PHASES;
        case GaitType::GECKO_EDGE: return GECKO_PHASES;
        default:                   return WAVE_PHASES;
    }
}

void GaitController::step(std::array<std::unique_ptr<Leg>, 6>& legs,
                           const Vec3& body_velocity, double yaw_rate, double dt)
{
    if (body_velocity.norm() < 0.1 && std::abs(yaw_rate) < 0.01) {
        //standing still
        return;
    }

    cycle_phase_ += dt / params_.cycle_time_s;
    if (cycle_phase_ >= 1.0) cycle_phase_ -= 1.0;

    double lift_h = params_.wall_mode
                  ? params_.step_height_wall
                  : params_.step_height_mm;

    for (int i = 0; i < 6; ++i) {
        double leg_phase = std::fmod(cycle_phase_ + phases_[i], 1.0);

        //swing phase
        bool in_swing = (leg_phase >= params_.duty_factor);

        if (in_swing) {
            //check stable
            if (params_.wall_mode && !can_lift_leg(i, legs)) {
                //force stance
                legs[i]->stance_step(body_velocity * dt);
                continue;
            }

            double swing_progress = (leg_phase - params_.duty_factor)
                                  / (1.0 - params_.duty_factor);

            Vec3 next_fh = compute_next_foothold(i, body_velocity, yaw_rate);
            legs[i]->swing_step(next_fh, lift_h, swing_progress);
            legs[i]->set_planted(false);
        } else {
            //push
            legs[i]->stance_step(body_velocity * dt);
            legs[i]->set_planted(true);
        }
    }
}

bool GaitController::can_lift_leg(int idx,
    const std::array<std::unique_ptr<Leg>, 6>& legs) const
{
    int adhered = 0;
    for (int i = 0; i < 6; ++i)
        if (i != idx && legs[i]->is_adhered()) ++adhered;

    return adhered >= gecko::GeckoAdhesion::MIN_ADHERED_WALL;
}

Vec3 GaitController::compute_next_foothold(int leg_idx, const Vec3& body_vel,
                                            double yaw) const
{
    //default footholf=current tip + step_length in body_vel direction
    double step = params_.step_length_mm;
    Vec3 dir = body_vel.norm() > 0.01 ? body_vel.normalised() : Vec3{1,0,0};
    return dir * step;
}

std::vector<int> GaitController::swing_legs() const {
    std::vector<int> sw;
    for (int i = 0; i < 6; ++i) {
        double p = std::fmod(cycle_phase_ + phases_[i], 1.0);
        if (p >= params_.duty_factor) sw.push_back(i);
    }
    return sw;
}

std::vector<int> GaitController::stance_legs() const {
    std::vector<int> st;
    for (int i = 0; i < 6; ++i) {
        double p = std::fmod(cycle_phase_ + phases_[i], 1.0);
        if (p < params_.duty_factor) st.push_back(i);
    }
    return st;
}

} 
