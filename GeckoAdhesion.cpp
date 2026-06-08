/**
 * @file GeckoAdhesion.cpp
 * @brief Gecko-tape adhesion state machine and hardware interface
 */

#include "gecko/GeckoAdhesion.hpp"
#include <stdexcept>
#include <cmath>

namespace gecko {


std::string to_string(PadState s) {
    switch (s) {
        case PadState::FREE:       return "FREE";
        case PadState::CONTACTING: return "CONTACTING";
        case PadState::PRESSING:   return "PRESSING";
        case PadState::ADHERED:    return "ADHERED";
        case PadState::PEELING:    return "PEELING";
        case PadState::FAILED:     return "FAILED";
        default:                   return "UNKNOWN";
    }
}


GeckoAdhesion::GeckoAdhesion(const Config& cfg, bool simulation)
    : cfg_(cfg), simulation_(simulation)
{
    for (int i = 0; i < NUM_PADS; ++i)
        state_entry_time_[i] = std::chrono::steady_clock::now();
}

void GeckoAdhesion::init() {
    LOG_INFO("GeckoAdhesion: initialising " +
             std::to_string(NUM_PADS) + " gecko pads");

    // Load per-pad config
    for (int i = 0; i < NUM_PADS; ++i) {
        pad_cfg_[i].foot_fsr_channel   = cfg_.get<int>(
            "gecko.pad" + std::to_string(i) + ".fsr_ch",  i);
        pad_cfg_[i].peel_servo_channel = cfg_.get<int>(
            "gecko.pad" + std::to_string(i) + ".peel_ch", 18 + i);
        pad_cfg_[i].peel_angle_deg     = cfg_.get<double>(
            "gecko.peel_angle_deg", 15.0);
        pad_cfg_[i].press_time_ms      = cfg_.get<double>(
            "gecko.press_time_ms",  80.0);
        pad_cfg_[i].min_adhesion_n     = cfg_.get<double>(
            "gecko.min_adhesion_n",  2.0);

        //set peel to 0
        set_peel_servo(i, 0.0);
    }

    LOG_INFO("GeckoAdhesion: all pads initialised in FREE state");
}


void GeckoAdhesion::update() {
    for (int i = 0; i < NUM_PADS; ++i) {
        update_pad(i);
    }
}


void GeckoAdhesion::update_pad(int idx) {
    auto& d = pad_data_[idx];
    auto  now = std::chrono::steady_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(
                            now - state_entry_time_[idx]).count();

    //read sensor
    d.contact_force_n = read_fsr(idx);

    switch (d.state) {
        case PadState::FREE:
            //detect unexpected cont
            if (d.contact_force_n > pad_cfg_[idx].min_adhesion_n * 0.5) {
                d.surface_detected = true;
                LOG_DEBUG("Pad " + std::to_string(idx) + ": surface detected");
            }
            break;

        case PadState::CONTACTING:
            //wait for confirmed cont
            if (d.contact_force_n >= pad_cfg_[idx].min_adhesion_n * 0.5) {
                transition(idx, PadState::PRESSING);
            } else if (elapsed_ms > 500.0) {
                LOG_WARN("Pad " + std::to_string(idx) +
                         ": no contact after 500ms, returning to FREE");
                transition(idx, PadState::FREE);
            }
            break;

        case PadState::PRESSING:
            //maintain press
            if (elapsed_ms >= pad_cfg_[idx].press_time_ms) {
                if (d.contact_force_n >= pad_cfg_[idx].min_adhesion_n) {
                    transition(idx, PadState::ADHERED);
                    ++d.engage_count;
                    LOG_INFO("Pad " + std::to_string(idx) + ": ADHERED ✓ (" +
                             std::to_string(d.contact_force_n) + " N)");
                } else {
                    LOG_WARN("Pad " + std::to_string(idx) +
                             ": adhesion force too low: " +
                             std::to_string(d.contact_force_n) + " N");
                    transition(idx, PadState::FREE);
                }
            }
            break;

        case PadState::ADHERED:
            //detect peel!!!!!!!!!!!!
            if (d.contact_force_n < pad_cfg_[idx].min_adhesion_n * 0.3) {
                LOG_ERROR("Pad " + std::to_string(idx) +
                          ": ADHESION LOST unexpectedly!");
                transition(idx, PadState::FAILED);
                d.fault = true;
                if (fault_cb_)
                    fault_cb_(idx, "Adhesion lost during stance");
            }
            break;

        case PadState::PEELING:
            //peel act working
            set_peel_servo(idx, pad_cfg_[idx].peel_angle_deg);
            if (d.contact_force_n < pad_cfg_[idx].min_adhesion_n * 0.2
                || elapsed_ms > 300.0) {
                //peel all done C :
                set_peel_servo(idx, 0.0);
                transition(idx, PadState::FREE);
                LOG_INFO("Pad " + std::to_string(idx) + ": disengaged ✓");
            }
            break;

        case PadState::FAILED:
            //somefing is fukt clear rn!!!!!!!! 
            break;
    }
}


void GeckoAdhesion::transition(int idx, PadState new_state) {
    PadState old_state = pad_data_[idx].state;
    if (old_state == new_state) return;

    pad_data_[idx].state = new_state;
    state_entry_time_[idx] = std::chrono::steady_clock::now();

    if (callback_) callback_(idx, new_state);
}


void GeckoAdhesion::engage(int pad_idx) {
    if (pad_idx < 0 || pad_idx >= NUM_PADS) return;
    auto& d = pad_data_[pad_idx];

    if (d.state == PadState::ADHERED) {
        LOG_DEBUG("Pad " + std::to_string(pad_idx) + ": already adhered");
        return;
    }

    LOG_INFO("Pad " + std::to_string(pad_idx) + ": engaging...");
    d.fault = false;
    transition(pad_idx, PadState::CONTACTING);
}


bool GeckoAdhesion::disengage(int pad_idx) {
    if (pad_idx < 0 || pad_idx >= NUM_PADS) return false;

    if (!can_disengage(pad_idx, true)) {
        LOG_WARN("Pad " + std::to_string(pad_idx) +
                 ": disengage refused — would violate safety minimum");
        return false;
    }

    LOG_INFO("Pad " + std::to_string(pad_idx) + ": disengaging...");
    transition(pad_idx, PadState::PEELING);
    return true;
}


void GeckoAdhesion::disengage_all() {
    LOG_INFO("GeckoAdhesion: disengaging ALL pads");
    for (int i = 0; i < NUM_PADS; ++i) {
        set_peel_servo(i, pad_cfg_[i].peel_angle_deg);
        transition(i, PadState::FREE);
    }
}


bool GeckoAdhesion::is_adhered(int pad_idx) const {
    return pad_data_[pad_idx].state == PadState::ADHERED;
}


bool GeckoAdhesion::can_disengage(int pad_idx, bool wall_mode) const {
    int minimum = wall_mode ? MIN_ADHERED_WALL : MIN_ADHERED_FLOOR;

    // Count how many other pads are adhered
    int other_adhered = 0;
    for (int i = 0; i < NUM_PADS; ++i) {
        if (i != pad_idx && is_adhered(i)) ++other_adhered;
    }
    return other_adhered >= minimum;
}


int GeckoAdhesion::adhered_count() const {
    int n = 0;
    for (int i = 0; i < NUM_PADS; ++i)
        if (is_adhered(i)) ++n;
    return n;
}

AdhesionStatus GeckoAdhesion::status() const {
    AdhesionStatus s;
    s.adhered_count   = 0;
    s.total_adhesion_n = 0.0;

    for (int i = 0; i < NUM_PADS; ++i) {
        s.pad_states[i] = pad_data_[i].state;
        if (pad_data_[i].state == PadState::ADHERED) {
            ++s.adhered_count;
            s.total_adhesion_n += pad_data_[i].contact_force_n;
        }
    }

    s.free_count   = NUM_PADS - s.adhered_count;
    s.minimum_met  = (s.adhered_count >= MIN_ADHERED_FLOOR);
    s.wall_safe    = (s.adhered_count >= MIN_ADHERED_WALL);
    s.system_ok    = s.minimum_met;

    //check for fault
    for (int i = 0; i < NUM_PADS; ++i)
        if (pad_data_[i].fault) { s.system_ok = false; break; }

    return s;
}


void GeckoAdhesion::log_status() const {
    auto s = status();
    std::string line = "Gecko pads [";
    for (int i = 0; i < NUM_PADS; ++i) {
        switch (pad_data_[i].state) {
            case PadState::ADHERED:    line += "A"; break;
            case PadState::FREE:       line += "."; break;
            case PadState::PEELING:    line += "P"; break;
            case PadState::PRESSING:   line += "p"; break;
            case PadState::CONTACTING: line += "c"; break;
            case PadState::FAILED:     line += "F"; break;
        }
    }
    line += "]  adhered=" + std::to_string(s.adhered_count) +
            "  total_force=" + std::to_string(s.total_adhesion_n) + "N" +
            (s.wall_safe ? "  [WALL SAFE]" : "  [!]");
    LOG_INFO(line);
}


double GeckoAdhesion::read_fsr(int pad_idx) const {
    if (simulation_) return sim_contact_force(pad_idx);

#ifdef PLATFORM_RASPI
    // TODO: Read ADC via I2C (e.g. ADS1115)
    // return adc_driver_.read_voltage(pad_cfg_[pad_idx].foot_fsr_channel)
    //        * FSR_VOLTS_TO_NEWTONS;
#endif
    return 0.0;
}

double GeckoAdhesion::sim_contact_force(int pad_idx) const {
    auto& d = pad_data_[pad_idx];
    switch (d.state) {
        case PadState::CONTACTING: return pad_cfg_[pad_idx].min_adhesion_n * 0.6;
        case PadState::PRESSING:   return pad_cfg_[pad_idx].min_adhesion_n * 1.2;
        case PadState::ADHERED:    return pad_cfg_[pad_idx].min_adhesion_n * 2.5;
        case PadState::PEELING:    return pad_cfg_[pad_idx].min_adhesion_n * 0.1;
        default:                   return 0.0;
    }
}


void GeckoAdhesion::set_peel_servo(int pad_idx, double angle_deg) {
    if (simulation_) {
        LOG_DEBUG("Pad " + std::to_string(pad_idx) +
                  " peel servo → " + std::to_string(angle_deg) + "°");
        return;
    }
#ifdef PLATFORM_RASPI
    // TODO: servo_driver_.set_angle(pad_cfg_[pad_idx].peel_servo_channel, angle_deg);
    (void)pad_idx;
    (void)angle_deg;
#endif
}

void GeckoAdhesion::run_self_test() {
    LOG_INFO("GeckoAdhesion: running self-test on all pads");
    for (int i = 0; i < NUM_PADS; ++i) {
        set_peel_servo(i, pad_cfg_[i].peel_angle_deg);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        set_peel_servo(i, 0.0);
        LOG_INFO("Pad " + std::to_string(i) + ": peel actuator OK");
    }
    LOG_INFO("GeckoAdhesion: self-test complete");
}

]
