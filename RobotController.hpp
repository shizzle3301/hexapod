// RobotController.hpp
#pragma once
#include "locomotion/Hexapod.hpp"
#include "gecko/GeckoAdhesion.hpp"
#include "sensors/SensorFusion.hpp"
#include "control/SafetyMonitor.hpp"
#include "utils/Config.hpp"

namespace control {
class RobotController {
public:
    RobotController(const Config& cfg, hexapod::Hexapod& hex,
                    gecko::GeckoAdhesion& gecko,
                    sensors::SensorFusion& sensors,
                    SafetyMonitor& safety)
        : cfg_(cfg), hexapod_(hex), gecko_(gecko),
          sensors_(sensors), safety_(safety) {}

    void update(uint64_t tick) {
        (void)tick;
        // High-level state dispatch — handled by WallClimbController
    }

private:
    const Config& cfg_;
    hexapod::Hexapod& hexapod_;
    gecko::GeckoAdhesion& gecko_;
    sensors::SensorFusion& sensors_;
    SafetyMonitor& safety_;
};
} // namespace control
