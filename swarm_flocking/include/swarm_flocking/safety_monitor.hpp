// Copyright (c) 2024 Swarm Robotics Lab. BSD-3-Clause License.
// safety_monitor.hpp — Multi-layered safety system
// Provides: collision threat detection, geofence enforcement,
// communication loss handling, battery monitoring, and emergency protocols.

#pragma once

#include "swarm_flocking/types.hpp"
#include "swarm_flocking/communication_manager.hpp"
#include <map>
#include <chrono>
#include <functional>
#include <string>
#include <vector>

namespace swarm_flocking {

// ─── Safety threat levels ────────────────────────────────────────────────────

enum class ThreatLevel : uint8_t {
    NONE     = 0,
    ADVISORY = 1,   // log only
    CAUTION  = 2,   // slow down, increase spacing
    WARNING  = 3,   // active avoidance maneuver
    CRITICAL = 4,   // emergency: halt or RTL
    PANIC    = 5    // immediate motor kill if collision imminent
};

struct SafetyEvent {
    ThreatLevel level;
    std::string source;     // e.g. "collision", "geofence", "battery"
    std::string description;
    std::chrono::steady_clock::time_point timestamp;
};

// ─── Safety configuration ────────────────────────────────────────────────────

struct SafetyConfig {
    // Collision zones (meters)
    double r_caution  = 15.0;   // yellow zone — advisory
    double r_warning  = 8.0;    // orange zone — active avoidance
    double r_critical = 5.0;    // red zone — emergency brake
    double r_panic    = 2.0;    // failsafe — motor kill distance

    // Geofence
    double geofence_warn_margin = 20.0;  // warn this far inside boundary
    double geofence_hard_margin = 5.0;   // force RTL this far outside
    double max_altitude = 50.0;          // meters AGL
    double min_altitude = 2.0;           // meters AGL

    // Communication
    double comm_loss_warn_s   = 2.0;   // no neighbor heard for this long
    double comm_loss_critical = 5.0;   // trigger RTL
    double all_comm_loss_s    = 8.0;   // all neighbors lost → land

    // Battery
    float battery_warn_v    = 14.0f;  // 4S LiPo warning
    float battery_critical_v = 13.2f; // force land
    float battery_panic_v    = 12.8f; // emergency land

    // GPS
    float gps_accuracy_warn  = 3.0f;  // meters
    float gps_accuracy_crit  = 5.0f;  // too inaccurate to fly
    uint8_t min_satellites    = 8;

    // Speed limits
    double max_speed_override = 12.0;  // absolute max regardless of params
};

// ─── Safety Monitor ──────────────────────────────────────────────────────────

class SafetyMonitor {
public:
    using EmergencyCallback = std::function<void(const SafetyEvent&)>;

    explicit SafetyMonitor(const SafetyConfig& cfg = {});

    void set_config(const SafetyConfig& cfg) { config_ = cfg; }

    /// Register callback for emergency events
    void on_emergency(EmergencyCallback cb) { emergency_cb_ = std::move(cb); }

    /// Main evaluation: check all safety conditions
    /// Returns the highest threat level found
    ThreatLevel evaluate(
        const AgentState& self,
        const std::map<uint16_t, AgentState>& neighbors,
        const ArenaConfig& arena,
        const CommunicationManager::CommHealth& comm_health);

    /// Apply safety modifications to desired velocity
    /// May reduce speed, add emergency avoidance, or zero velocity
    Vec2 apply_safety_filter(
        const Vec2& desired_vel,
        const AgentState& self,
        const std::map<uint16_t, AgentState>& neighbors,
        const ArenaConfig& arena);

    /// Get current threat level
    ThreatLevel current_threat() const { return current_threat_; }

    /// Get recent safety events
    std::vector<SafetyEvent> recent_events(size_t max_count = 10) const;

    /// Should the agent perform emergency landing?
    bool should_emergency_land() const { return emergency_land_; }

    /// Should the agent return to launch?
    bool should_rtl() const { return rtl_requested_; }

    /// Clear RTL / emergency flags (for recovery)
    void clear_emergency() { emergency_land_ = false; rtl_requested_ = false; }

    /// Reset the comm-loss clock to now (call on first odometry so the 8s
    /// window starts from when position data is actually available, not from
    /// node construction time).
    void reset_comm_timer() { last_neighbor_heard_ = std::chrono::steady_clock::now(); }

private:
    SafetyConfig config_;
    ThreatLevel current_threat_ = ThreatLevel::NONE;
    bool emergency_land_ = false;
    bool rtl_requested_ = false;
    EmergencyCallback emergency_cb_;
    std::vector<SafetyEvent> event_log_;
    std::chrono::steady_clock::time_point last_neighbor_heard_;

    void log_event(ThreatLevel level, const std::string& source,
                   const std::string& desc);

    // Individual checks
    ThreatLevel check_collision_threat(
        const AgentState& self,
        const std::map<uint16_t, AgentState>& neighbors);

    ThreatLevel check_geofence(
        const AgentState& self,
        const ArenaConfig& arena);

    ThreatLevel check_communication(
        const CommunicationManager::CommHealth& health);

    ThreatLevel check_battery(const AgentState& self);
    ThreatLevel check_gps(const AgentState& self);

    /// Emergency avoidance: repulsive velocity away from closest threat
    Vec2 emergency_avoidance(const AgentState& self,
                              const std::map<uint16_t, AgentState>& neighbors) const;
};

}  // namespace swarm_flocking
