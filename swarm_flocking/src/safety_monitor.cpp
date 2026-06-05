// Copyright (c) 2024 Swarm Robotics Lab. BSD-3-Clause License.
// safety_monitor.cpp

#include "swarm_flocking/safety_monitor.hpp"
#include "swarm_flocking/math_utils.hpp"
#include <algorithm>
#include <limits>

namespace swarm_flocking {

using namespace math;

SafetyMonitor::SafetyMonitor(const SafetyConfig& cfg)
    : config_(cfg), last_neighbor_heard_(std::chrono::steady_clock::now())
{}

void SafetyMonitor::log_event(ThreatLevel level, const std::string& source,
                               const std::string& desc) {
    SafetyEvent evt{level, source, desc, std::chrono::steady_clock::now()};
    event_log_.push_back(evt);
    // Keep log bounded
    if (event_log_.size() > 1000) {
        event_log_.erase(event_log_.begin(), event_log_.begin() + 500);
    }
    if (level >= ThreatLevel::CRITICAL && emergency_cb_) {
        emergency_cb_(evt);
    }
}

std::vector<SafetyEvent> SafetyMonitor::recent_events(size_t max_count) const {
    size_t start = (event_log_.size() > max_count)
        ? event_log_.size() - max_count : 0;
    return {event_log_.begin() + start, event_log_.end()};
}

// ─── Collision threat ────────────────────────────────────────────────────────

ThreatLevel SafetyMonitor::check_collision_threat(
    const AgentState& self,
    const std::map<uint16_t, AgentState>& neighbors)
{
    // Only check collisions when actively flying. In IDLE or EMERGENCY the
    // position frame may be inconsistent (each vehicle uses its own local NED
    // origin) and triggering PANIC blocks the transition to FLOCKING.
    if (self.status == AgentStatus::IDLE     ||
        self.status == AgentStatus::TAKEOFF  ||
        self.status == AgentStatus::EMERGENCY) return ThreatLevel::NONE;

    ThreatLevel worst = ThreatLevel::NONE;

    for (const auto& [id, nb] : neighbors) {
        double dist = (self.pos - nb.pos).norm();

        if (dist < config_.r_panic) {
            log_event(ThreatLevel::PANIC, "collision",
                      "PANIC: Agent " + std::to_string(id) +
                      " at " + std::to_string(dist) + "m");
            worst = ThreatLevel::PANIC;
        } else if (dist < config_.r_critical) {
            if (worst < ThreatLevel::CRITICAL) {
                log_event(ThreatLevel::CRITICAL, "collision",
                          "CRITICAL proximity: Agent " + std::to_string(id));
                worst = ThreatLevel::CRITICAL;
            }
        } else if (dist < config_.r_warning) {
            worst = std::max(worst, ThreatLevel::WARNING);
        } else if (dist < config_.r_caution) {
            worst = std::max(worst, ThreatLevel::CAUTION);
        }

        // Also check closure rate (time to collision estimate)
        Vec2 rel_pos = nb.pos - self.pos;
        Vec2 rel_vel = nb.vel - self.vel;
        double closing_speed = -rel_pos.dot(rel_vel) / std::max(dist, kEpsilon);
        if (closing_speed > 0.5 && dist < config_.r_warning * 2.0) {
            double ttc = dist / closing_speed;
            if (ttc < 2.0) {
                worst = std::max(worst, ThreatLevel::WARNING);
                log_event(ThreatLevel::WARNING, "collision",
                          "TTC=" + std::to_string(ttc) + "s with Agent " +
                          std::to_string(id));
            }
        }
    }
    return worst;
}

// ─── Geofence ────────────────────────────────────────────────────────────────

ThreatLevel SafetyMonitor::check_geofence(
    const AgentState& self,
    const ArenaConfig& arena)
{
    if (arena.boundary.vertices.empty()) return ThreatLevel::NONE;

    double signed_dist = arena.boundary.signed_distance(self.pos);
    // signed_dist > 0 means inside for CCW polygon

    if (signed_dist < -config_.geofence_hard_margin) {
        log_event(ThreatLevel::CRITICAL, "geofence",
                  "Outside hard geofence by " + std::to_string(-signed_dist) + "m");
        rtl_requested_ = true;
        return ThreatLevel::CRITICAL;
    } else if (signed_dist < 0) {
        log_event(ThreatLevel::WARNING, "geofence", "Outside arena boundary");
        return ThreatLevel::WARNING;
    } else if (signed_dist < config_.geofence_warn_margin) {
        return ThreatLevel::CAUTION;
    }

    // Altitude check
    if (self.altitude > config_.max_altitude) {
        log_event(ThreatLevel::WARNING, "geofence",
                  "Altitude " + std::to_string(self.altitude) + "m exceeds max");
        return ThreatLevel::WARNING;
    }
    if (self.altitude < config_.min_altitude && self.status == AgentStatus::FLOCKING) {
        return ThreatLevel::CAUTION;
    }

    return ThreatLevel::NONE;
}

// ─── Communication ───────────────────────────────────────────────────────────

ThreatLevel SafetyMonitor::check_communication(
    const CommunicationManager::CommHealth& health)
{
    if (health.active_count > 0) {
        last_neighbor_heard_ = std::chrono::steady_clock::now();
    }

    double since_any = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_neighbor_heard_).count();

    if (since_any > config_.all_comm_loss_s) {
        log_event(ThreatLevel::CRITICAL, "communication",
                  "All communication lost for " + std::to_string(since_any) + "s");
        emergency_land_ = true;
        return ThreatLevel::CRITICAL;
    }

    if (health.max_age_s > config_.comm_loss_critical) {
        return ThreatLevel::WARNING;
    }

    if (health.any_critical) {
        log_event(ThreatLevel::WARNING, "communication",
                  "Close neighbor with stale data");
        return ThreatLevel::WARNING;
    }

    if (health.max_age_s > config_.comm_loss_warn_s) {
        return ThreatLevel::CAUTION;
    }

    return ThreatLevel::NONE;
}

// ─── Battery ─────────────────────────────────────────────────────────────────

ThreatLevel SafetyMonitor::check_battery(const AgentState& self) {
    if (self.battery_voltage <= 0.01f) return ThreatLevel::NONE; // not reporting

    if (self.battery_voltage < config_.battery_panic_v) {
        log_event(ThreatLevel::PANIC, "battery",
                  "Battery critically low: " + std::to_string(self.battery_voltage) + "V");
        emergency_land_ = true;
        return ThreatLevel::PANIC;
    }
    if (self.battery_voltage < config_.battery_critical_v) {
        log_event(ThreatLevel::CRITICAL, "battery", "Battery critical");
        rtl_requested_ = true;
        return ThreatLevel::CRITICAL;
    }
    if (self.battery_voltage < config_.battery_warn_v) {
        return ThreatLevel::CAUTION;
    }
    return ThreatLevel::NONE;
}

// ─── GPS quality ─────────────────────────────────────────────────────────────

ThreatLevel SafetyMonitor::check_gps(const AgentState& self) {
    if (self.gps_accuracy > config_.gps_accuracy_crit) {
        log_event(ThreatLevel::WARNING, "gps",
                  "GPS accuracy degraded: " + std::to_string(self.gps_accuracy) + "m");
        return ThreatLevel::WARNING;
    }
    if (self.gps_accuracy > config_.gps_accuracy_warn) {
        return ThreatLevel::CAUTION;
    }
    if (self.num_sats < config_.min_satellites) {
        return ThreatLevel::CAUTION;
    }
    return ThreatLevel::NONE;
}

// ─── Emergency avoidance ─────────────────────────────────────────────────────

Vec2 SafetyMonitor::emergency_avoidance(
    const AgentState& self,
    const std::map<uint16_t, AgentState>& neighbors) const
{
    Vec2 escape{0.0, 0.0};
    for (const auto& [id, nb] : neighbors) {
        Vec2 diff = self.pos - nb.pos;
        double dist = diff.norm();
        if (dist < config_.r_critical && dist > kEpsilon) {
            // Strong repulsion inversely proportional to distance
            double force = config_.r_critical / dist;
            escape += diff.normalized() * force;
        }
    }
    return escape;
}

// ─── Main evaluation ─────────────────────────────────────────────────────────

ThreatLevel SafetyMonitor::evaluate(
    const AgentState& self,
    const std::map<uint16_t, AgentState>& neighbors,
    const ArenaConfig& arena,
    const CommunicationManager::CommHealth& comm_health)
{
    ThreatLevel worst = ThreatLevel::NONE;

    worst = std::max(worst, check_collision_threat(self, neighbors));
    worst = std::max(worst, check_geofence(self, arena));
    worst = std::max(worst, check_communication(comm_health));
    worst = std::max(worst, check_battery(self));
    worst = std::max(worst, check_gps(self));

    current_threat_ = worst;
    return worst;
}

// ─── Safety filter on desired velocity ───────────────────────────────────────

Vec2 SafetyMonitor::apply_safety_filter(
    const Vec2& desired_vel,
    const AgentState& self,
    const std::map<uint16_t, AgentState>& neighbors,
    const ArenaConfig& arena)
{
    if (emergency_land_) {
        return {0.0, 0.0};  // zero horizontal velocity, descend
    }

    Vec2 filtered = desired_vel;

    // Layer 1: absolute speed limit
    filtered = filtered.clamped(config_.max_speed_override);

    // Layer 2: emergency avoidance injection
    if (current_threat_ >= ThreatLevel::WARNING) {
        Vec2 escape = emergency_avoidance(self, neighbors);
        if (escape.norm() > kEpsilon) {
            // Blend: higher threat → more escape authority
            double blend = (current_threat_ >= ThreatLevel::CRITICAL) ? 0.8 : 0.4;
            filtered = filtered * (1.0 - blend) + escape.clamped(config_.max_speed_override) * blend;
        }
    }

    // Layer 3: speed reduction near threats
    if (current_threat_ >= ThreatLevel::CAUTION) {
        double factor = 1.0;
        switch (current_threat_) {
            case ThreatLevel::CAUTION:  factor = 0.8; break;
            case ThreatLevel::WARNING:  factor = 0.5; break;
            case ThreatLevel::CRITICAL: factor = 0.3; break;
            case ThreatLevel::PANIC:    factor = 0.0; break;
            default: break;
        }
        filtered *= factor;
    }

    // Layer 4: hard geofence clamp — push back if outside
    if (!arena.boundary.vertices.empty()) {
        double sd = arena.boundary.signed_distance(self.pos);
        if (sd < 0) {
            // Outside: override velocity to point inward
            Vec2 cp = arena.boundary.closest_point(self.pos);
            Vec2 to_inside = (cp - self.pos).normalized();
            filtered = to_inside * std::min(filtered.norm(), config_.max_speed_override * 0.5);
        }
    }

    return filtered;
}

}  // namespace swarm_flocking
