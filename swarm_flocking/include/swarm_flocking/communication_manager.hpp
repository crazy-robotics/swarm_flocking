// Copyright (c) 2024 Swarm Robotics Lab. BSD-3-Clause License.
// communication_manager.hpp — DDS communication layer for swarm
// Handles: broadcasting agent state, receiving neighbor states,
// tracking communication health/outages, linear extrapolation of
// stale positions (per 2018 paper Section: Drone setup).

#pragma once

#include "swarm_flocking/types.hpp"
#include <map>
#include <mutex>
#include <functional>
#include <chrono>

namespace swarm_flocking {

/// Configuration for the communication layer
struct CommConfig {
    double broadcast_rate_hz = 10.0;   // status packet rate
    double max_comm_range    = 80.0;   // meters — packets beyond this are dropped
    double stale_timeout_s   = 3.0;    // seconds before a neighbor is declared lost
    double extrapolation_max = 2.0;    // max seconds to extrapolate position
    bool   enable_extrapolation = true;
};

/// Manages the neighbor state table with timeout tracking and position
/// extrapolation to compensate for communication outages (Fig. 8 from paper).
class CommunicationManager {
public:
    using Clock = std::chrono::steady_clock;

    explicit CommunicationManager(uint16_t own_id, const CommConfig& cfg = {});

    /// Update own state (to be broadcast)
    void update_own_state(const AgentState& state);

    /// Process a received neighbor state packet
    void receive_neighbor_state(const AgentState& state);

    /// Get current own state
    AgentState own_state() const;

    /// Get all active (non-stale) neighbors, with extrapolated positions
    std::map<uint16_t, AgentState> get_active_neighbors() const;

    /// Get raw neighbor table (including stale entries)
    std::map<uint16_t, AgentState> get_all_neighbors() const;

    /// Check if a specific neighbor is alive
    bool is_neighbor_alive(uint16_t id) const;

    /// Number of active neighbors
    size_t active_neighbor_count() const;

    /// Prune stale neighbors from the table
    void prune_stale();

    /// Communication health metrics
    struct CommHealth {
        size_t active_count = 0;
        size_t stale_count = 0;
        double avg_age_s = 0.0;
        double max_age_s = 0.0;
        bool any_critical = false;  // any neighbor dangerously close AND stale
    };
    CommHealth get_health() const;

    void set_config(const CommConfig& cfg) { config_ = cfg; }

private:
    uint16_t own_id_;
    CommConfig config_;
    AgentState own_state_;
    std::map<uint16_t, AgentState> neighbors_;
    mutable std::mutex mutex_;

    /// Apply linear extrapolation to compensate for communication delay
    AgentState extrapolate(const AgentState& state) const;
};

}  // namespace swarm_flocking
