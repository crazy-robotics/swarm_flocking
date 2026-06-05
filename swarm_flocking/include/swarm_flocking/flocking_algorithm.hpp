// Copyright (c) 2024 Swarm Robotics Lab. BSD-3-Clause License.
// flocking_algorithm.hpp — Core flocking model
// Implements: Repulsion (Eq 2-3), Velocity alignment with braking curve
// (Eq 4-7), Wall/obstacle shill agents (Eq 8-9), Self-propelling,
// and final desired velocity with v_max clamp (Eq 10-11).

#pragma once

#include "swarm_flocking/types.hpp"
#include "swarm_flocking/math_utils.hpp"
#include <map>
#include <mutex>

namespace swarm_flocking {

/// Computes the desired velocity for a single agent given the states of
/// all visible neighbors, the arena, and current parameters.
class FlockingAlgorithm {
public:
    FlockingAlgorithm();

    /// Set parameters (can be updated at runtime)
    void set_params(const FlockingParams& params);
    const FlockingParams& params() const { return params_; }

    /// Set arena geometry
    void set_arena(const ArenaConfig& arena);

    /// Main computation: returns desired velocity for agent `self`
    /// given the map of visible neighbor states.
    Vec2 compute_desired_velocity(
        const AgentState& self,
        const std::map<uint16_t, AgentState>& neighbors) const;

    /// Compute individual interaction terms (for diagnostics / debugging)
    Vec2 compute_repulsion(const AgentState& self,
                           const std::map<uint16_t, AgentState>& neighbors) const;

    Vec2 compute_alignment(const AgentState& self,
                           const std::map<uint16_t, AgentState>& neighbors) const;

    Vec2 compute_wall_avoidance(const AgentState& self) const;

    Vec2 compute_obstacle_avoidance(const AgentState& self) const;

    Vec2 compute_self_propulsion(const AgentState& self) const;

    Vec2 compute_migration(const AgentState& self) const;

    /// Order parameter computation for diagnostics (Eq 13, 16, 1)
    FlockDiagnostics compute_diagnostics(
        const AgentState& self,
        const std::map<uint16_t, AgentState>& neighbors) const;

private:
    FlockingParams params_;
    ArenaConfig arena_;
    mutable std::mutex arena_mutex_;

    /// Pairwise repulsion (Eq 2): v_rep_ij
    Vec2 pairwise_repulsion(const Vec2& r_i, const Vec2& r_j) const;

    /// Pairwise velocity alignment (Eq 5-6): v_frict_ij
    Vec2 pairwise_alignment(const Vec2& r_i, const Vec2& r_j,
                            const Vec2& v_i, const Vec2& v_j) const;

    /// Wall shill interaction for a single polygon edge (Eq 8-9)
    Vec2 wall_shill_interaction(const Vec2& r_i, const Vec2& v_i,
                                const Vec2& edge_a, const Vec2& edge_b,
                                bool is_obstacle = false) const;

    /// Obstacle shill interaction — single closest point (Eq 9 variant)
    Vec2 obstacle_shill_interaction(const Vec2& r_i, const Vec2& v_i,
                                    const ConvexPolygon& obstacle) const;
};

}  // namespace swarm_flocking
