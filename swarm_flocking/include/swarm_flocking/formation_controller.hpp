// Copyright (c) 2024 Swarm Robotics Lab. BSD-3-Clause License.
// formation_controller.hpp — Formation flight control (grid, ring, line)
// Based on: Vásárhelyi et al., IROS 2014 (Eq. 6-8)

#pragma once

#include "swarm_flocking/types.hpp"
#include "swarm_flocking/math_utils.hpp"
#include <map>

namespace swarm_flocking {

/// Computes formation-specific velocity for target tracking with shapes.
/// Replaces the self-propelling + wall terms when in formation mode.
class FormationController {
public:
    FormationController();

    void set_params(const FormationParams& params) { params_ = params; }
    void set_total_agents(uint16_t n) { total_agents_ = n; }
    void set_target(const Vec2& target) { target_pos_ = target; }

    /// Compute the formation velocity component for agent `self`.
    /// Returns: v_target + v_formation (Eq. 6 from 2014 paper)
    Vec2 compute_formation_velocity(
        const AgentState& self,
        const std::map<uint16_t, AgentState>& neighbors) const;

    /// Get the desired position in formation for agent with given rank
    Vec2 get_formation_position(uint16_t agent_id,
                                 const Vec2& center_of_mass,
                                 const std::map<uint16_t, AgentState>& all_agents) const;

private:
    FormationParams params_;
    Vec2 target_pos_;
    uint16_t total_agents_ = 1;

    // Formation-specific position calculators
    Vec2 grid_position(uint16_t rank, uint16_t n, const Vec2& com) const;
    Vec2 ring_position(uint16_t agent_id, const Vec2& com,
                        const std::map<uint16_t, AgentState>& agents) const;
    Vec2 line_position(uint16_t agent_id, const Vec2& com,
                        const std::map<uint16_t, AgentState>& agents) const;

    /// Compute center of mass from visible neighbors
    Vec2 local_center_of_mass(const AgentState& self,
                               const std::map<uint16_t, AgentState>& neighbors) const;

    /// Rank agents by angle around COM (for ring) or projection (for line)
    uint16_t compute_rank(uint16_t agent_id,
                           const Vec2& com,
                           const std::map<uint16_t, AgentState>& agents) const;
};

}  // namespace swarm_flocking
