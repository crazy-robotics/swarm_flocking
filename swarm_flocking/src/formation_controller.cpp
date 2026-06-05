// Copyright (c) 2024 Swarm Robotics Lab. BSD-3-Clause License.
// formation_controller.cpp — Grid, ring, line formation control

#include "swarm_flocking/formation_controller.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace swarm_flocking {

using namespace math;

FormationController::FormationController() = default;

Vec2 FormationController::local_center_of_mass(
    const AgentState& self,
    const std::map<uint16_t, AgentState>& neighbors) const
{
    Vec2 com = self.pos;
    double count = 1.0;
    for (const auto& [id, nb] : neighbors) {
        if (id == self.id) continue;
        com += nb.pos;
        count += 1.0;
    }
    return com / count;
}

uint16_t FormationController::compute_rank(
    uint16_t agent_id,
    const Vec2& com,
    const std::map<uint16_t, AgentState>& agents) const
{
    // Sort agents by angle around COM
    std::vector<std::pair<double, uint16_t>> angles;
    for (const auto& [id, st] : agents) {
        Vec2 diff = st.pos - com;
        double angle = std::atan2(diff.y, diff.x);
        angles.push_back({angle, id});
    }
    std::sort(angles.begin(), angles.end());

    for (uint16_t i = 0; i < angles.size(); ++i) {
        if (angles[i].second == agent_id) return i;
    }
    return 0;
}

// ─── Grid formation ──────────────────────────────────────────────────────────
// Agents pack into a tight grid around COM. The pairwise repulsion and
// alignment handle the actual grid arrangement; we just define the target
// radius. (2014 paper: R_grid = g(N) where g is smallest enclosing circle)

Vec2 FormationController::grid_position(
    uint16_t /*rank*/, uint16_t n, const Vec2& com) const
{
    // For grid: target position IS the COM. Agents self-organize via repulsion.
    // R is set so all agents fit: R ≈ spacing * sqrt(N) / 2
    (void)n;
    return com;
}

// ─── Ring formation ──────────────────────────────────────────────────────────
// Each agent targets a point on a circle around COM, positioned between
// its two angular neighbors (bisectrix method from 2014 paper).

Vec2 FormationController::ring_position(
    uint16_t agent_id,
    const Vec2& com,
    const std::map<uint16_t, AgentState>& agents) const
{
    uint16_t n = static_cast<uint16_t>(agents.size());
    if (n == 0) return com;

    // Ring radius: fit N agents with desired spacing
    double circumference = n * params_.spacing;
    double radius = circumference / (2.0 * kPi);

    // Agent's angular position: evenly spaced by rank
    uint16_t rank = compute_rank(agent_id, com, agents);
    double angle = 2.0 * kPi * rank / n;

    Vec2 target;
    target.x = com.x + radius * std::cos(angle);
    target.y = com.y + radius * std::sin(angle);
    return target;
}

// ─── Line formation ──────────────────────────────────────────────────────────
// Agents arrange on a line through COM. Each targets the midpoint between
// its two projected neighbors (2014 paper method).

Vec2 FormationController::line_position(
    uint16_t agent_id,
    const Vec2& com,
    const std::map<uint16_t, AgentState>& agents) const
{
    uint16_t n = static_cast<uint16_t>(agents.size());
    if (n == 0) return com;

    // Determine line direction from PCA of positions (self-organized)
    Vec2 mean = {0, 0};
    for (const auto& [id, st] : agents) mean += st.pos;
    mean = mean / static_cast<double>(n);

    // Covariance for principal direction
    double cov_xx = 0, cov_xy = 0, cov_yy = 0;
    for (const auto& [id, st] : agents) {
        Vec2 d = st.pos - mean;
        cov_xx += d.x * d.x;
        cov_xy += d.x * d.y;
        cov_yy += d.y * d.y;
    }

    // Principal eigenvector (largest eigenvalue)
    double trace = cov_xx + cov_yy;
    double det = cov_xx * cov_yy - cov_xy * cov_xy;
    double lambda1 = trace / 2.0 + std::sqrt(std::max(0.0, trace * trace / 4.0 - det));

    Vec2 line_dir;
    if (std::abs(cov_xy) > kEpsilon) {
        line_dir = Vec2(lambda1 - cov_yy, cov_xy).normalized();
    } else {
        line_dir = (cov_xx >= cov_yy) ? Vec2(1, 0) : Vec2(0, 1);
    }

    // Project agents onto line and sort
    std::vector<std::pair<double, uint16_t>> projections;
    for (const auto& [id, st] : agents) {
        double proj = (st.pos - com).dot(line_dir);
        projections.push_back({proj, id});
    }
    std::sort(projections.begin(), projections.end());

    // Find this agent's rank on the line
    int rank = -1;
    for (int i = 0; i < static_cast<int>(projections.size()); ++i) {
        if (projections[i].second == agent_id) { rank = i; break; }
    }
    if (rank < 0) rank = 0;

    // Target: evenly spaced along line centered at COM
    double half_length = (n - 1) * params_.spacing / 2.0;
    double offset = -half_length + rank * params_.spacing;
    return com + line_dir * offset;
}

// ─── Main formation velocity (Eq. 6-8 from 2014 paper) ──────────────────────

Vec2 FormationController::get_formation_position(
    uint16_t agent_id,
    const Vec2& com,
    const std::map<uint16_t, AgentState>& all_agents) const
{
    switch (params_.type) {
        case FormationType::GRID:
            return grid_position(0, total_agents_, com);
        case FormationType::RING:
            return ring_position(agent_id, com, all_agents);
        case FormationType::LINE:
            return line_position(agent_id, com, all_agents);
        default:
            return com;
    }
}

Vec2 FormationController::compute_formation_velocity(
    const AgentState& self,
    const std::map<uint16_t, AgentState>& neighbors) const
{
    if (params_.type == FormationType::NONE) return {0.0, 0.0};

    // All agents including self
    std::map<uint16_t, AgentState> all_agents = neighbors;
    all_agents[self.id] = self;

    // Local center of mass
    Vec2 com = local_center_of_mass(self, neighbors);

    // Desired position in formation (Eq. 7 from 2014 paper)
    Vec2 formation_target = get_formation_position(self.id, com, all_agents);

    // === v_trg: velocity toward formation position ===
    Vec2 to_target = formation_target - self.pos;
    double dist_to_target = to_target.norm();
    double f_trg = smooth_step(dist_to_target, params_.spacing, params_.spacing);
    Vec2 v_trg = (dist_to_target > kEpsilon)
        ? to_target.normalized() * (params_.alpha * params_.v_max_tracking * f_trg)
        : Vec2{0.0, 0.0};

    // === v_com: velocity to make COM follow the global target ===
    Vec2 to_global = target_pos_ - com;
    double dist_to_global = to_global.norm();
    double f_com = smooth_step(dist_to_global, params_.spacing * 2.0,
                                params_.spacing * 2.0);
    Vec2 v_com = (dist_to_global > kEpsilon)
        ? to_global.normalized() * (params_.beta * params_.v_max_tracking * f_com)
        : Vec2{0.0, 0.0};

    Vec2 result = v_trg + v_com;

    // Add tangential rotation for ring
    if (params_.type == FormationType::RING && std::abs(params_.rotation_speed) > kEpsilon) {
        Vec2 radial = self.pos - com;
        double r = radial.norm();
        if (r > kEpsilon) {
            Vec2 tangential(-radial.y, radial.x);
            tangential = tangential.normalized();
            result += tangential * params_.rotation_speed;
        }
    }

    return result.clamped(params_.v_max_tracking);
}

}  // namespace swarm_flocking
