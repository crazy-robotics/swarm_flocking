// Copyright (c) 2024 Swarm Robotics Lab. BSD-3-Clause License.
// flocking_algorithm.cpp — Full implementation of the Vásárhelyi 2018 model

#include "swarm_flocking/flocking_algorithm.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace swarm_flocking {

using namespace math;

// ─── ConvexPolygon implementation ────────────────────────────────────────────

Vec2 ConvexPolygon::closest_point(const Vec2& p) const {
    if (vertices.size() < 2) return p;
    Vec2 best = vertices[0];
    double best_dist = (p - best).norm_sq();
    size_t n = vertices.size();
    for (size_t i = 0; i < n; ++i) {
        Vec2 cp = closest_point_on_segment(vertices[i], vertices[(i + 1) % n], p);
        double d = (p - cp).norm_sq();
        if (d < best_dist) {
            best_dist = d;
            best = cp;
        }
    }
    return best;
}

double ConvexPolygon::signed_distance(const Vec2& p) const {
    // For convex polygon: positive if inside, negative if outside
    // (using convention that vertices are CCW and we want inside = positive)
    if (vertices.size() < 3) return 0.0;
    double min_signed = std::numeric_limits<double>::max();
    size_t n = vertices.size();
    for (size_t i = 0; i < n; ++i) {
        double sd = signed_dist_to_edge(vertices[i], vertices[(i + 1) % n], p);
        min_signed = std::min(min_signed, sd);
    }
    return min_signed;  // positive inside CCW polygon
}

Vec2 ConvexPolygon::inward_normal(const Vec2& p) const {
    Vec2 cp = closest_point(p);
    Vec2 diff = p - cp;
    double d = diff.norm();
    if (d < kEpsilon) {
        // On boundary — use first edge normal as fallback
        if (vertices.size() >= 2) {
            Vec2 edge = vertices[1] - vertices[0];
            return Vec2(-edge.y, edge.x).normalized();
        }
        return {0.0, 0.0};
    }
    // If point is inside, inward points toward center; we want
    // the normal from boundary pointing inward, which is away from point
    // when point is outside.
    double sd = signed_distance(p);
    if (sd > 0) {
        // Inside: normal points from boundary toward center
        return (cp - p).normalized();
    }
    // Outside: normal points from boundary inward (away from p)
    return (p - cp).normalized() * (-1.0);
}

// ─── FlockingParams presets (Tables S1, S4) ──────────────────────────────────

void FlockingParams::load_preset(double speed) {
    v_flock = speed;

    // comm_range is NOT set here — it is a deployment parameter loaded from YAML/ROS params
    // and must not be overwritten by speed presets (it would silently undo the config).
    if (speed <= 4.0) {
        // Optimized for 4 m/s (Table S1 column 1, adjusted per Table S4)
        r_rep_0 = 15.0;   p_rep = 0.3;
        r_frict_0 = 45.0; C_frict = 0.5;  v_frict = 1.0;
        p_frict = 1.0;    a_frict = 3.0;
        r_shill_0 = 0.0;  v_shill = 10.0; p_shill = 2.0; a_shill = 2.5;
        v_max = 6.0;      arena_size = 200.0;
    } else if (speed <= 6.0) {
        // 6 m/s (Table S4)
        r_rep_0 = 35.0;   p_rep = 0.1;
        r_frict_0 = 65.0; C_frict = 0.5;  v_frict = 2.0;
        p_frict = 1.0;    a_frict = 3.0;
        r_shill_0 = 5.0;  v_shill = 14.0; p_shill = 2.0; a_shill = 2.5;
        v_max = 8.0;      arena_size = 260.0;
    } else if (speed <= 8.0) {
        // 8 m/s (Table S4)
        r_rep_0 = 50.0;   p_rep = 0.1;
        r_frict_0 = 80.0; C_frict = 0.5;  v_frict = 2.0;
        p_frict = 1.0;    a_frict = 3.0;
        r_shill_0 = 30.0; v_shill = 14.0; p_shill = 2.0; a_shill = 2.5;
        v_max = 10.0;     arena_size = 260.0;
    } else if (speed <= 16.0) {
        // 16 m/s (Table S1)
        r_rep_0 = 103.0;  p_rep = 0.03;
        r_frict_0 = 182.0; C_frict = 0.07; v_frict = 2.47;
        p_frict = 5.92;   a_frict = 2.57;
        r_shill_0 = 0.5;  v_shill = 49.2; p_shill = 9.49; a_shill = 9.50;
        v_max = 20.0;     arena_size = 500.0;
    } else {
        // 32 m/s (Table S1)
        r_rep_0 = 200.0;  p_rep = 0.03;
        r_frict_0 = 228.0; C_frict = 0.09; v_frict = 3.46;
        p_frict = 5.38;   a_frict = 0.70;
        r_shill_0 = 4.2;  v_shill = 76.4; p_shill = 4.87; a_shill = 9.99;
        v_max = 40.0;     arena_size = 1000.0;
    }
}

// ─── FlockingAlgorithm ───────────────────────────────────────────────────────

FlockingAlgorithm::FlockingAlgorithm() {
    params_.load_preset(4.0);
    arena_.boundary = make_square_arena(params_.arena_size);
}

void FlockingAlgorithm::set_params(const FlockingParams& params) {
    params_ = params;
}

void FlockingAlgorithm::set_arena(const ArenaConfig& arena) {
    std::lock_guard<std::mutex> lock(arena_mutex_);
    arena_ = arena;
}

// ─── Pairwise Repulsion (Eq. 2) ─────────────────────────────────────────────
// v_rep_ij = p_rep * (r_rep_0 - r_ij) * (r_i - r_j) / r_ij   if r_ij < r_rep_0

Vec2 FlockingAlgorithm::pairwise_repulsion(const Vec2& r_i, const Vec2& r_j) const {
    Vec2 diff = r_i - r_j;
    double r_ij = diff.norm();
    if (r_ij < kEpsilon || r_ij >= params_.r_rep_0) {
        return {0.0, 0.0};
    }
    double magnitude = params_.p_rep * (params_.r_rep_0 - r_ij);
    return diff.normalized() * magnitude;
}

// ─── Pairwise Velocity Alignment (Eq. 5-6) ──────────────────────────────────
// Uses braking curve D(.) to determine max allowed velocity difference.
// v_frict_max_ij = max(v_frict, D(r_ij - r_frict_0, a_frict, p_frict))
// v_frict_ij = C_frict * (v_ij - v_frict_max_ij) * (v_i - v_j)/v_ij
//              if v_ij > v_frict_max_ij, else 0

Vec2 FlockingAlgorithm::pairwise_alignment(
    const Vec2& r_i, const Vec2& r_j,
    const Vec2& v_i, const Vec2& v_j) const
{
    double r_ij = (r_i - r_j).norm();
    Vec2 dv = v_i - v_j;
    double v_ij = dv.norm();

    if (v_ij < kEpsilon) return {0.0, 0.0};

    // Maximum allowed velocity difference at this distance (Eq. 5)
    double D_val = braking_curve(r_ij - params_.r_frict_0,
                                  params_.a_frict, params_.p_frict);
    double v_frict_max = std::max(params_.v_frict, D_val);

    if (v_ij <= v_frict_max) return {0.0, 0.0};

    // Alignment force proportional to excess velocity difference (Eq. 6)
    double error = params_.C_frict * (v_ij - v_frict_max);
    return dv.normalized() * (-error);  // negative: reduces velocity difference
}

// ─── Wall Shill Interaction (Eq. 8-9) ────────────────────────────────────────
// For each arena wall edge, a virtual "shill" agent sits at the closest point
// heading inward with speed v_shill. The real agent aligns with this shill.

Vec2 FlockingAlgorithm::wall_shill_interaction(
    const Vec2& r_i, const Vec2& v_i,
    const Vec2& edge_a, const Vec2& edge_b,
    bool is_obstacle) const
{
    // Find shill agent position: closest point on edge to agent
    Vec2 r_s = closest_point_on_segment(edge_a, edge_b, r_i);
    double r_is = (r_i - r_s).norm();

    if (r_is < kEpsilon) r_is = kEpsilon;

    // Shill agent velocity: perpendicular to edge, pointing inward
    Vec2 edge_dir = (edge_b - edge_a);
    Vec2 inward;
    if (is_obstacle) {
        // Obstacle shills point outward from obstacle
        inward = Vec2(edge_dir.y, -edge_dir.x).normalized();
        // Check orientation: should point away from obstacle center
        // We flip if the shill velocity would point toward the agent from inside
    } else {
        // Arena wall shills point inward
        inward = Vec2(-edge_dir.y, edge_dir.x).normalized();
    }
    Vec2 v_s = inward * params_.v_shill;

    // Velocity difference
    Vec2 dv = v_i - v_s;
    double v_is = dv.norm();

    if (v_is < kEpsilon) return {0.0, 0.0};

    // Max allowed velocity diff from wall (Eq. 8)
    // No velocity slack for walls (C_shill = 1 implicitly)
    double v_shill_max = braking_curve(r_is - params_.r_shill_0,
                                        params_.a_shill, params_.p_shill);

    if (v_is <= v_shill_max) return {0.0, 0.0};

    // Wall alignment (Eq. 9): full error reduction (C=1)
    double error = v_is - v_shill_max;
    return dv.normalized() * (-error);
}

// ─── Obstacle Shill (single closest point variant) ───────────────────────────

Vec2 FlockingAlgorithm::obstacle_shill_interaction(
    const Vec2& r_i, const Vec2& v_i,
    const ConvexPolygon& obstacle) const
{
    Vec2 r_s = obstacle.closest_point(r_i);
    double r_is = (r_i - r_s).norm();
    if (r_is < kEpsilon) r_is = kEpsilon;

    // Shill velocity: pointing outward from obstacle (away from obstacle center)
    Vec2 outward = (r_i - r_s).normalized();
    Vec2 v_s = outward * params_.v_shill;

    Vec2 dv = v_i - v_s;
    double v_is = dv.norm();
    if (v_is < kEpsilon) return {0.0, 0.0};

    double v_shill_max = braking_curve(r_is - params_.r_shill_0,
                                        params_.a_shill, params_.p_shill);
    if (v_is <= v_shill_max) return {0.0, 0.0};

    double error = v_is - v_shill_max;
    return dv.normalized() * (-error);
}

// ─── Aggregate interaction terms ─────────────────────────────────────────────

Vec2 FlockingAlgorithm::compute_repulsion(
    const AgentState& self,
    const std::map<uint16_t, AgentState>& neighbors) const
{
    Vec2 v_rep{0.0, 0.0};
    for (const auto& [id, nb] : neighbors) {
        if (id == self.id) continue;
        double dist = (self.pos - nb.pos).norm();
        if (dist > params_.comm_range) continue;
        v_rep += pairwise_repulsion(self.pos, nb.pos);
    }
    return v_rep;
}

Vec2 FlockingAlgorithm::compute_alignment(
    const AgentState& self,
    const std::map<uint16_t, AgentState>& neighbors) const
{
    Vec2 v_frict{0.0, 0.0};
    for (const auto& [id, nb] : neighbors) {
        if (id == self.id) continue;
        double dist = (self.pos - nb.pos).norm();
        if (dist > params_.comm_range) continue;
        v_frict += pairwise_alignment(self.pos, nb.pos, self.vel, nb.vel);
    }
    return v_frict;
}

Vec2 FlockingAlgorithm::compute_wall_avoidance(const AgentState& self) const {
    std::lock_guard<std::mutex> lock(arena_mutex_);
    Vec2 v_wall{0.0, 0.0};
    const auto& verts = arena_.boundary.vertices;
    size_t n = verts.size();
    if (n < 2) return v_wall;
    // Each edge generates a separate shill agent (per paper)
    for (size_t i = 0; i < n; ++i) {
        v_wall += wall_shill_interaction(self.pos, self.vel,
                                          verts[i], verts[(i + 1) % n], false);
    }
    return v_wall;
}

Vec2 FlockingAlgorithm::compute_obstacle_avoidance(const AgentState& self) const {
    std::lock_guard<std::mutex> lock(arena_mutex_);
    Vec2 v_obs{0.0, 0.0};
    for (const auto& obstacle : arena_.obstacles) {
        v_obs += obstacle_shill_interaction(self.pos, self.vel, obstacle);
    }
    return v_obs;
}

Vec2 FlockingAlgorithm::compute_self_propulsion(const AgentState& self) const {
    double v_norm = self.vel.norm();
    if (v_norm < kEpsilon) {
        // No current velocity — use heading as direction
        return Vec2(std::cos(self.heading), std::sin(self.heading)) * params_.v_flock;
    }
    return self.vel.normalized() * params_.v_flock;
}

// ─── Migration toward global target (optional bias term) ─────────────────────
// v_mig = migration_gain * v_flock * (target - pos) / |target - pos|
// Adds a constant directional bias that steers the flock toward the target
// while leaving repulsion/alignment intact. Zeroes out within arrive_r of target.

Vec2 FlockingAlgorithm::compute_migration(const AgentState& self) const {
    if (!params_.has_migration_target) return {0.0, 0.0};
    Vec2 to_target = params_.migration_target - self.pos;
    double dist = to_target.norm();
    if (dist < params_.migration_arrive_r) return {0.0, 0.0};
    return to_target.normalized() * (params_.migration_gain * params_.v_flock);
}

// ─── Main desired velocity computation (Eq. 10-11) ──────────────────────────

Vec2 FlockingAlgorithm::compute_desired_velocity(
    const AgentState& self,
    const std::map<uint16_t, AgentState>& neighbors) const
{
    // Self-propelling term (Eq. 10, first term).
    // When a migration target is set, redirect self-propulsion toward the target
    // so the flock can steer even if it was previously heading in the wrong direction.
    // Without this, v_spp (4 m/s in old heading) dominates v_mig (1.6 m/s toward target)
    // and the flock cannot be steered once it has built up velocity.
    Vec2 v_spp;
    if (params_.has_migration_target) {
        Vec2 to_target = params_.migration_target - self.pos;
        double dist = to_target.norm();
        if (dist > params_.migration_arrive_r) {
            // Approaching target: drive self-propulsion toward it so the flock
            // steers correctly regardless of its previous heading.
            v_spp = to_target.normalized() * params_.v_flock;
        } else {
            // Arrived: zero self-propulsion so PX4 decelerates the vehicle
            // to a stop. Repulsion then spreads agents across the target area
            // instead of letting them overshoot and collide at a single point.
            v_spp = {0.0, 0.0};
        }
    } else {
        v_spp = compute_self_propulsion(self);
    }

    // Repulsion: Σ v_rep_ij  (Eq. 3)
    Vec2 v_rep = compute_repulsion(self, neighbors);

    // Velocity alignment: Σ v_frict_ij  (Eq. 7)
    Vec2 v_frict = compute_alignment(self, neighbors);

    // Wall avoidance: Σ v_wall_is  (Eq. 9 for arena walls)
    Vec2 v_wall = compute_wall_avoidance(self);

    // Obstacle avoidance: Σ v_obstacle_is
    Vec2 v_obs = compute_obstacle_avoidance(self);

    // Migration bias toward global target (optional)
    Vec2 v_mig = compute_migration(self);

    // Superposition (Eq. 10)
    Vec2 v_desired = v_spp + v_rep + v_frict + v_wall + v_obs + v_mig;

    // Clamp to v_max (Eq. 11)
    return v_desired.clamped(params_.v_max);
}

// ─── Diagnostics ─────────────────────────────────────────────────────────────

FlockDiagnostics FlockingAlgorithm::compute_diagnostics(
    const AgentState& self,
    const std::map<uint16_t, AgentState>& neighbors) const
{
    FlockDiagnostics diag;
    diag.desired_velocity = compute_desired_velocity(self, neighbors);
    diag.n_neighbors = 0;

    double sum_corr = 0.0;
    double sum_speed = self.vel.norm();
    double min_dist = std::numeric_limits<double>::max();
    double sum_min_dist = 0.0;
    uint32_t n_valid = 0;

    for (const auto& [id, nb] : neighbors) {
        if (id == self.id) continue;
        double dist = (self.pos - nb.pos).norm();
        if (dist > params_.comm_range) continue;

        diag.n_neighbors++;

        // Velocity correlation (per Eq. 13 inner sum)
        double vi_norm = self.vel.norm();
        double vj_norm = nb.vel.norm();
        if (vi_norm > kEpsilon && vj_norm > kEpsilon) {
            sum_corr += self.vel.dot(nb.vel) / (vi_norm * vj_norm);
            n_valid++;
        }

        sum_speed += nb.vel.norm();

        if (dist < min_dist) min_dist = dist;
        sum_min_dist += dist;  // simplified: not per-agent minimum, but ok for local diag

        // Collision check
        if (dist < params_.r_coll) {
            diag.collision_warnings++;
        }
    }

    if (n_valid > 0) {
        diag.phi_corr = sum_corr / static_cast<double>(n_valid);
    }
    diag.phi_vel = sum_speed / static_cast<double>(diag.n_neighbors + 1);
    diag.min_min_dist = (min_dist < 1e10) ? min_dist : 0.0;
    diag.avg_min_dist = (diag.n_neighbors > 0)
        ? sum_min_dist / static_cast<double>(diag.n_neighbors)
        : 0.0;

    // LAP parameter (Eq. 1 from paper): rotational order
    // phi_LAP = (vy*rx - vx*ry) / (v_flock * sqrt(rx^2 + ry^2))
    // Here we compute it relative to arena center (assumed at origin)
    double rx = self.pos.x;
    double ry = self.pos.y;
    double r_mag = std::sqrt(rx * rx + ry * ry);
    if (r_mag > kEpsilon && params_.v_flock > kEpsilon) {
        diag.phi_lap = (self.vel.y * rx - self.vel.x * ry)
                       / (params_.v_flock * r_mag);
    }

    // Closest wall distance
    {
        std::lock_guard<std::mutex> lock(arena_mutex_);
        Vec2 wall_cp = arena_.boundary.closest_point(self.pos);
        diag.closest_wall = (self.pos - wall_cp).norm();

        diag.closest_obstacle = std::numeric_limits<double>::max();
        for (const auto& obs : arena_.obstacles) {
            Vec2 obs_cp = obs.closest_point(self.pos);
            diag.closest_obstacle = std::min(diag.closest_obstacle,
                                              (self.pos - obs_cp).norm());
        }
        if (diag.closest_obstacle > 1e8) diag.closest_obstacle = -1.0;
    }

    return diag;
}

}  // namespace swarm_flocking
