// Copyright (c) 2024 Swarm Robotics Lab. BSD-3-Clause License.
// types.hpp — Core data structures for the swarm flocking system
// Based on: Vásárhelyi et al., Sci. Robot. 3, eaat3536 (2018)

#pragma once

#include <cstdint>
#include <cmath>
#include <vector>
#include <chrono>
#include <string>

namespace swarm_flocking {

// ─── 2D/3D Vector ────────────────────────────────────────────────────────────

struct Vec2 {
    double x = 0.0;
    double y = 0.0;

    Vec2() = default;
    Vec2(double x_, double y_) : x(x_), y(y_) {}

    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(double s) const { return {x * s, y * s}; }
    Vec2 operator/(double s) const { return {x / s, y / s}; }
    Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
    Vec2& operator-=(const Vec2& o) { x -= o.x; y -= o.y; return *this; }
    Vec2& operator*=(double s) { x *= s; y *= s; return *this; }

    double norm() const { return std::sqrt(x * x + y * y); }
    double norm_sq() const { return x * x + y * y; }
    double dot(const Vec2& o) const { return x * o.x + y * o.y; }

    Vec2 normalized() const {
        double n = norm();
        if (n < 1e-12) return {0.0, 0.0};
        return {x / n, y / n};
    }

    /// Clamp magnitude to max_val, preserving direction
    Vec2 clamped(double max_val) const {
        double n = norm();
        if (n <= max_val || n < 1e-12) return *this;
        return normalized() * max_val;
    }
};

inline Vec2 operator*(double s, const Vec2& v) { return v * s; }

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vec3() = default;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vec2 xy() const { return {x, y}; }
    double norm() const { return std::sqrt(x * x + y * y + z * z); }
};

// ─── Agent State ─────────────────────────────────────────────────────────────

enum class AgentStatus : uint8_t {
    IDLE        = 0,
    TAKEOFF     = 1,
    FLOCKING    = 2,
    FORMATION   = 3,
    LANDING     = 4,
    EMERGENCY   = 5,
    RTL         = 6
};

struct AgentState {
    uint16_t id = 0;
    Vec2     pos;                           // local ENU, meters
    double   altitude = 0.0;               // meters AGL
    Vec2     vel;                           // m/s
    double   heading = 0.0;                // radians
    AgentStatus status = AgentStatus::IDLE;

    // Health
    float battery_voltage = 0.0f;
    float gps_accuracy    = 0.0f;          // meters
    uint8_t num_sats      = 0;
    bool  comm_healthy     = false;

    // Timing
    using Clock = std::chrono::steady_clock;
    Clock::time_point last_received;        // when we last heard from this agent
    double age_seconds() const {
        auto now = Clock::now();
        return std::chrono::duration<double>(now - last_received).count();
    }
};

// ─── Flocking Parameters (Table S1/S3/S4 from 2018 paper) ───────────────────

struct FlockingParams {
    // === Repulsion (Eq. 2-3) ===
    double r_rep_0 = 25.6;    // m   — repulsion range
    double p_rep   = 0.13;    // 1/s — repulsion gain (spring constant)

    // === Velocity alignment / friction (Eq. 4-7) ===
    double r_frict_0 = 85.3;  // m   — stopping point offset
    double C_frict   = 0.05;  // -   — alignment coefficient
    double v_frict   = 0.63;  // m/s — velocity slack
    double p_frict   = 3.20;  // 1/s — braking curve gain
    double a_frict   = 4.16;  // m/s² — braking curve acceleration

    // === Wall / shill agent (Eq. 8-9) ===
    double r_shill_0 = 0.3;   // m   — wall stopping offset
    double v_shill   = 13.6;  // m/s — shill agent velocity
    double p_shill   = 3.55;  // 1/s — wall braking gain
    double a_shill   = 3.02;  // m/s² — wall braking acceleration

    // === Self-propelling ===
    double v_flock = 4.0;     // m/s — desired flocking speed
    double v_max   = 6.0;     // m/s — maximum allowed speed

    // === Migration (global target bias) ===
    bool   has_migration_target = false;
    Vec2   migration_target     = {0.0, 0.0};  // NED local meters
    double migration_gain       = 0.4;          // fraction of v_flock [0..1]
    double migration_arrive_r   = 6.0;          // m — stop migrating within this radius (needs room to decelerate at 4 m/s)

    // === Environment ===
    double arena_size = 250.0; // m — square arena side length
    double comm_range = 80.0;  // m — max communication range

    // === Safety ===
    double r_coll   = 3.0;    // m — collision danger zone
    double r_safety = 5.0;    // m — extra safety margin for real flights

    // === Simulation / realism ===
    double comm_delay = 1.0;   // s — communication delay model
    double sensor_period = 0.2;// s — GPS refresh period
    double ctrl_tau = 1.0;     // s — velocity relaxation time constant

    /// Load preset for a given flocking speed (from Table S1 / S4)
    void load_preset(double speed);
};

// ─── Formation Parameters (from 2014 paper) ──────────────────────────────────

enum class FormationType : uint8_t {
    NONE = 0,
    GRID = 1,
    RING = 2,
    LINE = 3
};

struct FormationParams {
    FormationType type = FormationType::NONE;
    double spacing     = 10.0;    // m — desired inter-agent distance
    double rotation_speed = 0.0;  // m/s — tangential speed for rotating ring
    double alpha = 1.0;           // formation tracking gain
    double beta  = 1.0;           // COM tracking gain
    double v_max_tracking = 4.0;  // m/s — max tracking speed
};

// ─── Arena / Obstacle geometry ───────────────────────────────────────────────

struct ConvexPolygon {
    std::vector<Vec2> vertices;

    /// Closest point on the polygon boundary to point p
    Vec2 closest_point(const Vec2& p) const;

    /// Signed distance: negative inside, positive outside
    double signed_distance(const Vec2& p) const;

    /// Normal pointing inward at closest point
    Vec2 inward_normal(const Vec2& p) const;
};

struct ArenaConfig {
    ConvexPolygon boundary;              // the flight arena
    std::vector<ConvexPolygon> obstacles;
};

// ─── Diagnostics / Order Parameters ──────────────────────────────────────────

struct FlockDiagnostics {
    double phi_corr = 0.0;       // velocity correlation (Eq. 13)
    double phi_vel  = 0.0;       // average speed (Eq. 16)
    double avg_min_dist = 0.0;   // avg closest neighbor distance
    double min_min_dist = 0.0;   // minimum of closest neighbor distances
    double phi_lap  = 0.0;       // LAP rotational order (Eq. 1)
    uint32_t n_neighbors = 0;
    uint32_t collision_warnings = 0;
    double closest_wall = 0.0;
    double closest_obstacle = 0.0;
    Vec2 desired_velocity;
};

}  // namespace swarm_flocking
