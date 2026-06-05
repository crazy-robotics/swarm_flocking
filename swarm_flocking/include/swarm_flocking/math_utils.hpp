// Copyright (c) 2024 Swarm Robotics Lab. BSD-3-Clause License.
// math_utils.hpp — Mathematical primitives for the flocking model
// Implements: braking curve D(r,a,p) [Eq. 4], transfer functions [Eq. 17-20],
// polygon geometry, and noise generation.

#pragma once

#include "swarm_flocking/types.hpp"
#include <algorithm>
#include <cmath>
#include <random>
#include <limits>

namespace swarm_flocking {
namespace math {

// ─── Constants ───────────────────────────────────────────────────────────────

constexpr double kEpsilon = 1e-12;
constexpr double kPi      = 3.14159265358979323846;

// ─── Braking / Deceleration curve  (Eq. 4 from 2018 paper) ──────────────────
// Smooth velocity decay: constant-acceleration phase at high speed,
// exponential approach at low speed. Returns the maximum allowed speed at
// distance r from the stopping point.
//   r — distance to stopping point
//   a — preferred (maximum) deceleration
//   p — linear gain (sets crossover between exponential and sqrt phases)

inline double braking_curve(double r, double a, double p) {
    if (r <= 0.0) return 0.0;
    double rp = r * p;
    double threshold = a / p;
    if (rp < threshold) {
        return rp;                                  // linear / exponential phase
    }
    return std::sqrt(2.0 * a * r - (a * a) / (p * p));  // constant-accel phase
}

/// Inverse: given a target speed v, what distance is needed to brake?
inline double braking_distance(double v, double a, double p) {
    if (v <= 0.0) return 0.0;
    double threshold = a / p;
    if (v < threshold) {
        return v / p;                               // linear phase inverse
    }
    return (v * v + (a * a) / (p * p)) / (2.0 * a); // sqrt phase inverse
}

// ─── Transfer / fitness functions (Eq. 17-20) ────────────────────────────────

/// Sigmoid with smooth sinusoidal decay from (x0-d) to x0
inline double sigmoid_S(double x, double x0, double d) {
    if (x < x0 - d) return 1.0;
    if (x >= x0) return 0.0;
    return 0.5 * (1.0 - std::cos(kPi / d * (x - x0)));
}

/// F1: monotonically growing, converges to 1 (Eq. 17)
inline double transfer_F1(double f, double f0, double d) {
    return 1.0 - sigmoid_S(f, f0, d);
}

/// F2: Gaussian-like peak at f=0 (Eq. 19)
inline double transfer_F2(double f, double sigma) {
    return std::exp(-(f * f) / (sigma * sigma));
}

/// F3: sharp peak at f=0 (Eq. 20)
inline double transfer_F3(double f, double alpha) {
    return (alpha * alpha) / ((f + alpha) * (f + alpha));
}

// ─── Smooth transfer function from 2014 paper (Eq. 5) ───────────────────────
// Used for formation control: smooth 0→1 transition
//   x — input distance
//   R — center of transition
//   d — width of transition region

inline double smooth_step(double x, double R, double d) {
    if (x <= R - d) return 0.0;
    if (x >= R) return 1.0;
    return 0.5 * (1.0 + std::sin(kPi / d * (x - R + d / 2.0)));
}

// ─── Geometry utilities ──────────────────────────────────────────────────────

/// Closest point on line segment AB to point P
inline Vec2 closest_point_on_segment(const Vec2& A, const Vec2& B, const Vec2& P) {
    Vec2 AB = B - A;
    double len_sq = AB.norm_sq();
    if (len_sq < kEpsilon) return A;                // degenerate segment
    double t = std::clamp((P - A).dot(AB) / len_sq, 0.0, 1.0);
    return A + AB * t;
}

/// Signed distance from point to a half-plane defined by edge (A→B),
/// positive on the left side (inside for CCW polygon)
inline double signed_dist_to_edge(const Vec2& A, const Vec2& B, const Vec2& P) {
    Vec2 edge = B - A;
    // left normal
    Vec2 n(-edge.y, edge.x);
    double len = n.norm();
    if (len < kEpsilon) return 0.0;
    return (P - A).dot(n) / len;
}

/// Check if polygon vertices are in counter-clockwise order
inline bool is_ccw(const std::vector<Vec2>& verts) {
    double sum = 0.0;
    size_t n = verts.size();
    for (size_t i = 0; i < n; ++i) {
        const Vec2& a = verts[i];
        const Vec2& b = verts[(i + 1) % n];
        sum += (b.x - a.x) * (b.y + a.y);
    }
    return sum < 0.0;  // negative sum → CCW in standard math coords
}

/// Make a square arena polygon centered at origin
inline ConvexPolygon make_square_arena(double side_length) {
    double h = side_length / 2.0;
    ConvexPolygon poly;
    // CCW winding
    poly.vertices = {{-h, -h}, {h, -h}, {h, h}, {-h, h}};
    return poly;
}

// ─── Noise generation ────────────────────────────────────────────────────────

class NoiseGenerator {
public:
    explicit NoiseGenerator(unsigned seed = 42)
        : rng_(seed), normal_(0.0, 1.0) {}

    /// Gaussian noise vector with given standard deviation
    Vec2 gaussian_2d(double sigma) {
        return {sigma * normal_(rng_), sigma * normal_(rng_)};
    }

    double gaussian(double sigma) {
        return sigma * normal_(rng_);
    }

private:
    std::mt19937 rng_;
    std::normal_distribution<double> normal_;
};

// ─── GPS coordinate conversion ───────────────────────────────────────────────
// Converts a vehicle's local NED position (x=North, y=East in metres relative
// to ref_lat/ref_lon) into a common global NED frame relative to
// global_ref_lat/global_ref_lon.  Uses flat-earth approximation, accurate to
// <1 cm for inter-vehicle distances up to ~500 m.

inline Vec2 local_ned_to_global_ned(
    double x_north, double y_east,
    double ref_lat_deg, double ref_lon_deg,
    double global_ref_lat_deg, double global_ref_lon_deg)
{
    const double R = 6371000.0;
    const double d2r = kPi / 180.0;

    // Local NED → absolute geodetic (small-angle)
    double lat = ref_lat_deg + (x_north / R) * (180.0 / kPi);
    double lon = ref_lon_deg + (y_east / (R * std::cos(ref_lat_deg * d2r))) * (180.0 / kPi);

    // Absolute geodetic → global NED
    double gx = (lat - global_ref_lat_deg) * d2r * R;
    double gy = (lon - global_ref_lon_deg) * d2r * R * std::cos(global_ref_lat_deg * d2r);
    return {gx, gy};
}

// ─── Angle utilities ─────────────────────────────────────────────────────────

inline double wrap_angle(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a <= -kPi) a += 2.0 * kPi;
    return a;
}

inline double angle_of(const Vec2& v) {
    return std::atan2(v.y, v.x);
}

}  // namespace math
}  // namespace swarm_flocking
