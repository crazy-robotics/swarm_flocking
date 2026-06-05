// Copyright (c) 2024 Swarm Robotics Lab. BSD-3-Clause License.
// test_math_utils.cpp — GTest tests for braking curve, geometry, etc.

#include <gtest/gtest.h>
#include "swarm_flocking/math_utils.hpp"
#include "swarm_flocking/types.hpp"

using namespace swarm_flocking;
using namespace swarm_flocking::math;

// ─── Braking curve D(r, a, p) from Eq. 4 ────────────────────────────────────

TEST(BrakingCurve, ZeroAtOrigin) {
    EXPECT_DOUBLE_EQ(braking_curve(0.0, 5.0, 2.0), 0.0);
}

TEST(BrakingCurve, NegativeDistanceGivesZero) {
    EXPECT_DOUBLE_EQ(braking_curve(-1.0, 5.0, 2.0), 0.0);
}

TEST(BrakingCurve, LinearPhaseNearOrigin) {
    // When r*p < a/p, D(r) = r*p
    double a = 6.0, p = 3.0;
    double r = 0.5;  // r*p = 1.5, a/p = 2.0, so linear phase
    EXPECT_NEAR(braking_curve(r, a, p), r * p, 1e-10);
}

TEST(BrakingCurve, SqrtPhaseAtLargeDistance) {
    double a = 6.0, p = 3.0;
    double r = 10.0;  // well into sqrt phase
    double expected = std::sqrt(2.0 * a * r - (a * a) / (p * p));
    EXPECT_NEAR(braking_curve(r, a, p), expected, 1e-10);
}

TEST(BrakingCurve, MonotonicallyIncreasing) {
    double a = 4.0, p = 2.0;
    double prev = 0.0;
    for (double r = 0.1; r < 100.0; r += 0.5) {
        double val = braking_curve(r, a, p);
        EXPECT_GE(val, prev - 1e-10);
        prev = val;
    }
}

TEST(BrakingCurve, InverseIsConsistent) {
    double a = 5.0, p = 3.0;
    for (double r = 0.5; r < 50.0; r += 2.0) {
        double v = braking_curve(r, a, p);
        double r_inv = braking_distance(v, a, p);
        EXPECT_NEAR(r, r_inv, 1e-6);
    }
}

// ─── Transfer functions (Eq. 17-20) ──────────────────────────────────────────

TEST(TransferFunctions, F1ConvergesToOne) {
    EXPECT_NEAR(transfer_F1(100.0, 10.0, 5.0), 1.0, 1e-10);
}

TEST(TransferFunctions, F2PeakAtZero) {
    EXPECT_NEAR(transfer_F2(0.0, 1.0), 1.0, 1e-10);
    EXPECT_LT(transfer_F2(1.0, 1.0), 1.0);
}

TEST(TransferFunctions, F3PeakAtZero) {
    EXPECT_NEAR(transfer_F3(0.0, 1.0), 1.0, 1e-10);
    EXPECT_LT(transfer_F3(1.0, 1.0), 1.0);
}

// ─── Geometry ────────────────────────────────────────────────────────────────

TEST(Geometry, ClosestPointOnSegment) {
    Vec2 A{0, 0}, B{10, 0}, P{5, 3};
    Vec2 cp = closest_point_on_segment(A, B, P);
    EXPECT_NEAR(cp.x, 5.0, 1e-10);
    EXPECT_NEAR(cp.y, 0.0, 1e-10);
}

TEST(Geometry, ClosestPointOnSegmentEndpoint) {
    Vec2 A{0, 0}, B{10, 0}, P{15, 0};
    Vec2 cp = closest_point_on_segment(A, B, P);
    EXPECT_NEAR(cp.x, 10.0, 1e-10);
    EXPECT_NEAR(cp.y, 0.0, 1e-10);
}

TEST(Geometry, SquareArenaContainsCenter) {
    auto arena = make_square_arena(100.0);
    double sd = arena.signed_distance({0, 0});
    EXPECT_GT(sd, 0.0);  // inside
}

TEST(Geometry, SquareArenaExcludesOutside) {
    auto arena = make_square_arena(100.0);
    double sd = arena.signed_distance({60, 0});
    EXPECT_LT(sd, 0.0);  // outside
}

// ─── Vec2 operations ─────────────────────────────────────────────────────────

TEST(Vec2, Clamped) {
    Vec2 v{3.0, 4.0};  // norm = 5
    Vec2 c = v.clamped(3.0);
    EXPECT_NEAR(c.norm(), 3.0, 1e-10);
    // Direction preserved
    EXPECT_NEAR(std::atan2(c.y, c.x), std::atan2(v.y, v.x), 1e-10);
}

TEST(Vec2, NormalizedZero) {
    Vec2 v{0, 0};
    Vec2 n = v.normalized();
    EXPECT_NEAR(n.norm(), 0.0, 1e-10);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
