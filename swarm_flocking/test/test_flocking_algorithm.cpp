// Copyright (c) 2024 Swarm Robotics Lab. BSD-3-Clause License.
// test_flocking_algorithm.cpp — GTest unit tests for the flocking model

#include <gtest/gtest.h>
#include "swarm_flocking/flocking_algorithm.hpp"
#include "swarm_flocking/math_utils.hpp"
#include <cmath>

using namespace swarm_flocking;

class FlockingTest : public ::testing::Test {
protected:
    void SetUp() override {
        params_.load_preset(4.0);
        algo_.set_params(params_);

        ArenaConfig arena;
        arena.boundary = math::make_square_arena(200.0);
        algo_.set_arena(arena);
    }

    FlockingParams params_;
    FlockingAlgorithm algo_;

    AgentState make_agent(uint16_t id, double x, double y,
                           double vx, double vy) {
        AgentState s;
        s.id = id;
        s.pos = {x, y};
        s.vel = {vx, vy};
        s.heading = std::atan2(vy, vx);
        s.last_received = AgentState::Clock::now();
        return s;
    }
};

// ─── Repulsion tests ─────────────────────────────────────────────────────────

TEST_F(FlockingTest, RepulsionZeroWhenFarApart) {
    AgentState self = make_agent(0, 0, 0, 2, 0);
    std::map<uint16_t, AgentState> neighbors;
    neighbors[1] = make_agent(1, 50, 0, 2, 0);  // 50m away > r_rep_0=15m

    Vec2 rep = algo_.compute_repulsion(self, neighbors);
    EXPECT_NEAR(rep.norm(), 0.0, 1e-10);
}

TEST_F(FlockingTest, RepulsionNonZeroWhenClose) {
    AgentState self = make_agent(0, 0, 0, 2, 0);
    std::map<uint16_t, AgentState> neighbors;
    neighbors[1] = make_agent(1, 10, 0, 2, 0);  // 10m < r_rep_0=15m

    Vec2 rep = algo_.compute_repulsion(self, neighbors);
    EXPECT_GT(rep.norm(), 0.0);
    // Should push self away from neighbor (negative x direction)
    EXPECT_LT(rep.x, 0.0);
}

TEST_F(FlockingTest, RepulsionIsSymmetric) {
    AgentState a = make_agent(0, 0, 0, 2, 0);
    AgentState b = make_agent(1, 10, 0, 2, 0);

    std::map<uint16_t, AgentState> nb_a, nb_b;
    nb_a[1] = b;
    nb_b[0] = a;

    Vec2 rep_a = algo_.compute_repulsion(a, nb_a);
    Vec2 rep_b = algo_.compute_repulsion(b, nb_b);

    // Opposite direction, same magnitude
    EXPECT_NEAR(rep_a.x + rep_b.x, 0.0, 1e-10);
    EXPECT_NEAR(rep_a.y + rep_b.y, 0.0, 1e-10);
}

// ─── Alignment tests ─────────────────────────────────────────────────────────

TEST_F(FlockingTest, AlignmentZeroWhenSameVelocity) {
    AgentState self = make_agent(0, 0, 0, 4, 0);
    std::map<uint16_t, AgentState> neighbors;
    neighbors[1] = make_agent(1, 20, 0, 4, 0);  // same velocity

    Vec2 align = algo_.compute_alignment(self, neighbors);
    EXPECT_NEAR(align.norm(), 0.0, 1e-10);
}

TEST_F(FlockingTest, AlignmentReducesVelocityDifference) {
    AgentState self = make_agent(0, 0, 0, 4, 0);
    std::map<uint16_t, AgentState> neighbors;
    neighbors[1] = make_agent(1, 10, 0, -4, 0);  // opposite velocity

    Vec2 align = algo_.compute_alignment(self, neighbors);
    // Alignment should push self toward neighbor's velocity (negative x)
    EXPECT_LT(align.x, 0.0);
}

// ─── Wall avoidance tests ────────────────────────────────────────────────────

TEST_F(FlockingTest, WallAvoidanceZeroAtCenter) {
    AgentState self = make_agent(0, 0, 0, 2, 0);
    Vec2 wall = algo_.compute_wall_avoidance(self);
    // At center of 200m arena, walls are 100m away — should be negligible
    EXPECT_NEAR(wall.norm(), 0.0, 0.1);
}

TEST_F(FlockingTest, WallAvoidanceNonZeroNearBoundary) {
    AgentState self = make_agent(0, 95, 0, 4, 0);  // 5m from right wall
    Vec2 wall = algo_.compute_wall_avoidance(self);
    // Should push away from wall (negative x)
    EXPECT_LT(wall.x, 0.0);
}

// ─── Full velocity computation ───────────────────────────────────────────────

TEST_F(FlockingTest, DesiredVelocityRespectsVmax) {
    AgentState self = make_agent(0, 0, 0, 4, 0);
    std::map<uint16_t, AgentState> neighbors;

    Vec2 v_d = algo_.compute_desired_velocity(self, neighbors);
    EXPECT_LE(v_d.norm(), params_.v_max + 1e-10);
}

TEST_F(FlockingTest, SelfPropulsionMaintainsDirection) {
    AgentState self = make_agent(0, 0, 0, 3, 1);
    Vec2 spp = algo_.compute_self_propulsion(self);

    double heading = std::atan2(self.vel.y, self.vel.x);
    double spp_heading = std::atan2(spp.y, spp.x);
    EXPECT_NEAR(heading, spp_heading, 1e-10);
    EXPECT_NEAR(spp.norm(), params_.v_flock, 1e-10);
}

// ─── Diagnostics ─────────────────────────────────────────────────────────────

TEST_F(FlockingTest, DiagnosticsReportCorrectNeighborCount) {
    AgentState self = make_agent(0, 0, 0, 4, 0);
    std::map<uint16_t, AgentState> neighbors;
    neighbors[1] = make_agent(1, 20, 0, 4, 0);
    neighbors[2] = make_agent(2, -20, 0, 4, 0);
    neighbors[3] = make_agent(3, 0, 20, 4, 0);

    auto diag = algo_.compute_diagnostics(self, neighbors);
    EXPECT_EQ(diag.n_neighbors, 3u);
}

TEST_F(FlockingTest, CorrelationIsOneForParallelVelocities) {
    AgentState self = make_agent(0, 0, 0, 4, 0);
    std::map<uint16_t, AgentState> neighbors;
    neighbors[1] = make_agent(1, 30, 0, 4, 0);
    neighbors[2] = make_agent(2, 0, 30, 4, 0);

    auto diag = algo_.compute_diagnostics(self, neighbors);
    EXPECT_NEAR(diag.phi_corr, 1.0, 1e-10);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
