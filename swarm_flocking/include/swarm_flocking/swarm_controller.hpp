// Copyright (c) 2024 Swarm Robotics Lab. BSD-3-Clause License.
// swarm_controller.hpp — Main ROS2 node orchestrating flocking, formation,
// communication, and safety subsystems.
//
// Architecture:
//   [PX4 VehicleOdometry] → own_state → CommunicationManager ← [DDS neighbors]
//                    ↓
//              FlockingAlgorithm / FormationController
//                    ↓
//              SafetyMonitor (filter)
//                    ↓
//              [TrajectorySetpoint + OffboardControlMode → PX4 FMU]

#pragma once
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// PX4 interface messages
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include "swarm_flocking/types.hpp"
#include "swarm_flocking/flocking_algorithm.hpp"
#include "swarm_flocking/formation_controller.hpp"
#include "swarm_flocking/communication_manager.hpp"
#include "swarm_flocking/safety_monitor.hpp"

// Forward-declare generated messages
#include "swarm_flocking/msg/agent_state.hpp"
#include "swarm_flocking/msg/swarm_command.hpp"
#include "swarm_flocking/msg/flocking_diagnostics.hpp"
#include "swarm_flocking/msg/arena_config.hpp"

namespace swarm_flocking {

class SwarmController : public rclcpp::Node {
public:
    explicit SwarmController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~SwarmController() override = default;

private:
    // ─── Parameters ──────────────────────────────────────────────────────
    uint16_t agent_id_;
    double control_rate_hz_;
    double broadcast_rate_hz_;

    // ─── Subsystems ──────────────────────────────────────────────────────
    FlockingAlgorithm flocking_;
    FormationController formation_;
    std::unique_ptr<CommunicationManager> comm_;
    SafetyMonitor safety_;

    FlockingParams flock_params_;
    FormationParams form_params_;
    ArenaConfig arena_;

    // Current operating mode
    AgentStatus mode_ = AgentStatus::IDLE;

    // Guard: do not broadcast state or run safety checks until PX4 has sent
    // at least one valid odometry message.  Without this guard every agent
    // broadcasts position (0,0,0) immediately on startup, all agents appear
    // to be co-located, and the collision safety monitor trips PANIC within
    // 200 ms — long before the operator can switch to Offboard mode.
    bool odometry_received_ = false;

    // ─── ROS2 Communication ──────────────────────────────────────────────

    // Target flight altitude (meters AGL, +up). All agents converge to this height
    // so the 2D flocking plane is level. Loaded from flight_altitude_m param.
    double flight_altitude_m_ = 5.0;

    // Global NED reference (common across all agents — set via ROS2 param)
    // Default: PX4 SITL GPS home (lat=47.397742, lon=8.545594)
    double global_ref_lat_ = 47.397742;
    double global_ref_lon_ = 8.545594;
    bool   global_ref_set_ = false;   // true once first VehicleLocalPosition arrives

    // Subscribers
    // PX4 VehicleOdometry: BEST_EFFORT QoS — velocity + attitude
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    // PX4 VehicleLocalPosition: BEST_EFFORT QoS — global-frame position via ref_lat/ref_lon
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
    rclcpp::Subscription<msg::AgentState>::SharedPtr neighbor_sub_;
    rclcpp::Subscription<msg::SwarmCommand>::SharedPtr command_sub_;
    rclcpp::Subscription<msg::ArenaConfig>::SharedPtr arena_sub_;

    // Publishers — PX4 offboard control (BEST_EFFORT QoS to match DDS bridge input)
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

    // Swarm / diagnostics publishers
    rclcpp::Publisher<msg::AgentState>::SharedPtr state_pub_;
    rclcpp::Publisher<msg::FlockingDiagnostics>::SharedPtr diag_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_pub_;

    // Auto-takeoff: arm → offboard → climb to flight_altitude_m_ automatically
    bool auto_takeoff_ = false;

    // Timers
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr broadcast_timer_;
    rclcpp::TimerBase::SharedPtr safety_timer_;
    rclcpp::TimerBase::SharedPtr prune_timer_;
    rclcpp::TimerBase::SharedPtr offboard_keepalive_timer_;
    rclcpp::TimerBase::SharedPtr offboard_mode_timer_;
    rclcpp::TimerBase::SharedPtr arm_timer_;        // one-shot: sends ARM 1 s after first odometry
    rclcpp::TimerBase::SharedPtr offboard_arm_timer_; // one-shot: sends offboard 3 s after arm

    // ─── Callbacks ───────────────────────────────────────────────────────

    void on_odometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
    void on_local_position(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void on_neighbor_state(const msg::AgentState::SharedPtr msg);
    void on_swarm_command(const msg::SwarmCommand::SharedPtr msg);
    void on_arena_config(const msg::ArenaConfig::SharedPtr msg);

    void control_loop();            // main 20 Hz control
    void broadcast_state();         // 10 Hz state broadcast
    void safety_check();            // 5 Hz safety evaluation
    void prune_neighbors();         // 1 Hz stale neighbor cleanup
    void offboard_keepalive();      // 10 Hz PX4 offboard mode heartbeat

    // ─── Helpers ─────────────────────────────────────────────────────────

    void declare_parameters();
    void load_parameters();

    void send_offboard_mode_command();  // send DO_SET_MODE → Offboard
    void send_arm_command();            // send COMPONENT_ARM_DISARM (arm=1, force)

    // vel is in the internal 2D frame (North=x, East=y); altitude_rate in m/s down (NED)
    void publish_velocity_command(const Vec2& vel, double altitude_rate = 0.0);
    void publish_diagnostics(const FlockDiagnostics& diag);
    void publish_visualization();
    void transition_to(AgentStatus new_mode);
    void handle_emergency(const SafetyEvent& event);

    /// Build PX4 topic prefix from agent_id_:
    ///   agent 1 → ""            (no prefix, bare /fmu/...)
    ///   agent N → "/px4_{N-1}"  (e.g. "/px4_1" for agent 2)
    std::string px4_topic_prefix() const;

    /// Convert between ROS and internal types
    static AgentState from_ros(const msg::AgentState& ros_msg);
    static msg::AgentState to_ros(const AgentState& state);
};

}  // namespace swarm_flocking
