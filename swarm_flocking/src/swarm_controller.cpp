// Copyright (c) 2024 Swarm Robotics Lab. BSD-3-Clause License.
// swarm_controller.cpp — Full ROS2 node implementation
//
// PX4 Interface:
//   Odometry input:  /fmu/out/vehicle_odometry         (agent 1)
//                    /px4_{N-1}/fmu/out/vehicle_odometry (agent N)
//   Velocity output: /fmu/in/trajectory_setpoint        (agent 1)
//                    /px4_{N-1}/fmu/in/trajectory_setpoint (agent N)
//   Offboard mode:   /fmu/in/offboard_control_mode      (agent 1)
//                    /px4_{N-1}/fmu/in/offboard_control_mode (agent N)
//
// All PX4 DDS topics require BEST_EFFORT QoS (not the ROS2 default RELIABLE).
// VehicleOdometry.position is NED float32[3]; velocity is NED float32[3].
// TrajectorySetpoint.velocity is NED float32[3]; position must be NaN for
// pure velocity control.

#include "swarm_flocking/swarm_controller.hpp"
#include "swarm_flocking/math_utils.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace swarm_flocking {

// ─── Constructor ─────────────────────────────────────────────────────────────

SwarmController::SwarmController(const rclcpp::NodeOptions& options)
    : Node("swarm_controller", options),
      comm_(nullptr)
{
    declare_parameters();
    load_parameters();

    // Initialize subsystems
    flocking_.set_params(flock_params_);
    formation_.set_params(form_params_);
    safety_.on_emergency(std::bind(&SwarmController::handle_emergency, this, _1));

    // Setup arena
    arena_.boundary = math::make_square_arena(flock_params_.arena_size);
    flocking_.set_arena(arena_);

    // ─── QoS profiles ────────────────────────────────────────────────────
    // PX4 DDS bridge publishes /fmu/out/* with BEST_EFFORT + VOLATILE.
    // Subscribers MUST use BEST_EFFORT; RELIABLE will never match and
    // the subscriber will receive nothing.
    auto qos_px4_sub = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .durability_volatile();

    // PX4 DDS bridge expects /fmu/in/* with BEST_EFFORT + VOLATILE.
    // Publishers MUST use BEST_EFFORT; RELIABLE causes QoS mismatch warnings
    // and PX4 may not process the setpoints.
    auto qos_px4_pub = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .durability_volatile();

    // Best-effort for high-rate swarm state broadcasts (mimics XBee/WiFi UDP)
    auto qos_swarm = rclcpp::QoS(rclcpp::KeepLast(10))
        .best_effort()
        .durability_volatile();

    // Commands: RELIABLE + TRANSIENT_LOCAL so agents that start after the command
    // is published (or reconnect briefly) still receive the last command.
    // The ros2 topic pub command MUST include --qos-durability transient_local to match.
    auto qos_command = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliable()
        .transient_local();

    // Reliable VOLATILE for diagnostics / safety events (no late-join replay needed)
    auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliable()
        .durability_volatile();

    // ─── PX4 topic prefix (depends on agent_id_) ─────────────────────────
    // agent 1  →  ""            →  /fmu/out/vehicle_odometry
    // agent 2  →  "/px4_1"     →  /px4_1/fmu/out/vehicle_odometry
    // agent N  →  "/px4_{N-1}" →  /px4_{N-1}/fmu/out/vehicle_odometry
    std::string prefix = px4_topic_prefix();

    std::string odom_topic      = prefix + "/fmu/out/vehicle_odometry";
    std::string local_pos_topic = prefix + "/fmu/out/vehicle_local_position_v1";
    std::string ocm_topic       = prefix + "/fmu/in/offboard_control_mode";
    std::string traj_topic      = prefix + "/fmu/in/trajectory_setpoint";
    std::string vcmd_topic      = prefix + "/fmu/in/vehicle_command";

    // Read optional global reference override from params (default: PX4 SITL home)
    global_ref_lat_ = this->declare_parameter("global_ref_lat", 47.397742);
    global_ref_lon_ = this->declare_parameter("global_ref_lon", 8.545594);

    // ─── Subscribers ─────────────────────────────────────────────────────

    // VehicleOdometry — velocity + attitude (BEST_EFFORT)
    odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        odom_topic, qos_px4_sub,
        std::bind(&SwarmController::on_odometry, this, _1));

    // VehicleLocalPosition — provides x,y in local NED + ref_lat/ref_lon for
    // converting to a common global NED frame shared across all agents.
    local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        local_pos_topic, qos_px4_sub,
        std::bind(&SwarmController::on_local_position, this, _1));

    // All agents publish/subscribe to the same swarm topics (DDS multicast)
    neighbor_sub_ = this->create_subscription<msg::AgentState>(
        "/swarm/agent_states", qos_swarm,
        std::bind(&SwarmController::on_neighbor_state, this, _1));

    command_sub_ = this->create_subscription<msg::SwarmCommand>(
        "/swarm/commands", qos_command,
        std::bind(&SwarmController::on_swarm_command, this, _1));

    arena_sub_ = this->create_subscription<msg::ArenaConfig>(
        "/swarm/arena", qos_command,
        std::bind(&SwarmController::on_arena_config, this, _1));

    // ─── Publishers ──────────────────────────────────────────────────────

    // PX4 offboard control mode keepalive — BEST_EFFORT required
    offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        ocm_topic, qos_px4_pub);

    // PX4 trajectory setpoint (velocity control) — BEST_EFFORT required
    trajectory_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        traj_topic, qos_px4_pub);

    // PX4 vehicle command (mode switch, arm/disarm) — BEST_EFFORT required
    vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        vcmd_topic, qos_px4_pub);

    // Swarm state broadcast
    state_pub_ = this->create_publisher<msg::AgentState>(
        "/swarm/agent_states", qos_swarm);

    diag_pub_ = this->create_publisher<msg::FlockingDiagnostics>(
        "flocking_diagnostics", qos_reliable);

    viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/swarm/visualization", qos_reliable);

    safety_pub_ = this->create_publisher<std_msgs::msg::String>(
        "safety_events", qos_reliable);

    // ─── Timers ──────────────────────────────────────────────────────────

    // Main control loop
    auto ctrl_period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    control_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(ctrl_period),
        std::bind(&SwarmController::control_loop, this));

    // Swarm state broadcast
    auto bcast_period = std::chrono::duration<double>(1.0 / broadcast_rate_hz_);
    broadcast_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(bcast_period),
        std::bind(&SwarmController::broadcast_state, this));

    // Safety evaluation
    safety_timer_ = this->create_wall_timer(
        200ms, std::bind(&SwarmController::safety_check, this));

    // Stale neighbor pruning
    prune_timer_ = this->create_wall_timer(
        1s, std::bind(&SwarmController::prune_neighbors, this));

    // PX4 offboard keepalive: must fire at >2 Hz or PX4 exits offboard mode.
    // We use 10 Hz for a comfortable margin. This publishes OffboardControlMode
    // continuously — PX4 requires this even when not actively sending setpoints.
    offboard_keepalive_timer_ = this->create_wall_timer(
        100ms, std::bind(&SwarmController::offboard_keepalive, this));

    // One-shot mode switch: after 1.5 s PX4 has seen enough OffboardControlMode
    // + TrajectorySetpoint messages to accept the DO_SET_MODE command.
    // The timer cancels itself inside the callback after firing once.
    offboard_mode_timer_ = this->create_wall_timer(
        1500ms, std::bind(&SwarmController::send_offboard_mode_command, this));

    RCLCPP_INFO(get_logger(),
        "SwarmController initialized: agent_id=%d, px4_prefix='%s', "
        "odom='%s', v_flock=%.1f m/s, arena=%.0fm, control=%.0fHz",
        agent_id_, prefix.c_str(), odom_topic.c_str(),
        flock_params_.v_flock, flock_params_.arena_size, control_rate_hz_);
}

// ─── PX4 topic prefix ────────────────────────────────────────────────────────
// agent_id_ is 1-indexed (set by launch file parameter).
// PX4 SITL multi-vehicle instances use:
//   instance 0 (agent 1): no prefix  → topics at /fmu/...
//   instance 1 (agent 2): /px4_1     → topics at /px4_1/fmu/...
//   instance N (agent N+1): /px4_N

std::string SwarmController::px4_topic_prefix() const {
    if (agent_id_ <= 1) {
        return "";  // first vehicle — no namespace prefix
    }
    return "/px4_" + std::to_string(agent_id_ - 1);
}

// ─── Parameter declaration and loading ───────────────────────────────────────

void SwarmController::declare_parameters() {
    this->declare_parameter("agent_id", 1);
    this->declare_parameter("control_rate_hz", 20.0);
    this->declare_parameter("broadcast_rate_hz", 10.0);

    // Flocking params
    this->declare_parameter("flock.v_flock", 4.0);
    this->declare_parameter("flock.v_max", 6.0);
    this->declare_parameter("flock.arena_size", 200.0);
    this->declare_parameter("flock.comm_range", 500.0);
    this->declare_parameter("flock.r_rep_0", 5.0);
    this->declare_parameter("flock.p_rep", 0.5);
    this->declare_parameter("flock.r_frict_0", 15.0);
    this->declare_parameter("flock.C_frict", 0.05);
    this->declare_parameter("flock.v_frict", 1.5);
    this->declare_parameter("flock.p_frict", 1.0);
    this->declare_parameter("flock.a_frict", 3.0);
    this->declare_parameter("flock.r_shill_0", 0.0);
    this->declare_parameter("flock.v_shill", 10.0);
    this->declare_parameter("flock.p_shill", 2.0);
    this->declare_parameter("flock.a_shill", 2.5);

    // Formation params
    this->declare_parameter("formation.type", 0);
    this->declare_parameter("formation.spacing", 10.0);
    this->declare_parameter("formation.rotation_speed", 0.0);

    // Safety params
    this->declare_parameter("safety.r_caution", 4.0);
    this->declare_parameter("safety.r_warning", 3.0);
    this->declare_parameter("safety.r_critical", 2.0);
    this->declare_parameter("safety.r_panic", 1.0);
    this->declare_parameter("safety.max_speed_override", 12.0);

    // Flight altitude for offboard velocity control (meters AGL, positive up)
    this->declare_parameter("flight_altitude_m", 5.0);

    // When true: automatically arm, switch to offboard, and climb to flight_altitude_m
    // on startup. Set false for real flights where arming must be done manually via QGC.
    this->declare_parameter("auto_takeoff", false);
}

void SwarmController::load_parameters() {
    agent_id_ = static_cast<uint16_t>(this->get_parameter("agent_id").as_int());
    control_rate_hz_ = this->get_parameter("control_rate_hz").as_double();
    broadcast_rate_hz_ = this->get_parameter("broadcast_rate_hz").as_double();

    // Load flocking params — first apply preset, then override individual values
    double v_flock = this->get_parameter("flock.v_flock").as_double();
    flock_params_.load_preset(v_flock);

    flock_params_.v_max      = this->get_parameter("flock.v_max").as_double();
    flock_params_.arena_size = this->get_parameter("flock.arena_size").as_double();
    flock_params_.comm_range = this->get_parameter("flock.comm_range").as_double();
    flock_params_.r_rep_0    = this->get_parameter("flock.r_rep_0").as_double();
    flock_params_.p_rep      = this->get_parameter("flock.p_rep").as_double();
    flock_params_.r_frict_0  = this->get_parameter("flock.r_frict_0").as_double();
    flock_params_.C_frict    = this->get_parameter("flock.C_frict").as_double();
    flock_params_.v_frict    = this->get_parameter("flock.v_frict").as_double();
    flock_params_.p_frict    = this->get_parameter("flock.p_frict").as_double();
    flock_params_.a_frict    = this->get_parameter("flock.a_frict").as_double();
    flock_params_.r_shill_0  = this->get_parameter("flock.r_shill_0").as_double();
    flock_params_.v_shill    = this->get_parameter("flock.v_shill").as_double();
    flock_params_.p_shill    = this->get_parameter("flock.p_shill").as_double();
    flock_params_.a_shill    = this->get_parameter("flock.a_shill").as_double();

    // Formation
    form_params_.type = static_cast<FormationType>(
        this->get_parameter("formation.type").as_int());
    form_params_.spacing = this->get_parameter("formation.spacing").as_double();
    form_params_.rotation_speed = this->get_parameter("formation.rotation_speed").as_double();

    // Reinitialize comm manager with correct ID
    CommConfig cc;
    cc.max_comm_range = flock_params_.comm_range;
    comm_ = std::make_unique<CommunicationManager>(agent_id_, cc);

    // Safety
    SafetyConfig sc;
    sc.r_caution  = this->get_parameter("safety.r_caution").as_double();
    sc.r_warning  = this->get_parameter("safety.r_warning").as_double();
    sc.r_critical = this->get_parameter("safety.r_critical").as_double();
    sc.r_panic    = this->get_parameter("safety.r_panic").as_double();
    sc.max_speed_override = this->get_parameter("safety.max_speed_override").as_double();
    safety_.set_config(sc);

    flight_altitude_m_ = this->get_parameter("flight_altitude_m").as_double();
    auto_takeoff_      = this->get_parameter("auto_takeoff").as_bool();
}

// ─── PX4 VehicleOdometry callback ────────────────────────────────────────────
// Position and velocity are in NED frame (North=+X, East=+Y, Down=+Z).
// The internal AgentState uses the same NED convention for pos/vel so that
// the flocking algorithm operates in the same frame as the PX4 output.
// Heading is extracted from the NED quaternion (q[0]=w, q[1]=x, q[2]=y, q[3]=z)
// as yaw (rotation around Down axis), measured CCW from North in radians.

void SwarmController::on_odometry(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    // Position comes from on_local_position (global NED frame).
    // Here we only update velocity and heading, preserving the global position
    // already set by on_local_position.
    AgentState state = comm_->own_state();
    state.id = agent_id_;

    // NED velocity: vNorth=vel[0], vEast=vel[1]
    state.vel.x = static_cast<double>(msg->velocity[0]);
    state.vel.y = static_cast<double>(msg->velocity[1]);

    // Yaw from quaternion: q[0]=w, q[1]=x, q[2]=y, q[3]=z
    float qw = msg->q[0], qx = msg->q[1], qy = msg->q[2], qz = msg->q[3];
    state.heading = std::atan2(2.0f * (qw * qz + qx * qy),
                               1.0f - 2.0f * (qy * qy + qz * qz));

    state.status = mode_;
    comm_->update_own_state(state);
}

// ─── VehicleLocalPosition callback — global NED position ─────────────────────
// Each PX4 instance has its own local NED origin (its home position).
// VehicleLocalPosition gives (x, y) in that local frame + the GPS reference
// (ref_lat, ref_lon) that defines the origin.  We convert to a common global
// NED frame using global_ref_lat_ / global_ref_lon_ so that inter-vehicle
// distances are computed in the same coordinate system.

void SwarmController::on_local_position(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
    if (!msg->xy_valid || !msg->xy_global) return;

    Vec2 global_pos = math::local_ned_to_global_ned(
        static_cast<double>(msg->x),
        static_cast<double>(msg->y),
        msg->ref_lat, msg->ref_lon,
        global_ref_lat_, global_ref_lon_);

    // Update only the position component of own_state; velocity/heading come
    // from on_odometry which runs at the same rate.
    AgentState state = comm_->own_state();
    state.pos = global_pos;
    // altitude: negate Down (z is negative-up in NED)
    state.altitude = -static_cast<double>(msg->z);
    comm_->update_own_state(state);

    if (!odometry_received_) {
        odometry_received_ = true;
        safety_.reset_comm_timer();
        RCLCPP_INFO(get_logger(),
            "First local position received — global NED ref: (%.6f, %.6f)",
            global_ref_lat_, global_ref_lon_);

        if (auto_takeoff_) {
            // Cancel the blind 1.5 s offboard timer — we will send DO_SET_MODE
            // after arming instead, so we know the vehicle is ready to fly.
            offboard_mode_timer_->cancel();

            transition_to(AgentStatus::TAKEOFF);
            RCLCPP_INFO(get_logger(), "Auto-takeoff: arming in 1 s…");

            // Step 1: arm after 1 s (keepalive has been running since t=0,
            //         giving PX4 enough OffboardControlMode messages to accept
            //         the mode switch that follows).
            arm_timer_ = this->create_wall_timer(1000ms, [this]() {
                arm_timer_->cancel();
                send_arm_command();
                RCLCPP_INFO(get_logger(), "Auto-takeoff: switching offboard in 3 s…");

                // Step 2: switch to offboard 3 s after arm so PX4 is fully armed
                offboard_arm_timer_ = this->create_wall_timer(3000ms, [this]() {
                    offboard_arm_timer_->cancel();
                    send_offboard_mode_command();
                    // offboard_mode_timer_ is cancelled inside send_offboard_mode_command
                    RCLCPP_INFO(get_logger(),
                        "Auto-takeoff: offboard active — climbing to %.1f m AGL",
                        flight_altitude_m_);
                });
            });
        }
    }
}

// ─── Neighbor state received via DDS broadcast ───────────────────────────────

void SwarmController::on_neighbor_state(const msg::AgentState::SharedPtr ros_msg) {
    AgentState state = from_ros(*ros_msg);
    comm_->receive_neighbor_state(state);
}

// ─── Command handler ─────────────────────────────────────────────────────────

void SwarmController::on_swarm_command(const msg::SwarmCommand::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "Received swarm command: %d", msg->command);

    switch (msg->command) {
        case msg::SwarmCommand::CMD_START_FLOCKING:
            safety_.clear_emergency();
            transition_to(AgentStatus::FLOCKING);
            if (msg->flock_speed > 0.1) {
                // Only update speed — do NOT call load_preset() which would wipe
                // all yaml-tuned physics params (r_rep_0, r_frict_0, comm_range…).
                flock_params_.v_flock = msg->flock_speed;
                flock_params_.v_max   = msg->flock_speed * 1.5;
            }
            {
                // Optional migration target: non-zero target_position sets a global
                // direction bias. x=North, y=East (NED local meters from origin).
                double tx = msg->target_position.x;
                double ty = msg->target_position.y;
                if (std::abs(tx) > 0.1 || std::abs(ty) > 0.1) {
                    flock_params_.has_migration_target = true;
                    flock_params_.migration_target     = {tx, ty};
                    RCLCPP_INFO(get_logger(),
                        "Migration target set: North=%.1f m, East=%.1f m", tx, ty);
                } else {
                    flock_params_.has_migration_target = false;
                }
            }
            flocking_.set_params(flock_params_);
            break;

        case msg::SwarmCommand::CMD_STOP:
            flock_params_.has_migration_target = false;
            flocking_.set_params(flock_params_);
            transition_to(AgentStatus::IDLE);
            break;

        case msg::SwarmCommand::CMD_FORMATION_GRID:
            form_params_.type = FormationType::GRID;
            form_params_.spacing = (msg->formation_spacing > 0)
                ? msg->formation_spacing : form_params_.spacing;
            formation_.set_params(form_params_);
            formation_.set_target({msg->target_position.x, msg->target_position.y});
            transition_to(AgentStatus::FORMATION);
            break;

        case msg::SwarmCommand::CMD_FORMATION_RING:
            form_params_.type = FormationType::RING;
            form_params_.spacing = (msg->formation_spacing > 0)
                ? msg->formation_spacing : form_params_.spacing;
            formation_.set_params(form_params_);
            formation_.set_target({msg->target_position.x, msg->target_position.y});
            transition_to(AgentStatus::FORMATION);
            break;

        case msg::SwarmCommand::CMD_FORMATION_LINE:
            form_params_.type = FormationType::LINE;
            form_params_.spacing = (msg->formation_spacing > 0)
                ? msg->formation_spacing : form_params_.spacing;
            formation_.set_params(form_params_);
            formation_.set_target({msg->target_position.x, msg->target_position.y});
            transition_to(AgentStatus::FORMATION);
            break;

        case msg::SwarmCommand::CMD_LAND_ALL:
            transition_to(AgentStatus::LANDING);
            break;

        case msg::SwarmCommand::CMD_EMERGENCY_STOP:
            transition_to(AgentStatus::EMERGENCY);
            break;

        case msg::SwarmCommand::CMD_RTL:
            transition_to(AgentStatus::RTL);
            break;

        case msg::SwarmCommand::CMD_SET_SPEED:
            flock_params_.v_flock = msg->flock_speed;
            flock_params_.load_preset(msg->flock_speed);
            flocking_.set_params(flock_params_);
            break;

        case msg::SwarmCommand::CMD_SET_ARENA:
            flock_params_.arena_size = msg->arena_size;
            arena_.boundary = math::make_square_arena(msg->arena_size);
            flocking_.set_arena(arena_);
            break;
    }
}

// ─── Arena configuration ─────────────────────────────────────────────────────

void SwarmController::on_arena_config(const msg::ArenaConfig::SharedPtr msg) {
    ArenaConfig new_arena;

    for (const auto& pt : msg->arena_vertices) {
        new_arena.boundary.vertices.push_back({pt.x, pt.y});
    }

    size_t idx = 0;
    for (size_t i = 0; i < msg->obstacle_counts.size(); ++i) {
        ConvexPolygon obs;
        for (uint32_t j = 0; j < msg->obstacle_counts[i] && idx < msg->obstacle_vertices.size(); ++j) {
            obs.vertices.push_back({msg->obstacle_vertices[idx].x,
                                    msg->obstacle_vertices[idx].y});
            idx++;
        }
        new_arena.obstacles.push_back(obs);
    }

    arena_ = new_arena;
    flocking_.set_arena(arena_);
    RCLCPP_INFO(get_logger(), "Arena updated: %zu boundary verts, %zu obstacles",
                arena_.boundary.vertices.size(), arena_.obstacles.size());
}

// ─── Main Control Loop (20 Hz) ───────────────────────────────────────────────

void SwarmController::control_loop() {
    if (!odometry_received_) {
        publish_velocity_command({0.0, 0.0}, 0.0);
        return;
    }

    AgentState self = comm_->own_state();

    // Altitude P-controller: converge to flight_altitude_m_ at ±1.5 m/s.
    // Active in all flying modes so the 2D flock operates on a level plane.
    double alt_error = flight_altitude_m_ - self.altitude;
    double alt_rate  = std::clamp(0.5 * alt_error, -1.5, 1.5);

    // ── TAKEOFF: climb until within 0.3 m of target altitude ────────────
    if (mode_ == AgentStatus::TAKEOFF) {
        if (self.altitude >= flight_altitude_m_ - 0.3) {
            RCLCPP_INFO(get_logger(),
                "Agent %d takeoff complete — holding at %.1f m AGL",
                agent_id_, self.altitude);
            transition_to(AgentStatus::IDLE);
        }
        // Horizontal velocity zero; altitude P-controller climbs the vehicle
        publish_velocity_command({0.0, 0.0}, alt_rate);
        return;
    }

    // ── Non-flying modes: publish zero so PX4 keeps the offboard stream ─
    if (mode_ == AgentStatus::IDLE ||
        mode_ == AgentStatus::EMERGENCY ||
        mode_ == AgentStatus::LANDING) {
        publish_velocity_command({0.0, 0.0}, 0.0);
        return;
    }

    auto neighbors = comm_->get_active_neighbors();
    Vec2 desired_vel{0.0, 0.0};

    if (mode_ == AgentStatus::FLOCKING) {
        // Full Vásárhelyi 2018 flocking model
        desired_vel = flocking_.compute_desired_velocity(self, neighbors);

    } else if (mode_ == AgentStatus::FORMATION) {
        Vec2 v_form  = formation_.compute_formation_velocity(self, neighbors);
        Vec2 v_rep   = flocking_.compute_repulsion(self, neighbors);
        Vec2 v_align = flocking_.compute_alignment(self, neighbors);
        desired_vel = v_form + v_rep + v_align;
        desired_vel = desired_vel.clamped(flock_params_.v_max);

    } else if (mode_ == AgentStatus::RTL) {
        Vec2 to_home = Vec2{0.0, 0.0} - self.pos;
        double dist = to_home.norm();
        if (dist > 2.0) {
            desired_vel = to_home.normalized() * std::min(flock_params_.v_flock, dist);
        }
        desired_vel += flocking_.compute_repulsion(self, neighbors);
        desired_vel = desired_vel.clamped(flock_params_.v_max);
    }

    // Apply safety filter (may modify or zero the velocity)
    desired_vel = safety_.apply_safety_filter(desired_vel, self, neighbors, arena_);

    publish_velocity_command(desired_vel, alt_rate);

    // Publish diagnostics at reduced rate (every 5th cycle → 4 Hz)
    static int diag_counter = 0;
    if (++diag_counter >= 5) {
        diag_counter = 0;
        auto diag = flocking_.compute_diagnostics(self, neighbors);
        diag.desired_velocity = desired_vel;
        publish_diagnostics(diag);
    }
}

// ─── State broadcast (10 Hz) ─────────────────────────────────────────────────

void SwarmController::broadcast_state() {
    // Do not broadcast until PX4 has confirmed our real position.
    // Broadcasting (0,0,0) before first odometry makes every peer agent
    // see us as co-located and triggers a spurious PANIC collision event.
    if (!odometry_received_) return;

    AgentState self = comm_->own_state();
    self.status = mode_;
    auto ros_msg = to_ros(self);
    state_pub_->publish(ros_msg);
}

// ─── Safety check (5 Hz) ─────────────────────────────────────────────────────

void SwarmController::safety_check() {
    // Skip until our own position is known from real odometry.
    if (!odometry_received_) return;

    AgentState self = comm_->own_state();
    auto neighbors = comm_->get_active_neighbors();
    auto health = comm_->get_health();

    ThreatLevel threat = safety_.evaluate(self, neighbors, arena_, health);

    if (safety_.should_emergency_land()) {
        transition_to(AgentStatus::EMERGENCY);
    } else if (safety_.should_rtl() && mode_ != AgentStatus::RTL) {
        transition_to(AgentStatus::RTL);
    }

    if (threat >= ThreatLevel::WARNING) {
        auto events = safety_.recent_events(1);
        if (!events.empty()) {
            std_msgs::msg::String safety_msg;
            safety_msg.data = "[" + events.back().source + "] " +
                              events.back().description;
            safety_pub_->publish(safety_msg);
            RCLCPP_WARN(get_logger(), "Safety: %s", safety_msg.data.c_str());
        }
    }
}

// ─── Stale neighbor cleanup (1 Hz) ──────────────────────────────────────────

void SwarmController::prune_neighbors() {
    comm_->prune_stale();
    formation_.set_total_agents(
        static_cast<uint16_t>(comm_->active_neighbor_count() + 1));
}

// ─── PX4 Offboard Keepalive (10 Hz) ─────────────────────────────────────────
// PX4 requires OffboardControlMode to be published at >2 Hz continuously.
// If it stops, PX4 will exit offboard mode within 500 ms.
// velocity=true means we are sending velocity setpoints (TrajectorySetpoint).
// All other control fields must be false.

void SwarmController::offboard_keepalive() {
    px4_msgs::msg::OffboardControlMode ocm;
    ocm.timestamp   = this->get_clock()->now().nanoseconds() / 1000;  // microseconds
    ocm.position    = false;
    ocm.velocity    = true;   // velocity control mode
    ocm.acceleration = false;
    ocm.attitude    = false;
    ocm.body_rate   = false;
    offboard_mode_pub_->publish(ocm);
}

// ─── Mode transitions ────────────────────────────────────────────────────────

void SwarmController::transition_to(AgentStatus new_mode) {
    if (mode_ == new_mode) return;
    RCLCPP_INFO(get_logger(), "Mode transition: %d -> %d",
                static_cast<int>(mode_), static_cast<int>(new_mode));
    mode_ = new_mode;
}

void SwarmController::handle_emergency(const SafetyEvent& event) {
    RCLCPP_ERROR(get_logger(), "EMERGENCY [%s]: %s",
                 event.source.c_str(), event.description.c_str());
    if (event.level >= ThreatLevel::PANIC) {
        transition_to(AgentStatus::EMERGENCY);
    }
}

// ─── ARM command ─────────────────────────────────────────────────────────────
// VEHICLE_CMD_COMPONENT_ARM_DISARM (400): param1=1 → arm, param2=21196 → force
// arm bypass (skips pre-arm checks in SITL; safe for simulation only).

void SwarmController::send_arm_command() {
    px4_msgs::msg::VehicleCommand cmd;
    cmd.timestamp        = this->get_clock()->now().nanoseconds() / 1000;
    cmd.command          = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    cmd.param1           = 1.0f;       // 1 = arm
    cmd.param2           = 21196.0f;   // force arm — bypass pre-arm checks (SITL only)
    cmd.target_system    = static_cast<uint8_t>(agent_id_);
    cmd.target_component = 1;
    cmd.source_system    = 1;
    cmd.source_component = 1;
    cmd.confirmation     = 0;
    cmd.from_external    = true;
    vehicle_command_pub_->publish(cmd);
    RCLCPP_INFO(get_logger(), "Sent ARM command (agent %d)", agent_id_);
}

// ─── Offboard mode switch (one-shot, 1.5 s after startup) ────────────────────
// Sends VEHICLE_CMD_DO_SET_MODE (176) with:
//   param1 = 1.0  (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
//   param2 = 6.0  (PX4_CUSTOM_MAIN_MODE_OFFBOARD)
// target_system = agent_id_ (1-indexed, matching PX4 SITL instance MAVLink ID)
// This is a one-shot: the timer is cancelled immediately after sending.

void SwarmController::send_offboard_mode_command() {
    // Cancel the timer immediately — this must fire exactly once.
    offboard_mode_timer_->cancel();

    px4_msgs::msg::VehicleCommand vcmd;
    vcmd.timestamp        = this->get_clock()->now().nanoseconds() / 1000;  // microseconds
    vcmd.command          = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
    vcmd.param1           = 1.0f;   // base mode: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    vcmd.param2           = 6.0f;   // custom main mode: OFFBOARD (PX4_CUSTOM_MAIN_MODE_OFFBOARD)
    vcmd.param3           = 0.0f;
    vcmd.param4           = 0.0f;
    vcmd.param5           = 0.0;
    vcmd.param6           = 0.0;
    vcmd.param7           = 0.0f;
    vcmd.target_system    = static_cast<uint8_t>(agent_id_);   // 1 for vehicle 1, 2 for vehicle 2, …
    vcmd.target_component = 1;
    vcmd.source_system    = 1;
    vcmd.source_component = 1;
    vcmd.confirmation     = 0;
    vcmd.from_external    = true;

    vehicle_command_pub_->publish(vcmd);

    RCLCPP_INFO(get_logger(),
        "Sent DO_SET_MODE → Offboard on topic '%s' (target_system=%d)",
        vehicle_command_pub_->get_topic_name(), agent_id_);
}

// ─── Publishing helpers ──────────────────────────────────────────────────────
// vel is in the NED frame (North=x, East=y) matching the AgentState internal
// frame, which was populated directly from VehicleOdometry.position (NED).
// altitude_rate: positive = ascending (converted to NED Down = negative).

void SwarmController::publish_velocity_command(const Vec2& vel, double altitude_rate) {
    px4_msgs::msg::TrajectorySetpoint sp;
    sp.timestamp = this->get_clock()->now().nanoseconds() / 1000;  // microseconds

    // NED velocity: North=vel.x, East=vel.y, Down=-altitude_rate
    sp.velocity[0] = static_cast<float>(vel.x);          // North m/s
    sp.velocity[1] = static_cast<float>(vel.y);          // East  m/s
    sp.velocity[2] = static_cast<float>(-altitude_rate); // Down  m/s (negative = up)

    // Position must be NaN for pure velocity control
    sp.position[0] = std::numeric_limits<float>::quiet_NaN();
    sp.position[1] = std::numeric_limits<float>::quiet_NaN();
    sp.position[2] = std::numeric_limits<float>::quiet_NaN();

    // Acceleration must be NaN (not commanding acceleration)
    sp.acceleration[0] = std::numeric_limits<float>::quiet_NaN();
    sp.acceleration[1] = std::numeric_limits<float>::quiet_NaN();
    sp.acceleration[2] = std::numeric_limits<float>::quiet_NaN();

    // Jerk must be NaN
    sp.jerk[0] = std::numeric_limits<float>::quiet_NaN();
    sp.jerk[1] = std::numeric_limits<float>::quiet_NaN();
    sp.jerk[2] = std::numeric_limits<float>::quiet_NaN();

    // Yaw: NaN means hold current yaw (no active yaw command)
    sp.yaw      = std::numeric_limits<float>::quiet_NaN();
    sp.yawspeed = std::numeric_limits<float>::quiet_NaN();

    trajectory_pub_->publish(sp);
}

void SwarmController::publish_diagnostics(const FlockDiagnostics& diag) {
    msg::FlockingDiagnostics ros_diag;
    ros_diag.stamp = this->now();
    ros_diag.agent_id = agent_id_;
    ros_diag.velocity_correlation = diag.phi_corr;
    ros_diag.avg_velocity = diag.phi_vel;
    ros_diag.avg_nearest_distance = diag.avg_min_dist;
    ros_diag.min_nearest_distance = diag.min_min_dist;
    ros_diag.lap_parameter = diag.phi_lap;
    ros_diag.num_neighbors_visible = diag.n_neighbors;
    ros_diag.collision_warnings = diag.collision_warnings;
    ros_diag.closest_wall_distance = diag.closest_wall;
    ros_diag.closest_obstacle_distance = diag.closest_obstacle;
    ros_diag.desired_velocity.x = diag.desired_velocity.x;
    ros_diag.desired_velocity.y = diag.desired_velocity.y;
    ros_diag.desired_speed = diag.desired_velocity.norm();
    diag_pub_->publish(ros_diag);
}

void SwarmController::publish_visualization() {
    visualization_msgs::msg::MarkerArray markers;

    if (!arena_.boundary.vertices.empty()) {
        visualization_msgs::msg::Marker arena_marker;
        arena_marker.header.frame_id = "world";
        arena_marker.header.stamp = this->now();
        arena_marker.ns = "arena";
        arena_marker.id = 0;
        arena_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        arena_marker.action = visualization_msgs::msg::Marker::ADD;
        arena_marker.scale.x = 1.0;
        arena_marker.color.r = 1.0;
        arena_marker.color.a = 0.8;

        for (const auto& v : arena_.boundary.vertices) {
            geometry_msgs::msg::Point p;
            p.x = v.x; p.y = v.y; p.z = 0.0;
            arena_marker.points.push_back(p);
        }
        if (!arena_.boundary.vertices.empty()) {
            geometry_msgs::msg::Point p;
            p.x = arena_.boundary.vertices[0].x;
            p.y = arena_.boundary.vertices[0].y;
            arena_marker.points.push_back(p);
        }
        markers.markers.push_back(arena_marker);
    }

    viz_pub_->publish(markers);
}

// ─── Type conversions ────────────────────────────────────────────────────────

AgentState SwarmController::from_ros(const msg::AgentState& ros_msg) {
    AgentState s;
    s.id = ros_msg.agent_id;
    // AgentState.position is stored in NED: x=North, y=East, z=altitude(+up)
    s.pos.x = ros_msg.position.x;
    s.pos.y = ros_msg.position.y;
    s.altitude = ros_msg.position.z;
    s.vel.x = ros_msg.velocity.x;
    s.vel.y = ros_msg.velocity.y;
    s.heading = ros_msg.heading;
    s.status = static_cast<AgentStatus>(ros_msg.status);
    s.battery_voltage = ros_msg.battery_voltage;
    s.gps_accuracy = ros_msg.gps_accuracy;
    s.num_sats = ros_msg.num_satellites;
    s.comm_healthy = ros_msg.comm_healthy;
    s.last_received = AgentState::Clock::now();
    return s;
}

msg::AgentState SwarmController::to_ros(const AgentState& state) {
    msg::AgentState m;
    m.stamp = rclcpp::Clock().now();
    m.agent_id = state.id;
    // Broadcast NED position so all agents share the same frame
    m.position.x = state.pos.x;       // North (m)
    m.position.y = state.pos.y;       // East  (m)
    m.position.z = state.altitude;    // altitude AGL (+up)
    m.velocity.x = state.vel.x;       // vNorth (m/s)
    m.velocity.y = state.vel.y;       // vEast  (m/s)
    m.velocity.z = 0.0;
    m.heading = state.heading;
    m.status = static_cast<uint8_t>(state.status);
    m.battery_voltage = state.battery_voltage;
    m.gps_accuracy = state.gps_accuracy;
    m.num_satellites = state.num_sats;
    m.comm_healthy = state.comm_healthy;
    return m;
}

}  // namespace swarm_flocking
