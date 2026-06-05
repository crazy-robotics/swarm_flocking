# ROS2 Swarm Flocking

[![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue?logo=ros)](https://docs.ros.org/en/jazzy/)
[![PX4](https://img.shields.io/badge/PX4-Autopilot-purple)](https://px4.io/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)](https://gazebosim.org/)
[![License: BSD-3](https://img.shields.io/badge/License-BSD--3--Clause-green.svg)](LICENSE)
[![GitHub](https://img.shields.io/badge/GitHub-crazy--robotics-black?logo=github)](https://github.com/crazy-robotics/swarm_flocking)

> Autonomous multi-UAV swarm flocking for PX4 SITL + Gazebo — implementing the bio-inspired optimised flocking model of Vásárhelyi et al. (2018) with automatic arm/takeoff, global NED coordinate sharing, layered safety monitoring, and runtime formation commands.

---

![Swarm Flocking Demo](media/swarm_flocking.gif)

*5-drone flock converging on a migration target in Gazebo SITL — all vehicles arm, climb, and switch to Offboard autonomously on launch.*

---

## Table of Contents

1. [Overview](#1-overview)
2. [Features](#2-features)
3. [Research Background](#3-research-background)
4. [Mathematical Model](#4-mathematical-model)
5. [System Architecture](#5-system-architecture)
6. [Global NED Coordinate Frame](#6-global-ned-coordinate-frame)
7. [Safety System](#7-safety-system)
8. [Auto-Takeoff Sequence](#8-auto-takeoff-sequence)
9. [Requirements](#9-requirements)
10. [Installation & Build](#10-installation--build)
11. [Running](#11-running)
12. [Swarm Commands](#12-swarm-commands)
13. [Parameters Reference](#13-parameters-reference)
14. [Diagnostics & Monitoring](#14-diagnostics--monitoring)
15. [References](#15-references)

---

## 1. Overview

This package implements a fully autonomous swarm controller for **N PX4 quadrotors** in Gazebo SITL. Each UAV runs an independent `SwarmController` ROS2 node that:

- Arms, switches to Offboard mode, and climbs to a target altitude automatically on launch
- Converts its per-vehicle local NED position to a **shared global NED frame** using a flat-earth GPS projection — solving the multi-instance coordinate problem
- Runs the Vásárhelyi 2018 bio-inspired flocking algorithm with a migration bias for goal-directed flight
- Applies a multi-layer safety monitor (collision zones, geofence, comm-loss, battery, GPS quality)
- Responds to runtime operator commands: flock to target, ring/line/grid formation, RTL, emergency stop

The swarm requires **no central coordinator** — each agent computes its velocity setpoint from local observations of its neighbours only and sends it directly to PX4 via the Micro XRCE-DDS bridge.

---

## 2. Features

| Feature | Detail |
|---------|--------|
| **Auto arm + takeoff** | Arm → Offboard → climb to altitude — all automatic on `ros2 launch` |
| **Decentralised flocking** | Vásárhelyi 2018: repulsion, velocity alignment, self-propulsion, wall shill agents |
| **Goal-directed migration** | Flock steers as a unit toward an operator-specified NED target while maintaining cohesion |
| **Formation modes** | Grid, Ring, Line — with configurable inter-agent spacing |
| **Global NED frame** | Flat-earth GPS projection unifies all per-vehicle local NED origins |
| **5-layer safety** | Collision zones, geofence, comm-loss, battery, GPS quality — each with graduated response |
| **10 runtime commands** | Full mission control via ROS2 topic with TRANSIENT_LOCAL QoS (guaranteed delivery) |
| **Auto PX4 topic naming** | Correct `/px4_N/fmu/...` prefix computed automatically from `agent_id` |
| **Full yaml config** | All physics, safety, and flight parameters tunable without recompile |

---

## 3. Research Background

This implementation is based on:

> **Vásárhelyi, G., Virágh, C., Somorjai, G., Nepusz, T., Eiben, A. E., & Vicsek, T. (2018).**
> *Optimized flocking of autonomous drones in confined environments.*
> Science Robotics, **3**(20), eaat3536.
> <https://doi.org/10.1126/scirobotics.aat3536>

The paper derives a parameter-optimised flocking model validated on 30 real outdoor drones. The model produces cohesion, alignment, and separation from purely local pairwise interactions with no global state required.

Additional references:

| Ref | Authors | Contribution to this work |
|-----|---------|--------------------------|
| [2] | Vásárhelyi et al. (2014) | Formation controller equations (Eq. 6-8) used in `FormationController` |
| [3] | Reynolds (1987) | Classic boid rules — conceptual foundation |
| [4] | Olfati-Saber (2006) | Convergence analysis of flocking algorithms |
| [5] | Mesbahi & Egerstedt (2010) | Graph theoretic multi-agent coordination |

---

## 4. Mathematical Model

### 4.1 Notation

| Symbol | Meaning |
|--------|---------|
| $\mathbf{r}_i$ | Position of agent $i$ in global NED (m) |
| $\mathbf{v}_i$ | Velocity of agent $i$ (m/s) |
| $r_{ij} = \|\mathbf{r}_i - \mathbf{r}_j\|$ | Euclidean distance between agents $i$, $j$ |
| $v_{ij} = \|\mathbf{v}_i - \mathbf{v}_j\|$ | Relative speed |
| $v_{flock}$ | Desired flocking speed |
| $v_{max}$ | Absolute speed limit |
| $\hat{(\cdot)}$ | Unit vector |

### 4.2 Braking Curve (Eq. 4)

Defines the maximum allowed speed at distance $r$ from a stopping point given deceleration $a$ and gain $p$:

$$D(r,\,a,\,p) = \begin{cases} p\,r & \text{if }p\,r < a/p \\ \sqrt{2a\,r - a^2/p^2} & \text{otherwise} \end{cases}$$

The linear phase gives smooth exponential approach; the square-root phase gives constant-acceleration deceleration. This function is reused in alignment and wall-shill terms.

**Code:** `math_utils.hpp::braking_curve()`

### 4.3 Pairwise Repulsion (Eq. 2–3)

Each agent is repelled by any neighbour within range $r_{rep,0}$:

$$\mathbf{v}_{rep,ij} = p_{rep}\,(r_{rep,0} - r_{ij})\,\hat{(\mathbf{r}_i - \mathbf{r}_j)}, \quad r_{ij} < r_{rep,0}$$

Magnitude grows linearly as separation decreases below $r_{rep,0}$.

**Code:** `flocking_algorithm.cpp::pairwise_repulsion()`

### 4.4 Velocity Alignment / Friction (Eq. 5–7)

Agents align velocities with neighbours within $r_{frict,0}$. The maximum *allowed* velocity difference at separation $r_{ij}$ is:

$$v_{frict,max}(r_{ij}) = \max\!\bigl(v_{frict},\; D(r_{ij} - r_{frict,0},\; a_{frict},\; p_{frict})\bigr)$$

When $v_{ij} > v_{frict,max}$, agent $i$ applies:

$$\mathbf{v}_{frict,ij} = -C_{frict}\,(v_{ij} - v_{frict,max})\,\hat{(\mathbf{v}_i - \mathbf{v}_j)}$$

> **Tuning note (5-drone SITL):** with $N=5$ agents, the summed alignment force can be $4 \times C_{frict} \times v_{flock} \approx 10\ \text{m/s}$ when $C_{frict}=0.5$ — overwhelming the migration drive. We use $C_{frict} = 0.05$ so alignment contributes $\approx 0.8\ \text{m/s}$, letting migration dominate.

**Code:** `flocking_algorithm.cpp::pairwise_alignment()`

### 4.5 Self-Propulsion

$$\mathbf{v}_{spp,i} = v_{flock}\,\hat{\mathbf{v}}_i$$

When a migration target is active and $\|\mathbf{r}_{target} - \mathbf{r}_i\| > r_{arrive}$, self-propulsion is redirected toward the target so the flock steers correctly regardless of prior heading:

$$\mathbf{v}_{spp,i} = v_{flock}\,\hat{(\mathbf{r}_{target} - \mathbf{r}_i)}$$

When the agent arrives ($\|\mathbf{r}_{target} - \mathbf{r}_i\| \le r_{arrive}$), $\mathbf{v}_{spp,i} = \mathbf{0}$ so PX4 decelerates the vehicle and repulsion spreads agents across the target area instead of causing collisions.

**Code:** `flocking_algorithm.cpp::compute_desired_velocity()`

### 4.6 Wall / Shill Agent Interaction (Eq. 8–9)

For each arena boundary edge, a virtual *shill* agent sits at the nearest point on that edge moving inward at $v_{shill}$. The real agent aligns with it:

$$v_{wall,max}(r_{is}) = D(r_{is} - r_{shill,0},\; a_{shill},\; p_{shill})$$

$$\mathbf{v}_{wall,is} = -(v_{is} - v_{wall,max})\,\hat{(\mathbf{v}_i - \mathbf{v}_s)}, \quad v_{is} > v_{wall,max}$$

where $r_{is}$ is the distance from agent $i$ to the shill position, and $\mathbf{v}_s = v_{shill}\,\hat{n}_{inward}$.

**Code:** `flocking_algorithm.cpp::wall_shill_interaction()`

### 4.7 Migration Bias

A persistent bias steers the flock toward an operator target $\mathbf{r}_{target}$:

$$\mathbf{v}_{mig,i} = \begin{cases} g_{mig}\,v_{flock}\,\hat{(\mathbf{r}_{target} - \mathbf{r}_i)} & \|\mathbf{r}_{target}-\mathbf{r}_i\| > r_{arrive} \\ \mathbf{0} & \text{otherwise} \end{cases}$$

Default migration gain $g_{mig} = 0.4$ (40% of $v_{flock}$ additional push).

### 4.8 Composite Desired Velocity (Eq. 10–11)

$$\mathbf{v}_{desired,i} = \mathbf{v}_{spp,i} + \sum_{j \neq i} \mathbf{v}_{rep,ij} + \sum_{j \neq i} \mathbf{v}_{frict,ij} + \sum_{s} \mathbf{v}_{wall,is} + \mathbf{v}_{mig,i}$$

$$\mathbf{v}_{cmd,i} = \mathrm{clip}\!\left(\mathbf{v}_{desired,i},\; v_{max}\right)$$

This is published as a `TrajectorySetpoint` (NED velocity control) to PX4 at 20 Hz.

### 4.9 Altitude Controller

A proportional controller drives all agents to a common flight altitude $z_{ref}$:

$$\dot{z}_{cmd,i} = \mathrm{clip}\!\left(K_p\,(z_{ref} - z_i),\; -\dot{z}_{max},\; +\dot{z}_{max}\right)$$

with $K_p = 0.5\ \text{s}^{-1}$ and $\dot{z}_{max} = 1.5\ \text{m/s}$, ensuring all drones occupy the same 2D flight plane.

### 4.10 Order Parameters

Velocity correlation (polarisation, Eq. 13) — quantifies how aligned the flock is:

$$\phi_{corr} = \frac{1}{N(N-1)} \sum_{i \neq j} \frac{\mathbf{v}_i \cdot \mathbf{v}_j}{\|\mathbf{v}_i\|\|\mathbf{v}_j\|}$$

$\phi_{corr} \to 1$ means all agents fly in the same direction (perfect flock). Published in `FlockingDiagnostics`.

---

## 5. System Architecture

### 5.1 Overview

```
┌──────────────────────────────────────────────────────────────────────┐
│                      Gazebo Harmonic + PX4 SITL                      │
│  Instance 0 (agent 1)    Instance 1 (agent 2)    ...  Instance N-1   │
│  /fmu/out/*              /px4_1/fmu/out/*              /px4_N-1/...  │
└──────────────────┬───────────────────┬──────────────────────┬────────┘
                   │  Micro XRCE-DDS   │  udp4 port 8888      │
                   ▼                   ▼                       ▼
      ┌────────────────────────────────────────────────────────────┐
      │                  ROS2 DDS (FastDDS)                        │
      │   BEST_EFFORT+VOLATILE  for /fmu/* topics                  │
      │   RELIABLE+TRANSIENT_LOCAL for /swarm/commands             │
      └────────────────────────────────────────────────────────────┘
              ▲  ▼                             ▲  ▼
   ┌──────────────────────┐       ┌──────────────────────┐
   │  /agent_1/           │       │  /agent_N/           │
   │  swarm_agent_1       │       │  swarm_agent_N       │
   │                      │       │                      │
   │  ┌────────────────┐  │       │  ┌────────────────┐  │
   │  │SwarmController │  │◄─────►│  │SwarmController │  │
   │  │                │  │       │  │                │  │
   │  │ FlockingAlgo   │  │       │  │ FlockingAlgo   │  │
   │  │ FormationCtrl  │  │       │  │ FormationCtrl  │  │
   │  │ CommManager    │  │       │  │ CommManager    │  │
   │  │ SafetyMonitor  │  │       │  │ SafetyMonitor  │  │
   │  └────────────────┘  │       │  └────────────────┘  │
   └──────────────────────┘       └──────────────────────┘
              │                               │
              └───────── /swarm/ topics ──────┘
                         (DDS multicast)
```

### 5.2 Node Data Flow

```
VehicleOdometry  ──► on_odometry()        → vel (NED), heading
VehicleLocalPos  ──► on_local_position()  → global NED position (GPS projection)
                                                      │
                                          CommunicationManager
                                          (own_state + neighbors[])
                                                      │
                         ┌────────────────────────────┤
                    FlockingAlgorithm         FormationController
                    compute_desired_vel()     compute_formation_vel()
                         └────────────────────────────┤
                                                      │
                                        SafetyMonitor::apply_safety_filter()
                                                      │
               TrajectorySetpoint (20 Hz) ◄───────────┘
               OffboardControlMode (10 Hz) ──────────────► PX4
```

### 5.3 PX4 Topic Naming Convention

| PX4 instance | Agent ID | Topic prefix | Odometry topic |
|-------------|---------|-------------|----------------|
| 0 | 1 | *(none)* | `/fmu/out/vehicle_odometry` |
| 1 | 2 | `/px4_1` | `/px4_1/fmu/out/vehicle_odometry` |
| 2 | 3 | `/px4_2` | `/px4_2/fmu/out/vehicle_odometry` |
| N−1 | N | `/px4_N-1` | `/px4_{N-1}/fmu/out/vehicle_odometry` |

`SwarmController::px4_topic_prefix()` computes the correct prefix from `agent_id` so no remappings are needed in the launch file.

### 5.4 QoS Profiles

| Topic | Reliability | Durability | Why |
|-------|-------------|------------|-----|
| `/fmu/out/*` | BEST_EFFORT | VOLATILE | PX4 DDS bridge requirement — RELIABLE silently drops |
| `/fmu/in/*` | BEST_EFFORT | VOLATILE | Same |
| `/swarm/agent_states` | BEST_EFFORT | VOLATILE | High-rate, loss-tolerant broadcast |
| `/swarm/commands` | RELIABLE | TRANSIENT_LOCAL | Guaranteed delivery; late-joining nodes receive last command |
| `flocking_diagnostics` | RELIABLE | VOLATILE | Low-rate monitoring |

### 5.5 Agent Status Enum

| Value | Name | Description |
|-------|------|-------------|
| 0 | IDLE | Hovering in Offboard, awaiting command |
| 1 | TAKEOFF | Climbing to `flight_altitude_m` |
| 2 | FLOCKING | Running Vásárhelyi model with optional migration |
| 3 | FORMATION | Running formation controller |
| 4 | LANDING | Descending to land |
| 5 | EMERGENCY | Velocity zeroed, awaiting reset |
| 6 | RTL | Returning to global NED origin (0, 0) |

### 5.6 State Machine

```
   BOOT ──(auto_takeoff)──► TAKEOFF ──(alt reached)──► IDLE
                                                         │
   IDLE ◄────────────────────────────── CMD_STOP (1) ◄──┤
                                                         │
   CMD_START_FLOCKING (0) ──────────────────────────►FLOCKING
   CMD_FORMATION_RING/LINE/GRID (2,3,4) ─────────► FORMATION
   CMD_RTL (9) ─────────────────────────────────────── RTL
   CMD_EMERGENCY_STOP (6) ──────────────────────── EMERGENCY
   CMD_LAND_ALL (5) ────────────────────────────── LANDING
   safety: emergency_land_ ─────────────────────── EMERGENCY
```

---

## 6. Global NED Coordinate Frame

### 6.1 Problem

Each PX4 SITL instance uses its own GPS home position as the origin of its local NED frame. All five vehicles report position $(0, 0)$ in their own frames on startup → every agent sees all neighbours at the same location → immediate PANIC collision detected → swarm cannot flock.

### 6.2 Solution: Flat-Earth GPS Projection

`VehicleLocalPosition` publishes `ref_lat` / `ref_lon` — the GPS coordinates of each vehicle's local NED origin. We convert every vehicle's local NED to a **shared global NED frame** in two steps:

**Step 1 — Local NED → geodetic (absolute lat/lon):**

$$\lambda_i = \lambda_{ref,i} + \dfrac{x_i}{R} \cdot \dfrac{180°}{\pi}$$

$$\phi_i = \phi_{ref,i} + \dfrac{y_i}{R\cos(\lambda_{ref,i} \pi/180°)} \cdot \dfrac{180°}{\pi}$$

**Step 2 — Geodetic → global NED** relative to shared reference $(\lambda_0, \phi_0)$:

$$x_{global,i} = (\lambda_i - \lambda_0)\,\dfrac{\pi}{180°}\,R$$

$$y_{global,i} = (\phi_i - \phi_0)\,\dfrac{\pi}{180°}\,R\,\cos\!\left(\lambda_0\,\dfrac{\pi}{180°}\right)$$

$R = 6{,}371{,}000\ \text{m}$. Default shared reference: PX4 SITL GPS home (lat = 47.397742°, lon = 8.545594°).

**Error**: flat-earth approximation accumulates $< 1\ \text{cm}$ error for inter-vehicle separations $\le 500\ \text{m}$.

**Code:** `math_utils.hpp::local_ned_to_global_ned()`

---

## 7. Safety System

`SafetyMonitor` runs five independent checks at 5 Hz and applies a graduated velocity filter.

### 7.1 Threat Levels & Filter Response

| Level | Trigger | Speed factor | Avoidance blend |
|-------|---------|-------------|-----------------|
| NONE | — | 1.00 | 0% |
| CAUTION | within `r_caution` (4 m) | 0.80 | 0% |
| WARNING | within `r_warning` (3 m) | 0.50 | 40% |
| CRITICAL | within `r_critical` (2 m) | 0.30 | 80% |
| PANIC | within `r_panic` (1 m) | 0.00 | → EMERGENCY |

### 7.2 Check Sources

| Source | Condition | Max effect |
|--------|-----------|-----------|
| Collision | neighbour within any zone | PANIC → EMERGENCY |
| Geofence | outside arena polygon | force velocity inward |
| Comm-loss | no neighbour heard > `all_comm_loss_s` (8 s) | `emergency_land_` |
| Battery | voltage below thresholds | RTL or emergency land |
| GPS | accuracy > threshold / sats < minimum | CAUTION |

### 7.3 Four-Layer Filter

```
desired_vel
    │
    ├─ Layer 1: clamp to max_speed_override (12 m/s hard limit)
    ├─ Layer 2: blend emergency avoidance when threat ≥ WARNING
    │           avoidance = Σ strong repulsion from nearest threats
    ├─ Layer 3: scale speed by threat factor (table above)
    └─ Layer 4: hard geofence — override to point inward if outside boundary
         │
         ▼
    filtered_vel → TrajectorySetpoint
```

Collision check is skipped in IDLE, TAKEOFF, and EMERGENCY states to prevent false alarms during startup and landing.

---

## 8. Auto-Takeoff Sequence

When `auto_takeoff:=true` (default), the following sequence runs automatically for each agent in parallel:

```
t = 0 s   Node starts
          OffboardControlMode keepalive begins at 10 Hz

t + GPS   First VehicleLocalPosition received (GPS fix, ~3–5 s)
          mode → TAKEOFF
          Global NED reference set

t + 1 s   VEHICLE_CMD_COMPONENT_ARM_DISARM sent
          (param2 = 21196 force-arm for SITL)

t + 4 s   DO_SET_MODE → Offboard sent
          PX4 enters Offboard velocity control

t + 4–8 s Altitude P-controller climbs at ≤1.5 m/s
          vDown = clip(0.5 × (z_target − z), −1.5, +1.5)

altitude  mode → IDLE
reached   Agent ready for swarm commands
```

Set `auto_takeoff:=false` to arm manually via QGroundControl. The one-shot Offboard mode timer still fires 1.5 s after the first odometry.

---

## 9. Requirements

| Software | Version | Notes |
|----------|---------|-------|
| Ubuntu | 22.04 LTS | Host OS |
| ROS2 | Jazzy | Middleware |
| PX4-Autopilot | ≥ v1.14 | SITL firmware |
| Gazebo | Harmonic | Physics / rendering |
| Micro XRCE-DDS Agent | 2.4.x | PX4 ↔ ROS2 bridge |
| px4_msgs | matching PX4 branch | Read-only dependency |
| Python | ≥ 3.10 | Launch files |

**Recommended hardware:** 8+ core CPU, 16 GB RAM (5 PX4 instances + Gazebo + ROS2).

---

## 10. Installation & Build

```bash
# 1. Create workspace
mkdir -p ~/ros2_swarm_ws/src
cd ~/ros2_swarm_ws/src

# 2. Clone this package
git clone https://github.com/crazy-robotics/swarm_flocking.git

# 3. Clone px4_msgs (must match your PX4 version — do not modify)
git clone https://github.com/PX4/px4_msgs.git

# 4. Build
cd ~/ros2_swarm_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 5. Source
source install/setup.bash
```

`--symlink-install` symlinks yaml config files so parameter changes take effect immediately on node restart without rebuilding.

---

## 11. Running

### 11.1 Terminal Layout

Open 7 terminals. Run each command in its own terminal:

```bash
# T1 — PX4 instance 0 (agent 1)
cd ~/PX4-Autopilot && PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 0

# T2 — PX4 instance 1 (agent 2)
cd ~/PX4-Autopilot && PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1

# T3 — PX4 instance 2 (agent 3)
cd ~/PX4-Autopilot && PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 2

# T4 — PX4 instance 3 (agent 4)
cd ~/PX4-Autopilot && PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 3

# T5 — PX4 instance 4 (agent 5)
cd ~/PX4-Autopilot && PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 4

# T6 — DDS bridge (one instance for all vehicles)
MicroXRCEAgent udp4 -p 8888

# T7 — Swarm launch
source ~/ros2_swarm_ws/install/setup.bash
ros2 launch swarm_flocking swarm.launch.py \
  num_agents:=5        \
  v_flock:=4.0         \
  flight_altitude:=5.0 \
  auto_takeoff:=true
```

### 11.2 Wait for Takeoff

Monitor in another terminal:

```bash
ros2 topic echo /swarm/agent_states | grep -E "agent_id|status"
```

Wait until all 5 agents show `status: 0` (IDLE).

### 11.3 Send Flocking Mission

```bash
ros2 topic pub -t 5 -r 1 /swarm/commands swarm_flocking/msg/SwarmCommand \
  "{command: 0, target_position: {x: 30.0, y: 30.0, z: 0.0}, flock_speed: 4.0}" \
  --qos-durability transient_local
```

> **Always use `-t 5 -r 1`** (5 repetitions, 1 Hz) and `--qos-durability transient_local` to guarantee all 5 DDS subscribers receive the command.

### 11.4 Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `num_agents` | 3 | Number of drones (must equal running PX4 instances) |
| `v_flock` | 4.0 | Flocking speed (m/s) |
| `flight_altitude` | 5.0 | Target altitude AGL (m) |
| `auto_takeoff` | true | Auto arm + takeoff + offboard |
| `arena_size` | 200.0 | Square geofence side (m) |
| `config_file` | *(default yaml)* | Path to custom config yaml |

---

## 12. Swarm Commands

Publish to `/swarm/commands` as `swarm_flocking/msg/SwarmCommand`.

| Constant | Value | Description | Fields used |
|----------|-------|-------------|------------|
| `CMD_START_FLOCKING` | **0** | Flock to target as a unit | `target_position` (NED m), `flock_speed` |
| `CMD_STOP` | 1 | Halt all → IDLE | — |
| `CMD_FORMATION_GRID` | 2 | Agents pack via repulsion, COM tracks target | `target_position`, `formation_spacing` |
| `CMD_FORMATION_RING` | 3 | Ring of radius = N·spacing/(2π) around target | `target_position`, `formation_spacing` |
| `CMD_FORMATION_LINE` | 4 | PCA-aligned line through target | `target_position`, `formation_spacing` |
| `CMD_LAND_ALL` | 5 | Descend and land all agents | — |
| `CMD_EMERGENCY_STOP` | 6 | Immediate zero-velocity halt | — |
| `CMD_SET_SPEED` | 7 | Update flocking speed (stays in current mode) | `flock_speed` |
| `CMD_SET_ARENA` | 8 | Resize geofence | `arena_size` |
| `CMD_RTL` | 9 | Return to global NED origin (0, 0) | — |

### 12.1 Example Commands

```bash
# Flock to N=30 E=30 at 4 m/s
ros2 topic pub -t 5 -r 1 /swarm/commands swarm_flocking/msg/SwarmCommand \
  "{command: 0, target_position: {x: 30.0, y: 30.0, z: 0.0}, flock_speed: 4.0}" \
  --qos-durability transient_local

# Ring formation around origin, 10 m spacing
ros2 topic pub -t 5 -r 1 /swarm/commands swarm_flocking/msg/SwarmCommand \
  "{command: 3, target_position: {x: 0.0, y: 0.0, z: 0.0}, formation_spacing: 10.0}" \
  --qos-durability transient_local

# Line formation to N=50, 8 m spacing
ros2 topic pub -t 5 -r 1 /swarm/commands swarm_flocking/msg/SwarmCommand \
  "{command: 4, target_position: {x: 50.0, y: 0.0, z: 0.0}, formation_spacing: 8.0}" \
  --qos-durability transient_local

# Return to launch
ros2 topic pub -t 5 -r 1 /swarm/commands swarm_flocking/msg/SwarmCommand \
  "{command: 9}" --qos-durability transient_local

# Emergency stop mid-flight
ros2 topic pub --once /swarm/commands swarm_flocking/msg/SwarmCommand \
  "{command: 6}" --qos-durability transient_local
```

---

## 13. Parameters Reference

Edit `config/flocking_params.yaml` (yaml key `/**:` — matches all agent nodes).

### 13.1 Flocking Physics

| Parameter | Default | Unit | Notes |
|-----------|---------|------|-------|
| `flock.v_flock` | 4.0 | m/s | Desired flocking speed |
| `flock.v_max` | 6.0 | m/s | Hard speed cap |
| `flock.comm_range` | 500.0 | m | Neighbour visibility radius (500 > arena diagonal) |
| `flock.r_rep_0` | 5.0 | m | Repulsion range; equilibrium separation ≈ r_rep_0/2 |
| `flock.p_rep` | 0.5 | 1/s | Repulsion spring constant |
| `flock.r_frict_0` | 15.0 | m | Alignment range |
| `flock.C_frict` | 0.05 | — | Alignment gain (small prevents alignment overpowering migration) |
| `flock.v_frict` | 1.5 | m/s | Velocity slack — no alignment if $v_{ij} < v_{frict}$ |
| `flock.p_frict` | 1.0 | 1/s | Braking curve linear gain |
| `flock.a_frict` | 3.0 | m/s² | Braking curve deceleration |
| `flock.r_shill_0` | 0.0 | m | Wall shill stopping offset |
| `flock.v_shill` | 10.0 | m/s | Wall shill speed |
| `flock.arena_size` | 200.0 | m | Square geofence side |

### 13.2 Safety

| Parameter | Default | Unit | Notes |
|-----------|---------|------|-------|
| `safety.r_panic` | 1.0 | m | Must be < PX4 SITL spawn spacing (~1.7 m) |
| `safety.r_critical` | 2.0 | m | Emergency brake zone |
| `safety.r_warning` | 3.0 | m | Active avoidance zone |
| `safety.r_caution` | 4.0 | m | Advisory slow-down zone |
| `safety.max_speed_override` | 12.0 | m/s | Absolute cap |

### 13.3 Flight

| Parameter | Default | Unit | Notes |
|-----------|---------|------|-------|
| `flight_altitude_m` | 5.0 | m | AGL target for all agents |
| `auto_takeoff` | true | bool | Arm + takeoff + offboard on startup |
| `global_ref_lat` | 47.397742 | ° | Shared NED reference (PX4 SITL default) |
| `global_ref_lon` | 8.545594 | ° | Shared NED reference |
| `control_rate_hz` | 20.0 | Hz | Main control loop |
| `broadcast_rate_hz` | 10.0 | Hz | AgentState broadcast |

---

## 14. Diagnostics & Monitoring

```bash
# All agent states (position, velocity, status)
ros2 topic echo /swarm/agent_states

# Per-agent algorithm diagnostics (4 Hz)
ros2 topic echo /agent_1/flocking_diagnostics

# Safety events
ros2 topic echo /agent_1/safety_events

# Confirm vehicle in Offboard + velocity control
ros2 topic echo /fmu/out/vehicle_control_mode | grep -E "flag_armed|flag_control_offboard|flag_control_velocity"

# Verify active parameters
ros2 param get /agent_1/swarm_agent_1 flock.C_frict
ros2 param get /agent_1/swarm_agent_1 safety.r_panic
```

### FlockingDiagnostics Fields

| Field | Meaning |
|-------|---------|
| `velocity_correlation` | $\phi_{corr}$ — 1.0 = perfect flock alignment |
| `desired_velocity` | Computed velocity before safety filter (m/s NED) |
| `avg_nearest_distance` | Mean distance to all visible neighbours |
| `min_nearest_distance` | Closest neighbour (collision indicator) |
| `num_neighbors_visible` | Active neighbours within `comm_range` |
| `collision_warnings` | Neighbours inside `r_coll` this cycle |

---

## 15. References

```
[1] Vásárhelyi, G. et al. (2018). Optimized flocking of autonomous drones in
    confined environments. Science Robotics 3(20), eaat3536.
    https://doi.org/10.1126/scirobotics.aat3536

[2] Vásárhelyi, G. et al. (2014). Outdoor flocking and formation flight with
    autonomous aerial robots. IROS 2014.
    https://doi.org/10.1109/IROS.2014.6943105

[3] Reynolds, C. W. (1987). Flocks, herds, and schools: A distributed
    behavioral model. SIGGRAPH '87.
    https://doi.org/10.1145/37401.37406

[4] Olfati-Saber, R. (2006). Flocking for multi-agent dynamic systems:
    algorithms and theory. IEEE TAC 51(3), 401–420.

[5] PX4 Autopilot Documentation. https://docs.px4.io/main/en/

[6] Micro XRCE-DDS Agent. https://micro-xrce-dds.docs.eprosima.com/

[7] ROS2 Jazzy. https://docs.ros.org/en/jazzy/
```

---

## License

BSD 3-Clause License. See [LICENSE](LICENSE).

---

*[crazy-robotics](https://github.com/crazy-robotics) — ROS2 Jazzy · PX4 · Gazebo Harmonic*
