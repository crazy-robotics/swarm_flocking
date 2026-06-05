# FlockingDiagnostics.msg
# Published by each agent with local diagnostic info

builtin_interfaces/Time stamp
uint16 agent_id

# Order parameters (per Vasarhelyi 2018 Eq 13-16)
float64 velocity_correlation         # phi_corr: [-1, 1]
float64 avg_velocity                 # phi_vel: m/s
float64 avg_nearest_distance         # mean closest neighbor dist (m)
float64 min_nearest_distance         # minimum closest neighbor dist (m)
float64 lap_parameter                # phi_LAP: rotational order [-1, 1]

# Safety metrics
uint32 num_neighbors_visible
uint32 collision_warnings            # count of r_ij < r_coll events
float64 closest_wall_distance        # meters
float64 closest_obstacle_distance    # meters

# Control outputs (desired velocity magnitude and direction)
geometry_msgs/Vector3 desired_velocity
float64 desired_speed
