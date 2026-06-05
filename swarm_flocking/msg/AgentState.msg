# AgentState.msg
# Broadcast message for inter-agent communication (analogous to XBee status packets)
# Each agent broadcasts this at 10 Hz over DDS

builtin_interfaces/Time stamp
uint16 agent_id

# Position (GNSS-based, WGS84 or local ENU frame)
geometry_msgs/Point position        # x, y, z in meters (local ENU)
geometry_msgs/Vector3 velocity      # vx, vy, vz in m/s

# Heading and attitude
float64 heading                     # radians
float64 roll
float64 pitch

# Status flags
uint8 STATUS_IDLE = 0
uint8 STATUS_TAKEOFF = 1
uint8 STATUS_FLOCKING = 2
uint8 STATUS_FORMATION = 3
uint8 STATUS_LANDING = 4
uint8 STATUS_EMERGENCY = 5
uint8 STATUS_RTL = 6
uint8 status

# Health
float32 battery_voltage
float32 gps_accuracy               # meters, horizontal DOP
uint8 num_satellites
bool comm_healthy
