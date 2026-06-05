# SwarmCommand.msg
# Commands issued to the swarm (from ground station or mission planner)

builtin_interfaces/Time stamp

uint8 CMD_START_FLOCKING = 0
uint8 CMD_STOP = 1
uint8 CMD_FORMATION_GRID = 2
uint8 CMD_FORMATION_RING = 3
uint8 CMD_FORMATION_LINE = 4
uint8 CMD_LAND_ALL = 5
uint8 CMD_EMERGENCY_STOP = 6
uint8 CMD_SET_SPEED = 7
uint8 CMD_SET_ARENA = 8
uint8 CMD_RTL = 9
uint8 command

# Target position for formation tracking (ENU)
geometry_msgs/Point target_position

# Parameters
float64 flock_speed                  # m/s
float64 arena_size                   # m (square arena side length)
float64 formation_spacing            # m (inter-agent spacing)
