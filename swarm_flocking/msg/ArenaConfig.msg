# ArenaConfig.msg
# Defines the bounded flight area and obstacles

builtin_interfaces/Time stamp

# Arena boundary as convex polygon vertices (ENU, z ignored)
geometry_msgs/Point[] arena_vertices

# Obstacles as convex polygons: each obstacle is a list of vertices
# Flattened: obstacle_counts[i] gives the vertex count for obstacle i
# obstacle_vertices contains all vertices sequentially
uint32[] obstacle_counts
geometry_msgs/Point[] obstacle_vertices
