# Autonomous Vehicle Control System

Autonomous navigation stack for a simulated robot: costmap from lidar, persistent global map, A* path planning, and Pure Pursuit for path following. Built as four ROS 2 nodes that communicate over standard topics.

**In this repo:** [costmap](src/robot/costmap/) turns lidar into an occupancy grid and inflates obstacles; [map_memory](src/robot/map_memory/) merges costmaps into a global map using odometry; [planner](src/robot/planner/) plans paths with A* and publishes them; [control](src/robot/control/) tracks the path using Pure Pursuit and publishes twist commands.

[![Watch the video](https://img.youtube.com/vi/X-JIZ522HMU/maxresdefault.jpg)](https://www.youtube.com/watch?v=X-JIZ522HMU)

---

### Techniques used in the code

- **A* on a grid** — [planner_node.cpp](src/robot/planner/src/planner_node.cpp) uses a min-heap open set ([`std::priority_queue`](https://en.cppreference.com/w/cpp/container/priority_queue)) with a custom comparator and [`std::unordered_map`](https://en.cppreference.com/w/cpp/container/unordered_map) with a [custom hash](https://en.cppreference.com/w/cpp/utility/hash) for `CellIndex` so cells can be used as keys for `came_from` and `g_score`.
- **Quaternion → yaw** — Control and map_memory use [tf2](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Quaternion-Stuff-Ros2.html) `Quaternion` and `Matrix3x3::getRPY()` to get robot yaw for heading and for transforming local costmap cells into the global map frame.
- **Periodic work with wall timers** — Planning, control, and map updates are driven by [`create_wall_timer`](https://docs.ros.org/en/rolling/How-To-Guides/Timers.html) (e.g. 500 ms planner, 100 ms control loop) instead of blocking loops.
- **Occupancy grid as 1D buffer** — Grids are stored as `std::vector<int8_t>` with row-major indexing (`y * width + x`) in [costmap_node.cpp](src/robot/costmap/src/costmap_node.cpp), [map_memory_node.cpp](src/robot/map_memory/src/map_memory_node.cpp), and planner.
- **Oriented robot footprint** — [planner_node.cpp](src/robot/planner/src/planner_node.cpp) samples a rectangle in robot frame and transforms points to world frame with the current yaw to check occupancy for the robot's footprint along the path.
- **Costmap inflation** — [costmap_node.cpp](src/robot/costmap/src/costmap_node.cpp) inflates each lidar hit with a distance-based cost decay (e.g. `100 - (dis/inflation_radius)*100`) over a local grid window.
- **Pure Pursuit** — [control_node.cpp](src/robot/control/src/control_node.cpp) uses a fixed lookahead distance to pick a target pose on the path, then computes linear and angular velocity from the heading error and distance to goal (proportional gains).

---

### Technologies and libraries

- **ROS 2** — Node runtime, topics, and build integration.
- **C++** — C++17; used for all robot packages.
- **Docker** — Containerization for build and run.
- **Foxglove** — Visualization (see [config](config/) for layout).
- **CMake** — Build system via [ament_cmake](https://docs.ros.org/en/rolling/How-To-Guides/Ament-CMake-Python-Documentation.html) for ROS 2 packages.

---

### Project structure

```
├── README.md
├── LICENSE
├── modules/                    # Env / secrets (e.g. .env)
├── config/                     # Root config (e.g. Foxglove layout)
└── src/
    ├── robot/                  # Four ROS 2 navigation packages
    │   ├── costmap/            # Lidar → occupancy grid + inflation
    │   │   ├── config/
    │   │   ├── include/
    │   │   └── src/
    │   ├── control/            # Pure Pursuit path following, cmd_vel
    │   │   ├── config/
    │   │   ├── include/
    │   │   └── src/
    │   ├── map_memory/         # Costmap → global map with odometry
    │   │   ├── config/
    │   │   ├── include/
    │   │   └── src/
    │   └── planner/            # A* path planning
    │       ├── config/
    │       ├── include/
    │       └── src/
    └── samples/                # Sample C++ and Python packages
        ├── cpp/
        │   ├── producer/
        │   └── transformer/
        └── python/
            ├── producer/
            └── transformer/
```
