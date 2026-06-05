// Copyright (c) 2024 Swarm Robotics Lab. BSD-3-Clause License.
// swarm_node.cpp — Executable entry point

#include <rclcpp/rclcpp.hpp>
#include "swarm_flocking/swarm_controller.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<swarm_flocking::SwarmController>();

    // Multi-threaded executor for parallel timer callbacks
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
