/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <cmath>  // For std::nanf
#include <px4_ros2/control/setpoint_types/constant_rate.hpp>
#include <px4_ros2/utils/message_version.hpp>

namespace px4_ros2 {

ConstantZDescned::ConstantZDescned(Context& context)
    : SetpointBase(context), _node(context.node()) {
    _constant_z_descned_pub = context.node().create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        context.topicNamespacePrefix() + "fmu/in/goto_setpoint" +
            px4_ros2::getMessageNameVersion<px4_msgs::msg::TrajectorySetpoint>(),
        1);
}

void ConstantZDescned::update(const Eigen::Vector3f& position,
                              const float& z_rate,
                              const float& heading) {
    onUpdate();

    px4_msgs::msg::TrajectorySetpoint sp{};

    // setpoints
    sp.position[0] = position(0);
    sp.position[1] = position(1);
    sp.position[2] = std::nanf("");
    sp.heading = heading.value_or(0.f);

    sp.velocity[0] = std::nanf("");
    sp.velocity[1] = std::nanf("");
    sp.velocity[2] = z_rate;

    // not controlled parameters
    sp.acceleration[0] = std::nanf("");
    sp.acceleration[1] = std::nanf("");
    sp.acceleration[2] = std::nanf("");
    sp.jerk[0] = std::nanf("");
    sp.jerk[1] = std::nanf("");
    sp.jerk[2] = std::nanf("");
    sp.yawspeed = std::nanf("");

    sp.timestamp = 0;  // Let PX4 set the timestamp
    _constant_z_descned_pub->publish(sp);
}

SetpointBase::Configuration ConstantZDescned::getConfiguration() {
    Configuration config{};
    config.control_allocation_enabled = true;
    config.rates_enabled = true;
    config.attitude_enabled = true;
    config.altitude_enabled = true;
    config.acceleration_enabled = true;
    config.velocity_enabled = true;
    config.position_enabled = true;
    config.climb_rate_enabled = true;
    return config;
}

}  // namespace px4_ros2
