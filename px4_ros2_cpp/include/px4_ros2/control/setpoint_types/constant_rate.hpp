/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#pragma once

#include <Eigen/Eigen>
#include <optional>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <px4_ros2/common/setpoint_base.hpp>
#include <px4_ros2/utils/geodesic.hpp>

namespace px4_ros2 {
/** \ingroup setpoint_types
 *  @{
 */

/**
 * @brief Setpoint type for smooth position and heading control
 */
class ConstantZDescned : public SetpointBase {
   public:
    explicit ConstantZDescned(Context& context);

    ~ConstantZDescned() override = default;

    Configuration getConfiguration() override;

    /**
     * @brief Go-to setpoint update
     *
     * Unset optional values are not controlled
     *
     * @param position [m] NED earth-fixed frame, z not used
     * @param z_rate [m/s] desired descend rate in z axis
     * @param heading [rad] from North
     */
    void update(const Eigen::Vector3f& position, const float& z_rate, const float& heading);

    float desiredUpdateRateHz() override { return 30.f; }

   private:
    rclcpp::Node& _node;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _constant_z_descned_pub;
};

/** @}*/
} /* namespace px4_ros2 */
