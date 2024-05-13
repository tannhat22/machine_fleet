/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef MACHINE_FLEET_SERVER_ROS2__SRC__UTILITIES_HPP
#define MACHINE_FLEET_SERVER_ROS2__SRC__UTILITIES_HPP

#include <machine_fleet_msgs/msg/delivery_request.hpp>
#include <machine_fleet_msgs/msg/machine_state.hpp>
#include <machine_fleet_msgs/msg/machine_request.hpp>
#include <machine_fleet_msgs/msg/station_request.hpp>

#include <machine_fleet/messages/DeliveryRequest.hpp>
#include <machine_fleet/messages/MachineState.hpp>
#include <machine_fleet/messages/MachineRequest.hpp>
#include <machine_fleet/messages/StationRequest.hpp>

namespace machine_fleet
{
namespace ros2
{

void to_mf_message(
    const machine_fleet_msgs::msg::MachineRequest& in_msg, 
    messages::MachineRequest& out_msg);

void to_mf_message(
    const machine_fleet_msgs::msg::StationRequest& in_msg, 
    messages::StationRequest& out_msg);

// ----------------------------------------------------------------------------

void to_ros_message(
    const messages::MachineState& in_msg,
    machine_fleet_msgs::msg::MachineState& out_msg);

void to_ros_message(
    const messages::DeliveryRequest& in_msg,
    machine_fleet_msgs::msg::DeliveryRequest& out_msg);

} // namespace ros2
} // namespace machine_fleet


#endif // MACHINE_FLEET_SERVER_ROS2__SRC__UTILITIES_HPP
