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

#ifndef MACHINE_FLEET__ROS2__CLIENTNODECONFIG_HPP
#define MACHINE_FLEET__ROS2__CLIENTNODECONFIG_HPP

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <machine_fleet/ClientConfig.hpp>

namespace machine_fleet
{
namespace ros2
{

struct ClientNodeConfig
{

  std::string machine_name = "machine_name";
  std::string fleet_name = "fleet_name";

  std::string machine_state_topic = "/machine_state";
  std::string delivery_request_topic = "/delivery_request";
  std::string station_request_topic = "/station_request";
  std::string machine_trigger_server_name = "machine_server";

  int dds_domain = 52;
  std::string dds_state_topic = "machine_state";
  std::string dds_delivery_request_topic = "delivery_request";
  std::string dds_machine_request_topic = "machine_request";
  std::string dds_station_request_topic = "station_request";

  double wait_timeout = 10.0;
  double update_frequency = 10.0;
  double publish_frequency = 1.0;

  void print_config() const;

  ClientConfig get_client_config() const;
};

} // namespace ros2
} // namespace machine_fleet

#endif // MACHINE_FLEET__ROS2__CLIENTNODECONFIG_HPP