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

#include <cstdio>

#include "machine_fleet/ros2/client_node_config.hpp"

namespace machine_fleet
{
namespace ros2
{

void ClientNodeConfig::print_config() const
{
  printf("ROS 2 CLIENT CONFIGURATION\n");
  printf("  fleet name: %s\n", fleet_name.c_str());
  printf("  machine name: %s\n", machine_name.c_str());
  printf("  wait timeout: %.1f\n", wait_timeout);
  printf("  update request frequency: %.1f\n", update_frequency);
  printf("  publish state frequency: %.1f\n", publish_frequency);
  printf("  TOPICS\n");
  printf("    machine state: %s\n", machine_state_topic.c_str());
  printf("    delivery request: %s\n", delivery_request_topic.c_str());
  printf("    station request: %s\n", station_request_topic.c_str());
  printf("    machine trigger server: %s\n", machine_trigger_server_name.c_str());
  printf("CLIENT-SERVER DDS CONFIGURATION\n");
  printf("  dds domain: %d\n", dds_domain);
  printf("  TOPICS\n");
  printf("    machine state: %s\n", dds_state_topic.c_str());
  printf("    delivery request: %s\n", dds_delivery_request_topic.c_str());
  printf("    machine request: %s\n", dds_machine_request_topic.c_str());
  printf("    station request: %s\n", dds_station_request_topic.c_str());
}
  
ClientConfig ClientNodeConfig::get_client_config() const
{
  ClientConfig client_config;
  client_config.dds_domain = dds_domain;
  client_config.dds_state_topic = dds_state_topic;
  client_config.dds_delivery_request_topic = dds_delivery_request_topic;
  client_config.dds_machine_request_topic = dds_machine_request_topic;
  client_config.dds_station_request_topic = dds_station_request_topic;
  return client_config;
}

} // namespace ros2
} // namespace machine_fleet