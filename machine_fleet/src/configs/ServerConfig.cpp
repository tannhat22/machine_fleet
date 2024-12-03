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

#include <machine_fleet/ServerConfig.hpp>

#include <cstdio>

namespace machine_fleet {

std::string ServerConfig::get_namespaced_topic(const std::string &machine_name,
                                               const std::string &base_topic) const {
  return machine_name + "/" + base_topic;
}

void ServerConfig::print_config() const {
  printf("SERVER-CLIENT DDS CONFIGURATION\n");
  printf("  dds domain: %d\n", dds_domain);
  printf("  TOPICS\n");
  for (size_t i = 0; i < machine_names.size(); ++i) {
    printf("machine_name[%zu]: %s\n", i, machine_names[i].c_str());
  }
  printf("    machine state: %s\n", dds_machine_state_topic.c_str());
  printf("    machine request: %s\n", dds_machine_request_topic.c_str());
  printf("    station request: %s\n", dds_station_request_topic.c_str());
}

} // namespace machine_fleet
