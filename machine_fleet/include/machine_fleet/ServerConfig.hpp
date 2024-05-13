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

#ifndef MACHINE_FLEET__INCLUDE__MACHINE_FLEET__SERVERCONFIG_HPP
#define MACHINE_FLEET__INCLUDE__MACHINE_FLEET__SERVERCONFIG_HPP

#include <string>

namespace machine_fleet {

struct ServerConfig
{
  int dds_domain = 52;
  std::string dds_machine_state_topic = "machine_state";

  // Client request -> server:
  std::string dds_delivery_request_topic = "delivery_request";

  // Server request -> client:
  std::string dds_machine_request_topic = "machine_request";
  std::string dds_station_request_topic = "station_request";

  void print_config() const;
};

} // namespace machine_fleet

#endif // MACHINE_FLEET__INCLUDE__MACHINE_FLEET__SERVERCONFIG_HPP
