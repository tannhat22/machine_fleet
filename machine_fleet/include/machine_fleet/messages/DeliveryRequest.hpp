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

#ifndef MACHINE_FLEET__INCLUDE__MACHINE_FLEET__MESSAGES__DELIVERYREQUEST_HPP
#define MACHINE_FLEET__INCLUDE__MACHINE_FLEET__MESSAGES__DELIVERYREQUEST_HPP

#include <string>
#include "DeliveryMode.hpp"

namespace machine_fleet {
namespace messages {

struct DeliveryRequest
{
  std::string machine_name;
  std::string fleet_name;
  std::string station_name;
  std::string request_id;
  DeliveryMode mode;
};

} // namespace messages
} // namespace machine_fleet

#endif // MACHINE_FLEET__INCLUDE__MACHINE_FLEET__MESSAGES__DELIVERYREQUEST_HPP
