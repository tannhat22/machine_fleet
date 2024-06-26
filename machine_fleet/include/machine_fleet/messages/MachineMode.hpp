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

#ifndef MACHINE_FLEET__INCLUDE__MACHINE_FLEET__MESSAGES__MACHINEMODE_HPP
#define MACHINE_FLEET__INCLUDE__MACHINE_FLEET__MESSAGES__MACHINEMODE_HPP

#include <cstdint>

namespace machine_fleet {
namespace messages {

struct MachineMode
{
  uint32_t mode;
  static const uint32_t MODE_IDLE = 0;
  static const uint32_t MODE_PK_RELEASE = 1;
  static const uint32_t MODE_PK_CLAMP = 2;
  static const uint32_t MODE_DF_RELEASE = 3;
  static const uint32_t MODE_DF_CLAMP = 4;
  static const uint32_t MODE_ERROR = 200;
};

} // namespace messages
} // namespace machine_fleet

#endif // MACHINE_FLEET__INCLUDE__MACHINE_FLEET__MESSAGES__MACHINEMODE_HPP
