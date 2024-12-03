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

#ifndef MACHINE_FLEET__SRC__MESSAGES__MESSAGE_UTILS_HPP
#define MACHINE_FLEET__SRC__MESSAGES__MESSAGE_UTILS_HPP

#include <machine_fleet/messages/DeviceMode.hpp>
#include <machine_fleet/messages/MachineRequest.hpp>
#include <machine_fleet/messages/MachineState.hpp>
#include <machine_fleet/messages/StationRequest.hpp>
#include <machine_fleet/messages/StationState.hpp>

#include "FleetMessages.h"

namespace machine_fleet {
namespace messages {

void convert(const DeviceMode& _input, MachineFleetData_DeviceMode& _output);

void convert(const MachineFleetData_DeviceMode& _input, DeviceMode& _output);

void convert(const MachineRequest& _input, MachineFleetData_MachineRequest& _output);

void convert(const MachineFleetData_MachineRequest& _input, MachineRequest& _output);

void convert(const MachineState& _input, MachineFleetData_MachineState& _output);

void convert(const MachineFleetData_MachineState& _input, MachineState& _output);

void convert(const StationRequest& _input, MachineFleetData_StationRequest& _output);

void convert(const MachineFleetData_StationRequest& _input, StationRequest& _output);

void convert(const StationState& _input, MachineFleetData_StationState& _output);

void convert(const MachineFleetData_StationState& _input, StationState& _output);
} // namespace 
} // namespace machine_fleet

#endif // MACHINE_FLEET__SRC__MESSAGES__MESSAGE_UTILS_HPP
