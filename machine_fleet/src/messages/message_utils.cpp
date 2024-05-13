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

#include <dds/dds.h>

#include "../dds_utils/common.hpp"

#include "message_utils.hpp"

namespace machine_fleet {
namespace messages {

void convert(const DeliveryMode& _input, MachineFleetData_DeliveryMode& _output)
{
  // Consequently, machine fleet delivery modes need to be ordered similarly as 
  // delivery modes.
  _output.mode = _input.mode;
}

void convert(const MachineFleetData_DeliveryMode& _input, DeliveryMode& _output)
{
  // Consequently, machine fleet delivery modes need to be ordered similarly as 
  // delivery modes.
  _output.mode = _input.mode;
}

void convert(const DeliveryRequest& _input, MachineFleetData_DeliveryRequest& _output)
{
  _output.machine_name = common::dds_string_alloc_and_copy(_input.machine_name);
  _output.fleet_name = common::dds_string_alloc_and_copy(_input.fleet_name);
  _output.station_name = common::dds_string_alloc_and_copy(_input.station_name);
  _output.request_id = common::dds_string_alloc_and_copy(_input.request_id);
  convert(_input.mode, _output.mode);
}

void convert(const MachineFleetData_DeliveryRequest& _input, DeliveryRequest& _output)
{
  _output.machine_name =  std::string(_input.machine_name);
  _output.fleet_name = std::string(_input.fleet_name);
  _output.station_name = std::string(_input.station_name);
  _output.request_id = std::string(_input.request_id);
  convert(_input.mode, _output.mode);
}

void convert(const MachineMode& _input, MachineFleetData_MachineMode& _output)
{
  // Consequently, machine fleet machine modes need to be ordered similarly as 
  // machine modes.
  _output.mode = _input.mode;
}

void convert(const MachineFleetData_MachineMode& _input, MachineMode& _output)
{
  // Consequently, machine fleet machine modes need to be ordered similarly as 
  // machine modes.
  _output.mode = _input.mode;
}

void convert(const MachineRequest& _input, MachineFleetData_MachineRequest& _output)
{
  _output.machine_name = common::dds_string_alloc_and_copy(_input.machine_name);
  _output.fleet_name = common::dds_string_alloc_and_copy(_input.fleet_name);
  _output.request_id = common::dds_string_alloc_and_copy(_input.request_id);
  convert(_input.mode, _output.mode);
}

void convert(const MachineFleetData_MachineRequest& _input, MachineRequest& _output)
{
  _output.machine_name =  std::string(_input.machine_name);
  _output.fleet_name = std::string(_input.fleet_name);
  _output.request_id = std::string(_input.request_id);
  convert(_input.mode, _output.mode);
}

void convert(const MachineState& _input, MachineFleetData_MachineState& _output)
{
  _output.machine_name = common::dds_string_alloc_and_copy(_input.machine_name);
  _output.fleet_name = common::dds_string_alloc_and_copy(_input.fleet_name);
  _output.error_message = common::dds_string_alloc_and_copy(_input.error_message);
  _output.request_id = common::dds_string_alloc_and_copy(_input.request_id);
  convert(_input.mode, _output.mode);
}

void convert(const MachineFleetData_MachineState& _input, MachineState& _output)
{
  _output.machine_name = std::string(_input.machine_name);
  _output.error_message = std::string(_input.error_message);
  _output.request_id = std::string(_input.request_id);
  _output.fleet_name = std::string(_input.fleet_name);
  convert(_input.mode, _output.mode);
}

void convert(const StationMode& _input, MachineFleetData_StationMode& _output)
{
  // Consequently, machine fleet station modes need to be ordered similarly as 
  // station modes.
  _output.mode = _input.mode;
}

void convert(const MachineFleetData_StationMode& _input, StationMode& _output)
{
  // Consequently, machine fleet station modes need to be ordered similarly as 
  // station modes.
  _output.mode = _input.mode;
}

void convert(const StationRequest& _input, MachineFleetData_StationRequest& _output)
{
  _output.machine_name = common::dds_string_alloc_and_copy(_input.machine_name);
  _output.fleet_name = common::dds_string_alloc_and_copy(_input.fleet_name);
  _output.request_id = common::dds_string_alloc_and_copy(_input.request_id);
  _output.station_name = common::dds_string_alloc_and_copy(_input.station_name);
  convert(_input.mode, _output.mode);
}

void convert(const MachineFleetData_StationRequest& _input, StationRequest& _output)
{
  _output.machine_name =  std::string(_input.machine_name);
  _output.fleet_name = std::string(_input.fleet_name);
  _output.request_id = std::string(_input.request_id);
  _output.station_name = std::string(_input.station_name);
  convert(_input.mode, _output.mode);
}



} // namespace messages
} // namespace machine_fleet
