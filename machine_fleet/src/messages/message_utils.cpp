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

void convert(const DeviceMode& _input, MachineFleetData_DeviceMode& _output)
{
  // Consequently, machine fleet delivery modes need to be ordered similarly as 
  // delivery modes.
  _output.mode = _input.mode;
}

void convert(const MachineFleetData_DeviceMode& _input, DeviceMode& _output)
{
  // Consequently, machine fleet delivery modes need to be ordered similarly as 
  // delivery modes.
  _output.mode = _input.mode;
}

void convert(const StationRequest& _input, MachineFleetData_StationRequest& _output)
{
  _output.machine_name = common::dds_string_alloc_and_copy(_input.machine_name);
  _output.station_name = common::dds_string_alloc_and_copy(_input.station_name);
  _output.station_type = _input.station_type;
  _output.mode = _input.mode;
}

void convert(const MachineFleetData_StationRequest& _input, StationRequest& _output)
{
  _output.machine_name =  std::string(_input.machine_name);
  _output.station_name = std::string(_input.station_name);
  _output.station_type = _input.station_type;
  _output.mode = _input.mode;
}

void convert(const StationState& _input, MachineFleetData_StationState& _output)
{
  // Consequently, machine fleet station modes need to be ordered similarly as 
  // station modes.
  _output.station_name = common::dds_string_alloc_and_copy(_input.station_name);
  _output.mode = _input.mode;
}

void convert(const MachineFleetData_StationState& _input, StationState& _output)
{
  // Consequently, machine fleet station modes need to be ordered similarly as 
  // station modes.
  _output.station_name = std::string(_input.station_name);
  _output.mode = _input.mode;
}

void convert(const MachineRequest& _input, MachineFleetData_MachineRequest& _output)
{
  _output.machine_name = common::dds_string_alloc_and_copy(_input.machine_name);
  _output.request_type = _input.request_type;
  convert(_input.mode, _output.mode);
  _output.request_id = common::dds_string_alloc_and_copy(_input.request_id);
}

void convert(const MachineFleetData_MachineRequest& _input, MachineRequest& _output)
{
  _output.machine_name =  std::string(_input.machine_name);
  _output.request_type = _input.request_type;
  convert(_input.mode, _output.mode);
  _output.request_id = std::string(_input.request_id);
}

void convert(const MachineState& _input, MachineFleetData_MachineState& _output)
{
  _output.machine_name = common::dds_string_alloc_and_copy(_input.machine_name);
  _output.machine_mode = _input.machine_mode;
  _output.request_pickup = _input.request_pickup;
  convert(_input.dispenser_mode, _output.dispenser_mode);
  _output.dispenser_request_id = common::dds_string_alloc_and_copy(_input.dispenser_request_id);
  _output.request_dropoff = _input.request_dropoff;
  convert(_input.ingestor_mode, _output.ingestor_mode);
  _output.ingestor_request_id = common::dds_string_alloc_and_copy(_input.ingestor_request_id);

  size_t station_states_length = _input.station_states.size();
  _output.station_states._maximum = static_cast<uint32_t>(station_states_length);
  _output.station_states._length = static_cast<uint32_t>(station_states_length);
  _output.station_states._buffer = 
      MachineFleetData_MachineState_station_states_seq_allocbuf(station_states_length);
  for (size_t i = 0; i < station_states_length; ++i)
    convert(_input.station_states[i], _output.station_states._buffer[i]);
}

void convert(const MachineFleetData_MachineState& _input, MachineState& _output)
{
  _output.machine_name = std::string(_input.machine_name);
  _output.machine_mode = _input.machine_mode;
  _output.request_pickup = _input.request_pickup;
  convert(_input.dispenser_mode, _output.dispenser_mode);
  _output.dispenser_request_id = std::string(_input.dispenser_request_id);
  _output.request_dropoff = _input.request_dropoff;
  convert(_input.ingestor_mode, _output.ingestor_mode);
  _output.ingestor_request_id = std::string(_input.ingestor_request_id);

  _output.station_states.clear();
  for (uint32_t i = 0; i < _input.station_states._length; ++i)
  {
    StationState tmp;
    convert(_input.station_states._buffer[i], tmp);
    _output.station_states.push_back(tmp);
  }
}

} // namespace messages
} // namespace machine_fleet
