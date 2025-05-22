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

#include "utilities.hpp"

namespace machine_fleet {
namespace ros2 {

void to_mf_message(const machine_fleet_msgs::msg::MachineRequest &_in_msg,
                   messages::MachineRequest &_out_msg) {
  _out_msg.machine_name = _in_msg.machine_name;
  _out_msg.request_type = _in_msg.request_type;
  _out_msg.request_mode.mode = _in_msg.request_mode.mode;
  _out_msg.request_id = _in_msg.request_id;
}

void to_mf_message(const machine_fleet_msgs::msg::StationRequest &_in_msg,
                   messages::StationRequest &_out_msg) {
  _out_msg.machine_name = _in_msg.machine_name;
  _out_msg.station_name = _in_msg.station_name;
  _out_msg.station_type = _in_msg.station_type;
  _out_msg.mode = _in_msg.mode;
}

void to_ros_message(const messages::StationState &_in_msg,
                    machine_fleet_msgs::msg::StationState &_out_msg) {
  _out_msg.station_name = _in_msg.station_name;
  _out_msg.mode = _in_msg.mode;
}

void to_ros_message(const messages::MachineState &_in_msg,
                    machine_fleet_msgs::msg::MachineState &_out_msg) {
  _out_msg.machine_time.sec = _in_msg.sec;
  _out_msg.machine_time.nanosec = _in_msg.nanosec;
  _out_msg.machine_name = _in_msg.machine_name;
  _out_msg.machine_mode = _in_msg.machine_mode;
  _out_msg.request_pickup = _in_msg.request_pickup;
  _out_msg.dispenser_mode.mode = _in_msg.dispenser_mode.mode;
  _out_msg.dispenser_request_id = _in_msg.dispenser_request_id;
  _out_msg.request_dropoff = _in_msg.request_dropoff;
  _out_msg.ingestor_mode.mode = _in_msg.ingestor_mode.mode;
  _out_msg.ingestor_request_id = _in_msg.ingestor_request_id;

  _out_msg.station_states = {};
  for (size_t i = 0; i < _in_msg.station_states.size(); ++i) {
    machine_fleet_msgs::msg::StationState tmp_state;
    to_ros_message(_in_msg.station_states[i], tmp_state);
    _out_msg.station_states.push_back(tmp_state);
  }
}

} // namespace ros2
} // namespace machine_fleet
