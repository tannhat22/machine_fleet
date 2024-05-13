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

namespace machine_fleet
{
namespace ros2
{

void to_mf_message(
    const machine_fleet_msgs::msg::MachineRequest& _in_msg, 
    messages::MachineRequest& _out_msg)
{
  _out_msg.machine_name = _in_msg.machine_name;
  _out_msg.fleet_name = _in_msg.fleet_name;
  _out_msg.request_id = _in_msg.request_id;
  _out_msg.mode.mode = _in_msg.mode.mode;
}

void to_mf_message(
    const machine_fleet_msgs::msg::StationRequest& _in_msg, 
    messages::StationRequest& _out_msg)
{
  _out_msg.machine_name = _in_msg.machine_name;
  _out_msg.fleet_name = _in_msg.fleet_name;
  _out_msg.request_id = _in_msg.request_id;
  _out_msg.station_name = _in_msg.station_name;
  _out_msg.mode.mode = _in_msg.mode.mode;
}

void to_ros_message(
    const messages::MachineState& _in_msg,
    machine_fleet_msgs::msg::MachineState& _out_msg)
{
  _out_msg.machine_name = _in_msg.machine_name;
  _out_msg.fleet_name = _in_msg.fleet_name;
  _out_msg.error_message = _in_msg.error_message;
  _out_msg.request_id = _in_msg.request_id;
  _out_msg.mode.mode = _in_msg.mode.mode;
}

void to_ros_message(
    const messages::DeliveryRequest& _in_msg,
    machine_fleet_msgs::msg::DeliveryRequest& _out_msg)
{
  _out_msg.machine_name = _in_msg.machine_name;
  _out_msg.fleet_name = _in_msg.fleet_name;
  _out_msg.station_name = _in_msg.station_name;
  _out_msg.request_id = _in_msg.request_id;
  _out_msg.mode.mode = _in_msg.mode.mode;
}

} // namespace ros2
} // namespace machine_fleet
