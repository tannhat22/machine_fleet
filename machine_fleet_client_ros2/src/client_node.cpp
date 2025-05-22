/*
 * Copyright (C) 2019 Open Source machineics Foundation
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

#include <exception>
#include <iostream>
#include <thread>

#include <rcl/time.h>
#include <rclcpp/rclcpp.hpp>

#include "machine_fleet/ros2/client_node.hpp"
#include "machine_fleet/ros2/client_node_config.hpp"

namespace machine_fleet {
namespace ros2 {
ClientNode::ClientNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("machine_fleet_client_ros2", options) {
  /// Starting the machine fleet client
  RCLCPP_INFO(get_logger(), "Greetings from %s", get_name());

  // parameter declarations
  // declare_parameter("fleet_name", client_node_config.fleet_name);
  declare_parameter("machine_name", client_node_config.machine_name);
  // defaults declared in header
  declare_parameter("machine_state_topic", client_node_config.machine_state_topic);
  declare_parameter("station_request_topic", client_node_config.station_request_topic);
  declare_parameter("machine_service_name", client_node_config.machine_service_name);
  declare_parameter("dds_domain", client_node_config.dds_domain);
  declare_parameter("dds_state_topic", client_node_config.dds_state_topic);
  declare_parameter("dds_machine_request_topic", client_node_config.dds_machine_request_topic);
  declare_parameter("dds_station_request_topic", client_node_config.dds_station_request_topic);
  declare_parameter("wait_timeout", client_node_config.wait_timeout);
  declare_parameter("update_frequency", client_node_config.update_frequency);
  declare_parameter("publish_frequency", client_node_config.publish_frequency);

  // getting new values for parameters or keep defaults
  // get_parameter("fleet_name", client_node_config.fleet_name);
  get_parameter("machine_name", client_node_config.machine_name);
  get_parameter("machine_state_topic", client_node_config.machine_state_topic);
  get_parameter("station_request_topic", client_node_config.station_request_topic);
  get_parameter("machine_service_name", client_node_config.machine_service_name);
  get_parameter("dds_domain", client_node_config.dds_domain);
  get_parameter("dds_state_topic", client_node_config.dds_state_topic);
  get_parameter("dds_machine_request_topic", client_node_config.dds_machine_request_topic);
  get_parameter("dds_station_request_topic", client_node_config.dds_station_request_topic);
  get_parameter("wait_timeout", client_node_config.wait_timeout);
  get_parameter("update_frequency", client_node_config.update_frequency);
  get_parameter("publish_frequency", client_node_config.publish_frequency);
  print_config();

  ClientConfig client_config = client_node_config.get_client_config();
  Client::SharedPtr client = Client::make(client_config);
  if (!client) {
    throw std::runtime_error("Unable to create machine_fleet Client from config.");
  }

  /// Setting up the machine server client, if required, wait for server
  rclcpp::Client<machine_fleet_msgs::srv::Machine>::SharedPtr machine_service_client = nullptr;
  if (client_node_config.machine_service_name != "") {
    machine_service_client =
        create_client<machine_fleet_msgs::srv::Machine>(client_node_config.machine_service_name);
    RCLCPP_INFO(get_logger(), "waiting for connection with machine_service server: %s",
                client_node_config.machine_service_name.c_str());
    while (!machine_service_client->wait_for_service(
        std::chrono::duration<double>(client_node_config.wait_timeout))) {
      RCLCPP_ERROR(get_logger(), "timed out waiting for machine_service server: %s",
                   client_node_config.machine_service_name.c_str());
      if (!rclcpp::ok()) {
        throw std::runtime_error("exited rclcpp while constructing client_node");
      }
    }
  }

  start(Fields{std::move(client), std::move(machine_service_client)});
}

ClientNode::~ClientNode() {}

void ClientNode::start(Fields _fields) {
  fields = std::move(_fields);

  // Publishers:
  station_request_pub = create_publisher<machine_fleet_msgs::msg::StationRequest>(
      client_node_config.station_request_topic, 10);

  // Subcribers:
  machine_state_sub = create_subscription<machine_fleet_msgs::msg::MachineState>(
      client_node_config.machine_state_topic, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&ClientNode::machine_state_callback_fn, this, std::placeholders::_1));

  request_error = false;

  RCLCPP_INFO(get_logger(), "starting update timer.");
  std::chrono::duration<double> update_period =
      std::chrono::duration<double>(1.0 / client_node_config.update_frequency);
  update_timer = create_wall_timer(update_period, std::bind(&ClientNode::update_fn, this));

  RCLCPP_INFO(get_logger(), "starting publish timer.");
  std::chrono::duration<double> publish_period =
      std::chrono::duration<double>(1.0 / client_node_config.publish_frequency);
  publish_timer = create_wall_timer(publish_period, std::bind(&ClientNode::publish_fn, this));
}

void ClientNode::print_config() { client_node_config.print_config(); }

void ClientNode::machine_state_callback_fn(
    const machine_fleet_msgs::msg::MachineState::SharedPtr _msg) {
  WriteLock machine_state_lock(machine_state_mutex);
  current_machine_state = *_msg;
}

messages::MachineState ClientNode::get_machine_state() {
  messages::MachineState machineState;

  /// Checks if machine has just received a request that causes an adapter error
  if (request_error) {
    machineState.machine_mode = messages::MachineState::MODE_ERROR;
  } else {
    machineState.machine_mode = current_machine_state.machine_mode;
  }
  return machineState;
}

void ClientNode::publish_machine_state() {
  rclcpp::Time now = get_clock()->now();
  messages::MachineState new_machine_state;
  new_machine_state.sec = static_cast<int32_t>(now.nanoseconds() / 1000000000);
  new_machine_state.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000);
  new_machine_state.machine_name = client_node_config.machine_name;
  {
    ReadLock request_id_lock(request_id_mutex);
    new_machine_state.dispenser_request_id = current_dispenser_request_id;
    new_machine_state.ingestor_request_id = current_ingestor_request_id;
  }

  ReadLock machine_state_lock(machine_state_mutex);
  {
    machine_fleet::messages::MachineState machineState;
    machineState = get_machine_state();
    new_machine_state.machine_mode = machineState.machine_mode;
    new_machine_state.request_pickup = current_machine_state.request_pickup;
    new_machine_state.dispenser_mode.mode = current_machine_state.dispenser_mode.mode;
    new_machine_state.request_dropoff = current_machine_state.request_dropoff;
    new_machine_state.ingestor_mode.mode = current_machine_state.ingestor_mode.mode;

    new_machine_state.station_states.clear();
    for (size_t i = 0; i < current_machine_state.station_states.size(); ++i) {
      new_machine_state.station_states.push_back(
          messages::StationState{current_machine_state.station_states[i].station_name,
                                 current_machine_state.station_states[i].mode});
    }
  }

  if (!fields.client->send_machine_state(new_machine_state)) {
    RCLCPP_WARN(get_logger(), "failed to send machine state");
  }
}

bool ClientNode::is_valid_request(const std::string &_request_machine_name,
                                  const std::string &_request_id, const uint8_t &_is_dispenser) {
  ReadLock request_id_lock(request_id_mutex);
  std::string current_request_id;
  if (_is_dispenser == messages::MachineRequest::REQUEST_DISPENSER)
    current_request_id = current_dispenser_request_id;
  else
    current_request_id = current_ingestor_request_id;

  if (current_request_id == _request_id ||
      client_node_config.machine_name != _request_machine_name) {
    return false;
  }
  return true;
}

bool ClientNode::is_valid_request(const std::string &_request_machine_name) {
  if (client_node_config.machine_name != _request_machine_name) {
    return false;
  }
  return true;
}

bool ClientNode::read_machine_request() {
  messages::MachineRequest machine_request;
  if (fields.client->read_machine_request(machine_request) &&
      is_valid_request(machine_request.machine_name, machine_request.request_id,
                       machine_request.request_type)) {
    if ((machine_request.request_mode.mode == messages::DeviceMode::MODE_IDLE) ||
        (machine_request.request_mode.mode == messages::DeviceMode::MODE_ACCEPT_DOCKIN) ||
        (machine_request.request_mode.mode == messages::DeviceMode::MODE_ROBOT_DOCKED_IN) ||
        (machine_request.request_mode.mode == messages::DeviceMode::MODE_ACCEPT_DOCKOUT) ||
        (machine_request.request_mode.mode == messages::DeviceMode::MODE_CANCEL) ||
        (machine_request.request_mode.mode == messages::DeviceMode::MODE_ROBOT_ERROR)) {
      WriteLock machine_state_lock(machine_state_mutex);
      RCLCPP_INFO(get_logger(),
                  "received a machine_request command: request_type: %d, request_mode: %d",
                  machine_request.request_type, machine_request.request_mode.mode);

      if (fields.machine_service_client && fields.machine_service_client->service_is_ready()) {
        using ServiceResponseFuture =
            rclcpp::Client<machine_fleet_msgs::srv::Machine>::SharedFuture;
        auto response_received_callback = [&](ServiceResponseFuture future) {
          auto response = future.get();
          if (!response->success) {
            RCLCPP_ERROR(get_logger(), "Failed to request machine, message: %s!",
                         response->message.c_str());
            request_error = true;
          } else {
            request_error = false;
          }
        };
        auto machine_srv = std::make_shared<machine_fleet_msgs::srv::Machine::Request>();
        machine_srv->request_type = machine_request.request_type;
        machine_srv->request_mode.mode = machine_request.request_mode.mode;

        // sync call would block indefinelty as we are in a spinning node
        fields.machine_service_client->async_send_request(machine_srv, response_received_callback);
      }
    } else {
      RCLCPP_ERROR(get_logger(), "received an INVALID/UNSUPPORTED command: %d.",
                   machine_request.request_mode.mode);
      request_error = true;
    }

    WriteLock request_id_lock(request_id_mutex);
    if (machine_request.request_type == messages::MachineRequest::REQUEST_DISPENSER)
      current_dispenser_request_id = machine_request.request_id;
    else
      current_ingestor_request_id = machine_request.request_id;
    return true;
  }
  return false;
}

bool ClientNode::read_station_request() {
  messages::StationRequest station_request;
  if (fields.client->read_station_request(station_request) &&
      is_valid_request(station_request.machine_name)) {
    if ((station_request.mode == messages::StationRequest::MODE_EMPTY) ||
        (station_request.mode == messages::StationRequest::MODE_FILLED)) {
      machine_fleet_msgs::msg::StationRequest _msg;
      _msg.machine_name = station_request.machine_name;
      _msg.station_name = station_request.station_name;
      _msg.station_type = station_request.station_type;
      _msg.mode = station_request.mode;
      station_request_pub->publish(_msg);
      request_error = false;
    } else {
      RCLCPP_ERROR(get_logger(), "received an INVALID/UNSUPPORTED command: %d.",
                   station_request.mode);
      request_error = true;
    }
    return true;
  }
  return false;
}

void ClientNode::read_requests() {
  if (read_machine_request() || read_station_request()) {
    return;
  }
}

void ClientNode::handle_requests() {}

void ClientNode::update_fn() { read_requests(); }

void ClientNode::publish_fn() { publish_machine_state(); }

} // namespace ros2
} // namespace machine_fleet
