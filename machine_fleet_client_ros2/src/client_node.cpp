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

#include <iostream>
#include <exception>
#include <thread>

#include <rcl/time.h>
#include <rclcpp/rclcpp.hpp>
#include <machine_fleet_msgs/srv/machine_cart.hpp>

#include "machine_fleet/ros2/client_node.hpp"
#include "machine_fleet/ros2/client_node_config.hpp"

namespace machine_fleet
{
namespace ros2
{
ClientNode::ClientNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("machine_fleet_client_ros2", options)
{
  /// Starting the machine fleet client
  RCLCPP_INFO(get_logger(), "Greetings from %s", get_name());

  // parameter declarations
  declare_parameter("fleet_name", client_node_config.fleet_name);
  declare_parameter("machine_name", client_node_config.machine_name);
  // defaults declared in header
  declare_parameter("machine_state_topic", client_node_config.machine_state_topic);
  declare_parameter("delivery_request_topic", client_node_config.delivery_request_topic);
  declare_parameter("station_request_topic", client_node_config.station_request_topic);
  declare_parameter("machine_trigger_server_name", client_node_config.machine_trigger_server_name);
  declare_parameter("dds_domain", client_node_config.dds_domain);
  declare_parameter("dds_state_topic", client_node_config.dds_state_topic);
  declare_parameter("dds_delivery_request_topic", client_node_config.dds_delivery_request_topic);
  declare_parameter("dds_machine_request_topic", client_node_config.dds_machine_request_topic);
  declare_parameter("dds_station_request_topic", client_node_config.dds_station_request_topic);
  declare_parameter("wait_timeout", client_node_config.wait_timeout);
  declare_parameter("update_frequency", client_node_config.update_frequency);
  declare_parameter("publish_frequency", client_node_config.publish_frequency);

  // getting new values for parameters or keep defaults
  get_parameter("fleet_name", client_node_config.fleet_name);
  get_parameter("machine_name", client_node_config.machine_name);
  get_parameter("machine_state_topic", client_node_config.machine_state_topic);
  get_parameter("delivery_request_topic", client_node_config.delivery_request_topic);
  get_parameter("station_request_topic", client_node_config.station_request_topic);
  get_parameter("machine_trigger_server_name", client_node_config.machine_trigger_server_name);
  get_parameter("dds_domain", client_node_config.dds_domain);
  get_parameter("dds_state_topic", client_node_config.dds_state_topic);
  get_parameter("dds_delivery_request_topic", client_node_config.dds_delivery_request_topic);
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
  rclcpp::Client<machine_fleet_msgs::srv::MachineCart>::SharedPtr machine_trigger_client = nullptr;
  if (client_node_config.machine_trigger_server_name != "") {
    machine_trigger_client = create_client<machine_fleet_msgs::srv::MachineCart>(
      client_node_config.machine_trigger_server_name);
    RCLCPP_INFO(
      get_logger(), "waiting for connection with trigger server: %s",
      client_node_config.machine_trigger_server_name.c_str());
    while (!machine_trigger_client->wait_for_service(
        std::chrono::duration<double>(client_node_config.wait_timeout)))
    {
      RCLCPP_ERROR(
        get_logger(), "timed out waiting for charging trigger server: %s",
        client_node_config.machine_trigger_server_name.c_str());
      if (!rclcpp::ok()) {
        throw std::runtime_error("exited rclcpp while constructing client_node");
      }
    }
  }

  start(
    Fields{
        std::move(client),
        std::move(machine_trigger_client)
      });
}

ClientNode::~ClientNode()
{
}

void ClientNode::start(Fields _fields)
{
  fields = std::move(_fields);

  // Publishers:
  station_request_pub = create_publisher<machine_fleet_msgs::msg::StationRequest>(
    client_node_config.station_request_topic, 10);

  // Subcribers:
  machine_state_sub = create_subscription<machine_fleet_msgs::msg::MachineState>(
    client_node_config.machine_state_topic, rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&ClientNode::machine_state_callback_fn, this, std::placeholders::_1));

  delivery_request_sub = create_subscription<machine_fleet_msgs::msg::DeliveryRequest>(
    client_node_config.delivery_request_topic, rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&ClientNode::delivery_request_callback_fn, this, std::placeholders::_1));

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

void ClientNode::print_config()
{
  client_node_config.print_config();
}

void ClientNode::machine_state_callback_fn(
  const machine_fleet_msgs::msg::MachineState::SharedPtr _msg)
{
  WriteLock machine_state_lock(machine_state_mutex);
  current_machine_state = *_msg;
}

void ClientNode::delivery_request_callback_fn(
  const machine_fleet_msgs::msg::DeliveryRequest::SharedPtr _msg)
{
  machine_fleet::messages::DeliveryRequest new_delivery_request;
  new_delivery_request.machine_name = _msg->machine_name;
  new_delivery_request.fleet_name = _msg->fleet_name;
  new_delivery_request.request_id = _msg->request_id;
  new_delivery_request.station_name = _msg->station_name;
  new_delivery_request.mode.mode = _msg->mode.mode;
  if (!fields.client->send_delivery_request(new_delivery_request)) {
    RCLCPP_WARN(get_logger(), "failed to send delivery request");
  }
}

messages::MachineState ClientNode::get_machine_state()
{
  messages::MachineState machineState;

  /// Checks if machine has just received a request that causes an adapter error
  if (request_error) {
    machineState.mode.mode = messages::MachineMode::MODE_ERROR;
    machineState.error_message = "Request lasted was error!";
  } else {
    ReadLock machine_state_lock(machine_state_mutex);
    machineState.mode.mode = current_machine_state.mode.mode;
    machineState.error_message = current_machine_state.error_message;
  }
  return machineState;
}

void ClientNode::publish_machine_state()
{
  messages::MachineState new_machine_state;
  new_machine_state.fleet_name = client_node_config.fleet_name;
  new_machine_state.machine_name = client_node_config.machine_name;

  {
    ReadLock request_id_lock(request_id_mutex);
    new_machine_state.request_id = current_request_id;
  }

  machine_fleet::messages::MachineState machineState;
  machineState = get_machine_state();

  new_machine_state.error_message = machineState.error_message;
  new_machine_state.mode.mode = machineState.mode.mode;

  if (!fields.client->send_machine_state(new_machine_state)) {
    RCLCPP_WARN(get_logger(), "failed to send machine state");
  }
}

bool ClientNode::is_valid_request(
  const std::string & _request_fleet_name,
  const std::string & _request_machine_name,
  const std::string & _request_id)
{
  ReadLock request_id_lock(request_id_mutex);
  if (current_request_id == _request_id ||
    client_node_config.machine_name != _request_machine_name ||
    client_node_config.fleet_name != _request_fleet_name)
  {
    return false;
  }
  return true;
}

bool ClientNode::read_machine_request()
{
  messages::MachineRequest machine_request;
  if (fields.client->read_machine_request(machine_request) &&
      is_valid_request(
          machine_request.fleet_name,
          machine_request.machine_name,
          machine_request.request_id))
  {
    if ((machine_request.mode.mode == messages::MachineMode::MODE_RELEASE) ||
        (machine_request.mode.mode == messages::MachineMode::MODE_CLAMP) )
    {
      if (machine_request.mode.mode == messages::MachineMode::MODE_RELEASE) {
        RCLCPP_INFO(get_logger(), "received a RELEASE command.");
      } else {
        RCLCPP_INFO(get_logger(), "received a CLAMP command.");
      }
      
      if (fields.machine_trigger_client &&
        fields.machine_trigger_client->service_is_ready())
      {
        using ServiceResponseFuture =
          rclcpp::Client<machine_fleet_msgs::srv::MachineCart>::SharedFuture;
        auto response_received_callback = [&](ServiceResponseFuture future) {
          auto response = future.get();
          if (!response->success)
          {
            RCLCPP_ERROR(get_logger(), "Failed to request machine, message: %s!",
              response->message.c_str());
            request_error = true;
          } else {
            request_error = false;
          }
        };
        auto machine_srv = std::make_shared<machine_fleet_msgs::srv::MachineCart::Request>();
        machine_srv->mode = machine_request.mode.mode;

        // sync call would block indefinelty as we are in a spinning node
        fields.machine_trigger_client->async_send_request(machine_srv, response_received_callback);
      }

    } else {
      RCLCPP_ERROR(get_logger(), "received an INVALID/UNSUPPORTED command: %d.",
              machine_request.mode.mode);
      request_error = true;
    }
    
    WriteLock request_id_lock(request_id_mutex);
    current_request_id = machine_request.request_id;

    return true;
  }
  return false;
}

bool ClientNode::read_station_request()
{
  messages::StationRequest station_request;
  if (fields.client->read_station_request(station_request) &&
      is_valid_request(
          station_request.fleet_name,
          station_request.machine_name,
          station_request.request_id))
  {
    if ((station_request.mode.mode == messages::StationMode::MODE_EMPTY) ||
        (station_request.mode.mode == messages::StationMode::MODE_FILLED) )
    {
      machine_fleet_msgs::msg::StationRequest _msg;
      _msg.fleet_name = station_request.fleet_name;
      _msg.machine_name = station_request.machine_name;
      _msg.request_id = station_request.request_id;
      _msg.station_name = station_request.station_name;
      _msg.mode.mode = station_request.mode.mode;
      station_request_pub->publish(_msg);
      request_error = false;
    } else {
      RCLCPP_ERROR(get_logger(), "received an INVALID/UNSUPPORTED command: %d.",
              station_request.mode.mode);
      request_error = true;
    }

    WriteLock request_id_lock(request_id_mutex);
    current_request_id = station_request.request_id;
    return true;
  }
  return false;
}

void ClientNode::read_requests()
{
  if (read_machine_request() ||
      read_station_request())
  {
    return;
  }
}

void ClientNode::handle_requests()
{
}

void ClientNode::update_fn()
{
  read_requests();
}

void ClientNode::publish_fn()
{
  publish_machine_state();
}

} // namespace ros2
} // namespace machine_fleet
