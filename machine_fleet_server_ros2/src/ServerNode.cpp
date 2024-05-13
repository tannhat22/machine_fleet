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

#include <chrono>

#include <Eigen/Geometry>

#include <machine_fleet/Server.hpp>
#include <machine_fleet/ServerConfig.hpp>

#include <machine_fleet/messages/MachineRequest.hpp>
#include <machine_fleet/messages/StationRequest.hpp>


#include "utilities.hpp"
#include "ServerNode.hpp"

namespace machine_fleet
{
namespace ros2
{

ServerNode::SharedPtr ServerNode::make(
    const ServerNodeConfig& _config, const rclcpp::NodeOptions& _node_options)
{
  // Starting the machine fleet server node
  SharedPtr server_node(new ServerNode(_config, _node_options));

  auto start_time = std::chrono::steady_clock::now();
  auto end_time = std::chrono::steady_clock::now();
  while (
      std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time)
          .count() < 10)
  {
    rclcpp::spin_some(server_node);

    server_node->setup_config();
    if (server_node->is_ready())
      break;
    RCLCPP_INFO(
        server_node->get_logger(), "waiting for configuration parameters.");

    end_time = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  if (!server_node->is_ready())
  {
    RCLCPP_ERROR(
        server_node->get_logger(), "unable to initialize parameters.");
    return nullptr;
  }
  server_node->print_config();

  // Starting the machine fleet server
  ServerConfig server_config =
      server_node->server_node_config.get_server_config();
  Server::SharedPtr server = Server::make(server_config);
  if (!server)
    return nullptr;

  server_node->start(Fields{
    std::move(server)
  });

  return server_node;
}

ServerNode::~ServerNode()
{}

ServerNode::ServerNode(
    const ServerNodeConfig& _config,
    const rclcpp::NodeOptions& _node_options) :
  Node(_config.fleet_name + "_node", _node_options),
  server_node_config(_config)
{}

void ServerNode::print_config()
{
  server_node_config.print_config();
}

void ServerNode::setup_config()
{
  get_parameter("fleet_name", server_node_config.fleet_name);
  get_parameter("fleet_state_topic", server_node_config.fleet_state_topic);
  get_parameter("delivery_request_topic", server_node_config.delivery_request_topic);
  get_parameter("machine_request_topic", server_node_config.machine_request_topic);
  get_parameter("station_request_topic", server_node_config.station_request_topic);
  get_parameter("dds_domain", server_node_config.dds_domain);
  get_parameter("dds_machine_state_topic", server_node_config.dds_machine_state_topic);
  get_parameter("dds_delivery_request_topic", server_node_config.dds_delivery_request_topic);
  get_parameter("dds_machine_request_topic", server_node_config.dds_machine_request_topic);
  get_parameter("dds_station_request_topic", server_node_config.dds_station_request_topic);
  get_parameter("update_state_frequency", server_node_config.update_state_frequency);
  get_parameter("publish_state_frequency", server_node_config.publish_state_frequency);
}

bool ServerNode::is_ready()
{
  if (server_node_config.fleet_name == "fleet_name")
    return false;
  return true;
}

void ServerNode::start(Fields _fields)
{
  fields = std::move(_fields);

  {
    WriteLock machine_states_lock(machine_states_mutex);
    machine_states.clear();
  }

  using namespace std::chrono_literals;

  // --------------------------------------------------------------------------
  // First callback group that handles getting updates from all the clients
  // available

  update_state_callback_group = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  update_state_timer = create_wall_timer(
      100ms, std::bind(&ServerNode::update_state_callback, this),
      update_state_callback_group);

  // --------------------------------------------------------------------------
  // Second callback group that handles publishing fleet states to RMF, and
  // handling requests from RMF to be sent down to the clients

  fleet_state_pub_callback_group = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  fleet_state_pub =
      create_publisher<machine_fleet_msgs::msg::FleetMachineState>(
          server_node_config.fleet_state_topic, 10);

  delivery_request_pub =
      create_publisher<machine_fleet_msgs::msg::DeliveryRequest>(
          server_node_config.delivery_request_topic, 10);

  fleet_state_pub_timer = create_wall_timer(
      std::chrono::seconds(1) / server_node_config.publish_state_frequency,
      std::bind(&ServerNode::publish_fleet_state, this),
      fleet_state_pub_callback_group);

  // --------------------------------------------------------------------------
  // Mode request handling

  auto machine_request_sub_opt = rclcpp::SubscriptionOptions();

  machine_request_sub_opt.callback_group = fleet_state_pub_callback_group;

  machine_request_sub = create_subscription<machine_fleet_msgs::msg::MachineRequest>(
      server_node_config.machine_request_topic, rclcpp::QoS(10),
      [&](machine_fleet_msgs::msg::MachineRequest::UniquePtr msg)
      {
        handle_machine_request(std::move(msg));
      },
      machine_request_sub_opt);

  // --------------------------------------------------------------------------
  // Station request handling

  auto station_request_sub_opt = rclcpp::SubscriptionOptions();

  station_request_sub_opt.callback_group = fleet_state_pub_callback_group;

  station_request_sub = create_subscription<machine_fleet_msgs::msg::StationRequest>(
      server_node_config.station_request_topic, rclcpp::QoS(10),
      [&](machine_fleet_msgs::msg::StationRequest::UniquePtr msg)
      {
        handle_station_request(std::move(msg));
      },
      station_request_sub_opt);
}

bool ServerNode::is_request_valid(
    const std::string& _fleet_name, const std::string& _machine_name)
{
  if (_fleet_name != server_node_config.fleet_name)
    return false;

  ReadLock machine_states_lock(machine_states_mutex);
  auto it = machine_states.find(_machine_name);
  if (it == machine_states.end())
    return false;
  return true;
}

void ServerNode::handle_machine_request(
    machine_fleet_msgs::msg::MachineRequest::UniquePtr _msg)
{
  messages::MachineRequest mf_msg;
  to_mf_message(*(_msg.get()), mf_msg);
  fields.server->send_machine_request(mf_msg);
}

void ServerNode::handle_station_request(
    machine_fleet_msgs::msg::StationRequest::UniquePtr _msg)
{
  messages::StationRequest mf_msg;
  to_mf_message(*(_msg.get()), mf_msg);
  fields.server->send_station_request(mf_msg);
}

void ServerNode::update_state_callback()
{
  // Update State
  std::vector<messages::MachineState> new_machine_states;
  fields.server->read_machine_states(new_machine_states);

  for (const messages::MachineState& mf_ms : new_machine_states)
  {
    machine_fleet_msgs::msg::MachineState ros_ms;
    to_ros_message(mf_ms, ros_ms);

    WriteLock machine_states_lock(machine_states_mutex);
    auto it = machine_states.find(ros_ms.machine_name);
    if (it == machine_states.end())
      RCLCPP_INFO(
          get_logger(),
          "registered a new machine: [%s]",
          ros_ms.machine_name.c_str());

    machine_states[ros_ms.machine_name] = ros_ms;
  }

  // Read Delivery Request
  std::vector<messages::DeliveryRequest> new_delivery_requests;
  fields.server->read_delivery_requests(new_delivery_requests);

  for (const messages::DeliveryRequest& mf_dr : new_delivery_requests)
  {
    machine_fleet_msgs::msg::DeliveryRequest ros_dr;
    to_ros_message(mf_dr, ros_dr);

    if (is_request_valid(ros_dr.fleet_name,
                         ros_dr.machine_name))
    {
      RCLCPP_INFO(
          get_logger(),
          "receive a new delivery request from `[%s]`",
          ros_dr.machine_name.c_str());
      delivery_request_pub->publish(ros_dr);
    }
  }
}

void ServerNode::publish_fleet_state()
{
  machine_fleet_msgs::msg::FleetMachineState fleet_state;
  fleet_state.name = server_node_config.fleet_name;
  fleet_state.machines.clear();

  ReadLock machine_states_lock(machine_states_mutex);
  for (const auto it : machine_states)
  {
    const auto fleet_frame_rs = it.second;
    machine_fleet_msgs::msg::MachineState rmf_frame_ms;

    rmf_frame_ms.machine_name = fleet_frame_rs.machine_name;
    rmf_frame_ms.fleet_name = fleet_frame_rs.fleet_name;
    rmf_frame_ms.error_message = fleet_frame_rs.error_message;
    rmf_frame_ms.request_id = fleet_frame_rs.request_id;
    rmf_frame_ms.mode.mode = fleet_frame_rs.mode.mode;
    fleet_state.machines.push_back(rmf_frame_ms);
  }
  fleet_state_pub->publish(fleet_state);
}

} // namespace ros2
} // namespace machine_fleet
