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

#ifndef MACHINE_FLEET__ROS2__CLIENTNODE_HPP
#define MACHINE_FLEET__ROS2__CLIENTNODE_HPP

#include <atomic>
#include <deque>
#include <memory>
#include <shared_mutex>
#include <thread>
#include <vector>

#include <machine_fleet_msgs/msg/machine_state.hpp>
#include <machine_fleet_msgs/msg/station_request.hpp>
#include <machine_fleet_msgs/srv/machine.hpp>
#include <rclcpp/rclcpp.hpp>

#include <machine_fleet/Client.hpp>
#include <machine_fleet/ClientConfig.hpp>
#include <machine_fleet/messages/MachineRequest.hpp>
#include <machine_fleet/messages/MachineState.hpp>
#include <machine_fleet/messages/StationRequest.hpp>

#include <machine_fleet/Client.hpp>

#include "machine_fleet/ros2/client_node_config.hpp"

namespace machine_fleet {
namespace ros2 {

class ClientNode : public rclcpp::Node {
public:
  using SharedPtr = std::shared_ptr<ClientNode>;
  using Mutex = std::mutex;
  using ReadLock = std::unique_lock<Mutex>;
  using WriteLock = std::unique_lock<Mutex>;

  explicit ClientNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~ClientNode() override;

  struct Fields {
    /// Machine fleet client
    Client::SharedPtr client;

    // Machine server client
    rclcpp::Client<machine_fleet_msgs::srv::Machine>::SharedPtr machine_service_client;
  };

  void print_config();

private:
  // --------------------------------------------------------------------------
  // Machine state handling
  rclcpp::Subscription<machine_fleet_msgs::msg::MachineState>::SharedPtr machine_state_sub;
  Mutex machine_state_mutex;
  machine_fleet_msgs::msg::MachineState current_machine_state;
  void machine_state_callback_fn(const machine_fleet_msgs::msg::MachineState::SharedPtr msg);

  // --------------------------------------------------------------------------
  // Mode handling
  rclcpp::Publisher<machine_fleet_msgs::msg::StationRequest>::SharedPtr station_request_pub;

  // TODO: conditions to trigger emergency, however this is most likely for
  // indicating emergency within the fleet and not in RMF
  // TODO: figure out a better way to handle multiple triggered modes
  std::atomic<bool> request_error;

  messages::MachineState get_machine_state();

  bool read_machine_request();

  bool read_station_request();

  // Request handling

  bool is_valid_request(const std::string &_request_machine_name, const std::string &_request_id,
                        const uint8_t &_is_dispenser);

  bool is_valid_request(const std::string &_request_machine_name);

  Mutex request_id_mutex;
  std::string current_dispenser_request_id;
  std::string current_ingestor_request_id;

  void read_requests();
  void handle_requests();
  void publish_machine_state();
  // --------------------------------------------------------------------------
  // publish and update functions and timers

  std::shared_ptr<rclcpp::TimerBase> update_timer;
  std::shared_ptr<rclcpp::TimerBase> publish_timer;
  void update_fn();
  void publish_fn();

  // --------------------------------------------------------------------------

  ClientNodeConfig client_node_config;
  Fields fields;

  void start(Fields fields);
};

} // namespace ros2
} // namespace machine_fleet

#endif // MACHINE_FLEET__ROS2__CLIENTNODE_HPP