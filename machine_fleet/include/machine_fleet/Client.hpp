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

#ifndef MACHINE_FLEET__INCLUDE__MACHINE_FLEET__CLIENT_HPP
#define MACHINE_FLEET__INCLUDE__MACHINE_FLEET__CLIENT_HPP

#include <memory>

#include <machine_fleet/ClientConfig.hpp>

#include <machine_fleet/messages/MachineState.hpp>
#include <machine_fleet/messages/DeliveryRequest.hpp>
#include <machine_fleet/messages/MachineRequest.hpp>
#include <machine_fleet/messages/StationRequest.hpp>

namespace machine_fleet {

class Client
{
public:

  using SharedPtr = std::shared_ptr<Client>;

  /// Factory function that creates an instance of the Charger Fleet DDS Client.
  ///
  /// \param[in] config
  ///   Configuration that sets up the client to communicate with the server.
  /// \return
  ///   Shared pointer to a machine fleet client.
  static SharedPtr make(const ClientConfig& config);

  /// Attempts to send a new machine state to the machine fleet server, to be 
  /// registered by the fleet management system.
  ///
  /// \param[in] new_machine_state
  ///   Current machine state to be sent to the machine fleet server to update the
  ///   fleet management system.
  /// \return
  ///   True if machine state was successfully sent, false otherwise.
  bool send_machine_state(const messages::MachineState& new_machine_state);

  /// Attempts to send a new delivery request to the machine fleet server, to be 
  /// registered by the fleet management system.
  ///
  /// \param[in] new_delivery_request
  ///   New delivery request to be sent out to the server
  /// \return
  ///   True if delivery request was successfully sent, false otherwise.
  bool send_delivery_request(const messages::DeliveryRequest& new_delivery_request);

  /// Attempts to read and receive a new machine request from the machine fleet
  /// server, for commanding the machine client.
  ///
  /// \param[out] machine_request
  ///   Newly received machine request from the machine fleet server, to be
  ///   handled by the machine client.
  /// \return
  ///   True if a new machine request was received, false otherwise.
  bool read_machine_request(messages::MachineRequest& machine_request);

  /// Attempts to read and receive a new station request from the machine fleet
  /// server, for commanding the station client.
  ///
  /// \param[out] station_request
  ///   Newly received station request from the station fleet server, to be
  ///   handled by the station client.
  /// \return
  ///   True if a new station request was received, false otherwise.
  bool read_station_request(messages::StationRequest& station_request);

  /// Destructor
  ~Client();

private:

  /// Forward declaration and unique implementation
  class ClientImpl;

  std::unique_ptr<ClientImpl> impl;

  Client(const ClientConfig& config);

};

} // namespace machine_fleet

#endif // MACHINE_FLEET__INCLUDE__MACHINE_FLEET__CLIENT_HPP
