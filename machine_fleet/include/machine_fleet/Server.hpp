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

#ifndef MACHINE_FLEET__INCLUDE__MACHINE_FLEET__SERVER_HPP
#define MACHINE_FLEET__INCLUDE__MACHINE_FLEET__SERVER_HPP

#include <memory>
#include <vector>

#include <machine_fleet/ServerConfig.hpp>

#include <machine_fleet/messages/MachineState.hpp>
#include <machine_fleet/messages/DeliveryRequest.hpp>
#include <machine_fleet/messages/MachineRequest.hpp>
#include <machine_fleet/messages/StationRequest.hpp>

namespace machine_fleet {

class Server
{
public:

  using SharedPtr = std::shared_ptr<Server>;

  /// Factory function that creates an instance of the Machine Fleet Server.
  ///
  /// \param[in] config
  ///   Configuration that sets up the server to communicate with the clients.
  /// \return
  ///   Shared pointer to a machine fleet server.
  static SharedPtr make(const ServerConfig& config);

  /// Attempts to read new incoming machine states sent by machine fleet clients
  /// over DDS.
  ///
  /// \param[out] new_machine_states
  ///   A vector of new incoming machine states sent by clients to update the
  ///   fleet management system.
  /// \return
  ///   True if new machine states were received, false otherwise.
  bool read_machine_states(std::vector<messages::MachineState>& new_machine_states);

  /// Attempts to read new incoming delivery requests sent by machine fleet clients
  /// over DDS.
  ///
  /// \param[out] new_delivery_requests
  ///   A vector of new incoming delivery requests sent by clients to update the
  ///   fleet management system.
  /// \return
  ///   True if new delivery requests were received, false otherwise.
  bool read_delivery_requests(std::vector<messages::DeliveryRequest>& new_delivery_states);

  /// Attempts to send a new machine request to all the clients. Clients are in
  /// charge to identify if requests are targetted towards them.
  /// 
  /// \param[in] machine_request
  ///   New machine request to be sent out to the clients.
  /// \return
  ///   True if the machine request was successfully sent, false otherwise.
  bool send_machine_request(const messages::MachineRequest& machine_request);

  /// Attempts to send a new station request to all the clients. Clients are in
  /// charge to identify if requests are targetted towards them.
  /// 
  /// \param[in] station_request
  ///   New station request to be sent out to the clients.
  /// \return
  ///   True if the station request was successfully sent, false otherwise.
  bool send_station_request(const messages::StationRequest& station_request);

  /// Destructor
  ~Server();

private:

  /// Forward declaration and unique implementation
  class ServerImpl;

  std::unique_ptr<ServerImpl> impl;

  Server(const ServerConfig& config);

};

} // namespace machine_fleet

#endif // MACHINE_FLEET__INCLUDE__MACHINE_FLEET__SERVER_HPP
