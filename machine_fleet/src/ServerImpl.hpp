/*
 * Copyright (C) 2019 Open Source Machineics Foundation
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

#ifndef MACHINE_FLEET__SRC__SERVERIMPL_HPP
#define MACHINE_FLEET__SRC__SERVERIMPL_HPP

#include <machine_fleet/messages/MachineState.hpp>
#include <machine_fleet/messages/DeliveryRequest.hpp>
#include <machine_fleet/messages/MachineRequest.hpp>
#include <machine_fleet/messages/StationRequest.hpp>
#include <machine_fleet/Server.hpp>
#include <machine_fleet/ServerConfig.hpp>

#include <dds/dds.h>

#include "messages/FleetMessages.h"
#include "dds_utils/DDSPublishHandler.hpp"
#include "dds_utils/DDSSubscribeHandler.hpp"

namespace machine_fleet {

class Server::ServerImpl
{
public:

  /// DDS related fields required for the server to operate
  struct Fields
  {
    /// DDS participant that is tied to the configured dds_domain_id
    dds_entity_t participant;

    /// DDS subscribers for new incoming machine states from clients
    dds::DDSSubscribeHandler<MachineFleetData_MachineState, 10>::SharedPtr
        machine_state_sub;

    /// DDS subscribers for new incoming delivery requests from clients
    dds::DDSSubscribeHandler<MachineFleetData_DeliveryRequest, 10>::SharedPtr 
        delivery_request_sub;

    /// DDS publisher for machine requests to be sent to clients
    dds::DDSPublishHandler<MachineFleetData_MachineRequest>::SharedPtr
        machine_request_pub;

    /// DDS publisher for station requests to be sent to clients
    dds::DDSPublishHandler<MachineFleetData_StationRequest>::SharedPtr
        station_request_pub;
  };

  ServerImpl(const ServerConfig& config);

  ~ServerImpl();

  void start(Fields fields);

  bool read_machine_states(std::vector<messages::MachineState>& new_machine_states);

  bool read_delivery_requests(std::vector<messages::DeliveryRequest>& new_delivery_requests);

  bool send_machine_request(const messages::MachineRequest& machine_request);

  bool send_station_request(const messages::StationRequest& station_request);

private:

  Fields fields;

  ServerConfig server_config;

};

} // namespace machine_fleet

#endif // MACHINE_FLEET__SRC__SERVERIMPL_HPP
