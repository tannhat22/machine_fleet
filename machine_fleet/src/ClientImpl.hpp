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

#ifndef MACHINE_FLEET__SRC__CLIENTIMPL_HPP
#define MACHINE_FLEET__SRC__CLIENTIMPL_HPP

#include <machine_fleet/messages/MachineState.hpp>
#include <machine_fleet/messages/DeliveryRequest.hpp>
#include <machine_fleet/messages/MachineRequest.hpp>
#include <machine_fleet/messages/StationRequest.hpp>
#include <machine_fleet/Client.hpp>
#include <machine_fleet/ClientConfig.hpp>

#include <dds/dds.h>

#include "messages/FleetMessages.h"
#include "dds_utils/DDSPublishHandler.hpp"
#include "dds_utils/DDSSubscribeHandler.hpp"

namespace machine_fleet {

class Client::ClientImpl
{
public:

  /// DDS related fields required for the client to operate
  struct Fields
  {
    /// DDS participant that is tied to the configured dds_domain_id
    dds_entity_t participant;

    /// DDS publisher that handles sending out current machine states to the 
    /// server
    dds::DDSPublishHandler<MachineFleetData_MachineState>::SharedPtr
        state_pub;

    /// DDS subscriber for delivery requests coming from the server
    dds::DDSPublishHandler<MachineFleetData_DeliveryRequest>::SharedPtr 
        delivery_request_pub;

    /// DDS subscriber for machine requests coming from the server
    dds::DDSSubscribeHandler<MachineFleetData_MachineRequest>::SharedPtr 
        machine_request_sub;

    /// DDS subscriber for station requests coming from the server
    dds::DDSSubscribeHandler<MachineFleetData_StationRequest>::SharedPtr 
        station_request_sub;
  };

  ClientImpl(const ClientConfig& config);

  ~ClientImpl();

  void start(Fields fields);

  bool send_machine_state(const messages::MachineState& new_machine_state);

  bool send_delivery_request(const messages::DeliveryRequest& new_delivery_request);

  bool read_machine_request(messages::MachineRequest& machine_request);

  bool read_station_request(messages::StationRequest& station_request);

private:

  Fields fields;

  ClientConfig client_config;

};

} // namespace machine_fleet

#endif // MACHINE_FLEET__SRC__CLIENTIMPL_HPP
