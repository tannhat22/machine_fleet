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

#include <dds/dds.h>

#include <machine_fleet/Server.hpp>

#include "ServerImpl.hpp"

#include "messages/FleetMessages.h"
#include "dds_utils/DDSPublishHandler.hpp"
#include "dds_utils/DDSSubscribeHandler.hpp"

namespace machine_fleet {

Server::SharedPtr Server::make(const ServerConfig& _config)
{
  SharedPtr server = SharedPtr(new Server(_config));

  dds_entity_t participant = dds_create_participant(
      static_cast<dds_domainid_t>(_config.dds_domain), NULL, NULL);
  if (participant < 0)
  {
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));
    return nullptr;
  }

  dds::DDSSubscribeHandler<MachineFleetData_MachineState, 10>::SharedPtr state_sub(
      new dds::DDSSubscribeHandler<MachineFleetData_MachineState, 10>(
          participant, &MachineFleetData_MachineState_desc,
          _config.dds_machine_state_topic));

  dds::DDSSubscribeHandler<MachineFleetData_DeliveryRequest, 10>::SharedPtr delivery_request_sub(
      new dds::DDSSubscribeHandler<MachineFleetData_DeliveryRequest, 10>(
          participant, &MachineFleetData_DeliveryRequest_desc,
          _config.dds_delivery_request_topic));

  dds::DDSPublishHandler<MachineFleetData_MachineRequest>::SharedPtr 
      machine_request_pub(
          new dds::DDSPublishHandler<MachineFleetData_MachineRequest>(
              participant, &MachineFleetData_MachineRequest_desc,
              _config.dds_machine_request_topic));

  dds::DDSPublishHandler<MachineFleetData_StationRequest>::SharedPtr 
      station_request_pub(
          new dds::DDSPublishHandler<MachineFleetData_StationRequest>(
              participant, &MachineFleetData_StationRequest_desc,
              _config.dds_station_request_topic));

  if (!state_sub->is_ready() ||
      !delivery_request_sub->is_ready() ||
      !machine_request_pub->is_ready() ||
      !station_request_pub->is_ready())
    return nullptr;

  server->impl->start(ServerImpl::Fields{
      std::move(participant),
      std::move(state_sub),
      std::move(delivery_request_sub),
      std::move(machine_request_pub),
      std::move(station_request_pub)});
  return server;
}

Server::Server(const ServerConfig& _config)
{
  impl.reset(new ServerImpl(_config));
}

Server::~Server()
{}

bool Server::read_machine_states(
    std::vector<messages::MachineState>& _new_machine_states)
{
  return impl->read_machine_states(_new_machine_states);
}

bool Server::read_delivery_requests(
    std::vector<messages::DeliveryRequest>& _new_delivery_requests)
{
  return impl->read_delivery_requests(_new_delivery_requests);
}

bool Server::send_machine_request(const messages::MachineRequest& _machine_request)
{
  return impl->send_machine_request(_machine_request);
}

bool Server::send_station_request(const messages::StationRequest& _station_request)
{
  return impl->send_station_request(_station_request);
}

} // namespace machine_fleet
