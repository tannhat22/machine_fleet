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

#include <machine_fleet/Client.hpp>

#include "ClientImpl.hpp"

#include "messages/FleetMessages.h"
#include "dds_utils/DDSPublishHandler.hpp"
#include "dds_utils/DDSSubscribeHandler.hpp"

namespace machine_fleet {

Client::SharedPtr Client::make(const ClientConfig& _config)
{
  SharedPtr client = SharedPtr(new Client(_config));

  dds_entity_t participant = dds_create_participant(
      static_cast<dds_domainid_t>(_config.dds_domain), NULL, NULL);
  if (participant < 0)
  {
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));
    return nullptr;
  }

  dds::DDSPublishHandler<MachineFleetData_MachineState>::SharedPtr state_pub(
      new dds::DDSPublishHandler<MachineFleetData_MachineState>(
          participant, &MachineFleetData_MachineState_desc,
          _config.dds_state_topic));

  dds::DDSPublishHandler<MachineFleetData_DeliveryRequest>::SharedPtr delivery_request_pub(
      new dds::DDSPublishHandler<MachineFleetData_DeliveryRequest>(
          participant, &MachineFleetData_DeliveryRequest_desc,
          _config.dds_delivery_request_topic));

  dds::DDSSubscribeHandler<MachineFleetData_MachineRequest>::SharedPtr 
      machine_request_sub(
          new dds::DDSSubscribeHandler<MachineFleetData_MachineRequest>(
              participant, &MachineFleetData_MachineRequest_desc,
              _config.dds_machine_request_topic));

  dds::DDSSubscribeHandler<MachineFleetData_StationRequest>::SharedPtr 
      station_request_sub(
          new dds::DDSSubscribeHandler<MachineFleetData_StationRequest>(
              participant, &MachineFleetData_StationRequest_desc,
              _config.dds_station_request_topic));

  if (!state_pub->is_ready() ||
      !delivery_request_pub->is_ready() ||
      !machine_request_sub->is_ready() ||
      !station_request_sub->is_ready())
    return nullptr;

  client->impl->start(ClientImpl::Fields{
      std::move(participant),
      std::move(state_pub),
      std::move(delivery_request_pub),
      std::move(machine_request_sub),
      std::move(station_request_sub)});
  return client;
}

Client::Client(const ClientConfig& _config)
{
  impl.reset(new ClientImpl(_config));
}

Client::~Client()
{}

bool Client::send_machine_state(const messages::MachineState& _new_machine_state)
{
  return impl->send_machine_state(_new_machine_state);
}

bool Client::send_delivery_request(const messages::DeliveryRequest& _new_delivery_request)
{
  return impl->send_delivery_request(_new_delivery_request);
}

bool Client::read_machine_request(messages::MachineRequest& _machine_request)
{
  return impl->read_machine_request(_machine_request);
}

bool Client::read_station_request(messages::StationRequest& _station_request)
{
  return impl->read_station_request(_station_request);
}


} // namespace machine_fleet
