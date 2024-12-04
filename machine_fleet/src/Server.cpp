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

#include "dds_utils/DDSPublishHandler.hpp"
#include "dds_utils/DDSSubscribeHandler.hpp"
#include "messages/FleetMessages.h"

namespace machine_fleet {

Server::SharedPtr Server::make(const ServerConfig &_config) {
  SharedPtr server = SharedPtr(new Server(_config));

  dds_entity_t participant =
      dds_create_participant(static_cast<dds_domainid_t>(_config.dds_domain), NULL, NULL);
  if (participant < 0) {
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));
    return nullptr;
  }

  std::unordered_map<std::string,
                     dds::DDSSubscribeHandler<MachineFleetData_MachineState>::SharedPtr>
      state_subs;

  std::unordered_map<std::string,
                     dds::DDSPublishHandler<MachineFleetData_MachineRequest>::SharedPtr>
      machine_request_pubs;

  std::unordered_map<std::string,
                     dds::DDSPublishHandler<MachineFleetData_StationRequest>::SharedPtr>
      station_request_pubs;

  for (const auto &machine_name : _config.machine_names) {
    auto state_sub = std::make_shared<dds::DDSSubscribeHandler<MachineFleetData_MachineState>>(
        participant, &MachineFleetData_MachineState_desc,
        _config.get_namespaced_topic(machine_name, _config.dds_machine_state_topic));
    state_subs.insert({machine_name, state_sub});

    auto machine_request_pub =
        std::make_shared<dds::DDSPublishHandler<MachineFleetData_MachineRequest>>(
            participant, &MachineFleetData_MachineRequest_desc,
            _config.get_namespaced_topic(machine_name, _config.dds_machine_request_topic));
    machine_request_pubs.insert({machine_name, machine_request_pub});

    auto station_request_pub =
        std::make_shared<dds::DDSPublishHandler<MachineFleetData_StationRequest>>(
            participant, &MachineFleetData_StationRequest_desc,
            _config.get_namespaced_topic(machine_name, _config.dds_station_request_topic));
    station_request_pubs.insert({machine_name, station_request_pub});
  }

  // Kiểm tra các publishers, subcribers đã sẵn sàng hay chưa
  for (const auto &machine_name : _config.machine_names) {
    const auto state_sub = state_subs.find(machine_name);
    if (state_sub == state_subs.end() || !state_sub->second->is_ready()) {
      return nullptr;
    }

    const auto machine_request_pub = machine_request_pubs.find(machine_name);
    if (machine_request_pub == machine_request_pubs.end() ||
        !machine_request_pub->second->is_ready()) {
      return nullptr;
    }

    const auto station_request_pub = station_request_pubs.find(machine_name);
    if (station_request_pub == station_request_pubs.end() ||
        !station_request_pub->second->is_ready()) {
      return nullptr;
    }
  }

  server->impl->start(ServerImpl::Fields{std::move(participant), std::move(state_subs),
                                         std::move(machine_request_pubs),
                                         std::move(station_request_pubs)});
  return server;
}

Server::Server(const ServerConfig &_config) { impl.reset(new ServerImpl(_config)); }

Server::~Server() {}

bool Server::read_machine_states(std::vector<messages::MachineState> &_new_machine_states) {
  return impl->read_machine_states(_new_machine_states);
}

bool Server::send_machine_request(const messages::MachineRequest &_machine_request) {
  return impl->send_machine_request(_machine_request);
}

bool Server::send_station_request(const messages::StationRequest &_station_request) {
  return impl->send_station_request(_station_request);
}

} // namespace machine_fleet
