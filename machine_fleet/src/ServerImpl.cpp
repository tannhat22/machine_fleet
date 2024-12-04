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

#include "ServerImpl.hpp"
#include "messages/message_utils.hpp"

namespace machine_fleet {

Server::ServerImpl::ServerImpl(const ServerConfig &_config) : server_config(_config) {}

Server::ServerImpl::~ServerImpl() {
  dds_return_t return_code = dds_delete(fields.participant);
  if (return_code != DDS_RETCODE_OK) {
    DDS_FATAL("dds_delete: %s", dds_strretcode(-return_code));
  }
}

void Server::ServerImpl::start(Fields _fields) { fields = std::move(_fields); }

bool Server::ServerImpl::read_machine_states(
    std::vector<messages::MachineState> &_new_machine_states) {
  _new_machine_states.clear();
  for (const auto &sub : fields.machine_state_subs) {
    if (sub.second) {
      auto machine_states = sub.second->read();
      if (!machine_states.empty()) {
        messages::MachineState tmp_machine_state;
        convert(*(machine_states[0]), tmp_machine_state);
        _new_machine_states.push_back(tmp_machine_state);
      }
    }
  }
  return !_new_machine_states.empty(); // Trả về true nếu có ít nhất một trạng thái machine mới
}

bool Server::ServerImpl::send_machine_request(const messages::MachineRequest &_machine_request) {
  auto it = fields.machine_request_pubs.find(_machine_request.machine_name);
  if (it != fields.machine_request_pubs.end()) {
    MachineFleetData_MachineRequest *new_mr = MachineFleetData_MachineRequest__alloc();
    convert(_machine_request, *new_mr);
    bool sent = it->second->write(new_mr);
    MachineFleetData_MachineRequest_free(new_mr, DDS_FREE_ALL);
    return sent;
  }
  return false;
}

bool Server::ServerImpl::send_station_request(const messages::StationRequest &_station_request) {
  auto it = fields.station_request_pubs.find(_station_request.machine_name);
  if (it != fields.station_request_pubs.end()) {
    MachineFleetData_StationRequest *new_sr = MachineFleetData_StationRequest__alloc();
    convert(_station_request, *new_sr);
    bool sent = it->second->write(new_sr);
    MachineFleetData_StationRequest_free(new_sr, DDS_FREE_ALL);
    return sent;
  }
  return false;
}

} // namespace machine_fleet
