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

Server::ServerImpl::ServerImpl(const ServerConfig& _config) :
  server_config(_config)
{}

Server::ServerImpl::~ServerImpl()
{
  dds_return_t return_code = dds_delete(fields.participant);
  if (return_code != DDS_RETCODE_OK)
  {
    DDS_FATAL("dds_delete: %s", dds_strretcode(-return_code));
  }
}

void Server::ServerImpl::start(Fields _fields)
{
  fields = std::move(_fields);
}

bool Server::ServerImpl::read_machine_states(
    std::vector<messages::MachineState>& _new_machine_states)
{
  auto machine_states = fields.machine_state_sub->read();
  if (!machine_states.empty())
  {
    _new_machine_states.clear();
    for (size_t i = 0; i < machine_states.size(); ++i)
    {
      messages::MachineState tmp_machine_state;
      convert(*(machine_states[i]), tmp_machine_state);
      _new_machine_states.push_back(tmp_machine_state);
    }
    return true;
  }
  return false;
}

bool Server::ServerImpl::read_delivery_requests(
    std::vector<messages::DeliveryRequest>& _new_delivery_requests)
{
  auto delivery_requests = fields.delivery_request_sub->read();
  if (!delivery_requests.empty())
  {
    _new_delivery_requests.clear();
    for (size_t i = 0; i < delivery_requests.size(); ++i)
    {
      messages::DeliveryRequest tmp_machine_state;
      convert(*(delivery_requests[i]), tmp_machine_state);
      _new_delivery_requests.push_back(tmp_machine_state);
    }
    return true;
  }
  return false;
}

bool Server::ServerImpl::send_machine_request(
    const messages::MachineRequest& _machine_request)
{
  MachineFleetData_MachineRequest* new_mr = MachineFleetData_MachineRequest__alloc();
  convert(_machine_request, *new_mr);
  bool sent = fields.machine_request_pub->write(new_mr);
  MachineFleetData_MachineRequest_free(new_mr, DDS_FREE_ALL);
  return sent;
}

bool Server::ServerImpl::send_station_request(
    const messages::StationRequest& _station_request)
{
  MachineFleetData_StationRequest* new_mr = MachineFleetData_StationRequest__alloc();
  convert(_station_request, *new_mr);
  bool sent = fields.station_request_pub->write(new_mr);
  MachineFleetData_StationRequest_free(new_mr, DDS_FREE_ALL);
  return sent;
}

} // namespace machine_fleet
