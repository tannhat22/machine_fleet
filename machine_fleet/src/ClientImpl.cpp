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

#include "ClientImpl.hpp"
#include "messages/message_utils.hpp"

namespace machine_fleet {

Client::ClientImpl::ClientImpl(const ClientConfig& _config) :
  client_config(_config)
{}

Client::ClientImpl::~ClientImpl()
{
  dds_return_t return_code = dds_delete(fields.participant);
  if (return_code != DDS_RETCODE_OK)
  {
    DDS_FATAL("dds_delete: %s", dds_strretcode(-return_code));
  }
}

void Client::ClientImpl::start(Fields _fields)
{
  fields = std::move(_fields);
}

bool Client::ClientImpl::send_machine_state(
    const messages::MachineState& _new_machine_state)
{
  MachineFleetData_MachineState* new_rs = MachineFleetData_MachineState__alloc();
  convert(_new_machine_state, *new_rs);
  bool sent = fields.state_pub->write(new_rs);
  MachineFleetData_MachineState_free(new_rs, DDS_FREE_ALL);
  return sent;
}

bool Client::ClientImpl::send_delivery_request(
    const messages::DeliveryRequest& _new_delivery_request)
{
  MachineFleetData_DeliveryRequest* new_rs = MachineFleetData_DeliveryRequest__alloc();
  convert(_new_delivery_request, *new_rs);
  bool sent = fields.delivery_request_pub->write(new_rs);
  MachineFleetData_DeliveryRequest_free(new_rs, DDS_FREE_ALL);
  return sent;
}

bool Client::ClientImpl::read_machine_request
    (messages::MachineRequest& _machine_request)
{
  auto machine_requests = fields.machine_request_sub->read();
  if (!machine_requests.empty())
  {
    convert(*(machine_requests[0]), _machine_request);
    return true;
  }
  return false;
}

bool Client::ClientImpl::read_station_request
    (messages::StationRequest& _station_request)
{
  auto station_requests = fields.station_request_sub->read();
  if (!station_requests.empty())
  {
    convert(*(station_requests[0]), _station_request);
    return true;
  }
  return false;
}

} // namespace machine_fleet
