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

#include <iostream>
#include <limits>
#include <stdio.h>
#include <stdlib.h>

#include <dds/dds.h>

#include "../dds_utils/common.hpp"
#include "../messages/FleetMessages.h"

int main(int argc, char **argv) {
  if (argc < 4) {
    std::cout << "Please request using the following format," << std::endl;
    std::cout << "<Executable> <Machine Name> <Request type> <Task ID> <Mode>" << std::endl;
    return 1;
  }

  std::string machine_name(argv[1]);
  std::string request_type(argv[2]);
  std::string request_id(argv[3]);
  std::string mode(argv[4]);

  if (request_type != "dispenser" && request_type != "ingestor") {
    std::cout << "Supported request type are dispenser or ingestor" << std::endl;
    return 1;
  }

  if (mode != "0" && mode != "1" && mode != "2" && mode != "3" && mode != "4" && mode != "5") {
    std::cout << "Supported mode are 0, 1, 2, 3, 4, 5" << std::endl;
    return 1;
  }

  dds_entity_t participant;
  dds_entity_t topic;
  dds_entity_t writer;
  dds_return_t rc;
  dds_qos_t *qos;
  MachineFleetData_MachineRequest *msg;
  msg = MachineFleetData_MachineRequest__alloc();
  uint32_t status = 0;
  (void)argc;
  (void)argv;

  /* Create a Participant. */
  participant = dds_create_participant(52, NULL, NULL);
  if (participant < 0)
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));

  /* Create a Topic. */
  topic = dds_create_topic(participant, &MachineFleetData_MachineRequest_desc, "machine_request",
                           NULL, NULL);
  if (topic < 0)
    DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic));

  /* Create a Writer. */
  qos = dds_create_qos();
  dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, 0);
  writer = dds_create_writer(participant, topic, qos, NULL);
  if (writer < 0)
    DDS_FATAL("dds_create_write: %s\n", dds_strretcode(-writer));
  dds_delete_qos(qos);

  printf("=== [Publisher]  Waiting for a reader to be discovered ...\n");
  fflush(stdout);

  rc = dds_set_status_mask(writer, DDS_PUBLICATION_MATCHED_STATUS);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_set_status_mask: %s\n", dds_strretcode(-rc));

  while (!(status & DDS_PUBLICATION_MATCHED_STATUS)) {
    rc = dds_get_status_changes(writer, &status);
    if (rc != DDS_RETCODE_OK)
      DDS_FATAL("dds_get_status_changes: %s\n", dds_strretcode(-rc));

    /* Polling sleep. */
    dds_sleepfor(DDS_MSECS(20));
  }

  /* Create a message to write. */
  msg->machine_name = machine_fleet::common::dds_string_alloc_and_copy(machine_name);
  msg->request_id = machine_fleet::common::dds_string_alloc_and_copy(request_id);

  if (request_type == "dispenser")
    msg->request_type == MachineFleetData_MachineRequest_Constants_REQUEST_DISPENSER;
  else if (request_type == "ingestor") {
    msg->request_type == MachineFleetData_MachineRequest_Constants_REQUEST_INGESTOR;
  }

  if (mode == "0")
    msg->request_mode.mode = MachineFleetData_DeviceMode_Constants_MODE_IDLE;
  else if (mode == "1")
    msg->request_mode.mode = MachineFleetData_DeviceMode_Constants_MODE_ACCEPT_DOCKIN;
  else if (mode == "2")
    msg->request_mode.mode = MachineFleetData_DeviceMode_Constants_MODE_ROBOT_DOCKED_IN;
  else if (mode == "3")
    msg->request_mode.mode = MachineFleetData_DeviceMode_Constants_MODE_ACCEPT_DOCKOUT;
  else if (mode == "4")
    msg->request_mode.mode = MachineFleetData_DeviceMode_Constants_MODE_CANCEL;
  else if (mode == "5")
    msg->request_mode.mode = MachineFleetData_DeviceMode_Constants_MODE_ROBOT_ERROR;

  printf("=== [Publisher]  Writing : ");
  printf("Message: machine_request %s\n", request_type.c_str());
  fflush(stdout);

  rc = dds_write(writer, msg);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_write: %s\n", dds_strretcode(-rc));

  /* Deleting the participant will delete all its children recursively as well. */
  rc = dds_delete(participant);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_delete: %s\n", dds_strretcode(-rc));

  MachineFleetData_MachineRequest_free(msg, DDS_FREE_ALL);

  return EXIT_SUCCESS;
}
