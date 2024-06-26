/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to C Translator
  File name: FleetMessages.c
  Source: FleetMessages.idl
  Cyclone DDS: V0.7.0

*****************************************************************/
#include "FleetMessages.h"


static const uint32_t MachineFleetData_DeliveryMode_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (MachineFleetData_DeliveryMode, mode),
  DDS_OP_RTS
};

const dds_topic_descriptor_t MachineFleetData_DeliveryMode_desc =
{
  sizeof (MachineFleetData_DeliveryMode),
  4u,
  0u,
  0u,
  "MachineFleetData::DeliveryMode",
  NULL,
  2,
  MachineFleetData_DeliveryMode_ops,
  "<MetaData version=\"1.0.0\"><Module name=\"MachineFleetData\"><Struct name=\"DeliveryMode\"><Member name=\"mode\"><ULong/></Member></Struct></Module></MetaData>"
};


static const uint32_t MachineFleetData_MachineMode_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (MachineFleetData_MachineMode, mode),
  DDS_OP_RTS
};

const dds_topic_descriptor_t MachineFleetData_MachineMode_desc =
{
  sizeof (MachineFleetData_MachineMode),
  4u,
  0u,
  0u,
  "MachineFleetData::MachineMode",
  NULL,
  2,
  MachineFleetData_MachineMode_ops,
  "<MetaData version=\"1.0.0\"><Module name=\"MachineFleetData\"><Struct name=\"MachineMode\"><Member name=\"mode\"><ULong/></Member></Struct></Module></MetaData>"
};


static const uint32_t MachineFleetData_StationMode_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (MachineFleetData_StationMode, mode),
  DDS_OP_RTS
};

const dds_topic_descriptor_t MachineFleetData_StationMode_desc =
{
  sizeof (MachineFleetData_StationMode),
  4u,
  0u,
  0u,
  "MachineFleetData::StationMode",
  NULL,
  2,
  MachineFleetData_StationMode_ops,
  "<MetaData version=\"1.0.0\"><Module name=\"MachineFleetData\"><Struct name=\"StationMode\"><Member name=\"mode\"><ULong/></Member></Struct></Module></MetaData>"
};


static const uint32_t MachineFleetData_DeliveryRequest_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (MachineFleetData_DeliveryRequest, machine_name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (MachineFleetData_DeliveryRequest, fleet_name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (MachineFleetData_DeliveryRequest, station_name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (MachineFleetData_DeliveryRequest, request_id),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (MachineFleetData_DeliveryRequest, mode.mode),
  DDS_OP_RTS
};

const dds_topic_descriptor_t MachineFleetData_DeliveryRequest_desc =
{
  sizeof (MachineFleetData_DeliveryRequest),
  sizeof (char *),
  DDS_TOPIC_NO_OPTIMIZE,
  0u,
  "MachineFleetData::DeliveryRequest",
  NULL,
  6,
  MachineFleetData_DeliveryRequest_ops,
  "<MetaData version=\"1.0.0\"><Module name=\"MachineFleetData\"><Struct name=\"DeliveryMode\"><Member name=\"mode\"><ULong/></Member></Struct><Struct name=\"DeliveryRequest\"><Member name=\"machine_name\"><String/></Member><Member name=\"fleet_name\"><String/></Member><Member name=\"station_name\"><String/></Member><Member name=\"request_id\"><String/></Member><Member name=\"mode\"><Type name=\"DeliveryMode\"/></Member></Struct></Module></MetaData>"
};


static const uint32_t MachineFleetData_MachineRequest_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (MachineFleetData_MachineRequest, machine_name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (MachineFleetData_MachineRequest, fleet_name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (MachineFleetData_MachineRequest, request_id),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (MachineFleetData_MachineRequest, mode.mode),
  DDS_OP_RTS
};

const dds_topic_descriptor_t MachineFleetData_MachineRequest_desc =
{
  sizeof (MachineFleetData_MachineRequest),
  sizeof (char *),
  DDS_TOPIC_NO_OPTIMIZE,
  0u,
  "MachineFleetData::MachineRequest",
  NULL,
  5,
  MachineFleetData_MachineRequest_ops,
  "<MetaData version=\"1.0.0\"><Module name=\"MachineFleetData\"><Struct name=\"MachineMode\"><Member name=\"mode\"><ULong/></Member></Struct><Struct name=\"MachineRequest\"><Member name=\"machine_name\"><String/></Member><Member name=\"fleet_name\"><String/></Member><Member name=\"request_id\"><String/></Member><Member name=\"mode\"><Type name=\"MachineMode\"/></Member></Struct></Module></MetaData>"
};


static const uint32_t MachineFleetData_MachineState_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (MachineFleetData_MachineState, machine_name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (MachineFleetData_MachineState, fleet_name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (MachineFleetData_MachineState, error_message),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (MachineFleetData_MachineState, request_id),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (MachineFleetData_MachineState, mode.mode),
  DDS_OP_RTS
};

const dds_topic_descriptor_t MachineFleetData_MachineState_desc =
{
  sizeof (MachineFleetData_MachineState),
  sizeof (char *),
  DDS_TOPIC_NO_OPTIMIZE,
  0u,
  "MachineFleetData::MachineState",
  NULL,
  6,
  MachineFleetData_MachineState_ops,
  "<MetaData version=\"1.0.0\"><Module name=\"MachineFleetData\"><Struct name=\"MachineMode\"><Member name=\"mode\"><ULong/></Member></Struct><Struct name=\"MachineState\"><Member name=\"machine_name\"><String/></Member><Member name=\"fleet_name\"><String/></Member><Member name=\"error_message\"><String/></Member><Member name=\"request_id\"><String/></Member><Member name=\"mode\"><Type name=\"MachineMode\"/></Member></Struct></Module></MetaData>"
};


static const uint32_t MachineFleetData_StationRequest_ops [] =
{
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (MachineFleetData_StationRequest, machine_name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (MachineFleetData_StationRequest, fleet_name),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (MachineFleetData_StationRequest, request_id),
  DDS_OP_ADR | DDS_OP_TYPE_STR, offsetof (MachineFleetData_StationRequest, station_name),
  DDS_OP_ADR | DDS_OP_TYPE_4BY, offsetof (MachineFleetData_StationRequest, mode.mode),
  DDS_OP_RTS
};

const dds_topic_descriptor_t MachineFleetData_StationRequest_desc =
{
  sizeof (MachineFleetData_StationRequest),
  sizeof (char *),
  DDS_TOPIC_NO_OPTIMIZE,
  0u,
  "MachineFleetData::StationRequest",
  NULL,
  6,
  MachineFleetData_StationRequest_ops,
  "<MetaData version=\"1.0.0\"><Module name=\"MachineFleetData\"><Struct name=\"StationMode\"><Member name=\"mode\"><ULong/></Member></Struct><Struct name=\"StationRequest\"><Member name=\"machine_name\"><String/></Member><Member name=\"fleet_name\"><String/></Member><Member name=\"request_id\"><String/></Member><Member name=\"station_name\"><String/></Member><Member name=\"mode\"><Type name=\"StationMode\"/></Member></Struct></Module></MetaData>"
};
