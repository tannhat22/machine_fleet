# machine_time records when the information in this message was generated
builtin_interfaces/Time machine_time

string machine_name

uint8 machine_mode
uint8 MODE_UNKNOWN=0
uint8 MODE_HUMAN=1
uint8 MODE_AGV=2
uint8 MODE_ERROR=3
uint8 MODE_EMERGENCY=4

bool request_pickup
DeviceMode dispenser_mode
string dispenser_request_id

bool request_dropoff
DeviceMode ingestor_mode
string ingestor_request_id


