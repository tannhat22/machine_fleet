import sys
import threading
import argparse
import yaml
import rclpy
from rclpy.node import Node
from .hostlinkprotocol.hostlink import HostLink

from machine_fleet_msgs.msg import DeviceMode, MachineState, StationState, StationRequest


class StationContext:
    _state: StationState
    _lock: threading.Lock

    def __init__(self, name: str, headdevice: str) -> None:
        self.name = name
        self.headdevice = headdevice
        self._state = StationState()
        self._state.station_name = name
        self._lock = threading.Lock()

    def set_state(self, mode: int) -> None:
        with self._lock:
            self._state.mode = mode

    def get_state(self) -> StationState:
        return self._state


class MachineStateUpdate(Node):
    station_context_dict: dict[str, StationContext]

    def __init__(self, config_yaml):
        super().__init__("machine_state_update")
        self.config_yaml = config_yaml

        # Params:
        # Cấu hình các thông số quan trọng:
        self.IP_addres_PLC = self.config_yaml["ip"]
        self.port_addres_PLC = self.config_yaml["port_machine_state"]
        self.frequency = self.config_yaml["frequency"]

        self.get_logger().info(f"PLC IP address: {self.IP_addres_PLC}")
        self.get_logger().info(f"PLC Port address: {self.port_addres_PLC}")
        self.get_logger().info(f"frequency: {self.frequency}")

        self.pyPLC = HostLink("KV")
        self.pyPLC.connect(self.IP_addres_PLC, self.port_addres_PLC)

        station_config = self.config_yaml["stations"]
        self.station_context_dict = {}
        for _name, _config in station_config.items():
            self.station_context_dict.update({_name: StationContext(_name, _config)})

        # Variables:
        self.machine_name = self.config_yaml["name"]
        self.mode_operation = self.config_yaml["mode_operation"]
        self.stations_quantity = len(self.station_context_dict)

        # ------ Address all device -------:
        # Bits:
        self.request_delivery_bit = self.config_yaml["bit"]["request_delivery"]
        self.station_states_bits = self.config_yaml["bit"]["station_states"]

        ## Registers:
        # Machine data:
        # 0: machine_mode (0: unknow, 1: human, 2: agv, 3: error, 4: emergency)
        # 1: dispenser_state (0: idle, 1: accept_dockin, 2: robot_docked, 3: accept_dockout)
        # 2: ingestor_state (0: idle, 1: accept_dockin, 2: robot_docked, 3: accept_dockout)
        self.machine_data_reg = self.config_yaml["register"]["machine_data"]

        # Publishers:
        self.machineStatePub = self.create_publisher(
            MachineState, f"/{self.machine_name}_machine_state", 10
        )

        # Subcribers:
        self.create_subscription(
            StationRequest,
            f"/{self.machine_name}_station_request",
            self.station_request_callback,
            10,
        )

        timer_period = 1 / self.frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("is running!!!!!!!!!!")

    def station_request_callback(self, msg: StationRequest):
        station = self.station_context_dict.get(msg.station_name, None)
        if station is not None:
            station.set_state(msg.mode)
            self.get_logger().info(f"station [{station.name}] receive request (mode: {msg.mode})")
            if msg.mode == StationRequest.MODE_EMPTY:
                self.pyPLC.write_data(station.headdevice, "", 0)
            else:
                self.pyPLC.write_data(station.headdevice, "", 1)
        else:
            self.get_logger().error(f"not found station [{msg.station_name}] in config!")

    def timer_callback(self):
        machineStateMsg = MachineState()
        machineStateMsg.machine_name = self.machine_name

        if self.mode_operation == "combine":
            machineData = self.pyPLC.continuous_read_data(self.machine_data_reg, 3, ".U")
            requestDelivery = self.pyPLC.continuous_read_data(self.request_delivery_bit, 2, "")

            # Machine mode:
            if machineData[0] == 1:
                machineStateMsg.machine_mode = MachineState.MODE_HUMAN
            elif machineData[0] == 2:
                machineStateMsg.machine_mode = MachineState.MODE_AGV
            elif machineData[0] == 3:
                machineStateMsg.machine_mode = MachineState.MODE_ERROR
            elif machineData[0] == 4:
                machineStateMsg.machine_mode = MachineState.MODE_EMERGENCY
            else:
                machineStateMsg.machine_mode = MachineState.MODE_UNKNOWN

            # dispenser_mode
            if machineData[1] == 0:
                machineStateMsg.dispenser_mode.mode = DeviceMode.MODE_IDLE
            elif machineData[1] == 1:
                machineStateMsg.dispenser_mode.mode = DeviceMode.MODE_ACCEPT_DOCKIN
            elif machineData[1] == 2:
                machineStateMsg.dispenser_mode.mode = DeviceMode.MODE_ROBOT_DOCKED_IN
            elif machineData[1] == 3:
                machineStateMsg.dispenser_mode.mode = DeviceMode.MODE_ACCEPT_DOCKOUT

            # ingestor_mode
            if machineData[2] == 0:
                machineStateMsg.ingestor_mode.mode = DeviceMode.MODE_IDLE
            elif machineData[2] == 1:
                machineStateMsg.ingestor_mode.mode = DeviceMode.MODE_ACCEPT_DOCKIN
            elif machineData[2] == 2:
                machineStateMsg.ingestor_mode.mode = DeviceMode.MODE_ROBOT_DOCKED_IN
            elif machineData[2] == 3:
                machineStateMsg.ingestor_mode.mode = DeviceMode.MODE_ACCEPT_DOCKOUT

            if requestDelivery[0]:
                machineStateMsg.request_pickup = True
            else:
                machineStateMsg.request_pickup = False

            if requestDelivery[1]:
                machineStateMsg.request_dropoff = True
            else:
                machineStateMsg.request_dropoff = False

        if self.stations_quantity > 0:
            stationData = self.pyPLC.continuous_read_data(
                self.station_states_bits, self.stations_quantity, ""
            )
            # Station states:
            if len(stationData) == self.stations_quantity:
                i = 0
                for station in self.station_context_dict.values():
                    if stationData[i]:
                        station.set_state(StationState.MODE_FILLED)
                    else:
                        station.set_state(StationState.MODE_EMPTY)
                    machineStateMsg.station_states.append(station.get_state())
                    i += 1
            else:
                self.get_logger().error(
                    f"length of station data ({len(stationData)}) not match witch stations_quantity!"
                )

        self.machineStatePub.publish(machineStateMsg)


def main(argv=sys.argv):
    rclpy.init(args=argv)
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="machine_server", description="Configure and spin up the machine_server"
    )
    parser.add_argument(
        "-c",
        "--config_file",
        type=str,
        required=True,
        help="Path to the config.yaml file",
    )
    args = parser.parse_args(args_without_ros[1:])
    config_path = args.config_file

    # Load config yamls
    with open(config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    machine_state_update = MachineStateUpdate(config_yaml=config_yaml["machine_info"])
    rclpy.spin(machine_state_update)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    machine_state_update.pyPLC.close()
    machine_state_update.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
