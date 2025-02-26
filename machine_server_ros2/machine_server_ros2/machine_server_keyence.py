import time
import sys
import argparse
import yaml
import rclpy
from rclpy.node import Node

from machine_fleet_msgs.srv import Machine

from .pymcprotocol import Type3E


class MachineService(Node):
    def __init__(self, config_yaml):
        super().__init__("machine_service")

        # Params:
        # Cấu hình các thông số quan trọng:
        self.IP_addres_PLC = config_yaml["ip"]
        self.port_addres_PLC = config_yaml["port"]
        self.timeout = config_yaml["time_out"]
        self.machine_name = config_yaml["name"]
        mode_operation = config_yaml["mode_operation"]

        self.get_logger().info(f"PLC IP address: {self.IP_addres_PLC}")
        self.get_logger().info(f"PLC Port address: {self.port_addres_PLC}")
        self.get_logger().info(f"timeout: {self.timeout}")

        self.pyPLC = Type3E("Q")
        self.pyPLC.connect(self.IP_addres_PLC, self.port_addres_PLC)

        # Việc cần làm là viết một bộ convert headdevice cho plc keyence khi giao tiếp theo mcprotocol
        convert_devicename = {
            # "R": "X",
            # "B": "B",
            # "MR": "M",
            "LR": "L",
            # "CR": "SM",
            # "CM": "SD",
            "DM": "D",
            # "EM": "D",
            # "FM": "R",
            # "ZF": "ZR",
            # "ZF": "ZR",
            # "W": "W",
        }

        # ------ Address all device -------:
        # Bits:
        # Dispenser:
        self.dispenser_trigger_bit = config_yaml["bit"]["dispenser_trigger"]
        # self.dispenser_trigger_bit = f"{convert_devicename[config_yaml["bit"]["dispenser_trigger"][0:2]]}{config_yaml["bit"]["dispenser_trigger"][2:]}"

        # Ingestor:
        self.ingestor_trigger_bit = config_yaml["bit"]["ingestor_trigger"]
        # self.ingestor_trigger_bit = f"{convert_devicename[config_yaml["bit"]["ingestor_trigger"][0:2]]}{config_yaml["bit"]["ingestor_trigger"][2:]}"

        # Registers:
        # Machine data:
        # 0: machine_mode (0: unknow, 1: human, 2: agv, 3: error, 4: emergency)
        # 1: dispenser_state (0: idle, 1: accept_dockin, 2: robot_docked, 3: accept_dockout)
        # 2: ingestor_state (0: idle, 1: accept_dockin, 2: robot_docked, 3: accept_dockout)
        self.machine_mode_reg = f"{convert_devicename[config_yaml["register"]["machine_mode"][0:2]]}{config_yaml["register"]["machine_mode"][2:]}"
        self.dispenser_state_reg = f"{convert_devicename[config_yaml["register"]["dispenser_state"][0:2]]}{config_yaml["register"]["dispenser_state"][2:]}"
        self.ingestor_state_reg = f"{convert_devicename[config_yaml["register"]["ingestor_state"][0:2]]}{config_yaml["register"]["ingestor_state"][2:]}"

        # 0: idle, 1: accept_dockin, 2: robot_docked, 3: accept_dockout, 4: cancel, 5: robot_error
        self.dispenser_control_reg = f"{convert_devicename[config_yaml["register"]["dispenser_control"][0:2]]}{config_yaml["register"]["dispenser_control"][2:]}"

        # 0: idle, 1: accept_dockin, 2: robot_docked, 3: accept_dockout, 4: cancel, 5: robot_error
        self.ingestor_control_reg = f"{convert_devicename[config_yaml["register"]["ingestor_control"][0:2]]}{config_yaml["register"]["ingestor_control"][2:]}"

        # Services server:
        self.machine_srv = self.create_service(
            Machine, f"/{self.machine_name}_server", self.machine_request_callback
        )

        self.get_logger().info("is running!!!!!!!!!!")

    def machine_request_callback(self, request: Machine.Request, response: Machine.Response):
        try:
            self.get_logger().info(
                f"Get request MACHINE:\n"
                f"  request_type: {request.request_type}\n"
                f"  request_mode: {request.request_mode.mode}"
            )

            response.success = False
            dispenserState = self.pyPLC.batchread_wordunits(self.dispenser_state_reg, 1)[0]
            ingestorState = self.pyPLC.batchread_wordunits(self.ingestor_state_reg, 1)[0]

            if request.request_type == Machine.Request.REQUEST_DISPENSER:
                control_reg = self.dispenser_control_reg
                trigger_bit = self.dispenser_trigger_bit
                workcell_state = dispenserState  # dispenser_state
            elif request.request_type == Machine.Request.REQUEST_INGESTOR:
                control_reg = self.ingestor_control_reg
                trigger_bit = self.ingestor_trigger_bit
                workcell_state = ingestorState  # ingestor_state
            else:
                self.get_logger().error(f"Invalid/Unsupport request_type!")
                response.message = "Invalid/Unsupport request_type!"
                return response

            if workcell_state != request.request_mode.mode:
                self.pyPLC.batchwrite_wordunits(control_reg, [request.request_mode.mode])
                self.pyPLC.batchwrite_bitunits(trigger_bit, [1])
                startTime = self.get_clock().now()
                while rclpy.ok():
                    if self.pyPLC.batchread_bitunits(trigger_bit, 1)[0]:
                        break

                    durationTime = (self.get_clock().now() - startTime).nanoseconds * (10 ** (-9))
                    if durationTime >= self.timeout:
                        self.get_logger().error(f"Timeout machine reaches!")
                        response.success = False
                        response.message = "Machine timeout error!"
                        return response
                    time.sleep(0.5)

            response.success = True
            response.message = "Process success!"
            return response

        except Exception as e:
            self.get_logger().error(e)
            response.success = False
            response.message = "error undefined!"
            return response


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

    machine_service = MachineService(config_yaml=config_yaml["machine_info"])
    rclpy.spin(machine_service)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    machine_service.pyPLC.close()
    machine_service.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
