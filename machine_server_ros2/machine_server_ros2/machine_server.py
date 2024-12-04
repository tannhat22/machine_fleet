import time
import sys
import argparse
import yaml
import rclpy
from rclpy.node import Node

from machine_fleet_msgs.srv import Machine
from machine_fleet_msgs.msg import MachineRequest

from .pymcprotocol import Type3E


class MachineService(Node):
    def __init__(self, config_yaml):
        super().__init__("machine_service")
        self.config_yaml = config_yaml

        # Params:
        # Cấu hình các thông số quan trọng:
        self.IP_addres_PLC = self.config_yaml["ip"]
        self.port_addres_PLC = self.config_yaml["port"]
        self.timeout = self.config_yaml["time_out"]
        self.machine_name = self.config_yaml["name"]
        mode_operation = self.config_yaml["mode_operation"]

        if mode_operation != "combine":
            return

        self.get_logger().info(f"PLC IP address: {self.IP_addres_PLC}")
        self.get_logger().info(f"PLC Port address: {self.port_addres_PLC}")
        self.get_logger().info(f"timeout: {self.timeout}")

        self.pyPLC = Type3E("Q")
        self.pyPLC.connect(self.IP_addres_PLC, self.port_addres_PLC)

        # ------ Address all device -------:
        # Bits:
        # Dispenser:
        self.dispenser_trigger_bit = self.config_yaml["bit"]["dispenser_trigger"]

        # Ingestor:
        self.ingestor_trigger_bit = self.config_yaml["bit"]["ingestor_trigger"]

        # Registers:
        # Machine data:
        # 0: machine_mode (0: unknow, 1: human, 2: agv, 3: error, 4: emergency)
        # 1: dispenser_state (0: idle, 1: accept_dockin, 2: robot_docked, 3: accept_dockout)
        # 2: ingestor_state (0: idle, 1: accept_dockin, 2: robot_docked, 3: accept_dockout)
        self.machine_data_reg = self.config_yaml["register"]["machine_data"]

        # 1: idle, 2: accept_dockin, 3: robot_docked, 4: accept_dockout, 5: cancel, 6: robot_error
        self.dispenser_control_reg = self.config_yaml["register"]["dispenser_control"]

        # 1: idle, 2: accept_dockin, 3: robot_docked, 4: accept_dockout, 5: cancel, 6: robot_error
        self.ingestor_control_reg = self.config_yaml["register"]["ingestor_control"]

        # Services server:
        self.srv = self.create_service(
            Machine, f"/{self.machine_name}_server", self.machine_request_callback
        )

        self.get_logger().info("is running!!!!!!!!!!")

    def machine_request_callback(self, request: Machine.Request, response: Machine.Response):
        try:
            self.get_logger().info(
                f"Get request MACHINE:\n"
                f"  request_type: {request.request_type}\n"
                f"  request_mode: {request.request_mode}"
            )

            response.success = False
            machineData = self.pyPLC.batchread_wordunits(self.machine_data_reg, 3)
            if request.request_type == Machine.Request.REQUEST_DISPENSER:
                control_reg = self.dispenser_control_reg
                trigger_bit = self.dispenser_trigger_bit
            elif request.request_type == Machine.Request.REQUEST_INGESTOR:
                control_reg = self.ingestor_control_reg
                trigger_bit = self.ingestor_trigger_bit
            else:
                self.get_logger().error(f"Invalid/Unsupport request_type!")
                response.message = "Invalid/Unsupport request_type!"
                return response

            if machineData[1] != request.request_mode:
                self.pyPLC.batchwrite_wordunits(control_reg, [request.request_mode + 1])
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
