import time
import socket
import rclpy
from rclpy.node import Node

from charger_fleet_msgs.srv import Charger
from charger_fleet_msgs.msg import ChargerState

class ChargerService(Node):
    def __init__(self):
        super().__init__("charger_service")
        # Params:
        # Cấu hình các thông số quan trọng:
        self.declare_parameter('PLC_IP_address','192.168.1.1')
        self.declare_parameter('PLC_Port_address',8501)
        self.declare_parameter('timeout', 10.0)
        self.declare_parameter('frequency', 5.0)

        self.IP_addres_PLC = self.get_parameter('PLC_IP_address').value
        self.port_addres_PLC = self.get_parameter('PLC_Port_address').value
        self.timeout = self.get_parameter('timeout').value
        self.frequency = self.get_parameter('frequency').value


        self.get_logger().info(f"PLC IP address: {self.IP_addres_PLC}")
        self.get_logger().info(f"PLC Port address: {self.port_addres_PLC}")
        self.get_logger().info(f"timeout: {self.timeout}")
        self.get_logger().info(f"frequency: {self.frequency}")

        self.soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._is_connected = False
        while self._is_connected == False:
            self._is_connected = self.socket_connect(self.IP_addres_PLC, self.port_addres_PLC)

        # Device PLC:
        # Bits:
        self.trigger_charge_bit = ['MR',100,'.U',1]
        self.trigger_uncharge_bit = ['MR',101,'.U',1]

        # Registers:
        self.charger_state_reg = ['DM',1000,'.U',1]
        self.process_state_reg = ['DM',2000,'.U',1]

        # Services server:
        self.srv = self.create_service(Charger, "charger_server", self.charger_callback)

        # Publishers:
        self.chargerModePub = self.create_publisher(ChargerState, "/charger_state", 10)

        timer_period = 1 / self.frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)


        self.get_logger().info("is running!!!!!!!!!!")


    #"Connect to PLC:"
    def socket_connect(self,host, port):
        try:
            self.soc.connect((host, port))
            self.get_logger().info("is connected to PLC success")
            return True
        except OSError:
            self.get_logger().info("can't connect to PLC, will auto reconneting after 2s")
            time.sleep(2)
            return False
        

    #"------------Read device-------------"
    # deviceType: 'DM'
    # deviceNo: 100
    # format: .U: Unsigned 16-bit DEC
    #         .S: Signed 16-bit DEC
    #         .D: Unsigned 32-bit DEC
    #         .L: Signed 32-bit DEC
    #         .H: 16-bit HEX
    # length: 3
    # return: []
    def read_device(self,deviceType: str = 'DM', deviceNo: int = 0, format: str = '', length: int = 1):
        try:
            if length > 1:
                device = 'RDS' + ' ' + deviceType + str(deviceNo) + format + ' ' + str(length) + '\x0D'
            else:
                device = 'RD' + ' ' + deviceType + str(deviceNo) + format + '\x0D' 
            dataformat = device.encode()
            self.soc.sendall(dataformat)
            dataRecv = self.soc.recv(4096)
            dataRecvDec = dataRecv.decode()
            dataResp = dataRecvDec.split(' ')
            # self.get_logger().warn(f'len data: {len(dataResp)}')
            dataResp[len(dataResp)-1] = dataResp[len(dataResp)-1][:-2]
            for i in range (0,len(dataResp)):
                dataResp[i] = int(dataResp[i])
            return dataResp
        except Exception as e:
            print(e)
            return False
        

    #"------------Write device-------------"
    # deviceType: 'DM'
    # deviceNo: 100
    # format: .U: Unsigned 16-bit DEC
    #         .S: Signed 16-bit DEC
    #         .D: Unsigned 32-bit DEC
    #         .L: Signed 32-bit DEC
    #         .H: 16-bit HEX
    # length: 3
    # data: [1,2,3]
    # return: []
    def write_device(self, deviceType: str = 'R', deviceNo: int = 0, format: str = '', length: int = 1, data: list = []):
        try:
            if length != len(data):
                print('write data error (length of data not correct!)')
                return False
            
            if length > 1:
                device = 'WRS' + ' ' + deviceType + str(deviceNo) + format + ' ' + str(length)
                for i in data:
                    device += (' ' + str(i))
                device += '\x0D'
            else:
                device = 'WR' + ' ' + deviceType + str(deviceNo) + format + ' ' + str(data[0]) + '\x0D'
            
            dataformat = device.encode()
            self.soc.sendall(dataformat)
            dataRecv = self.soc.recv(1024)
            dataRecvDec = dataRecv.decode()
            dataResp = dataRecvDec.split(' ')
            dataResp[len(dataResp)-1] = dataResp[len(dataResp)-1][:-2]
            
            if (dataResp[0]== 'OK'):
                return True
            return False
        except Exception as e:
            print(e)
            return False

    def charger_callback(self, request: Charger.Request, response: Charger.Response):
        try:
            self.get_logger().info(f"Get request charge mode: {request.mode}")
            if request.mode == Charger.Request.MODE_CHARGE:
                if self.write_device(self.trigger_charge_bit[0],
                                self.trigger_charge_bit[1],
                                self.trigger_charge_bit[2],
                                self.trigger_charge_bit[3],
                                [1]):
                    self.get_logger().info(f"writed bit CHARGE to PLC success")
                
            elif request.mode == Charger.Request.MODE_UNCHARGE:
                if self.write_device(self.trigger_uncharge_bit[0],
                                self.trigger_uncharge_bit[1],
                                self.trigger_uncharge_bit[2],
                                self.trigger_uncharge_bit[3],
                                [1]):
                    self.get_logger().info(f"writed bit UNCHARGE to PLC success")
            
            else:
                self.get_logger().error(f"Request charge mode not supported!")
                response.success = False
                response.message = "Request charge mode not supported!"
                return response
            
            process_state = 0
            startTime = self.get_clock().now()
            while process_state == 0:
                process_state = self.read_device(self.process_state_reg[0],
                                                 self.process_state_reg[1],
                                                 self.process_state_reg[2],
                                                 self.process_state_reg[3])[0]
                durationTime = (self.get_clock().now() - startTime).nanoseconds*(10**(-9))
                if durationTime >= self.timeout:
                    self.get_logger().error(f"Timeout reaches!")
                    break
                time.sleep(0.5)
                continue

            if process_state == 1000:
                self.get_logger().info(f"Request Success!")
                response.success = True
            else:
                response.success = False
                response.message = "PLC error!"

            # Reset lại thanh ghi trạng thái thực thi request:
            self.write_device(self.process_state_reg[0],
                            self.process_state_reg[1],
                            self.process_state_reg[2],
                            self.process_state_reg[3],
                            [0])

            return response
        
        except:
            response.success = False
            response.message = "error undefined!"
            return response

    def timer_callback(self):
        stateData = self.read_device(self.charger_state_reg[0],
                                     self.charger_state_reg[1],
                                     self.charger_state_reg[2],
                                     self.charger_state_reg[3])[0]
        
        msg = ChargerState()
        msg.state = stateData
        if stateData == 200:
            msg.state = 200
            msg.error_message = "Motor error"
        elif stateData == 201:
            msg.state = 200
            msg.error_message = "Sensor detect robot no response"
        elif stateData == 202:
            msg.state = 200
            msg.error_message = "Charger can't trigger "

        self.chargerModePub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    charger_service = ChargerService()
    rclpy.spin(charger_service)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    charger_service.soc.close()
    charger_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
