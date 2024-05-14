import time
import socket
from uuid import uuid4
import rclpy
from rclpy.node import Node

from machine_fleet_msgs.srv import MachineCart
from machine_fleet_msgs.msg import DeliveryMode, DeliveryRequest, MachineState, MachineMode, StationMode, StationRequest 

class MachineService(Node):
    def __init__(self):
        super().__init__("machine_service")
        # Params:
        # Cấu hình các thông số quan trọng:
        self.declare_parameter('PLC_IP_address','192.168.1.1')
        self.declare_parameter('PLC_Port_address',8501)
        self.declare_parameter('timeout', 10.0)
        self.declare_parameter('frequency', 5.0)
        self.declare_parameter('dropoff_station_name', 'station')
        self.declare_parameter('pickup_station_name', 'station')

        self.IP_addres_PLC = self.get_parameter('PLC_IP_address').value
        self.port_addres_PLC = self.get_parameter('PLC_Port_address').value
        self.timeout = self.get_parameter('timeout').value
        self.frequency = self.get_parameter('frequency').value
        self.dropoff_station_name = self.get_parameter('dropoff_station_name').value
        self.pickup_station_name = self.get_parameter('pickup_station_name').value


        self.get_logger().info(f"PLC IP address: {self.IP_addres_PLC}")
        self.get_logger().info(f"PLC Port address: {self.port_addres_PLC}")
        self.get_logger().info(f"timeout: {self.timeout}")
        self.get_logger().info(f"frequency: {self.frequency}")
        self.get_logger().info(f"dropoff_station_name: {self.dropoff_station_name}")
        self.get_logger().info(f"pickup_station_name: {self.pickup_station_name}")

        self.soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._is_connected = False
        while self._is_connected == False:
            self._is_connected = self.socket_connect(self.IP_addres_PLC, self.port_addres_PLC)

        # Device PLC:
        # Bits:
        # MR600-602: DROPOFF:
        #   600: alow input
        #   601: alow output
        #   602: Request dropoff
        # MR603-605: PICKUP:
        #   603: alow input
        #   604: alow output        
        #   605: Request pickup
        self.machine_signal_bit = ['MR',600,'.U',6]
        self.dropoff_in_location = ['MR',606,'.U',1]
        self.dropoff_out_location = ['MR',607,'.U',1]
        self.pickup_in_location = ['MR',608,'.U',1]
        self.pickup_out_location = ['MR',609,'.U',1]
        self.dropoff_station_state_bit = ['LR',1501,'.U',5]
        self.pickup_station_state_bit = ['LR',1511,'.U',2]

        self.machine_error_bit = ['MR',2001,'.U',1]

        # Services server:
        self.srv = self.create_service(MachineCart, "machine_server_rasp", self.machine_callback)

        # Publishers:
        self.deliveryRequestPub = self.create_publisher(DeliveryRequest, "/delivery_request_rasp",10)
        self.machineStatePub = self.create_publisher(MachineState, "/machine_state_rasp", 10)

        # Subcribers:
        self.stationRequestSub = self.create_subscription(StationRequest,"/station_request_rasp",self.station_request_callback,1)

        # vars:
        self.machine_mode = MachineMode.MODE_IDLE

        # Timer:
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

    def machine_callback(self, request: MachineCart.Request, response: MachineCart.Response):
        try:
            self.get_logger().info(f"Get request machine mode: {request.mode}")
            if request.mode == MachineCart.Request.MODE_PK_RELEASE:
                self.write_device(self.pickup_in_location[0],
                                  self.pickup_in_location[1],
                                  self.pickup_in_location[2],
                                  self.pickup_in_location[3],
                                  [1])
                self.get_logger().info(f"writed bit RELEASE pickup to PLC success")
                while self.read_device(self.pickup_in_location[0],
                                       self.pickup_in_location[1],
                                       self.pickup_in_location[2],
                                       self.pickup_in_location[3]):
                    self.get_logger().info(f"Waiting machine pickup RELEASE!")
                    time.sleep(0.5)

            elif request.mode == MachineCart.Request.MODE_PK_CLAMP:
                self.write_device(self.pickup_out_location[0],
                                  self.pickup_out_location[1],
                                  self.pickup_out_location[2],
                                  self.pickup_out_location[3],
                                  [1])
                self.get_logger().info(f"writed bit CLAMP pickup to PLC success")

            elif request.mode == MachineCart.Request.MODE_DF_RELEASE:
                self.write_device(self.dropoff_in_location[0],
                                  self.dropoff_in_location[1],
                                  self.dropoff_in_location[2],
                                  self.dropoff_in_location[3],
                                  [1])
                self.get_logger().info(f"writed bit RELEASE dropoff to PLC success")
            
            elif request.mode == MachineCart.Request.MODE_DF_CLAMP:
                self.write_device(self.dropoff_out_location[0],
                                  self.dropoff_out_location[1],
                                  self.dropoff_out_location[2],
                                  self.dropoff_out_location[3],
                                  [1])
                self.get_logger().info(f"writed bit CLAMP dropoff to PLC success")
            
            else:
                self.get_logger().error(f"Request charge mode not supported!")
                response.success = False
                response.message = "Request charge mode not supported!"
                self.machine_mode = MachineMode.MODE_IDLE
                return response
            
            self.machine_mode = MachineMode.MODE_IDLE
            response.success = True
            return response
        
        except:
            response.success = False
            response.message = "error undefined!"
            return response

    def station_request_callback(self, msg: StationRequest):
        stationName = msg.station_name
        if stationName.find(self.dropoff_station_name) != -1:
            idStation = int(stationName[len(self.dropoff_station_name):]) - 1
            bitAddr = self.dropoff_station_state_bit[1] + idStation
            if msg.mode.mode == StationMode.MODE_EMPTY:
                self.write_device(self.dropoff_station_state_bit[0], bitAddr,
                                  self.dropoff_station_state_bit, 1, [0])
            elif msg.mode.mode == StationMode.MODE_FILLED:
                self.write_device(self.dropoff_station_state_bit[0], bitAddr,
                                  self.dropoff_station_state_bit, 1, [1])
        elif stationName.find(self.pickup_station_name) != -1:
            idStation = int(stationName[len(self.pickup_station_name):]) - 1
            bitAddr = self.pickup_station_state_bit[1] + idStation
            if msg.mode.mode == StationMode.MODE_EMPTY:
                self.write_device(self.pickup_station_state_bit[0], bitAddr,
                                  self.pickup_station_state_bit, 1, [0])
            elif msg.mode.mode == StationMode.MODE_FILLED:
                self.write_device(self.pickup_station_state_bit[0], bitAddr,
                                  self.pickup_station_state_bit, 1, [1])
        else:
            self.get_logger().error("Not found station name match with dropoff or pickup station")

    def timer_callback(self):
        signalMachineData = self.read_device(self.machine_signal_bit[0],
                                             self.machine_signal_bit[1],
                                             self.machine_signal_bit[2],
                                             self.machine_signal_bit[3])
        
        # Request dropoff:
        if signalMachineData[2]:
            stationState = self.read_device(self.pickup_station_state_bit[0],
                                            self.pickup_station_state_bit[1],
                                            self.pickup_station_state_bit[2],
                                            self.pickup_station_state_bit[3])
            self.machine_mode = MachineMode.MODE_DF_RELEASE
            i = 1
            for state in stationState:
                if not state:
                    msgDR = DeliveryRequest()
                    msgDR.request_id = str(uuid4())[0:8]
                    msgDR.station_name = f"{self.pickup_station_name}{i}"
                    msgDR.mode.mode = DeliveryMode.MODE_DROPOFF
                    self.deliveryRequestPub.publish(msgDR)
                    self.write_device(self.machine_signal_bit[0],
                                      self.machine_signal_bit[0]+2,
                                      self.machine_signal_bit[0],
                                      1,[0])
                    break
                i+=1
        
        # Request pickup:
        elif signalMachineData[5]:
            stationState = self.read_device(self.dropoff_station_state_bit[0],
                                            self.dropoff_station_state_bit[1],
                                            self.dropoff_station_state_bit[2],
                                            self.dropoff_station_state_bit[3])
            self.machine_mode = MachineMode.MODE_PK_RELEASE
            i = 1
            for state in stationState:
                if not state:
                    msgDR = DeliveryRequest()
                    msgDR.request_id = str(uuid4())[0:8]
                    msgDR.station_name = f"{self.dropoff_station_name}{i}"
                    msgDR.mode.mode = DeliveryMode.MODE_PICKUP
                    self.deliveryRequestPub.publish(msgDR)
                    self.write_device(self.machine_signal_bit[0],
                                      self.machine_signal_bit[0] + 5,
                                      self.machine_signal_bit[0],
                                      1,[0])
                    break
                i+=1
        
        msgMS = MachineState()
        msgMS.mode = self.machine_mode
        self.machineStatePub.publish(msgMS)



def main(args=None):
    rclpy.init(args=args)
    charger_service = MachineService()
    rclpy.spin(charger_service)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    charger_service.soc.close()
    charger_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
