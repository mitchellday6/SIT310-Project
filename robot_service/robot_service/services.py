import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


from robot_interfaces.srv import Move, Turn
from std_srvs.srv import Empty

class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s

    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)


class RobotServer(Node):
    robot_response = "no_response"
    
    def __init__(self):
        super().__init__('robot_server')
        self.robot_response = "no_response"
        self.srv = self.create_service(Move, 'move_forward', self.move_forward_callback)
        self.srv = self.create_service(Move, 'move_backward', self.move_backward_callback)
        self.srv = self.create_service(Turn, 'turn_left', self.turn_left_callback)
        self.srv = self.create_service(Turn, 'turn_right', self.turn_right_callback)
        self.srv = self.create_service(Move, 'stop', self.stop_callback)
        
    def send_command(self, msg):
        print("Sending Command: "+ msg)
        ser = serial.Serial('/dev/ttyACM0', baudrate=9600, timeout=0.5)
        command = msg+'\n'
        ser.write(command.encode())
        reply = ser.readline().decode("utf-8").strip('\n').strip('\r')
        print("From robot:" +reply)
        ser.close()
        reply = "Msg Sent"
        return reply


    def move_forward_callback(self, request, response):
        self.get_logger().info('Incoming request: Move Forward: %dcm' % (request.distance))
        command = "MOVEF:%d" % request.distance
        # command = "MOVEF:1000"
        reply = self.send_command(command)
        response.result = "Sent"
        return response


    def move_backward_callback(self, request, response):
        self.get_logger().info('Incoming request: Move Backward: %dcm' % (request.distance))
        command = "MOVEB:%d" % request.distance
        # command = "MOVEB:1000"
        reply = self.send_command(command)
        response.result = "Sent"
        return response

    def turn_right_callback(self, request, response):
        self.get_logger().info('Incoming request: Turn Right: %d degrees' % (request.deg))
        command = "TURNR:%d" % request.deg
        # command = "TURNR:1000"
        reply = self.send_command(command)
        response.result = "Sent"
        return response
    
    def turn_left_callback(self, request, response):
        self.get_logger().info('Incoming request: Turn Left: %d degrees' % (request.deg))
        command = "TURNL:%d" % request.deg
        # command = "TURNL:1000"
        reply = self.send_command(command)
        response.result = "Sent"
        return response

    def stop_callback(self, request, response):
        self.get_logger().info('Incoming request: Stop')
        command = "STOPA:0000"
        reply = self.send_command(command)
        response.result = "Sent"
        return response


                

def main(args=None):
    rclpy.init(args=args)

    node = RobotServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - Done automatically when node is garbage collected)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

