#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from erp42_msgs.msg import SerialFeedBack
from erp42_control.msg import ControlMessage

import serial
import threading
import math as m

exitThread = False


class ERP42Serial(Node):
    def __init__(self):
        super().__init__('erp42_serial')

        # Packet Define
        S = 83
        T = 84
        X = 88
        AorM = 1
        ESTOP = 0
        ETX0 = 13
        ETX1 = 10
        ALIVE = 0

        self.DATA = bytearray(14)
        self.DATA[0] = S
        self.DATA[1] = T
        self.DATA[2] = X
        self.DATA[3] = AorM
        self.DATA[4] = ESTOP
        self.DATA[12] = ETX0
        self.DATA[13] = ETX1

        self.ALIVE = ALIVE
        self.ESTOP = False

        self.feedback_pub = self.create_publisher(SerialFeedBack, '/erp42_feedback', 1)
        self.control_sub = self.create_subscription(ControlMessage, '/cmd_msg', self.cmdCallback, 1)
        self.estop_sub = self.create_subscription(Bool, 'estop', self.callback_estop, 1)

        self.feedback_msg = SerialFeedBack()
        self.cmd_msg = ControlMessage()

        port = '/dev/ttyerp'  # Default port (change accordingly)
        self.ser = serial.Serial(
            port=port,
            baudrate=115200,
        )

        thread = threading.Thread(target=self.receive_data)
        thread.daemon = True
        thread.start()

    def cmdCallback(self, msg):
        self.cmd_msg = msg

    def callback_estop(self, msg):
        self.ESTOP = msg.data

    def send_data(self, estop, data=ControlMessage()):
        # The send_data function body remains the same

    def receive_data(self):
        # The receive_data function body remains the same

    def handle_data(self, line):
        # The handle_data function body remains the same

    """ Util """
    def kph2mps(self, value):
        return value * 0.277778

    def mps2kph(self, value):
        return value * 3.6


def main(args=None):
    rclpy.init(args=args)
    erp42_serial = ERP42Serial()
    rclpy.spin(erp42_serial)
    erp42_serial.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
