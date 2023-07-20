#!/usr/bin/env python3

# Import the necessary modules
import socket
import threading
import time

class TelloDrone:

    TIME_BTW_RC_CONTROL_COMMANDS = 0.001  # in seconds

    def __init__(self, tello_address, local_address):
        self.last_rc_control_timestamp = time.time()
        
        self.tello_address = tello_address
        self.local_address = local_address

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(local_address)

        self.receiveThread = threading.Thread(target=self.receive)
        self.receiveThread.daemon = True
        self.receiveThread.start()

    def receive(self):
        while True:
            try:
                response, ip = self.sock.recvfrom(128)
                print("Received message: from Tello EDU: " + response.decode(encoding='utf-8'))
            except Exception as e:
                self.sock.close()
                print("Error receiving: " + str(e))
                break

    def _send_message(self, message, delay):
        try:
            self.sock.sendto(message.encode(), self.tello_address)
            print("Sending message: " + message)
        except Exception as e:
            print("Error sending: " + str(e))
        time.sleep(delay)
        
    def command(self, delay):
        self._send_message("command", delay)

    def command(self, delay):
        self._send_message("mon", delay)

    def battery(self, delay):
        self._send_message("battery?", delay)

    def takeoff(self, delay):
        self._send_message("takeoff", 10)

    def land(self, delay):
        self._send_message("land", 5)

    def send_rc_control(self, left_right_velocity: int, forward_backward_velocity: int, up_down_velocity: int,
                        yaw_velocity: int):
        """Send RC control via four channels. Command is sent every self.TIME_BTW_RC_CONTROL_COMMANDS seconds.
        Arguments:
            left_right_velocity: -100~100 (left/right)
            forward_backward_velocity: -100~100 (forward/backward)
            up_down_velocity: -100~100 (up/down)
            yaw_velocity: -100~100 (yaw)
        """
        def clamp100(x: int) -> int:
            return max(-100, min(100, x))

        if time.time() - self.last_rc_control_timestamp > self.TIME_BTW_RC_CONTROL_COMMANDS:
            self.last_rc_control_timestamp = time.time()
            cmd = 'rc {} {} {} {}'.format(
                clamp100(left_right_velocity),
                clamp100(forward_backward_velocity),
                clamp100(up_down_velocity),
                clamp100(yaw_velocity)
            )
            self._send_message(cmd,delay=0.001)
