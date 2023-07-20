#!/usr/bin/env python3

# Import the necessary modules
import socket
import threading
import time

class TelloDrone:

    TIME_BTW_RC_CONTROL_COMMANDS = 0.001  # in seconds

    def __init__(self, tello_address, local_address):
        self.last_rc_control_timestamp = time.time()

        # Tello EDU telemetry format
        # mid:-1;x:0;y:0;z:0;mpry:0,0,0;pitch:1;roll:0;yaw:0;vgx:0;vgy:0;vgz:0;templ:66;temph:67;tof:10;h:0;bat:88;baro:328.69;time:0;agx:21.00;agy:15.00;agz:-1004.00;
        """self.tello_edu_telemetry_indices = {
            "pitch": 5,
            "roll": 6,
            "yaw": 7,
            "tof": 13,
            "altitude": 14,
            "battery": 15
        }"""

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

    def receive_telemetry(self):
        self.sock.bind(self.local_address)

        def recv():
            while True:
                try:
                    response, _ = self.sock.recvfrom(256)
                    response = response.decode(encoding="utf-8")
                    self.parse_telemetry(response)                
                except Exception as e:
                    print("Error receiving: " + str(e))
                    break

        thread = threading.Thread(target=recv)
        thread.start()

    def get_battery(self, delay):
        self._send_message("bat", delay=0.01)

    def get_pitch(self, delay):
        self._send_message("pitch", delay=0.01)

    def get_yaw(self, delay):
        self._send_message("yaw", delay=0.01)

    def get_roll(self, delay):
        self._send_message("roll", delay=0.01)

    def get_acceleration_x(self, delay):
        self._send_message("agx", delay=0.01)

    def get_acceleration_y(self, delay):
        self._send_message("agy", delay=0.01)

    def get_acceleration_z(self, delay):
        self._send_message("agz", delay=0.01)

    def get_speed_x(self, delay):
        self._send_message("vgx", delay=0.01)

    def get_speed_y(self, delay):
        self._send_message("vgy", delay=0.01)

    def get_speed_z(self, delay):
        self._send_message("vgz", delay=0.01)

    def get_barometer(self,delay):
        self._send_message("baro", delay=0.01)

    def get_distance_tof(self,delay):
        self._send_message("tof", delay=0.01)

    def get_flight_time(self,delay):
        self._send_message("time", delay=0.01)

    def get_lowest_temperature(self, delay):
        """Get lowest temperature
        Returns:
            int: lowest temperature (°C)
        """
        self._send_message('templ', delay=0.01)

    def get_highest_temperature(self, delay):
        """Get highest temperature
        Returns:
            float: highest temperature (°C)
        """
        self._send_message('temph', delay=0.01)


    def takeoff(self, delay):
        self._send_message("takeoff", delay=7)

    def land(self, delay):
        self._send_message("land", delay=5)

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
            msg = 'rc {} {} {} {}'.format(
                clamp100(left_right_velocity),
                clamp100(forward_backward_velocity),
                clamp100(up_down_velocity),
                clamp100(yaw_velocity)
            )
            self._send_message(msg,delay=0.001)
