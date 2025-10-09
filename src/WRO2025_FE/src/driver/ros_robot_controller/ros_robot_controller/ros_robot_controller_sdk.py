#!/usr/bin/env python3
# encoding: utf-8
# Simplified and hardened STM32 Python SDK for WRO2025 project (with Button support)

import enum
import time
import queue
import struct
import serial
import threading

# --- Protocol Definitions ---
class PacketControllerState(enum.IntEnum):
    PACKET_CONTROLLER_STATE_STARTBYTE1 = 0
    PACKET_CONTROLLER_STATE_STARTBYTE2 = 1
    PACKET_CONTROLLER_STATE_LENGTH = 2
    PACKET_CONTROLLER_STATE_FUNCTION = 3
    PACKET_CONTROLLER_STATE_ID = 4
    PACKET_CONTROLLER_STATE_DATA = 5
    PACKET_CONTROLLER_STATE_CHECKSUM = 6

class PacketFunction(enum.IntEnum):
    PACKET_FUNC_SYS = 0
    PACKET_FUNC_LED = 1
    PACKET_FUNC_BUZZER = 2
    PACKET_FUNC_MOTOR = 3
    PACKET_FUNC_PWM_SERVO = 4
    PACKET_FUNC_KEY = 6
    PACKET_FUNC_IMU = 7 # CRITICAL: Re-add IMU function ID
    # Removed bus servo, gamepad, sbus, oled, rgb
    PACKET_FUNC_NONE = 12

# Re-added for button support
class PacketReportKeyEvents(enum.IntEnum):
    KEY_EVENT_PRESSED = 0x01
    KEY_EVENT_LONGPRESS = 0x02
    KEY_EVENT_LONGPRESS_REPEAT = 0x04
    KEY_EVENT_RELEASE_FROM_LP = 0x08
    KEY_EVENT_RELEASE_FROM_SP = 0x10
    KEY_EVENT_CLICK = 0x20
    KEY_EVENT_DOUBLE_CLICK= 0x40
    KEY_EVENT_TRIPLE_CLICK = 0x80

crc8_table = [
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
]

def checksum_crc8(data):
    check = 0
    for b in data:
        check = crc8_table[check ^ b]
    return check & 0x00FF

class Board:
    def __init__(self, device="/dev/rrc", baudrate=1000000, timeout=10, logger=None):
        self.logger = logger
        self.enable_recv = False
        self.frame = []
        self.recv_count = 0

        # CRITICAL FIX: Set both read (timeout) and write (write_timeout) timeouts.
        self.port = serial.Serial(None, baudrate, timeout=0.1, write_timeout=0.1)
        self.port.rts = False
        self.port.dtr = False
        self.port.setPort(device)
        self.port.open()

        self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
        self.access_lock = threading.RLock()

        # Queues for necessary data
        self.sys_queue = queue.Queue(maxsize=1)
        self.pwm_servo_queue = queue.Queue(maxsize=1)
        self.key_queue = queue.Queue(maxsize=1) # Re-added for button support

        # Parsers for necessary data
        self.parsers = {
            PacketFunction.PACKET_FUNC_SYS: self.packet_report_sys,
            PacketFunction.PACKET_FUNC_PWM_SERVO: self.packet_report_pwm_servo,
            PacketFunction.PACKET_FUNC_KEY: self.packet_report_key, # Re-added for button support
            PacketFunction.PACKET_FUNC_IMU: self.packet_report_imu # CRITICAL: Re-add IMU parser
        }

        time.sleep(0.5)
        threading.Thread(target=self.recv_task, daemon=True).start()

    def packet_report_sys(self, data):
        try:
            self.sys_queue.put_nowait(data)
        except queue.Full:
            pass

    def packet_report_pwm_servo(self, data):
        try:
            self.pwm_servo_queue.put_nowait(data)
        except queue.Full:
            pass

    def packet_report_key(self, data):
        # Re-added for button support
        try:
            self.key_queue.put_nowait(data)
        except queue.Full:
            pass

    def packet_report_imu(self, data):
        # This function receives the IMU data from the recv_task,
        # but does nothing with it. This effectively discards the packet
        # without causing an error.
        pass

    def get_battery(self):
        if self.enable_recv:
            try:
                data = self.sys_queue.get(block=False)
                if data[0] == 0x04:
                    return struct.unpack('<H', data[1:])[0]
                else:
                    return None
            except queue.Empty:
                return None
        else:
            if self.logger:
                self.logger.warn('get_battery called but reception is not enabled!')
            return None

    def get_button(self):
        # Re-added for button support
        if self.enable_recv:
            try:
                data = self.key_queue.get(block=False)
                key_id = data[0]
                key_event = data[1]
                # This function now returns the raw event data for the main node to interpret
                return key_id, key_event
            except queue.Empty:
                return None
        else:
            if self.logger:
                self.logger.warn('get_button called but reception is not enabled!')
            return None

    def _buf_write(self, func, data):
        # This is a private helper. Lock is handled by the public-facing methods.
        # This method can now raise serial.SerialTimeoutException.
        buf = [0xAA, 0x55, int(func)]
        buf.append(len(data))
        buf.extend(data)
        buf.append(checksum_crc8(bytes(buf[2:])))
        buf = bytes(buf)
        self.port.write(buf)

    def set_motor_speed(self, speeds):
        data = [0x01, len(speeds)]
        for i in speeds:
            data.extend(struct.pack("<Bf", int(i[0] - 1), float(i[1])))
        
        with self.access_lock:
            try:
                self._buf_write(PacketFunction.PACKET_FUNC_MOTOR, data)
            except serial.SerialTimeoutException:
                log_msg = "Write timeout in set_motor_speed. Motor might be stalled."
                if self.logger:
                    self.logger.warn(log_msg)

    def pwm_servo_set_position(self, duration, positions):
        duration = int(duration * 1000)
        data = [0x01, duration & 0xFF, 0xFF & (duration >> 8), len(positions)]
        for i in positions:
            data.extend(struct.pack("<BH", i[0], i[1]))
        
        with self.access_lock:
            try:
                self._buf_write(PacketFunction.PACKET_FUNC_PWM_SERVO, data)
            except serial.SerialTimeoutException:
                log_msg = "Write timeout in pwm_servo_set_position."
                if self.logger:
                    self.logger.warn(log_msg)

    def pwm_servo_set_offset(self, servo_id, offset):
        data = struct.pack("<BBb", 0x07, servo_id, int(offset))
        with self.access_lock:
            try:
                self._buf_write(PacketFunction.PACKET_FUNC_PWM_SERVO, data)
            except serial.SerialTimeoutException:
                log_msg = f"Write timeout in pwm_servo_set_offset for servo {servo_id}."
                if self.logger:
                    self.logger.warn(log_msg)

    def pwm_servo_read_and_unpack(self, servo_id, cmd, unpack_format):
        with self.access_lock:
            try:
                while not self.pwm_servo_queue.empty():
                    self.pwm_servo_queue.get_nowait()
                
                self._buf_write(PacketFunction.PACKET_FUNC_PWM_SERVO, [cmd, servo_id])
                
                data = self.pwm_servo_queue.get(block=True, timeout=0.2)
                _servo_id, _cmd, info = struct.unpack(unpack_format, data)
                return info
            except queue.Empty:
                log_msg = f"Timeout waiting for PWM servo {servo_id} response to cmd {cmd}"
                if self.logger:
                    self.logger.warn(log_msg)
                return None
            except serial.SerialTimeoutException:
                log_msg = f"Write timeout when requesting data from PWM servo {servo_id}."
                if self.logger:
                    self.logger.warn(log_msg)
                return None

    def pwm_servo_read_offset(self, servo_id):
        return self.pwm_servo_read_and_unpack(servo_id, 0x09, "<BBb")

    def pwm_servo_read_position(self, servo_id):
        return self.pwm_servo_read_and_unpack(servo_id, 0x05, "<BBH")

    def enable_reception(self, enable=True):
        self.enable_recv = enable

    def recv_task(self):
        while True:
            try:
                if self.enable_recv:
                    if self.port.in_waiting > 0:
                        recv_data = self.port.read(self.port.in_waiting)
                        if recv_data:
                            for dat in recv_data:
                                # print("%0.2X "%dat)
                                if self.state == PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1:
                                    if dat == 0xAA:
                                        self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE2
                                    continue
                                elif self.state == PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE2:
                                    if dat == 0x55:
                                        self.state = PacketControllerState.PACKET_CONTROLLER_STATE_FUNCTION
                                    else:
                                        self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
                                    continue
                                elif self.state == PacketControllerState.PACKET_CONTROLLER_STATE_FUNCTION:
                                    if dat < int(PacketFunction.PACKET_FUNC_NONE):
                                        self.frame = [dat, 0]
                                        self.state = PacketControllerState.PACKET_CONTROLLER_STATE_LENGTH
                                    else:
                                        self.frame = []
                                        self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
                                    continue
                                elif self.state == PacketControllerState.PACKET_CONTROLLER_STATE_LENGTH:
                                    self.frame[1] = dat
                                    self.recv_count = 0
                                    if dat == 0:
                                        self.state = PacketControllerState.PACKET_CONTROLLER_STATE_CHECKSUM
                                    else:
                                        self.state = PacketControllerState.PACKET_CONTROLLER_STATE_DATA
                                    continue
                                elif self.state == PacketControllerState.PACKET_CONTROLLER_STATE_DATA:
                                    self.frame.append(dat)
                                    self.recv_count += 1
                                    if self.recv_count >= self.frame[1]:
                                        self.state = PacketControllerState.PACKET_CONTROLLER_STATE_CHECKSUM
                                    continue
                                elif self.state == PacketControllerState.PACKET_CONTROLLER_STATE_CHECKSUM:
                                    crc8 = checksum_crc8(bytes(self.frame))
                                    if crc8 == dat:
                                        func = PacketFunction(self.frame[0])
                                        data = bytes(self.frame[2:])
                                        if func in self.parsers:
                                            self.parsers[func](data)
                                    else:
                                        print("校验失败")
                                    self.state = PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
                                    continue
                    else:
                        # No data available, sleep briefly to prevent busy-waiting
                        time.sleep(0.005)
                else:
                    time.sleep(0.01)
            except Exception as e:
                log_msg = f"Error in recv_task, but continuing: {e}"
                if self.logger:
                    self.logger.error(log_msg)
                else:
                    print(log_msg)
                time.sleep(0.2)
        self.port.close()
        print("END...")


if __name__ == "__main__":
    board = Board()
    board.enable_reception()
    print("START...")
    #time.sleep(2)
    board.set_led(0.1, 0.9, 1,1)
    #board.set_led(0.1, 0.9, 5,2)
    board.set_buzzer(1900, 0.05, 0.01, 1)
    #time.sleep(1)
    #board.set_buzzer(1900, 0.05, 0.01, 1)
    #time.sleep(1)
    #board.set_rgb([[2, 100, 0, 0],[1,100,0,0]])
    #time.sleep(0.5)
    #board.set_rgb([[2, 0, 0, 255],[1,0,0,255]])
    #time.sleep(0.5)
    #board.set_rgb([[2, 255, 0, 0],[1,255,0,0]])
    #time.sleep(0.5)
    #board.set_rgb([[1, 0, 255, 0]])
    #board.set_motor_speed([[1, -0.6], [2, -0.6], [3, 0.6], [4, 0.6]])
    #time.sleep(1)
    #board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])
    
    #bus_servo_test(board)
    #board.bus_servo_set_position(1, [[1, 700], [2, 500]])
    # pwm_servo_test(board)
    # last_time = time.time()
    while True:
        try:
            # board.set_buzzer(3000, 0.05, 0.01, 1)
            res = board.get_imu()
            if res is not None:
                for item in res:
                   print("  {: .8f} ".format(item), end='')
                print()
            # res = board.get_button()
            # if res is not None:
                # print(res)
            # data = board.get_gamepad()
            # if data is not None:
                # print(data[0])
                # print(data[1])
            # res = board.get_sbus()
            # if res is not None:
                # print(res)
            # res = board.get_battery()
            # if res is not None:
                # print(res)
            #board.set_rgb([[2, 50, 0, 0],[1,50,0,0]])
            #time.sleep(0.05)
            #board.set_rgb([[2, 0, 50, 0],[1,0,50,0]])
            #time.sleep(0.05)
            #board.set_rgb([[2, 255, 0, 0],[1,255,0,0]])
            
            #time.sleep(0.1)
            # t = time.time()
            # print(1/(t - last_time))
            # last_time = t
        except KeyboardInterrupt:
            break
