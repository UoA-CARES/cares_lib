import logging
import time
import dynamixel_sdk as dxl

"""
The servo class contains methods used to change attributes of the servo motors
most useful for setting up the servos to move when they are chained togerther
Beth Cutler
"""

DXL_MOVING_STATUS_THRESHOLD = 10

class DynamixelServoError(IOError):
    pass

class Servo(object):
    addresses = {
        "shutdown" : 18,
        "torque_enable": 24,
        "led": 25,
        "goal_position": 30,
        "moving_speed": 32,
        "torque_limit": 35,
        "current_position": 37,
        "current_velocity": 38,
        "current_load": 41,
        "moving": 49
    }

    def __init__(self, port_handler, packet_handler, LED_colour, motor_id, torque_limit, speed_limit, max, min):
        self.port_handler = port_handler
        self.packet_handler = packet_handler

        self.LED_colour = LED_colour
        self.motor_id = motor_id

        self.torque_limit = torque_limit
        self.speed_limit = speed_limit

        self.max = max
        self.min = min

        self.target_position = self.current_position()

    def move(self, target_position, wait=True, timeout=5):
        if not self.verify_step(target_position):
            error_message = f"Dynamixel#{self.motor_id}: Target position {target_position} is out of bounds of min {self.min} max {self.max}"
            logging.error(error_message)
            raise DynamixelServoError(error_message)

        try:
            self.target_position = target_position

            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, self.motor_id, Servo.addresses["goal_position"], self.target_position)
            self.process_result(dxl_comm_result, dxl_error, message=f"successfully told to move to {self.target_position}")

            start_time = time.perf_counter()
            while wait and self.is_moving() and time.perf_time() < start_time + timeout:
                pass

            return self.current_position()
        except DynamixelServoError as error:
            raise DynamixelServoError(f"Dynamixel#{self.motor_id}: failed while moving") from error

    def stop_moving(self):
        try:
            return self.move(self.current_position())
        except DynamixelServoError as error:
            error_message = f"Dynamixel#{self.motor_id}: failed to stop"
            logging.error(error_message)
            raise DynamixelServoError(error_message) from error

    def turn_on_LED(self):
        try:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, self.motor_id, Servo.addresses["led"], self.LED_colour)
            self.process_result(dxl_comm_result, dxl_error, message="successfully turned on LEDs")
        except DynamixelServoError as error:
            error_message = f"Dynamixel#{self.motor_id}: failed to turn LED on"
            logging.error(error_message)
            raise DynamixelServoError(error_message) from error

    def limit_torque(self):
        try:
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, self.motor_id, Servo.addresses["torque_limit"], self.torque_limit)
            self.process_result(dxl_comm_result, dxl_error, message="has been successfully torque limited")
        except DynamixelServoError as error:
            error_message = f"Dynamixel#{self.motor_id}: failed to limit torque"
            logging.error(error_message)
            raise DynamixelServoError(error_message) from error

    def enable_torque(self):
        try:
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, self.motor_id, Servo.addresses["torque_enable"], 1)
            self.process_result(dxl_comm_result, dxl_error, message="has been successfully torque enabled")
        except DynamixelServoError as error:
            error_message = f"Dynamixel#{self.motor_id}: failed to enable torque"
            logging.error(error_message)
            raise DynamixelServoError(error_message) from error

    def disable_torque(self):
        try:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, self.motor_id, Servo.addresses["torque_enable"], 0)
            self.process_result(dxl_comm_result, dxl_error, message="has successfully disabled torque" )
        except DynamixelServoError as error:
            error_message = f"Dynamixel#{self.motor_id}: failed to disable torque"
            logging.error(error_message)
            raise DynamixelServoError(error_message) from error

    def limit_speed(self):
        try:
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, self.motor_id, Servo.addresses["moving_speed"], self.speed_limit)        
            self.process_result(dxl_comm_result, dxl_error, message="has been successfully speed limited")
        except DynamixelServoError as error:
            error_message = f"Dynamixel#{self.motor_id}: failed to limit speed"
            logging.error(error_message)
            raise DynamixelServoError(error_message) from error

    def is_moving(self):
        try:
            current_position = self.current_position()
            logging.debug(f"Dynamixel#{self.motor_id} is at position {current_position} and moving to {self.target_position}")
            return abs(current_position - self.target_position) > DXL_MOVING_STATUS_THRESHOLD
        except DynamixelServoError as error:
            error_message = f"Dynamixel#{self.motor_id}: failed to check if moving"
            logging.error(error_message)
            raise DynamixelServoError(error_message) from error

    def current_position(self):
        try:
            data_read, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, self.motor_id, Servo.addresses["current_position"])
            self.process_result(dxl_comm_result, dxl_error, f"measured position {data_read}")
            return data_read
        except DynamixelServoError as error:
            error_message = f"Dynamixel#{self.motor_id}: failed to read current poisiton"
            logging.error(error_message)
            raise DynamixelServoError(error_message) from error

    def current_load(self): 
        try:
            data_read, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, self.motor_id, Servo.addresses["current_load"])
            self.process_result(dxl_comm_result, dxl_error, f"current load is {data_read}")
            # Convert it to a value between 0 - 1023 regardless of direction and then maps this between 0-100
            # See section 2.4.21 link for details on why this is required
            # https://emanual.robotis.com/docs/en/dxl/x/xl320/ 
            current_load_percent = ((data_read % 1023) / 1023) * 100
            return current_load_percent
        except DynamixelServoError as error:
            error_message = f"Dynamixel#{self.motor_id}: failed to read load"
            logging.error(error_message)
            raise DynamixelServoError(error_message) from error

    def verify_step(self, step):
        return self.min <= step <= self.max

    def process_result(self, dxl_comm_result, dxl_error, message="success"):
        if dxl_comm_result != dxl.COMM_SUCCESS:
            error_message = f"Dynamixel#{self.motor_id} {self.packet_handler.getTxRxResult(dxl_comm_result)} {self.packet_handler.getRxPacketError(dxl_error)}"
            logging.error(error_message)
            raise DynamixelServoError(error_message)

        logging.debug(f"Dynamixel#{self.motor_id}: {message}")
        
    @staticmethod
    def step_to_angle(step):
        # 0 to 1023 steps to -150 to 150 degrees 
        return (step - 511.5) / 3.41

    @staticmethod
    def angle_to_step(angle):
        #  -150 to 150 degrees to 0 to 1023 steps
        return (3.41 * angle) + 511.5

# TODO expand the error code returns to fit the actual servo error messages
# ERRNUM_RESULT_FAIL = 1  # Failed to process the instruction packet.
# ERRNUM_INSTRUCTION = 2  # Instruction error
# ERRNUM_CRC = 3  # CRC check error
# ERRNUM_DATA_RANGE = 4  # Data range error
# ERRNUM_DATA_LENGTH = 5  # Data length error
# ERRNUM_DATA_LIMIT = 6  # Data limit error
# ERRNUM_ACCESS = 7  # Access error