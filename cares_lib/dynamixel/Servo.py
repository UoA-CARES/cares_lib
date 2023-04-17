import logging
import time
import dynamixel_sdk as dxl
from enum import Enum
import ctypes

"""
The servo class contains methods used to change attributes of the servo motors
most useful for setting up the servos to move when they are chained togerther
Beth Cutler
"""

DXL_MOVING_STATUS_THRESHOLD = 10


class ControlMode(Enum):
    WHEEL = 1
    JOINT = 2


class DynamixelServoError(IOError):
    pass


class Servo(object):
    addresses = {
        "control_mode": 11,
        "shutdown": 18,
        "torque_enable": 24,
        "led": 25,
        "goal_position": 30,
        "moving_speed": 32,
        "torque_limit": 35,
        "current_position": 37,
        "current_velocity": 39,
        "current_load": 41,
        "moving": 49,
    }

    def __init__(self, port_handler, packet_handler, protocol, motor_id, LED_colour, torque_limit, max_velocity, max, min):
        self.port_handler = port_handler
        self.packet_handler = packet_handler
        self.protocol = protocol

        self.motor_id = motor_id
        self.LED_colour = LED_colour

        self.torque_limit = torque_limit
        self.max_velocity = max_velocity

        self.max = max
        self.min = min

    def enable(self):
        try:
            self.disable_torque()
            self.set_control_mode(ControlMode.JOINT.value)  # default set to joint mode to avoid moving on start up
            self.limit_torque()
            self.limit_speed()
            self.enable_torque()
            self.turn_on_LED()
        except DynamixelServoError as error:
            raise DynamixelServoError(f"Dynamixel#{self.motor_id}: failed to enable") from error

    def state(self):
        current_state = {}
        current_state["position"] = self.current_position()
        current_state["velocity"] = self.velocity_to_int(self.current_velocity())
        current_state["load"] = self.current_load()

        return current_state

    def step(self):
        control_mode = self.control_mode()
        if control_mode == ControlMode.JOINT.value:
            return

        try:
            current_state = self.state()
            current_position = current_state["position"]
            current_velocity = Servo.velocity_to_int(current_state["velocity"])

            logging.debug(f"Current Velocity {current_velocity} : {self.min} < {current_position} < {self.max}")
            if (current_position >= self.max and current_velocity > 0) or \
                    (current_position <= self.min and current_velocity < 0):
                self.move_velocity(0)
                logging.warn(f"Dynamixel#{self.motor_id}: position out of boundry, stopping servo")

        except DynamixelServoError as error:
            raise DynamixelServoError(f"Dynamixel#{self.motor_id}: failed during step") from error

    def ping(self):
        try:
            model_number, dxl_comm_result, dxl_error = self.packet_handler.ping(self.port_handler, self.motor_id)
            self.process_result(dxl_comm_result, dxl_error,
                                message=f"successfully pinged Dynamixel#{self.motor_id} as model {model_number}")
        except DynamixelServoError as error:
            raise DynamixelServoError(f"Dynamixel#{self.motor_id}: failed to ping") from error

    def control_mode(self):
        try:
            current_mode, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(self.port_handler,
                                                                                         self.motor_id, Servo.addresses[
                                                                                             "control_mode"])
            self.process_result(dxl_comm_result, dxl_error,
                                message=f"Dynamixel#{self.motor_id}: successfully read control mode as {current_mode}")
            return current_mode
        except DynamixelServoError as error:
            raise DynamixelServoError(f"Dynamixel#{self.motor_id}: failed to read control mode") from error

    def set_control_mode(self, new_mode):
        try:
            self.disable_torque()  # disable to set servo parameters

            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, self.motor_id,
                                                                            Servo.addresses["control_mode"], new_mode)
            self.process_result(dxl_comm_result, dxl_error, message=f"successfully set control mode to {new_mode}")

            self.enable_torque()
        except DynamixelServoError as error:
            error_message = f"Dynamixel#{self.motor_id}: failed to set control mode to {new_mode}"
            logging.error(error_message)
            raise DynamixelServoError(error_message) from error

    def move(self, target_position, wait=True, timeout=5):
        if not self.verify_step(target_position):
            error_message = f"Dynamixel#{self.motor_id}: Target position {target_position} is out of bounds of min {self.min} max {self.max}"
            logging.error(error_message)
            raise DynamixelServoError(error_message)

        try:
            self.set_control_mode(ControlMode.JOINT.value)

            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, self.motor_id,
                                                                            Servo.addresses["goal_position"],
                                                                            target_position)
            self.process_result(dxl_comm_result, dxl_error,
                                message=f"Dynamixel#{self.motor_id}: successfully told to move to {target_position}")

            start_time = time.perf_counter()
            while wait and self.is_moving() and time.perf_counter() < start_time + timeout:
                pass

            return self.current_position()
        except DynamixelServoError as error:
            raise DynamixelServoError(f"Dynamixel#{self.motor_id}: failed while moving") from error

    def move_velocity(self, target_velocity):
        if not self.verify_velocity(target_velocity):
            error_message = f"Dynamixel#{self.motor_id}: Target velocity {target_velocity} over max velocity {self.max_velocity}"
            logging.error(error_message)
            raise DynamixelServoError(error_message)

        try:
            self.set_control_mode(ControlMode.WHEEL.value)

            if not self.validate_movement(target_velocity):
                return self.current_velocity()

            processed_velocity = Servo.velocity_to_bytes(target_velocity)
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, self.motor_id,
                                                                            Servo.addresses["moving_speed"],
                                                                            processed_velocity)
            self.process_result(dxl_comm_result, dxl_error,
                                message=f"Dynamixel#{self.motor_id}: successfully told to move to at {target_velocity}")

            return self.current_velocity()
        except DynamixelServoError as error:
            raise DynamixelServoError(f"Dynamixel#{self.motor_id}: failed while moving") from error

    def stop_moving(self):
        try:
            return self.move(self.current_position())
        except DynamixelServoError as error:
            error_message = f"Dynamixel#{self.motor_id}: failed to stop"
            logging.error(error_message)
            raise DynamixelServoError(error_message) from error

    def is_moving(self):
        try:
            current_position = self.current_position()
            target_position = self.current_target_position()

            logging.debug(
                f"Dynamixel#{self.motor_id} is at position {current_position} and moving to {target_position}")
            return abs(current_position - target_position) > DXL_MOVING_STATUS_THRESHOLD
        except DynamixelServoError as error:
            error_message = f"Dynamixel#{self.motor_id}: failed to check if moving"
            logging.error(error_message)
            raise DynamixelServoError(error_message) from error

    def current_position(self):
        try:
            data_read, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, self.motor_id,
                                                                                      Servo.addresses[
                                                                                          "current_position"])
            self.process_result(dxl_comm_result, dxl_error, f"Dynamixel#{self.motor_id}: measured position {data_read}")
            return data_read
        except DynamixelServoError as error:
            error_message = (f"Dynamixel#{self.motor_id}: failed to read current poisiton")
            logging.error(error_message)
            raise DynamixelServoError(error_message) from error

    def current_target_position(self):
        try:
            data_read, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, self.motor_id,
                                                                                      Servo.addresses["goal_position"])
            self.process_result(dxl_comm_result, dxl_error,
                                f"Dynamixel#{self.motor_id}: current goal position {data_read}")
            return data_read
        except DynamixelServoError as error:
            error_message = (f"Dynamixel#{self.motor_id}: failed to read current goal poisiton")
            logging.error(error_message)
            raise DynamixelServoError(error_message) from error

    def current_velocity(self):
        try:
            data_read, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, self.motor_id,
                                                                                      Servo.addresses[
                                                                                          "current_velocity"])
            self.process_result(dxl_comm_result, dxl_error, f"Dynamixel#{self.motor_id}: measured velocity {data_read}")
            return data_read
        except DynamixelServoError as error:
            error_message = (f"Dynamixel#{self.motor_id}: failed to read current velocity")
            logging.error(error_message)
            raise DynamixelServoError(error_message) from error

    def current_load(self):
        try:
            data_read, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, self.motor_id,
                                                                                      Servo.addresses["current_load"])
            self.process_result(dxl_comm_result, dxl_error, f"Dynamixel#{self.motor_id}: current load is {data_read}")
            # Convert it to a value between 0 - 1023 regardless of direction and then maps this between 0-100
            # See section 2.4.21 link for details on why this is required
            # https://emanual.robotis.com/docs/en/dxl/x/xl320/
            current_load_percent = ((data_read % 1023) / 1023) * 100
            return current_load_percent
        except DynamixelServoError as error:
            error_message = f"Dynamixel#{self.motor_id}: failed to read load"
            logging.error(error_message)
            raise DynamixelServoError(error_message) from error

    def turn_on_LED(self):
        try:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, self.motor_id,
                                                                            Servo.addresses["led"], self.LED_colour)
            self.process_result(dxl_comm_result, dxl_error,
                                message=f"Dynamixel#{self.motor_id}: successfully turned on LEDs")
        except DynamixelServoError as error:
            error_message = f"Dynamixel#{self.motor_id}: failed to turn LED on"
            logging.error(error_message)
            raise DynamixelServoError(error_message) from error

    def limit_speed(self):
        try:
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, self.motor_id,
                                                                            Servo.addresses["moving_speed"],
                                                                            self.max_velocity)
            self.process_result(dxl_comm_result, dxl_error,
                                message=f"Dynamixel#{self.motor_id}: has been successfully speed limited")
        except DynamixelServoError as error:
            error_message = f"Dynamixel#{self.motor_id}: failed to limit speed"
            logging.error(error_message)
            raise DynamixelServoError(error_message) from error

    def limit_torque(self):
        try:
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, self.motor_id,
                                                                            Servo.addresses["torque_limit"],
                                                                            self.torque_limit)
            self.process_result(dxl_comm_result, dxl_error,
                                message=f"Dynamixel#{self.motor_id}: has been successfully torque limited")
        except DynamixelServoError as error:
            error_message = f"Dynamixel#{self.motor_id}: failed to limit torque"
            logging.error(error_message)
            raise DynamixelServoError(error_message) from error

    def enable_torque(self):
        try:
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, self.motor_id,
                                                                            Servo.addresses["torque_enable"], 1)
            self.process_result(dxl_comm_result, dxl_error,
                                message=f"Dynamixel#{self.motor_id}: has been successfully torque enabled")
        except DynamixelServoError as error:
            error_message = f"Dynamixel#{self.motor_id}: failed to enable torque"
            logging.error(error_message)
            raise DynamixelServoError(error_message) from error

    def disable_torque(self):
        try:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, self.motor_id,
                                                                            Servo.addresses["torque_enable"], 0)
            self.process_result(dxl_comm_result, dxl_error,
                                message=f"Dynamixel#{self.motor_id}: has successfully disabled torque")
        except DynamixelServoError as error:
            error_message = f"Dynamixel#{self.motor_id}: failed to disable torque"
            logging.error(error_message)
            raise DynamixelServoError(error_message) from error

    def verify_step(self, step):
        return self.min <= step <= self.max

    def verify_velocity(self, velocity):
        if abs(velocity) > self.max_velocity:
            error_message = f"Dynamixel#{self.motor_id}: Target velocity {velocity} over max velocity {self.max_velocity}"
            logging.warn(error_message)
            return False
        return True

    def validate_movement(self, target_velocity):
        current_position = self.current_position()
        if (current_position >= self.max and target_velocity > 0) or \
                (current_position <= self.min and target_velocity < 0):
            logging.warn(f"Dynamixel#{self.motor_id}: position out of boundry, setting velocity not valid")
            return False
        return True

    def process_result(self, dxl_comm_result, dxl_error, message="success"):
        if (dxl_comm_result != dxl.COMM_SUCCESS) or (dxl_error != 0):
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

    @staticmethod
    def velocity_to_bytes(target_velocity):
        # If a value in the range of 0~1,023 is used, it is stopped by setting to 0 while rotating to CCW direction.
        # If a value in the range of 1,024~2,047 is used, it is stopped by setting to 1,024 while rotating to CW direction.
        if target_velocity < 0:
            return abs(target_velocity) + 1024  # CCW 0-1023
        return target_velocity  # CW 1024-2047

    @staticmethod
    def velocity_to_int(target_velocity):
        if target_velocity >= 1024:
            return -(target_velocity - 1024)

        else:
            return target_velocity

        # TODO expand the error code returns to fit the actual servo error messages
# ERRNUM_RESULT_FAIL = 1  # Failed to process the instruction packet.
# ERRNUM_INSTRUCTION = 2  # Instruction error
# ERRNUM_CRC = 3  # CRC check error
# ERRNUM_DATA_RANGE = 4  # Data range error
# ERRNUM_DATA_LENGTH = 5  # Data length error
# ERRNUM_DATA_LIMIT = 6  # Data limit error
# ERRNUM_ACCESS = 7  # Access error