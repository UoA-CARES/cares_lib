import logging
import dynamixel_sdk as dxl
import time

import numpy as np
from functools import wraps

from cares_lib.dynamixel.Servo import Servo, DynamixelServoError, OperatingMode
from cares_lib.dynamixel.servos_addresses import addresses
from cares_lib.dynamixel.gripper_configuration import GripperConfig

MOVING_STATUS_THRESHOLD = 20


def exception_handler(error_message):
    def decorator(function):
        @wraps(function)
        def wrapper(self, *args, **kwargs):
            try:
                return function(self, *args, **kwargs)
            except (GripperError, DynamixelServoError) as error:
                logging.error(f"Gripper#{self.gripper_id}: {error_message}")
                raise GripperError(f"Gripper#{self.gripper_id}: {error_message}") from error
        return wrapper
    return decorator

class GripperError(IOError):
    pass

class Gripper(object):
    def __init__(self, config: GripperConfig):

        self.config = config

        self.gripper_id = config.gripper_id

        self.num_motors = config.num_motors
        self.min_values = config.min_values
        self.max_values = config.max_values
        self.speed_limit = config.speed_limit
        self.velocity_min = config.velocity_min
        self.velocity_max = config.velocity_max

        self.home_sequence = config.home_sequence
        self.device_name = config.device_name
        self.baudrate = config.baudrate

        self.protocol = 2  # NOTE: All current servos uses protocol 2, update needed

        self.port_handler = dxl.PortHandler(self.device_name)
        self.packet_handler = dxl.PacketHandler(self.protocol)
        self.setup_handlers()

        self.group_bulk_write = dxl.GroupBulkWrite(self.port_handler, self.packet_handler)
        self.group_bulk_read = dxl.GroupBulkRead(self.port_handler, self.packet_handler)

        self.servos = {}
        self.servo_type = config.servo_type

        self.velocity_in_pos_control = "moving_speed" if self.servo_type == "XL-320" else "profile_velocity"

        try:
            for id in range(1, self.num_motors + 1):
                led = id % 7
                self.servos[id] = Servo(self.port_handler, self.packet_handler, self.protocol, id, led,
                                        config.torque_limit, config.speed_limit, self.max_values[id - 1],
                                        self.min_values[id - 1], self.servo_type)
            self.setup_servos()
        except (GripperError, DynamixelServoError) as error:
            raise GripperError(f"Gripper#{self.gripper_id}: Failed to initialise servos") from error
    
    @exception_handler("Failed to reboot servos")
    def reboot(self):
        for _, servo in self.servos.items():
            servo.reboot()
            time.sleep(0.5)

        self.close()
        self.__init__(self.config)
        logging.info(f"Gripper#{self.gripper_id}: reboot completed")

    @exception_handler("Failed to setup handlers")
    def setup_handlers(self):
        if not self.port_handler.openPort():
            error_message = f"Failed to open port {self.device_name}"
            logging.error(error_message)
            raise IOError(error_message)
        logging.info(f"Succeeded to open port {self.device_name}")

        if not self.port_handler.setBaudRate(self.baudrate):
            error_message = f"Failed to change the baudrate to {self.baudrate}"
            logging.error(error_message)
            raise IOError(error_message)
        logging.info(f"Succeeded to change the baudrate to {self.baudrate}")

    @exception_handler("Failed to setup servos")
    def setup_servos(self):
        for _, servo in self.servos.items():
            servo.enable()

    @exception_handler("Failed to ping")
    def ping(self):
        for _, servo in self.servos.items():
            servo.ping()

    @exception_handler("Failed to read state")
    def state(self):
        current_state = {}
        current_state["positions"] = self.current_positions()
        current_state["velocities"] = self.current_velocity()
        current_state["loads"] = self.current_load()

        return current_state        

    @exception_handler("Failed to step")
    def step(self):
        state = self.state()
        for motor_id, servo in self.servos.items():
            index = motor_id - 1
            current_position = state["positions"][index]
            current_velocity = Servo.velocity_to_int(state["velocities"][index])
            logging.debug(f"Current Velocity {current_velocity} : {servo.min} < {current_position} < {servo.max}")
            if (current_position >= servo.max and current_velocity > 0) or \
                (current_position <= servo.min and current_velocity < 0):
                logging.debug(f"Dynamixel#{motor_id}: position out of boundry, stopping servo")
                servo.move_velocity(0)
        return state


    @exception_handler("Failed to read current position")
    def current_positions(self):
        return self.bulk_read("current_position")

    @exception_handler("Failed to read current velocity")
    def current_velocity(self):
        return self.bulk_read("current_velocity")
    
    @exception_handler("Failed to read goal velocity")
    def goal_velocity(self):
        return self.bulk_read(self.velocity_in_pos_control)

    @exception_handler("Failed to read current load")
    def current_load(self):
        return self.bulk_read("current_load")

    @exception_handler("Failed to read current operating mode")
    def current_operating_mode(self):
        return self.bulk_read("operating_mode")
    
    @exception_handler("Failed to read shutdown")
    def read_shutdown(self):
        return self.bulk_read("shutdown") 

    @exception_handler("Failed to read hardware errors")
    def read_hardware_errors(self):
        return self.bulk_read("hardware_error_status") 
    
    @exception_handler("Failed to check if moving")
    def is_moving(self):
        gripper_moving = False
        for _, servo in self.servos.items():
            gripper_moving |= servo.is_moving()
        return gripper_moving

    @exception_handler("Failed to stop moving")
    def stop_moving(self):
        for _, servo in self.servos.items():
            servo.stop_moving()

    @exception_handler("Failed while moving servo")
    def move_servo(self, servo_id, target_step, wait=True, timeout=5):
        if servo_id not in self.servos:
            error_message = f"Dynamixel#{servo_id} is not associated to Gripper#{self.gripper_id}"
            logging.error(error_message)
            raise ValueError(error_message)

        self.servos[servo_id].move(target_step, wait=wait, timeout=timeout)

    @exception_handler("Failed while moving servo by velocity")
    def move_servo_velocity(self, servo_id, target_velocity):
        if servo_id not in self.servos:
            error_message = (f"Dynamixel#{servo_id} is not associated to Gripper#{self.gripper_id}")
            logging.error(error_message)
            raise ValueError(error_message)
        
        self.servos[servo_id].move_velocity(target_velocity)

    @exception_handler("Failed while trying to move by steps")
    def move(self, steps, wait=True, timeout=2):
        if not self.verify_steps(steps):
            error_message = f"Gripper#{self.gripper_id}: The move command provided is out of bounds: Step {steps}"
            logging.error(error_message)
            raise ValueError(error_message)

        # uncomment if use move_velocity_wheel
        # self.move_velocity(np.full(self.num_motors,self.speed_limit),True)
        # self.set_operating_mode(np.full(self.num_motors,OperatingMode.JOINT.value))
        
        for servo_id, servo in self.servos.items():
            target_position = steps[servo_id - 1]

            param_goal_position = servo.data_to_bytes(target_position, servo.addresses["goal_position_length"])
            dxl_result = self.group_bulk_write.addParam(servo_id, servo.addresses["goal_position"], servo.addresses["goal_position_length"], param_goal_position)

            if not dxl_result:
                error_message = f"Gripper#{self.gripper_id}: Failed to setup move command for Dynamixel#{servo_id}"
                logging.error(error_message)
                raise GripperError(error_message)

        dxl_comm_result = self.group_bulk_write.txPacket()
        if dxl_comm_result != dxl.COMM_SUCCESS:
            error_message = f"Gripper#{self.gripper_id}: Failed to send move command to gripper"
            logging.error(error_message)
            raise GripperError(error_message)

        logging.debug(f"Gripper#{self.gripper_id}: Sending move command succeeded")
    
        start_time = time.perf_counter()
        while wait and self.is_moving() and time.perf_counter() < start_time + timeout:
            pass

        self.group_bulk_write.clearParam()

    @exception_handler("Failed while trying to move by velocity")
    def set_velocity(self, velocities):
        if not self.verify_velocity(velocities):
            error_message = f"Gripper#{self.gripper_id}: The move velocity command provided is out of bounds velocities {velocities}"
            logging.error(error_message)
            raise ValueError(error_message)

        for servo_id, servo in self.servos.items():
            target_velocity = velocities[servo_id-1]
            target_velocity_b = servo.velocity_to_bytes(target_velocity)

            param_goal_velocity = servo.data_to_bytes(target_velocity_b, servo.addresses[self.velocity_in_pos_control+"_length"])
            dxl_result = self.group_bulk_write.addParam(servo_id, servo.addresses[self.velocity_in_pos_control], servo.addresses[self.velocity_in_pos_control+"_length"], param_goal_velocity)

            if not dxl_result:
                error_message = f"Gripper#{self.gripper_id}: Failed to setup set velocity command for Dynamixel#{servo_id}"
                logging.error(error_message)
                raise GripperError(error_message)

        dxl_comm_result = self.group_bulk_write.txPacket()
        if dxl_comm_result != dxl.COMM_SUCCESS:
            error_message = f"Gripper#{self.gripper_id}: Failed to send set velocity command to gripper"
            logging.error(error_message)
            raise GripperError(error_message)

        logging.debug(f"Gripper#{self.gripper_id}: Sending move velocity command succeeded")
        self.group_bulk_write.clearParam()

    @exception_handler("Failed while trying to move by velocity joint")
    def move_velocity_joint(self, velocities):
        steps = []
        for servo_id, servo in self.servos.items():
            step = servo.min if velocities[servo_id-1] < 0 else servo.max
            steps.append(step)
            velocities[servo_id-1] = np.abs(velocities[servo_id-1])
        
        self.move(steps, wait=False)
        self.set_velocity(velocities)

    @exception_handler("Failed while trying to move by velocity wheel")
    def move_velocity_wheel(self, velocities):
        if not self.verify_velocity(velocities):
            error_message = f"Gripper#{self.gripper_id}: The move velocity command provided is out of bounds velocities {velocities}"
            logging.error(error_message)
            raise ValueError(error_message)
        
        self.set_operating_mode(np.full(self.num_motors,OperatingMode.WHEEL.value))

        for servo_id, servo in self.servos.items():
            target_velocity = velocities[servo_id-1]
            target_velocity_b = servo.velocity_to_bytes(target_velocity)

            if not servo.validate_movement(target_velocity):
                continue

            param_goal_velocity = servo.data_to_bytes(target_velocity, servo.addresses[self.velocity_in_pos_control+"_length"])
            dxl_result = self.group_bulk_write.addParam(servo_id, servo.addresses[self.velocity_in_pos_control], servo.addresses[self.velocity_in_pos_control+"_length"], param_goal_velocity)

            if not dxl_result:
                error_message = f"Gripper#{self.gripper_id}: Failed to setup move velocity command for Dynamixel#{servo_id}"
                logging.error(error_message)
                raise GripperError(error_message)

        dxl_comm_result = self.group_bulk_write.txPacket()
        if dxl_comm_result != dxl.COMM_SUCCESS:
            error_message = f"Gripper#{self.gripper_id}: Failed to send move velocity command to gripper"
            logging.error(error_message)
            raise GripperError(error_message)

        logging.debug(f"Gripper#{self.gripper_id}: Sending move velocity command succeeded")
        self.group_bulk_write.clearParam()
    
    @exception_handler("Failed to home")
    def home(self):
        pose = self.home_sequence[-1]
        self.move(pose)

        if not self.is_home():
            error_message = f"Gripper#{self.gripper_id}: failed to Home"
            logging.error(error_message)
            raise GripperError(error_message)

        logging.debug(f"Gripper#{self.gripper_id}: in home position")

    @exception_handler("Failed to wiggle home")
    def wiggle_home(self):
        for pose in self.home_sequence:
            self.move(pose)

        if not self.is_home():
            error_message = f"Gripper#{self.gripper_id}: Failed to wiggle home"
            logging.error(error_message)
            raise GripperError(error_message)

        logging.debug(f"Gripper#{self.gripper_id}: in home position")
        return True
        
    @exception_handler("Failed to read if home")
    def is_home(self):
        return np.all(np.abs(self.home_sequence[-1] - np.array(self.current_positions())) <= MOVING_STATUS_THRESHOLD)
    
    @exception_handler("Failed to set operating mode")
    def set_operating_mode(self, new_mode):
        if (new_mode != self.current_operating_mode()).all():
            self.disable_torque()#disable to set servo parameters

            for servo_id, servo in self.servos.items():            
                dxl_result = self.group_bulk_write.addParam(servo_id, servo.addresses["operating_mode"], 1, [new_mode[servo_id-1]])

                if not dxl_result:
                    error_message = f"Gripper#{self.gripper_id}: Failed to add operating mode param for Dynamixel#{servo_id}"
                    logging.error(error_message)
                    raise GripperError(error_message)

            dxl_comm_result = self.group_bulk_write.txPacket()
            if dxl_comm_result != dxl.COMM_SUCCESS:
                error_message = f"Gripper#{self.gripper_id}: failed to send change operating mode command to gripper"
                logging.error(error_message)
                raise GripperError(error_message)

            logging.debug(f"Gripper#{self.gripper_id}: Change operating mode command succeeded")
            self.group_bulk_write.clearParam()

            self.enable_torque()

    @exception_handler("Failed to enable tourque")
    def enable_torque(self):
        for servo_id, servo in self.servos.items():   
            dxl_result = self.group_bulk_write.addParam(servo_id, servo.addresses["torque_enable"], 1, [1])

            if not dxl_result:
                error_message = f"Gripper#{self.gripper_id}: Failed to add torque enable param for Dynamixel#{servo_id}"
                logging.error(error_message)

        dxl_comm_result = self.group_bulk_write.txPacket()
        if dxl_comm_result != dxl.COMM_SUCCESS:
            error_message = f"Gripper#{self.gripper_id}: Failed to send enable torque command to gripper"
            logging.error(error_message)
            raise GripperError(error_message)

        self.group_bulk_write.clearParam()
        
    @exception_handler("Failed to disable tourque")
    def disable_torque(self):
        for servo_id, servo in self.servos.items():            
            dxl_result = self.group_bulk_write.addParam(servo_id, servo.addresses["torque_enable"], 1, [0])    

            if not dxl_result:
                error_message = f"Gripper#{self.gripper_id}: Failed to add torque disable param for Dynamixel#{servo_id}"
                logging.error(error_message)

        dxl_comm_result = self.group_bulk_write.txPacket()
        if dxl_comm_result != dxl.COMM_SUCCESS:
            error_message = f"Gripper#{self.gripper_id}: Failed to send disable torque command to gripper"
            logging.error(error_message)
            raise GripperError(error_message)

        self.group_bulk_write.clearParam()

    def verify_steps(self, steps):
        for servo_id, servo in self.servos.items():
            step = steps[servo_id - 1]
            if not servo.verify_step(step):
                logging.warn(
                    f"Gripper#{self.gripper_id}: step for servo {servo_id} is out of bounds: {servo.min} to {servo.max}")
                return False
        return True

    def verify_velocity(self, velocities):
        for servo_id, servo in self.servos.items():
            velocity = velocities[servo_id - 1]
            if not servo.verify_velocity(velocity):
                logging.warn(f"Gripper#{self.gripper_id}: velocity for servo {servo_id} is out of bounds")
                return False
        return True
    
    @exception_handler("Failed to bulk read")
    def bulk_read(self, address):
        readings = []
        length = addresses[self.servo_type][address+"_length"]
        for id, _ in self.servos.items():
            self.bulk_read_addparam(id, address, length)

        dxl_comm_result = self.group_bulk_read.txRxPacket()
        if dxl_comm_result != dxl.COMM_SUCCESS:
            error_message = f"Gripper#{self.gripper_id}: {self.packet_handler.getTxRxResult(dxl_comm_result)}"
            logging.error(error_message)
            raise GripperError(error_message)

        for id, servo in self.servos.items():
            readings.append(self.group_bulk_read.getData(id, servo.addresses[address], length))

        self.group_bulk_read.clearParam()
        return readings

    # do the same for length as above and test
    def bulk_read_addparam(self, servo_id, address, length):
        dxl_addparam_result = self.group_bulk_read.addParam(servo_id, self.servos[servo_id].addresses[address], length)
        if not dxl_addparam_result:
            error_message = f"Gripper#{self.gripper_id} - Dynamixel#{servo_id}: groupBulkRead addparam {address} failed"
            logging.error(error_message)
            raise GripperError(error_message)

    def check_bulk_read_avaliability(self, servo_id, address, length):
        dxl_getdata_result = self.group_bulk_read.isAvailable(servo_id, self.servos[servo_id].addresses[address], length)
        if not dxl_getdata_result:
            error_message = f"Gripper#{self.gripper_id} - Dynamixel#{servo_id}: groupBulkRead {address} unavaliable"
            logging.error(error_message)
            raise GripperError(error_message)
        
    def close(self):
        logging.info(f"Closing Gripper{self.gripper_id}")
        for _, servo in self.servos.items():
            self.disable_torque()
        
        self.port_handler.closePort()