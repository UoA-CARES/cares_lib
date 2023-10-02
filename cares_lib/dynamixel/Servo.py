import logging
import time
import dynamixel_sdk as dxl
from enum import Enum
from functools import wraps

"""
The servo class contains methods used to change attributes of the servo motors
most useful for setting up the servos to move when they are chained togerther
Beth Cutler
"""

DXL_MOVING_STATUS_THRESHOLD = 10

def exception_handler(error_message):
    def decorator(function):
        @wraps(function)
        def wrapper(self, *args, **kwargs):
            try:
                return function(self, *args, **kwargs)
            except DynamixelServoError as error:
                logging.error(f"Dynamixel#{error.servo.motor_id}: {error_message}")
                raise DynamixelServoError(error.servo, f"Dynamixel#{error.servo.motor_id}: {error_message}") from error
        
        return wrapper
    return decorator


class ControlMode(Enum):
    WHEEL = 1
    JOINT = 2
    POSITION = 3
    EXTENDED_POSITION = 4
    PWM_CONTROL = 16

class ShutdownStatus(Enum):
    OVERLOAD = 0
    OVERHEAT = 1
    INPUT_VOLTAGE = 2

class DynamixelServoError(IOError):
    def __init__(self, servo, message):
        self.servo = servo
        super().__init__(message)

addresses = {}
addresses["XL-320"] = {
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
    "moving": 49}
addresses["XL430-W250-T"] = {        
    "control_mode": 11,
    "shutdown": 63,
    "torque_enable": 64,
    "led": 65,
    "goal_position": 116,
    "moving_speed": 104,
    "current_position": 132,
    "current_velocity": 128,
    "moving": 122}
addresses["XC330-T288-T"] = {        
    "control_mode": 11,
    "shutdown": 63,
    "torque_enable": 64,
    "led": 65,
    "goal_position": 116,
    "moving_speed": 104,
    "current_position": 132,
    "current_velocity": 128,
    "moving": 122}
addresses["XM430-W350"] = {        
    "control_mode": 11,
    "shutdown": 63,
    "torque_enable": 64,
    "led": 65,
    "goal_position": 116,
    "moving_speed": 104,
    "current_position": 132,
    "current_velocity": 128,
    "moving": 122}


class Servo(object):
    def __init__(self, port_handler, packet_handler, protocol, motor_id, LED_colour, torque_limit, max_velocity, max, min, model="XL-320"):
        self.port_handler = port_handler
        self.packet_handler = packet_handler
        self.protocol = protocol

        self.motor_id = motor_id
        self.LED_colour = LED_colour

        self.torque_limit = torque_limit
        self.max_velocity = max_velocity

        self.max = max
        self.min = min

        self.model = model

        self.total_results = 0
        self.dxl_errors = 0 
        
        self.addresses = addresses[self.model]

    @exception_handler("Failed to enable")
    def enable(self):
        self.disable_torque()
        self.set_control_mode(ControlMode.JOINT.value)# default set to joint mode to avoid moving on start up
        if self.model == "XL-320":
            self.limit_torque()
            self.limit_speed()
            self.enable_torque()
        self.turn_on_LED()

    def state(self):
        current_state = {}
        current_state["position"] = self.current_position()
        if self.model == "XL-320":
            current_state["velocity"] = self.velocity_to_int(self.current_velocity())
            current_state["load"]     = self.current_load()
    
        return current_state

    @exception_handler("Failed during step")
    def step(self):
        control_mode = self.control_mode()
        if control_mode == ControlMode.JOINT.value:
            return
    
        current_state    = self.state()
        current_position = current_state["position"]
        current_velocity = Servo.velocity_to_int(current_state["velocity"])

        logging.debug(f"Current Velocity {current_velocity} : {self.min} < {current_position} < {self.max}")
        if (current_position >= self.max and current_velocity > 0) or \
            (current_position <= self.min and current_velocity < 0):
            self.move_velocity(0)
            logging.warn(f"Dynamixel#{self.motor_id}: position out of boundry, stopping servo")
 
    @exception_handler("Failed to ping")
    def ping(self): 
        model_number, dxl_comm_result, dxl_error = self.packet_handler.ping(self.port_handler, self.motor_id)
        self.process_result(dxl_comm_result, dxl_error, message=f"successfully pinged Dynamixel#{self.motor_id} as model {model_number}")
   
    @exception_handler("Failed to read control mode")
    def control_mode(self): 
        current_mode, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(self.port_handler, self.motor_id, self.addresses["control_mode"])
        self.process_result(dxl_comm_result, dxl_error, message=f"Dynamixel#{self.motor_id}: successfully read control mode as {current_mode}")
        return current_mode 

    @exception_handler("Failed to set control mode")
    def set_control_mode(self, new_mode): 
        self.disable_torque()#disable to set servo parameters
        
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, self.motor_id, self.addresses["control_mode"], new_mode)
        self.process_result(dxl_comm_result, dxl_error, message=f"successfully set control mode to {new_mode}")

        self.enable_torque() 
            
    @exception_handler("Failed while moving")
    def move(self, target_position, wait=True, timeout=5):
        if not self.verify_step(target_position):
            error_message = f"Dynamixel#{self.motor_id}: Target position {target_position} is out of bounds of min {self.min} max {self.max}"
            logging.error(error_message)
            raise DynamixelServoError(self, error_message)
 
        if self.model == "XL430-W250-T":
            self.set_control_mode(ControlMode.POSITION.value)
            dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, self.motor_id, self.addresses["goal_position"], target_position)
        else:
            self.set_control_mode(ControlMode.JOINT.value)
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, self.motor_id, self.addresses["goal_position"], target_position)
            
        self.process_result(dxl_comm_result, dxl_error, message=f"Dynamixel#{self.motor_id}: successfully told to move to {target_position}")

        start_time = time.perf_counter()
        while wait and self.is_moving() and time.perf_counter() < start_time + timeout:
            pass

        return self.current_position() 

    @exception_handler("Failed while moving by velocity")
    def move_velocity(self, target_velocity):
        if not self.verify_velocity(target_velocity):
            error_message = f"Dynamixel#{self.motor_id}: Target velocity {target_velocity} over max velocity {self.max_velocity}"
            logging.error(error_message)
            raise DynamixelServoError(self, error_message)
 
        self.set_control_mode(ControlMode.WHEEL.value)

        if not self.validate_movement(target_velocity):
            return self.current_velocity()
        
        processed_velocity = Servo.velocity_to_bytes(target_velocity)
        dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, self.motor_id, self.addresses["moving_speed"], processed_velocity)
        self.process_result(dxl_comm_result, dxl_error, message=f"Dynamixel#{self.motor_id}: successfully told to move to at {target_velocity}")
        
        return self.current_velocity() 

    @exception_handler("Failed to stop")
    def stop_moving(self): 
        return self.move(self.current_position())
         
    @exception_handler("Failed to check if moving")
    def is_moving(self): 
        current_position = self.current_position()
        target_position  = self.current_target_position()

        logging.debug(f"Dynamixel#{self.motor_id} is at position {current_position} and moving to {target_position}")
        return abs(current_position - target_position) > DXL_MOVING_STATUS_THRESHOLD 

    @exception_handler("Failed to read current position")
    def current_position(self): 
        if self.model == "XL430-W250-T":
            data_read, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, self.motor_id, self.addresses["current_position"])
        else:
            data_read, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, self.motor_id, self.addresses["current_position"])

        self.process_result(dxl_comm_result, dxl_error, f"Dynamixel#{self.motor_id}: measured position {data_read}")
        return data_read 

    @exception_handler("Failed to read current goal position")
    def current_target_position(self): 
        if self.model == "XL430-W250-T":
            data_read, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, self.motor_id, self.addresses["goal_position"])
        else:
            data_read, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, self.motor_id, self.addresses["goal_position"])
            
        self.process_result(dxl_comm_result, dxl_error, f"Dynamixel#{self.motor_id}: current goal position {data_read}")
        return data_read

    @exception_handler("Failed to read current velocity")
    def current_velocity(self): 
        data_read, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, self.motor_id, self.addresses["current_velocity"])
        self.process_result(dxl_comm_result, dxl_error, f"Dynamixel#{self.motor_id}: measured velocity {data_read}")
        return data_read 
    
    @exception_handler("Failed to read shutdown status")
    def current_shutdown(self): 
        # shutdown is 1 byte
        data_read, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(self.port_handler, self.motor_id, self.addresses["shutdown"])
        self.process_result(dxl_comm_result, dxl_error, f"Dynamixel#{self.motor_id}: measured shutdown status {data_read}")
        
        statuses = [] # an array of errors that occured 
        if(data_read & 1<<ShutdownStatus.OVERLOAD == 1): 
            statuses.append(ShutdownStatus.OVERLOAD)
        elif(data_read & 1<<ShutdownStatus.OVERHEAT == 2): 
            statuses.append(ShutdownStatus.OVERHEAT)
        elif(data_read & 1<<ShutdownStatus.INPUT_VOLTAGE == 4): 
            statuses.append(ShutdownStatus.INPUT_VOLTAGE)

        return statuses

    @exception_handler("Failed to read current load")
    def current_load(self): 
        data_read, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, self.motor_id, self.addresses["current_load"])
        self.process_result(dxl_comm_result, dxl_error, f"Dynamixel#{self.motor_id}: current load is {data_read}")
        # Convert it to a value between 0 - 1023 regardless of direction and then maps this between 0-100
        # See section 2.4.21 link for details on why this is required
        # https://emanual.robotis.com/docs/en/dxl/x/xl320/
        current_load_percent = ((data_read % 1023) / 1023) * 100
        return current_load_percent 

    @exception_handler("Failed to turn LED on")
    def turn_on_LED(self): 
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx( self.port_handler, self.motor_id, self.addresses["led"], self.LED_colour)
        self.process_result(dxl_comm_result, dxl_error, message=f"Dynamixel#{self.motor_id}: successfully turned on LEDs") 

    @exception_handler("Failed to limit speed")
    def limit_speed(self): 
        dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, self.motor_id, self.addresses["moving_speed"], self.max_velocity)
        self.process_result(dxl_comm_result, dxl_error, message=f"Dynamixel#{self.motor_id}: has been successfully speed limited")
         
    @exception_handler("Failed to limit torque")
    def limit_torque(self):
        dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, self.motor_id, self.addresses["torque_limit"], self.torque_limit)
        self.process_result(dxl_comm_result, dxl_error, message=f"Dynamixel#{self.motor_id}: has been successfully torque limited")
 
    @exception_handler("Failed to read enable torque")
    def enable_torque(self): 
        dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, self.motor_id, self.addresses["torque_enable"], 1)
        self.process_result(dxl_comm_result, dxl_error, message=f"Dynamixel#{self.motor_id}: has been successfully torque enabled")
         
    @exception_handler("Failed to disable torque")
    def disable_torque(self): 
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, self.motor_id, self.addresses["torque_enable"], 0)
        self.process_result(dxl_comm_result, dxl_error, message=f"Dynamixel#{self.motor_id}: has successfully disabled torque")
         
    @exception_handler("Failed to reboot servo")
    def reboot(self):
        dxl_comm_result, dxl_error = self.packet_handler.reboot(self.port_handler, self.motor_id)
        self.process_result(dxl_comm_result, dxl_error, message=f"Dynamixel#{self.motor_id}: has successfully rebooted servo")
    
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
        self.total_results += 1
        if (dxl_error != 0):
            self.dxl_errors += 1
            error_message = f"Dynamixel#{self.motor_id} {self.packet_handler.getRxPacketError(dxl_error)}  ({self.dxl_errors}/{self.total_results})"
            logging.debug(error_message)

        if (dxl_comm_result != dxl.COMM_SUCCESS): # or (dxl_error != 0): ignore hardware issues for now
            error_message = f"Dynamixel#{self.motor_id} {self.packet_handler.getTxRxResult(dxl_comm_result)} {self.packet_handler.getRxPacketError(dxl_error)}"
            logging.error(error_message)
            raise DynamixelServoError(self, error_message)
            
        logging.debug(f"Dynamixel#{self.motor_id}: {message}")

    def step_to_angle(self, step):
        if self.model == "XL430-W250-T":
            return ((step-2048)%4096)*360/4096
        elif self.model == "XL-320":
            return (step - 511.5) / 3.41

    def angle_to_step(self, angle):
        if self.model == "XL430-W250-T":
            return (angle*4096/360+2048)%4096
        elif self.model == "XL-320":
            return (3.41 * angle) + 511.5
    
    @staticmethod
    def velocity_to_bytes(target_velocity):
        if target_velocity < 0:
            return abs(target_velocity) + 1024#CCW 0-1023
        return target_velocity #CW 1024-2047
        
    @staticmethod
    def velocity_to_int(target_velocity):
        if target_velocity >= 1024:
            return -(target_velocity-1024)
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
