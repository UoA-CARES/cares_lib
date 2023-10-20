import logging
import time
import dynamixel_sdk as dxl
from enum import Enum
from functools import wraps
from cares_lib.dynamixel.servos_addresses import addresses

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
                raise DynamixelServoError(error.servo, message=f"Dynamixel#{error.servo.motor_id}: {error_message}") from error
        
        return wrapper
    return decorator


class OperatingMode(Enum):
    WHEEL = 1
    JOINT = 2
    POSITION = 3
    EXTENDED_POSITION = 4
    PWM_CONTROL = 16


class DynamixelServoError(IOError):
    def __init__(self, servo, message):
        self.servo = servo
        super().__init__(message)

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
        self.velocity_in_pos_control = "moving_speed" if self.model == "XL-320" else "profile_velocity"

    @exception_handler("Failed to enable")
    def enable(self):
        self.disable_torque()
        self.set_operating_mode(OperatingMode.JOINT.value)# default set to joint mode to avoid moving on start up
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
        operating_mode = self.operating_mode()
        if operating_mode == OperatingMode.JOINT.value:
            return
    
        current_state    = self.state()
        current_position = current_state["position"]
        current_velocity = Servo.velocity_to_int(current_state["velocity"])

        logging.debug(f"Current Velocity {current_velocity} : {self.min} < {current_position} < {self.max}")
        if (current_position >= self.max and current_velocity > 0) or \
            (current_position <= self.min and current_velocity < 0):
            self.stop_moving()
            logging.warn(f"Dynamixel#{self.motor_id}: position out of boundry, stopping servo")
 
    @exception_handler("Failed to ping")
    def ping(self): 
        model_number, dxl_comm_result, dxl_error = self.packet_handler.ping(self.port_handler, self.motor_id)
        self.process_result(dxl_comm_result, dxl_error, message=f"successfully pinged Dynamixel#{self.motor_id} as model {model_number}")
   
    @exception_handler("Failed to read operating mode")
    def operating_mode(self): 
        operating_mode, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(self.port_handler, self.motor_id, self.addresses["operating_mode"])
        self.process_result(dxl_comm_result, dxl_error, message=f"Dynamixel#{self.motor_id}: successfully read operating mode as {operating_mode}")
        return operating_mode 

    @exception_handler("Failed to set operating mode")
    def set_operating_mode(self, new_mode): 
        self.disable_torque()#disable to set servo parameters
        
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, self.motor_id, self.addresses["operating_mode"], new_mode)
        self.process_result(dxl_comm_result, dxl_error, message=f"successfully set operating mode to {new_mode}")

        self.enable_torque() 
            
    @exception_handler("Failed while moving")
    def move(self, target_position, wait=True, timeout=5):
        if not self.verify_step(target_position):
            error_message = f"Dynamixel#{self.motor_id}: Target position {target_position} is out of bounds of min {self.min} max {self.max}"
            logging.error(error_message)
            raise DynamixelServoError(self, error_message)
 
        if self.addresses["goal_position_length"] == 2:
            self.set_operating_mode(OperatingMode.JOINT.value)
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, self.motor_id, self.addresses["goal_position"], target_position)
        elif self.addresses["goal_position_length"] == 4:
            self.set_operating_mode(OperatingMode.POSITION.value)
            dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, self.motor_id, self.addresses["goal_position"], target_position)
            
            
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

        if not self.validate_movement(target_velocity):
            return self.current_velocity()
        

        processed_velocity = self.velocity_to_bytes(target_velocity)
        if self.addresses[self.velocity_in_pos_control+"_length"] == 2:
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, self.motor_id, self.addresses[self.velocity_in_pos_control], processed_velocity)
        elif self.addresses[self.velocity_in_pos_control+"_length"] == 4:
            dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, self.motor_id, self.addresses[self.velocity_in_pos_control], processed_velocity)
        
        self.process_result(dxl_comm_result, dxl_error, message=f"Dynamixel#{self.motor_id}: successfully told to move to at {target_velocity}")
        
        return self.current_velocity() 

    @exception_handler("Failed to stop")
    def stop_moving(self): 
        self.move_velocity(0)
        self.move(self.current_position())
        self.move_velocity(self.max_velocity)
        
    @exception_handler("Failed to check if moving")
    def is_moving(self): 
        current_position = self.current_position()
        goal_position  = self.current_goal_position()

        logging.debug(f"Dynamixel#{self.motor_id} is at position {current_position} and moving to {goal_position}")
        return abs(current_position - goal_position) > DXL_MOVING_STATUS_THRESHOLD 

    @exception_handler("Failed to read current position")
    def current_position(self): 
        if self.addresses["current_position_length"] == 2:
            data_read, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, self.motor_id, self.addresses["current_position"])
        elif self.addresses["current_position_length"] == 4:
            data_read, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, self.motor_id, self.addresses["current_position"])
            
        self.process_result(dxl_comm_result, dxl_error, message=f"Dynamixel#{self.motor_id}: measured position {data_read}")
        return data_read 

    @exception_handler("Failed to read current goal position")
    def current_goal_position(self): 
        if self.addresses["goal_position_length"] == 2:
            data_read, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, self.motor_id, self.addresses["goal_position"])
        elif self.addresses["goal_position_length"] == 4:
            data_read, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, self.motor_id, self.addresses["goal_position"])
            
        self.process_result(dxl_comm_result, dxl_error, message=f"Dynamixel#{self.motor_id}: current goal position {data_read}")
        return data_read

    @exception_handler("Failed to read current velocity")
    def current_velocity(self): 
        if self.addresses["current_velocity_length"] == 2:
            data_read, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, self.motor_id, self.addresses[self.velocity_in_pos_control])
        elif self.addresses["current_velocity_length"] == 4:
            data_read, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, self.motor_id, self.addresses[self.velocity_in_pos_control])
        self.process_result(dxl_comm_result, dxl_error, message=f"Dynamixel#{self.motor_id}: measured velocity {data_read}")
        return data_read 

    @exception_handler("Failed to read current load")
    def current_load(self): 
        data_read, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, self.motor_id, self.addresses["current_load"])
        self.process_result(dxl_comm_result, dxl_error, message=f"Dynamixel#{self.motor_id}: current load is {data_read}")
        # Convert it to a value between 0 - 1023 regardless of direction and then maps this between 0-100
        # See section 2.4.21 link for details on why this is required
        # https://emanual.robotis.com/docs/en/dxl/x/xl320/
        current_load_percent = ((data_read % 1023) / 1023) * 100
        return current_load_percent 
    
    @exception_handler("Failed to read shutdown status")
    def read_shutdown(self): 
        data_read, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(self.port_handler, self.motor_id, self.addresses["shutdown"])
        self.process_result(dxl_comm_result, dxl_error, message=f"Dynamixel#{self.motor_id}: measured shutdown status {data_read}")
        
        statuses = self.process_shutdown(data_read)

        return statuses
    
    # @exception_handler("Failed to read shutdown status")
    # def set_shutdown(self): 
    #     dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx( self.port_handler, self.motor_id, self.addresses["shutdown"], self.shutdown)
    #     self.process_result(dxl_comm_result, dxl_error, message=f"Dynamixel#{self.motor_id}: successfully set shutdown values {self.shutdown}") 


    @exception_handler("Failed to read hardware error status")
    def read_hardware_errors(self): 
        data_read, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(self.port_handler, self.motor_id, self.addresses["hardware_error_status"])
        self.process_result(dxl_comm_result, dxl_error,message=f"Dynamixel#{self.motor_id}: measured hardware error status {data_read}")
        
        statuses = self.process_hardware_error(data_read)

        return statuses

    @exception_handler("Failed to turn LED on")
    def turn_on_LED(self): 
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx( self.port_handler, self.motor_id, self.addresses["led"], self.LED_colour)
        self.process_result(dxl_comm_result, dxl_error, message=f"Dynamixel#{self.motor_id}: successfully turned on LEDs") 

    @exception_handler("Failed to limit speed")
    def limit_speed(self): 
        if self.addresses[self.velocity_in_pos_control+"_length"] == 2:
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(self.port_handler, self.motor_id, self.addresses[self.velocity_in_pos_control], self.max_velocity)
        elif self.addresses[self.velocity_in_pos_control+"_length"] == 4:
            dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, self.motor_id, self.addresses[self.velocity_in_pos_control], self.max_velocity)
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

    def process_hardware_errors(self, data_read):                 
        statuses = [] # an array of errors that occured 
        if(data_read & 1<<self.addresses["INPUT_VOLTAGE"]): 
            statuses.append(self.addresses["INPUT_VOLTAGE"])
        if(data_read & 1<<self.addresses["OVERHEAT"]): 
            statuses.append(self.addresses["OVERHEAT"])
        if(data_read & 1<<self.addresses["OVERLOAD"]):
            statuses.append(self.addresses["OVERLOAD"])

        if self.model != "XL-320":
            if (data_read & 1<<self.addresses["MOTOR_ENCODER_ERROR"]): 
                statuses.append(self.addresses["MOTOR_ENCODER_ERROR"])
            if (data_read & 1<<self.addresses["ELECTRICAL_SHOCK"]): 
                statuses.append(self.addresses["ELECTRICAL_SHOCK"])

    
        return statuses
    
    def data_to_bytes(self, data,length):
        if length == 2:
            return [dxl.DXL_LOBYTE(data), dxl.DXL_HIBYTE(data)]
        elif length == 4:
            return [dxl.DXL_LOBYTE(dxl.DXL_LOWORD(data)), dxl.DXL_HIBYTE(dxl.DXL_LOWORD(data)), dxl.DXL_LOBYTE(dxl.DXL_HIWORD(data)), dxl.DXL_HIBYTE(dxl.DXL_HIWORD(data))]

    def step_to_angle(self, step):
        if self.addresses["goal_position_length"] == 2:
            return (step - 511.5) / 3.41
        elif self.addresses["goal_position_length"] == 4:
            return ((step-2048)%4096)*360/4096

    def angle_to_step(self, angle):
        if self.addresses["goal_position_length"] == 2:
            return (3.41 * angle) + 511.5
        elif self.addresses["goal_position_length"] == 4:
            return (angle*4096/360+2048)%4096
    
    def velocity_to_bytes(self, target_velocity):
        if target_velocity < 0:
            if self.addresses[self.velocity_in_pos_control+"_length"] == 2:
                return abs(target_velocity) + 1024#CCW 0-1023
            elif self.addresses[self.velocity_in_pos_control+"_length"] == 4:
                return abs(target_velocity) + 2048
        return target_velocity #CW 1024-2047
        
    def velocity_to_int(self, target_velocity):
        if target_velocity >= 1024:
            if self.addresses[self.velocity_in_pos_control+"_length"] == 2:
                return -(target_velocity-1024)
            elif self.addresses[self.velocity_in_pos_control+"_length"] == 4:
                return -(target_velocity-2048)
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
