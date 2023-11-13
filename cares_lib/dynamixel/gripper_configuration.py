from pydantic import BaseModel
from typing import List

class GripperConfig(BaseModel):
    gripper_type: int
    gripper_id: int
    gripper_device_name: str
    gripper_baudrate: int
    servo_type: str
    action_type: str

    torque_limit: int
    speed_limit: int
    velocity_min: int
    velocity_max: int
    
    num_motors: int
    min_values: List[int]
    max_values: List[int]
    home_sequence: List[List[int]]