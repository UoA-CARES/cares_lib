from pydantic import BaseModel
from typing import List, Optional

class GripperConfig(BaseModel):
    gripper_type: int
    gripper_id: int
    device_name: str
    baudrate: int
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

    touch: Optional[bool] = False
    touch_port: Optional[str] = "/dev/ttyACM0"
    socket_port: Optional[int] = 12345
    num_touch_sensors: Optional[int] = 4