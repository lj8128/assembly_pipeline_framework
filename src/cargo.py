import rospy
from shared_constants import DUMMY_HEAD_WS_NAME

class Cargo:
    def __init__(self,
            last_placed,
            cargo_frame_id,
            cur_ws=DUMMY_HEAD_WS_NAME):
       self.cargo_frame_id = cargo_frame_id
       self.cur_ws = cur_ws
       self.last_placed = last_placed

