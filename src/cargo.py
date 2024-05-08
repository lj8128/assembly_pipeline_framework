import rospy

class Cargo:
    def __init__(self,
            last_placed,
            cargo_frame_id,
            cur_ws="None"):
       self.cargo_frame_id = cargo_frame_id
       self.cur_ws = cur_ws
       self.last_placed = last_placed

