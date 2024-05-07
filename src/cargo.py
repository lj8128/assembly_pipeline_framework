import rospy

class Cargo:
    def __init__(self,
            cargo_frame_id,
            cur_ws="None",
            last_placed=rospy.Time.now()):
       self.cargo_frame_id = cargo_frame_id
       self.cur_ws = cur_ws
       self.last_placed = last_placed

