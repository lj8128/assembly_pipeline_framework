import rospy
from threading import Thread
from shared_constants import DUMMY_HEAD_WS_NAME, DUMMY_TAIL_WS_NAME

class CargoEnqueuer(Thread):
    def __init__(self,
            target_queue,
            cargo_dict,
            cargo_dict_lock,
            ws_dir,
            quit_event):
        super().__init__()
        self.target_queue = target_queue
        self.cargo_dict = cargo_dict
        self.cargo_dict_lock = cargo_dict_lock
        self.ws_dir = ws_dir
        self.quit_event = quit_event

    def run(self):
        while not self.quit_event.is_set():
            with self.cargo_dict_lock:
                for cf_id, cargo_obj in self.cargo_dict.items():
                    if cargo_obj.cur_ws_name == DUMMY_HEAD_WS_NAME:
                        self.target_queue.put(cf_id)
                    elif cargo_obj.cur_ws_name != DUMMY_TAIL_WS_NAME:
                        cur_ws_name = cargo_obj.cur_ws_name
                        ws_obj = ws_dir.get(cur_ws_name)
                        ws_fin_time = (cargo_obj.last_placed
                                + rospy.Duration(ws_obj.latency))
                        cur_time = rospy.Time.now()
                        if cur_time >= ws_fin_time:
                            self.target_queue.put(cf_id)
 
