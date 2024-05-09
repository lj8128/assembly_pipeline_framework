import rospy
from threading import Thread
from shared_constants import DUMMY_HEAD_WS_NAME, DUMMY_TAIL_WS_NAME

class CargoEnqueuer(Thread):
    def __init__(self,
            creg,
            ws_dir,
            quit_event):
        super().__init__()
        self.target_queue = creg.pnp_queue
        self.set_of_q_els = creg.set_of_q_els
        self.cargo_dict = creg.cargo_dict
        self.cargo_dict_lock = creg.dict_lock
        self.ws_dir = ws_dir
        self.quit_event = quit_event

    def run(self):
        while not self.quit_event.is_set():
            with self.cargo_dict_lock:
                for cf_id, cargo_obj in self.cargo_dict.items():
                    if cf_id not in self.set_of_q_els:
                        ws_name = cargo_obj.cur_ws_name
                        if ws_name != DUMMY_TAIL_WS_NAME:
                            ws_obj = self.ws_dir.get(ws_name)

                            if ws_name == DUMMY_HEAD_WS_NAME:
                                next_ws_name = ws_obj.next.ws_name
                                cargo_obj.cur_ws_name = next_ws_name 
                                self.target_queue.put(cf_id)
                                self.set_of_q_els.add(cf_id)
                            else: 
                                ws_fin_time = (cargo_obj.last_placed
                                        + rospy.Duration(ws_obj.latency))
                                cur_time = rospy.Time.now()
                                if cur_time >= ws_fin_time:
                                    rospy.loginfo(f'cur_ws_name: {ws_name}')
                                    self.target_queue.put(cf_id)
                                    self.set_of_q_els.add(cf_id)
         
