#!/usr/bin/env python3

import rospy
import tf2_ros
from arm_controller import ArmController
from workspace_registrar import WorkspaceRegistrar
from cargo_registrar import CargoRegistrar
from cargo_enqueuer import CargoEnqueuer
from threading import Event

class AsmLineController:

    def __init__(self):
        self.arm_ctrl = ArmController()
        self.ws_dir = WorkspaceRegistrar().return_built_ws_directory()
        self.creg = CargoRegistrar()
        self.quit_enq_ev = Event()
        self.cargo_enq = CargoEnqueuer(self.creg,
                self.ws_dir,
                self.quit_enq_ev
                )
        self.cargo_enq.start()
        self.shutting_down = False
        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):
        self.quit_enq_ev.set()
        self.cargo_enq.join()
        self.shutting_down = True

    def pick_and_place(self, cf_id, ws_frame_name):
        self.arm_ctrl.pick_up(cf_id)
        self.arm_ctrl.place_at(ws_frame_name)

    def run(self):
        rate = rospy.Rate(10.0)
        while not self.shutting_down:
            with self.creg.dict_lock:
                 if not self.creg.pnp_queue.empty():
                    rospy.loginfo(f'QUEUE SIZE: {self.creg.pnp_queue.qsize()}')
                    cur_cf_id = self.creg.pnp_queue.get()
                    self.creg.set_of_q_els.remove(cur_cf_id)

                    cur_cargo_obj = self.creg.cargo_dict[cur_cf_id]
                    cur_ws_name = cur_cargo_obj.cur_ws_name

                    self.pick_and_place(cur_cf_id, cur_ws_name) 

                    cur_ws_obj = self.ws_dir.get(cur_ws_name)
                    next_ws_name = cur_ws_obj.next.ws_name
                    cur_cargo_obj.cur_ws_name = next_ws_name
                    cur_cargo_obj.last_placed = rospy.Time.now()

if __name__ == '__main__':
    rospy.init_node('asm_line_controller')
    AsmLineController().run()

