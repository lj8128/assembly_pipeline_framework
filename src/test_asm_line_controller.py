#!/usr/bin/env python3

import rospy
import tf2_ros
from arm_controller import ArmController
from workspace_registrar import WorkspaceRegistrar
from cargo_registrar import CargoRegistrar
from cargo_enqueuer import CargoEnqueuer
from threading import Event

class AsmLineControllerTester:

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

    def run(self):
        self._test_main_logic()

    def _test_main_logic(self):
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

    def pick_and_place(self, cf_id, ws_frame_name):
        self.arm_ctrl.pick_up(cf_id)
        self.arm_ctrl.place_at(ws_frame_name)

    def _test_carg_enq(self):
        rate = rospy.Rate(10.0)
        while not self.shutting_down:
            if not self.creg.pnp_queue.empty():
                rospy.loginfo('==============================')
                rospy.loginfo(f'Dequeued: {self.creg.pnp_queue.get()}')
                rospy.loginfo('==============================')

    def _test_cargo_registrar(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            with self.creg.dict_lock:
                for key, value in self.creg.cargo_dict.items():
                    rospy.loginfo('==============================')
                    rospy.loginfo(f'cargo_dict entry key: {key}')
                    rospy.loginfo(f'Cargo cf_id: {value.cargo_frame_id}')
                    rospy.loginfo(f'Cargo cur_ws: {value.cur_ws}')
                    rospy.loginfo(f'Cargo last_placed: {value.last_placed}')
                    rospy.loginfo('==============================')

    def _test_ws_registrar(self):
        for key, value in self.ws_dir.get_ws_dict().items():
            rospy.loginfo('==============================')
            rospy.loginfo(f'ws_dir entry key: {key}')
            rospy.loginfo(f'ws_node name: {value.ws_name}')
            rospy.loginfo(f'ws_node latency: {value.latency}')
            rospy.loginfo('==============================')

    def _test_pick_and_place(self):
        arm_command_sent = False
        rate = rospy.Rate(10.0)

        while not (rospy.is_shutdown() or arm_command_sent):
            try:
               self.pick_and_place('cargo_7', 'ws_A')
               self.pick_and_place('cargo_7', 'ws_B')
               self.pick_and_place('cargo_7', 'ws_C')
               self.pick_and_place('cargo_7', 'ws_D')

               self.arm_ctrl.retire_robot()
               arm_command_sent = True

            except (tf2_ros.LookupException,
                    tf2_ros.ExtrapolationException,
                    tf2_ros.ConnectivityException):
                continue

if __name__ == '__main__':
    rospy.init_node('test_asm_line_controller')
    AsmLineControllerTester().run()
