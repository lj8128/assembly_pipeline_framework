#!/usr/bin/env python3

import rospy
import tf2_ros
from arm_controller import ArmController
from workspace_registrar import WorkspaceRegistrar
from cargo_registrar import CargoRegistrar

class AsmLineControllerTester:

    def __init__(self):
        self.arm_ctrl = ArmController()
        self.ws_dir = WorkspaceRegistrar().return_built_ws_directory()
        self.creg = CargoRegistrar()

    def run(self):
        self._test_cargo_registrar()

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
    rospy.init_node('asm_line_controller')
    # res = 'n'
    # while res != 'y' and res != 'yes':
    #     res = input('Are all fiducials being detected? (y/n) ').lower()
    AsmLineControllerTester().run()
