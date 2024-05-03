#!/usr/bin/env python3

import rospy
import tf2_ros
from arm_controller import ArmController

class AsmLineController:

    def __init__(self):
        self.arm_ctrl = ArmController()

    def run(self):
        arm_command_sent = False
        rate = rospy.Rate(10.0)

        while not (rospy.is_shutdown() or arm_command_sent):
            try:
               self.pick_and_place('cargo_8', 'ws_A')
               self.pick_and_place('cargo_8', 'ws_B')
               self.pick_and_place('cargo_8', 'ws_C')
               self.pick_and_place('cargo_8', 'ws_D')

               self.arm_ctrl.retire_robot()
               arm_command_sent = True

            except (tf2_ros.LookupException,
                    tf2_ros.ExtrapolationException,
                    tf2_ros.ConnectivityException):
                continue

    def pick_and_place(self, cf_name, ws_name, moving_time=1.25):
        self.arm_ctrl.pick_up(cf_name, moving_time)
        self.arm_ctrl.place_at(ws_name, moving_time)


if __name__ == '__main__':
    rospy.init_node('asm_line_controller')
    res = 'n'
    while res != 'y' and res != 'yes':
        res = input('Are all fiducials being detected? (y/n) ').lower()
    AsmLineController().run()
