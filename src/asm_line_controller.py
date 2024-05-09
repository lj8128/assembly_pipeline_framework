#!/usr/bin/env python3

import rospy
import tf2_ros
from arm_controller import ArmController
from workspace_registrar import WorkspaceRegistrar
from cargo_registrar import CargoRegistrar

class AsmLineController:

    def __init__(self):
        self.arm_ctrl = ArmController()
        self.ws_dir = WorkspaceRegistrar().return_built_ws_directory()
        self.creg = CargoRegistrar()

    def run(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():

if __name__ == '__main__':
    rospy.init_node('asm_line_controller')
    AsmLineController().run()

