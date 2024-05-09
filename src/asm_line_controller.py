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
        self.quit_enq_ev = Event()
        self.cargo_enq = CargoEnqueuer(self.creg.pnp_queue,
                self.creg.cargo_dict,
                self.creg.dict_lock,
                self.ws_dir,
                self.quit_enq_ev
                )
        self.cargo_enq.start()
        self.shutting_down = False
        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):
        self.quit_enq_ev.set()
        self.shutting_down = True

    def run(self):
        rate = rospy.Rate(10.0)
        while not self.shutting_down:
            # main queue dequeueing and pnp logic should go here 

if __name__ == '__main__':
    rospy.init_node('asm_line_controller')
    AsmLineController().run()

