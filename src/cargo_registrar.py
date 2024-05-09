import rospy
from queue import Queue
from threading import Lock
from actionlib import SimpleActionServer
from cargo import Cargo
from assembly_pipeline_framework.msg import (
        CargoRegAction,
        CargoRegResult
        )

class CargoRegistrar:
    def __init__(self):
        self.pnp_queue = Queue()
        self.dict_lock = Lock()
        self.cargo_dict = {}
        self.cargo_reg_server = SimpleActionServer(
            'cargo_reg_server',
            CargoRegAction,
            self._register_new_cargo,
            False)
        self.cargo_reg_server.start()
    
    def _register_new_cargo(self, goal):
        cf_id = goal.cargo_frame_id
        new_cargo = Cargo(
                cargo_frame_id=cf_id,
                last_placed=goal.last_placed)
        with self.dict_lock:
            self.cargo_dict[cf_id] = new_cargo
        self._set_cargo_reg_server_result(cf_id, goal.last_placed)

    def _set_cargo_reg_server_result(self, cf_id, last_placed):
        result = CargoRegResult()
        result.log = (
                f'Registered new cargo with cargo frame id: {cf_id}, '
                f'which was last placed at: {last_placed}')
        result.success = True
        self.cargo_reg_server.set_succeeded(result)

