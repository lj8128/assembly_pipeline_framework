import rospy
from workstation_node import WsNode
from workstation_directory import WsDirectory

class WorkspaceRegistrar:
    def __init__(self):
        try:
            self.task_list = rospy.get_param('/task_list')
            self.ws_dir = WsDirectory()
        except KeyError:
            raise LookupError('Could not fetch ROS params for the '
                    'WorkspaceRegistrar!')

    def return_built_ws_directory(self):
        self._register_workspaces()
        return self.ws_dir

    def _register_workspaces(self):
        for task in self.task_list:
            cur_ws_node = WsNode(ws_name=task['task_name'],
                    latency=task['latency'])
            try:
                self.ws_dir.insert(cur_ws_node)
            except ValueError as e:
                rospy.loginfo(e)

