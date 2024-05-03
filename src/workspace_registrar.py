#!/usr/bin/env python3

class WorkspaceRegistrar:
    def __init__(self):
        try:
            self.task_list = rospy.get_param('/task_list')
        except KeyError:
            raise LookupError('Could not fetch ROS params for the'
                    'WorkspaceRegistrar!')

    def register(self):

