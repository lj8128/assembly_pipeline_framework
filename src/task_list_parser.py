#!/usr/bin/env python3

import os
import json
import rospy

class TaskListParser:
    def __init__(self):
        abs_path_to_cur_file = os.path.dirname(__file__)
        self.CONFIG_FILE_PATH = os.path.join(abs_path_to_cur_file,
                '../config/task_sequence.json')

    def run(self):
        task_list = self._parse_config_file()
        self._set_ros_params(task_list)

    def _set_ros_params(self, task_list):
        rospy.set_param('task_list', task_list)
        rospy.set_param('first_cargo_fid_id', len(task_list) + 1)
        rospy.set_param('aruco_dict_num_fid_ids', 50)

    def _parse_config_file(self):
        with open(self.CONFIG_FILE_PATH, 'r', encoding='utf-8') as f:
            return json.load(f)

if __name__ == '__main__':
    rospy.init_node('task_list_parser')
    TaskListParser().run()

