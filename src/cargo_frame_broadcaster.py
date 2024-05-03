#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

class CargoFrameBroadcaster:

    def __init__(self):
        try:
            self.first_cargo_fid_id = rospy.get_param('/first_cargo_fid_id')
            self.last_cargo_fid_id = rospy.get_param('/aruco_dict_num_fid_ids')
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
            self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        except KeyError:
            raise LookupError('Could not fetch ROS params for the '
                    'cargo frame broadcaster!')

    def run(self):
        rate = rospy.Rate(10.0)

        while not rospy.is_shutdown():
            self._broadcast_frames()                
            rate.sleep()
    
    def _broadcast_frames(self):
        for fid_id in range(self.first_cargo_fid_id, 
                self.last_cargo_fid_id):
            try:
                new_cf = TransformStamped()
                parent_fid = f'fiducial_{fid_id}'

                self._set_new_cf_header_and_child_frame_id(fid_id,
                        new_cf,
                        parent_fid)
                self._set_new_cf_translation(new_cf, parent_fid)
                self._set_new_cf_rotation(new_cf, parent_fid)
                
                self.tf_broadcaster.sendTransform(new_cf)
            except (tf2_ros.LookupException,
                    tf2_ros.ExtrapolationException, 
                    tf2_ros.ConnectivityException):
                continue

    def _set_new_cf_header_and_child_frame_id(self,
            fid_id,
            new_cf,
            parent_fid):
        new_cf.header.stamp = rospy.Time.now()
        new_cf.header.frame_id = parent_fid
        cargo_id = fid_id - self.first_cargo_fid_id + 1
        new_cf.child_frame_id = (f'cargo_{cargo_id}')

    def _set_new_cf_translation(self, new_cf, parent_fid):
        tf_cargo = self.tf_buffer.lookup_transform(parent_fid,
                parent_fid,
                rospy.Time()).transform
        new_cf.transform.translation = tf_cargo.translation
        new_cf.transform.translation.x = tf_cargo.translation.x
        new_cf.transform.translation.y = tf_cargo.translation.y

    def _set_new_cf_rotation(self, new_cf, parent_fid):
        tf_parent_fid_to_base_link = self.tf_buffer.lookup_transform(
                parent_fid,
                'px100/base_link',
                rospy.Time()).transform 
        new_cf.transform.rotation = tf_parent_fid_to_base_link.rotation

if __name__ == '__main__':
    rospy.init_node('cargo_frame_broadcaster')
    try:
        cfb = CargoFrameBroadcaster()
        cfb.run()
    except LookupError as e:
        rospy.loginfo(e) 

