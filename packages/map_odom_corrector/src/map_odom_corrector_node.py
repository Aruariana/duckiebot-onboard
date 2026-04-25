#!/usr/bin/env python3
import rospy
import numpy as np
import tf.transformations as tr
import tf2_ros
import collections

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped

# Import DTROS essentials
from duckietown.dtros import DTROS, NodeType

class MapOdomCorrectorNode(DTROS):
    def __init__(self, node_name):
        # Initialize the DTROS parent class with the correct node type
        super(MapOdomCorrectorNode, self).__init__(
            node_name=node_name, 
            node_type=NodeType.LOCALIZATION
        )

        # The offset matrix between the absolute map and the drifting odom frame.
        self.map_T_odom = np.identity(4)
        
        # A rolling buffer to store the last 3 seconds of wheel odometry with 10hz updates.
        # Stores tuples of: (timestamp_in_seconds, odom_T_footprint_matrix)
        self.odom_history = collections.deque(maxlen=30) 

        # TF Broadcaster
        self._tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Publisher for RViz visualization (shows the true map position)
        self.path_pub = rospy.Publisher("~true_path", Path, queue_size=1, latch=True)
        self.path = Path()

        # Setup subscribers
        self.sub_wheels = rospy.Subscriber(
            "~input_odom", Odometry, self.cb_wheel_odom, queue_size=10
        )
        self.sub_tags = rospy.Subscriber(
            "~input_pose", PoseStamped, self.cb_tag_pose, queue_size=1
        )

        # Use DTROS built-in logging methods instead of rospy.loginfo
        self.loginfo("Initialized. Publishing map -> odom TF...")

    def cb_tag_pose(self, msg):
        """ When an AprilTag is seen, calculate the new drift offset. """
        if len(self.odom_history) == 0:
            return

        tag_time = msg.header.stamp.to_sec()

        # 1. TIME MACHINE: Find where the wheels were exactly when the photo was taken
        historical_odom_T = None
        min_time_diff = float('inf')
        
        history_snapshot = list(self.odom_history)
        for hist_time, hist_T in history_snapshot:
            diff = abs(hist_time - tag_time)
            if diff < min_time_diff:
                min_time_diff = diff
                historical_odom_T = hist_T

        if min_time_diff > 0.5:
            self.logwarn("Tag data is too old! Ignoring to prevent bad math.")
            return

        # 2. Extract the absolute map position of the robot from the tag
        tag_pos = msg.pose.position
        tag_ori = msg.pose.orientation
        
        map_t_footprint = tr.translation_matrix((tag_pos.x, tag_pos.y, 0.0))
        map_R_footprint = tr.quaternion_matrix((tag_ori.x, tag_ori.y, tag_ori.z, tag_ori.w))
        map_T_footprint_past = tr.concatenate_matrices(map_t_footprint, map_R_footprint)

        # 3. Calculate the new offset: map_T_odom = map_T_footprint(past) * inverse(odom_T_footprint(past))
        odom_T_footprint_inv = tr.inverse_matrix(historical_odom_T)
        self.map_T_odom = tr.concatenate_matrices(map_T_footprint_past, odom_T_footprint_inv)

    def cb_wheel_odom(self, msg):
        """ Keep the TF tree alive by continuously publishing map -> odom. """
        
        # 1. Extract current wheel odometry
        odom_pos = msg.pose.pose.position
        odom_ori = msg.pose.pose.orientation
        
        odom_t_footprint = tr.translation_matrix((odom_pos.x, odom_pos.y, 0.0))
        odom_R_footprint = tr.quaternion_matrix((odom_ori.x, odom_ori.y, odom_ori.z, odom_ori.w))
        current_odom_T_footprint = tr.concatenate_matrices(odom_t_footprint, odom_R_footprint)

        # 2. Save to history buffer for the Time Machine
        self.odom_history.append((msg.header.stamp.to_sec(), current_odom_T_footprint))

        # 3. Broadcast the `map` -> `odom` TF link
        final_translation = tr.translation_from_matrix(self.map_T_odom)
        final_quaternion = tr.quaternion_from_matrix(self.map_T_odom)
        
        map_frame = rospy.get_namespace().strip('/') + "/map"
        odom_frame = msg.header.frame_id

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = map_frame
        t.child_frame_id = odom_frame
        
        t.transform.translation.x = final_translation[0]
        t.transform.translation.y = final_translation[1]
        t.transform.translation.z = 0.0
        t.transform.rotation.x = final_quaternion[0]
        t.transform.rotation.y = final_quaternion[1]
        t.transform.rotation.z = final_quaternion[2]
        t.transform.rotation.w = final_quaternion[3]
        
        self._tf_broadcaster.sendTransform(t)

        # 4. Optional: Publish the true global path for RViz
        map_T_footprint = tr.concatenate_matrices(self.map_T_odom, current_odom_T_footprint)
        path_trans = tr.translation_from_matrix(map_T_footprint)
        path_quat = tr.quaternion_from_matrix(map_T_footprint)

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = map_frame
        pose.pose.position.x = path_trans[0]
        pose.pose.position.y = path_trans[1]
        pose.pose.orientation.x = path_quat[0]
        pose.pose.orientation.y = path_quat[1]
        pose.pose.orientation.z = path_quat[2]
        pose.pose.orientation.w = path_quat[3]
        
        self.path.header.stamp = msg.header.stamp
        self.path.header.frame_id = map_frame
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)

if __name__ == "__main__":
    # Create the DTROS node
    node = MapOdomCorrectorNode("map_odom_corrector_node")
    rospy.spin()