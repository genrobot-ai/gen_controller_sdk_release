#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

class GripperDataConverter:
    def __init__(self):
        rospy.init_node('das_controller_converter', anonymous=True)
        
        self.publish_rate = 100  # rospy.get_param('~publish_rate', 50)
        self.latest_left_data = None
        self.latest_left_cmd = None
        
        rospy.Subscriber('/left_gripper/encoder', Float32, self.left_gripper_data_callback)
        self.left_gripper_feedback_pub = rospy.Publisher('/gripper/left/current_distance', PoseStamped, queue_size=10)      

        rospy.Subscriber('/target_gripper/left_gripper', PoseStamped, self.left_cmd_callback)
        self.left_gripper_cmd_pub = rospy.Publisher('/left_gripper/target_distance', Float32, queue_size=10)

        rospy.loginfo("Gripper Data Converter Node Started")
        rospy.loginfo(f"Publish rate: {self.publish_rate}Hz")
    
    def left_gripper_data_callback(self, msg):
        """Store latest left gripper feedback."""
        self.latest_left_data = msg
    
    def process_gripper_feedback(self, gripper_msg, publisher, gripper_name):
        """Convert Float32 distance to PoseStamped for the model."""
        pose_msg = PoseStamped()
        
        if hasattr(gripper_msg, 'header') and gripper_msg.header.stamp:
            pose_msg.header.stamp = gripper_msg.header.stamp
        else:
            pose_msg.header.stamp = rospy.Time.now()
            
        pose_msg.header.frame_id = f"{gripper_name}_gripper_frame"
        
        pose_msg.pose.position.x = gripper_msg.data
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.0
        
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        
        publisher.publish(pose_msg)

        # Used during calibration
        self.left_gripper_cmd_pub.publish(0.05)
        
        rospy.loginfo(f"Published {gripper_name} feedback distance: {gripper_msg.data}")

    def left_cmd_callback(self, msg):
        """Store latest left gripper command."""
        self.latest_left_cmd = msg
    
    def process_gripper_cmd(self, pos_msg, publisher, gripper_name):
        """PoseStamped -> Float32 target_distance."""
        gripper_cmd_msg = Float32()
        
        gripper_cmd_msg.data = pos_msg.pose.position.x
        
        publisher.publish(gripper_cmd_msg)
        
        # rospy.loginfo(f"Published {gripper_name} command distance: {pos_msg.pose.position.x}")
    
    def publish_all_data(self):
        """Publish at fixed rate."""
        if self.latest_left_data is not None:
            self.process_gripper_feedback(self.latest_left_data, self.left_gripper_feedback_pub, "left")
        
        if self.latest_left_cmd is not None:
            self.process_gripper_cmd(self.latest_left_cmd, self.left_gripper_cmd_pub, "left")
    
    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.publish_all_data()
            rate.sleep()

if __name__ == '__main__':
    try:
        converter = GripperDataConverter()
        converter.run()
    except rospy.ROSInterruptException:
        pass
