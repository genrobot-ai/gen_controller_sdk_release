#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

class GripperDataConverter:
    def __init__(self):
        rospy.init_node('das_controller_converter', anonymous=True)
        
        self.publish_rate = 100  # rospy.get_param('~publish_rate', 50)
        self.latest_right_data = None
        self.latest_right_cmd = None
        
        rospy.Subscriber('/right_gripper/encoder', Float32, self.right_gripper_data_callback)
        self.right_gripper_feedback_pub = rospy.Publisher('/gripper/right/current_distance', PoseStamped, queue_size=10)      

        rospy.Subscriber('/target_gripper/right_gripper', PoseStamped, self.right_cmd_callback)
        self.right_gripper_cmd_pub = rospy.Publisher('/right_gripper/target_distance', Float32, queue_size=10)

        rospy.loginfo("Gripper Data Converter Node Started")
        rospy.loginfo(f"Publish rate: {self.publish_rate}Hz")
    
    def right_gripper_data_callback(self, msg):
        """Store latest right gripper feedback."""
        self.latest_right_data = msg
    
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

        # self.right_gripper_cmd_pub.publish(0.1)
        
        rospy.loginfo(f"Published {gripper_name} feedback distance: {gripper_msg.data}")

    def right_cmd_callback(self, msg):
        """Store latest right gripper command."""
        self.latest_right_cmd = msg
    
    def process_gripper_cmd(self, pos_msg, publisher, gripper_name):
        """PoseStamped -> Float32 target_distance."""
        gripper_cmd_msg = Float32()
        
        gripper_cmd_msg.data = pos_msg.pose.position.x
        
        publisher.publish(gripper_cmd_msg)
        
        # rospy.loginfo(f"Published {gripper_name} command distance: {pos_msg.pose.position.x}")
    
    def publish_all_data(self):
        """Publish at fixed rate."""
        if self.latest_right_data is not None:
            self.process_gripper_feedback(self.latest_right_data, self.right_gripper_feedback_pub, "right")
        
        if self.latest_right_cmd is not None:
            self.process_gripper_cmd(self.latest_right_cmd, self.right_gripper_cmd_pub, "right")
    
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
