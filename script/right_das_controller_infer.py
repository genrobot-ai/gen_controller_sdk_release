#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

class GripperDataConverter:
    def __init__(self):
        # 初始化节点
        rospy.init_node('das_controller_converter', anonymous=True)
        
        self.publish_rate = 100
        self.latest_right_data = None
        self.latest_right_cmd = None
        
        # 订阅夹爪的数据
        rospy.Subscriber('/right_gripper/encoder', Float32, self.right_gripper_data_callback)
        # 发布夹爪数据给模型
        self.right_gripper_feedback_pub = rospy.Publisher('/gripper/right/current_distance', PoseStamped, queue_size=10)      

        # 订阅VR或推理的控制指令
        rospy.Subscriber('/target_gripper/right_gripper', PoseStamped, self.right_cmd_callback)
        # 然后发布控制指令到夹爪
        self.right_gripper_cmd_pub = rospy.Publisher('/right_gripper/target_distance', Float32, queue_size=10)

        rospy.loginfo("Gripper Data Converter Node Started")
        rospy.loginfo(f"Publish rate: {self.publish_rate}Hz")
    
    def right_gripper_data_callback(self, msg):
        """处理右侧夹爪反馈数据，存储最新数据"""
        self.latest_right_data = msg
    
    def process_gripper_feedback(self, gripper_msg, publisher, gripper_name):
        """处理Gripper反馈数据，提取distance并发布为PoseStamped"""
        # 创建PoseStamped消息
        pose_msg = PoseStamped()
        
        # 使用原始消息的时间戳，如果没有则使用当前时间
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
        
        # 发布消息
        publisher.publish(pose_msg)

        # 测试时可使用
        # self.right_gripper_cmd_pub.publish(0.1)
        
        rospy.loginfo(f"Published {gripper_name} feedback distance: {gripper_msg.data}")

    def right_cmd_callback(self, msg):
        """右侧夹爪控制指令回调函数，存储最新命令"""
        self.latest_right_cmd = msg
    
    def process_gripper_cmd(self, pos_msg, publisher, gripper_name):
        """处理PoseStamped控制指令并发布Gripper消息"""
        # 创建Float32消息
        gripper_cmd_msg = Float32()
        
        # 将PoseStamped的x位置作为控制指令
        gripper_cmd_msg.data = pos_msg.pose.position.x
        
        # 发布消息
        publisher.publish(gripper_cmd_msg)
        
        # rospy.loginfo(f"Published {gripper_name} command distance: {pos_msg.pose.position.x}")
    
    def publish_all_data(self):
        """定时发布所有数据，控制频率"""
        # 发布反馈数据
        if self.latest_right_data is not None:
            self.process_gripper_feedback(self.latest_right_data, self.right_gripper_feedback_pub, "right")
        
        # 发布控制指令
        if self.latest_right_cmd is not None:
            self.process_gripper_cmd(self.latest_right_cmd, self.right_gripper_cmd_pub, "right")
    
    def run(self):
        """主循环，控制发布频率"""
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