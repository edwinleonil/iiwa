#!/usr/bin/env python

import rospy
from math import pi
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def talker():
    pub_joints = rospy.Publisher('/iiwa/move_group/fake_controller_joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(10) # 10hz
    joint_states = JointState()
    joint_states.header = Header()
    joint_states.header.stamp.secs = rospy.get_time()
    joint_states.name = ["iiwa_joint_1", 
                         "iiwa_joint_2", 
                         "iiwa_joint_3", 
                         "iiwa_joint_4", 
                         "iiwa_joint_5", 
                         "iiwa_joint_6",
                         "iiwa_joint_7"]

    joint_states.position = [0.0, 
                             0.0, 
                             0.0, 
                             -pi/2, 
                             0.0,  
                             pi/2,
                             0.0]
    joint_states.velocity = []
    joint_states.effort = []
    rospy.loginfo(joint_states)
    
    while not rospy.is_shutdown():
    #   joint_states.header.stamp.secs = rospy.get_time()
      pub_joints.publish(joint_states)
      rate.sleep()
    #   rospy.spin()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
