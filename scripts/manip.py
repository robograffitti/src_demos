#!/usr/bin/env python

import copy
import time
import rospy

from numpy import append

from ihmc_msgs.msg import ArmTrajectoryRosMessage
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage

# PUSH_BUTTON = [-0.700, 0.725, 0.635, 1.155, 0.0, 0.0, 0.0] # push_button_pose
# PUSH_BUTTON = [-0.800, 0.800, 0.600, 1.000, 0.0, 0.0, 0.0] # push_button 1
PUSH_BUTTON = [-1.250, 0.800, 0.600, 0.150, 0.0, 0.0, 0.0] # push_button 1
LIFT_ARM = [-0.250, 1.000, 0.200, 1.125, 0.0, 0.0, 0.0]
TUCK_ARM = [0.0, 1.250, 0.4, 2.0, 0.0, 0.0, 0.0]

ROBOT_NAME = None

def sendRightArmTrajectory():
    msg = ArmTrajectoryRosMessage()

    msg.robot_side = ArmTrajectoryRosMessage.RIGHT

    msg = appendTrajectoryPoint(msg, 0.20, LIFT_ARM)
    msg = appendTrajectoryPoint(msg, 0.50, PUSH_BUTTON)
    msg = appendTrajectoryPoint(msg, 0.70, TUCK_ARM)

    msg.unique_id = -1

    rospy.loginfo('publishing right trajectory')
    armTrajectoryPublisher.publish(msg)

def appendTrajectoryPoint(arm_trajectory, time, positions):
    if not arm_trajectory.joint_trajectory_messages:
        arm_trajectory.joint_trajectory_messages = [copy.deepcopy(OneDoFJointTrajectoryRosMessage()) for i in range(len(positions))]
    for i, pos in enumerate(positions):
        point = TrajectoryPoint1DRosMessage()
        point.time = time
        point.position = pos
        point.velocity = 0
        arm_trajectory.joint_trajectory_messages[i].trajectory_points.append(point)
    return arm_trajectory

if __name__ == '__main__':
    try:
        rospy.init_node('ihmc_arm_demo1')

        ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')

        armTrajectoryPublisher = rospy.Publisher("/ihmc_ros/{0}/control/arm_trajectory".format(ROBOT_NAME), ArmTrajectoryRosMessage, queue_size=1)

        rate = rospy.Rate(10) # 10hz
        time.sleep(1)

        # make sure the simulation is running otherwise wait
        if armTrajectoryPublisher.get_num_connections() == 0:
            rospy.loginfo('waiting for subscriber...')
            while armTrajectoryPublisher.get_num_connections() == 0:
                rate.sleep()

        if not rospy.is_shutdown():
            sendRightArmTrajectory()
            time.sleep(2)

    except rospy.ROSInterruptException:
        pass
