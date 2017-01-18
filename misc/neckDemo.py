#!/usr/bin/env python

import copy
import time
import rospy

from numpy import append

from ihmc_msgs.msg import NeckTrajectoryRosMessage
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage

ZERO_VECTOR = [0.0, 0.0, 0.0]
TEST_VECTOR = [0.25, 0.5, -0.5]

ROBOT_NAME = None

def sendNeckTrajectory():
    msg = NeckTrajectoryRosMessage()

    # msg.robot_side = ArmTrajectoryRosMessage.RIGHT

    msg = appendTrajectoryPoint(msg, 2.0, ZERO_VECTOR)
    msg = appendTrajectoryPoint(msg, 3.0, TEST_VECTOR)

    msg.unique_id = -1

    rospy.loginfo('publishing neck trajectory')
    neckTrajectoryPublisher.publish(msg)

def appendTrajectoryPoint(neck_trajectory, time, positions):
    if not neck_trajectory.joint_trajectory_messages:
        neck_trajectory.joint_trajectory_messages = [copy.deepcopy(OneDoFJointTrajectoryRosMessage()) for i in range(len(positions))]
    for i, pos in enumerate(positions):
        point = TrajectoryPoint1DRosMessage()
        point.time = time
        point.position = pos
        point.velocity = 0
        neck_trajectory.joint_trajectory_messages[i].trajectory_points.append(point)
    return neck_trajectory

if __name__ == '__main__':
    try:
        rospy.init_node('ihmc_neck_demo1')

        ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')

        neckTrajectoryPublisher = rospy.Publisher("/ihmc_ros/{0}/control/neck_trajectory".format(ROBOT_NAME), NeckTrajectoryRosMessage, queue_size=1)

        rate = rospy.Rate(10) # 10hz
        time.sleep(1)

        # make sure the simulation is running otherwise wait
        if neckTrajectoryPublisher.get_num_connections() == 0:
            rospy.loginfo('waiting for subscriber...')
            while neckTrajectoryPublisher.get_num_connections() == 0:
                rate.sleep()

        if not rospy.is_shutdown():
            sendNeckTrajectory()
            time.sleep(2)

    except rospy.ROSInterruptException:
        pass
