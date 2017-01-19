#!/usr/bin/env python

import copy
import time
import rospy

from numpy import append

# from ihmc_msgs.msg import ArmTrajectoryRosMessage
# from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
# from ihmc_msgs.msg import TrajectoryPoint1DRosMessage
from ihmc_msgs.msg import HandTrajectoryRosMessage
from ihmc_msgs.msg import SE3TrajectoryPointRosMessage
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

TARGET_POSE = [0.6, -0.6, 1.1, 0.0, 0.0, 0.0, 1.0]

ROBOT_NAME = None

def sendRightHandTrajectory():
    msg = HandTrajectoryRosMessage()

    msg.robot_side = HandTrajectoryRosMessage.RIGHT
    msg.base_for_control = HandTrajectoryRosMessage.CHEST

    msg = appendTrajectoryPoint(msg, 2.0, TARGET_POSE)
    # msg = appendTrajectoryPoint(msg, 3.0, ELBOW_BENT_UP)

    msg.unique_id = -1

    rospy.loginfo('publishing right hand trajectory')
    handTrajectoryPublisher.publish(msg)

def appendTrajectoryPoint(hand_trajectory, time, pose):
    points =  SE3TrajectoryPointRosMessage()

    points.time = time

    position = Vector3()
    position.x = pose[0]
    position.y = pose[1]
    position.z = pose[2]
    points.position = position

    orientation = Quaternion()
    orientation.x = pose[3]
    orientation.y = pose[4]
    orientation.z = pose[5]
    orientation.w = pose[6]
    points.orientation = orientation

    lin_vel = Vector3()
    lin_vel.x = 0
    lin_vel.y = 0
    lin_vel.z = 0
    points.linear_velocity = lin_vel

    ang_vel = Vector3()
    ang_vel.x = 0
    ang_vel.y = 0
    ang_vel.z = 0
    points.angular_velocity = ang_vel

    hand_trajectory.taskspace_trajectory_points.append(points)

    return hand_trajectory

if __name__ == '__main__':
    try:
        rospy.init_node('ihmc_hand_demo1')

        ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')

        handTrajectoryPublisher = rospy.Publisher("/ihmc_ros/{0}/control/hand_trajectory".format(ROBOT_NAME), HandTrajectoryRosMessage, queue_size=1)

        rate = rospy.Rate(10) # 10hz
        time.sleep(1)

        # make sure the simulation is running otherwise wait
        if handTrajectoryPublisher.get_num_connections() == 0:
            rospy.loginfo('waiting for subscriber...')
            while handTrajectoryPublisher.get_num_connections() == 0:
                rate.sleep()

        if not rospy.is_shutdown():
            sendRightHandTrajectory()
            time.sleep(2)

    except rospy.ROSInterruptException:
        pass
