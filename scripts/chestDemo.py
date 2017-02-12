#!/usr/bin/env python

import copy
import time
import rospy

from numpy import append

# from ihmc_msgs.msg import ArmTrajectoryRosMessage
# from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
# from ihmc_msgs.msg import TrajectoryPoint1DRosMessage
from ihmc_msgs.msg import ChestTrajectoryRosMessage
from ihmc_msgs.msg import SO3TrajectoryPointRosMessage
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

POINT = [0.0, 0.2, 0.0, 1.0, 0.0, 0.5, 0.0]

ROBOT_NAME = None

def sendChestTrajectory():
    msg = ChestTrajectoryRosMessage()

    msg = appendTrajectoryPoint(msg, 2.0, POINT)

    msg.unique_id = -1

    rospy.loginfo('publishing chest trajectory')
    chestTrajectoryPublisher.publish(msg)

def appendTrajectoryPoint(chest_trajectory, time, point):
    points =  SO3TrajectoryPointRosMessage()

    points.time = time

    orientation = Quaternion()
    orientation.x = point[0]
    orientation.y = point[1]
    orientation.z = point[2]
    orientation.w = point[3]
    points.orientation = orientation

    angular_velocity = Vector3()
    angular_velocity.x = point[4]
    angular_velocity.y = point[5]
    angular_velocity.z = point[6]
    points.angular_velocity = angular_velocity

    chest_trajectory.taskspace_trajectory_points.append(points)

    return chest_trajectory

if __name__ == '__main__':
    try:
        rospy.init_node('ihmc_chest_demo1')

        ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')

        chestTrajectoryPublisher = rospy.Publisher("/ihmc_ros/{0}/control/chest_trajectory".format(ROBOT_NAME), ChestTrajectoryRosMessage, queue_size=1)

        rate = rospy.Rate(10) # 10hz
        time.sleep(1)

        # make sure the simulation is running otherwise wait
        if chestTrajectoryPublisher.get_num_connections() == 0:
            rospy.loginfo('waiting for subscriber...')
            while chestTrajectoryPublisher.get_num_connections() == 0:
                rate.sleep()

        if not rospy.is_shutdown():
            sendChestTrajectory()
            time.sleep(2)

    except rospy.ROSInterruptException:
        pass
