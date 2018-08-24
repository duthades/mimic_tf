#!/usr/bin/env python
"""
Listens to the tf and performs the computations
"""
import math
import rospy
import tf
from std_msgs.msg import String
import roslib
import geometry_msgs.msg
from turtlesim.msg import Pose
import turtlesim.srv
roslib.load_manifest('mimic_tf')


TOPIC = 'chatter'
PUB = rospy.Publisher(TOPIC, String, queue_size=10)
rospy.loginfo("I will publish to the topic %s", TOPIC)


class TurtlePose(object):
    """
    Data structure to save positions of turtle
    """
    def __init__(self):
        self.pose = Pose()

    def update_pose(self, data):
        """Callback function which is called when a new message of type
           Pose is received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)


if __name__ == '__main__':

    rospy.init_node('tf_listener')
    LISTENER = tf.TransformListener()

    rospy.wait_for_service('spawn')
    SPAWNER = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    SPAWNER(4, 2, 0, 'turtle2')

    TURTLE_VEL = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,
                                 queue_size=1)

#  Get the pose of the turtle2
    POSE_DATA = TurtlePose()
    rospy.Subscriber('/turtle2/pose', Pose, POSE_DATA.update_pose)

    RATE = rospy.Rate(10)

    i = 0

    while not rospy.is_shutdown():
        try:

            (TRANS, ROT) = LISTENER.lookupTransform('/turtle2', '/turtle1',
                                                    rospy.Time(0))
            if i == 0:
                DISTANCE = math.sqrt(TRANS[0] ** 2 + TRANS[1] ** 2)
                i = i + 1

        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            continue

        THETA = tf.transformations.euler_from_quaternion(ROT)[2]

        STR1 = str("x,y: %s" % str(TRANS))
        rospy.loginfo(STR1)
        PUB.publish(STR1)

        CURRENT_DISTANCE = math.sqrt(TRANS[0] ** 2 + TRANS[1] ** 2)

        CMD = geometry_msgs.msg.Twist()
        MULTIPLIER = 1
        if (CURRENT_DISTANCE - DISTANCE < -0.04 and TRANS[0] >= 0):
            LINEAR = -1 * MULTIPLIER * DISTANCE
        elif (CURRENT_DISTANCE - DISTANCE > 0.04 and TRANS[0] >= 0):
            LINEAR = 1 * MULTIPLIER * DISTANCE
        elif (CURRENT_DISTANCE - DISTANCE < -0.04 and TRANS[0] < 0):
            LINEAR = 1 * MULTIPLIER * DISTANCE
        elif (CURRENT_DISTANCE - DISTANCE > 0.04 and TRANS[0] < 0):
            LINEAR = -1 * MULTIPLIER * DISTANCE
        else:
            LINEAR = 0

        if THETA < -0.05:
            TIME = abs(THETA) * 0.5
            CMD.angular.z = -1 * abs(THETA) * 2
            rospy.sleep(TIME)
        elif THETA > 0.05:
            TIME = abs(THETA) * 0.5
            CMD.angular.z = 1 * abs(THETA) * 2
            rospy.sleep(TIME)
        elif THETA < -1:
            TIME = abs(THETA) * 0.5
            CMD.angular.z = -1 * abs(THETA) * 10
            rospy.sleep(TIME)
        elif THETA > 1:
            TIME = abs(THETA) * 0.5
            CMD.angular.z = 1 * abs(THETA) * 10
            rospy.sleep(TIME)
        else:
            CMD.angular.z = 0

        CMD.linear.x = LINEAR
        TURTLE_VEL.publish(CMD)
