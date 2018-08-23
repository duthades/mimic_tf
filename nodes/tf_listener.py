#!/usr/bin/env python
import rospy
import math
import tf
from std_msgs.msg import String
import roslib
from turtlesim.msg import Pose
import geometry_msgs.msg
import turtlesim.srv
roslib.load_manifest('mimic_tf')


topic = 'chatter'
pub = rospy.Publisher(topic, String, queue_size=10)
rospy.loginfo("I will publish to the topic %s", topic)


class turtlePose():
    pose = Pose()

    def update_pose(self, data):
        """Callback function which is called when a new message of type
           Pose is received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)


if __name__ == '__main__':

    rospy.init_node('tf_listener')
    listener = tf.TransformListener()

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2')

    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,
                                 queue_size=1)

#  Get the pose of the turtle2
    pose_data = turtlePose()
    rospy.Subscriber('/turtle2/pose', Pose, pose_data.update_pose)

    rate = rospy.Rate(10)

    i = 0

    while not rospy.is_shutdown():
        try:

            (trans, rot) = listener.lookupTransform('/turtle2', '/turtle1',
                                                    rospy.Time(0))
            if i == 0:
                distance = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
                i = i + 1

        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            continue

        theta = tf.transformations.euler_from_quaternion(rot)[2]

        str1 = str("x,y: %s" % str(trans))
        rospy.loginfo(str1)
        pub.publish(str1)

        distance1 = math.sqrt(trans[0] ** 2 + trans[1] ** 2)

        cmd = geometry_msgs.msg.Twist()
        multiplier = 1
        if (distance1 - distance < -0.04 and trans[0] >= 0):
            linear = -1 * multiplier * distance
        elif (distance1 - distance > 0.04 and trans[0] >= 0):
            linear = 1 * multiplier * distance
        elif (distance1 - distance < -0.04 and trans[0] < 0):
            linear = 1 * multiplier * distance
        elif (distance1 - distance > 0.04 and trans[0] < 0):
            linear = -1 * multiplier * distance
        else:
            linear = 0

        if theta < -0.05:
            time = abs(theta) * 0.5
            cmd.angular.z = -1*abs(theta)*2
            rospy.sleep(time)
        elif theta > 0.05:
            time = abs(theta) * 0.5
            cmd.angular.z = 1*abs(theta)*2
            rospy.sleep(time)
        elif theta < -1:
            time = abs(theta) * 0.5
            cmd.angular.z = -1*abs(theta)*5
            rospy.sleep(time)
        elif theta > 1:
            time = abs(theta) * 0.5
            cmd.angular.z = 1*abs(theta)*5
            rospy.sleep(time)
        else:
            cmd.angular.z = 0

        cmd.linear.x = linear
        turtle_vel.publish(cmd)
