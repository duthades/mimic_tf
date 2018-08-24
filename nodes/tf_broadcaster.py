#!/usr/bin/env python
"""
Broadcast positions of turtles with reference to world
"""
import rospy
import roslib
import tf
import turtlesim.msg
roslib.load_manifest('mimic_tf')


def handle_turtle_pose(msg, turtlename):
    """
    Function to broadcast turtle positions
    """
    br_tf = tf.TransformBroadcaster()
    br_tf.sendTransform((msg.x, msg.y, 0),
                        tf.transformations.quaternion_from_euler(0,
                                                                 0, msg.theta),
                        rospy.Time.now(),
                        turtlename,
                        "world")


if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    TURTLENAME = rospy.get_param('~turtle')
    rospy.Subscriber('/%s/pose' % TURTLENAME,
                     turtlesim.msg.Pose,
                     handle_turtle_pose,
                     TURTLENAME)
    rospy.spin()
