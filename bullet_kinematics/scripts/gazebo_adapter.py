#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray

pub = []
for leg in range(0, 6):
    pub.append([])
    for segment in range(0, 3):
        pub[leg].append(rospy.Publisher('bullet/leg_%d_%d_ctrl/command' % (leg + 1, segment + 1),Float64, queue_size=1))


def callback(data):
    for leg_number in range(0, 6):
        for servo in range (0, 3):
            publisher = pub[leg_number][servo]
            publisher.publish(data=data.data[leg_number*3+servo])


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('bullet_gazebo_adapter')

    rospy.Subscriber("bullet_driver/present_joints_goal_position", Float32MultiArray, callback)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
