#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from bullet_gait import srv as StandupLaydownService

dead_man_button_id = 4
stand_up_button_id = 0
lay_down_button_id = 1
lin_vel_x_axe_id = 1
lin_vel_y_axe_id = 0
ang_vel_axe_id = 2
mode_change_button_id = 2
body_pos_x_axe_id = 1
body_pos_y_axe_id = 0
body_orientation_x_axe_id = 3
body_orientation_y_axe_id = 2
body_orientation_max_angle = 0.15
body_pos_max = 0.03

max_lin_vel = 0.1
max_ang_vel = math.pi / 10

mode = 0
#0-velocity 1-body pose
mode_button_pressed = 0
stand_up_button_pressed = 0
lay_down_button_pressed = 0


###############################################################################
# ROS PUBLISHERS
###############################################################################


pub = rospy.Publisher('teleop/body_relative', Twist, queue_size=100)
body_position_pub = rospy.Publisher('body_position', Vector3, queue_size=100)
body_orientation_pub = rospy.Publisher('body_orientation', Vector3, queue_size=100)
rospy.loginfo("Publishing to topic: body_relative")


###############################################################################
# VARIABLES
###############################################################################


def joy_callback(data):

    ###########################################################################
    # VARIABLES
    ###########################################################################

    global mode
    global mode_button_pressed
    global stand_up_button_pressed
    global lay_down_button_pressed

    lin_vel_x = 0
    lin_vel_y = 0
    ang_vel = 0
    body_pos_x = 0
    body_pos_y = 0
    body_orientation_x = 0
    body_orientation_y = 0

    ###########################################################################
    # MODE CHANGE
    ###########################################################################

    if data.buttons[mode_change_button_id] is 1:
        print "MODE CHANGE BUTTON!", mode
        if mode_button_pressed is 0:
            mode = (mode + 1)%2
            mode_button_pressed = 1
    else:
        mode_button_pressed = 0

    ###########################################################################
    # DEADMAN SWITCH
    ###########################################################################
 
    if data.buttons[dead_man_button_id] is 1:

        #######################################################################
        # GO AHEAD 
        #######################################################################

        if mode is 0:
            lin_vel_x = data.axes[lin_vel_x_axe_id] * max_lin_vel
            lin_vel_y = data.axes[lin_vel_y_axe_id] * max_lin_vel
            ang_vel = data.axes[ang_vel_axe_id] * max_ang_vel

        #######################################################################
        # DANCE BABY, DANCE
        #######################################################################

        else:
            body_pos_x = data.axes[body_pos_x_axe_id] * body_pos_max
            body_pos_y = data.axes[body_pos_y_axe_id] * body_pos_max
            body_orientation_x = data.axes[body_orientation_x_axe_id] * body_orientation_max_angle
            body_orientation_y = data.axes[body_orientation_y_axe_id] * body_orientation_max_angle

        #######################################################################
        # STAND UP / LAY DOWN
        #######################################################################

        if data.buttons[stand_up_button_id] is 1:
            if stand_up_button_pressed is 0:
                print "SIMON SAYS: STAND!"
                rospy.ServiceProxy("stand_up", StandupLaydownService.StandUpLayDown).call()
                stand_up_button_pressed = 1
        else:
            stand_up_button_pressed = 0

        if data.buttons[lay_down_button_id] is 1:
            if lay_down_button_pressed is 0:
                rospy.ServiceProxy("lay_down", StandupLaydownService.StandUpLayDown).call()
                lay_down_button_pressed = 1
        else:
            lay_down_button_pressed = 0

        #######################################################################
        # SEND MESSAGES
        #######################################################################

    twist_msg = Twist()
    twist_msg.linear.x = lin_vel_x
    twist_msg.linear.y = lin_vel_y
    twist_msg.angular.z = ang_vel
    pub.publish(twist_msg)

    body_pos_msg = Vector3()
    body_pos_msg.x = body_pos_x
    body_pos_msg.y = body_pos_y
    body_position_pub.publish(body_pos_msg)

    body_orientation_msg = Vector3()
    body_orientation_msg.x = body_orientation_x
    body_orientation_msg.y = body_orientation_y
    body_orientation_pub.publish(body_orientation_msg)


###############################################################################
# MAIN
###############################################################################


if __name__ == '__main__':
    rospy.init_node('bullet_teleop')
    try:
        dead_man_button_id = rospy.get_param('joy/dead_man_switch_id')
        lin_vel_x_axe_id = rospy.get_param('linear_velocity_x_axe_id')
        lin_vel_y_axe_id = rospy.get_param('linear_velocity_y_axe_id')
        ang_vel_axe_id = rospy.get_param('joy/angular_velocity_axe_id')
        stand_up_button_id = rospy.get_param('standup_button_id')
        lay_down_button_id = rospy.get_param('lay_down_button_id')
        mode_change_button_id = rospy.get_param('mode_change_button_id')
        body_pos_x_axe_id = rospy.get_param('body_pos_x_axe_id')
        body_pos_y_axe_id = rospy.get_param('body_pos_y_axe_id')
        body_orientation_x_axe_id = rospy.get_param('body_orientation_x_axe_id')
        body_orientation_y_axe_id = rospy.get_param('body_orientation_y_axe_id')
        body_orientation_max_angle = rospy.get_param('body_orientation_max_angle')
        body_pos_max = rospy.get_param('body_pos_max')

    except KeyError:
        rospy.logwarn('No joystick button map found on parameter server. Using default mapping.')

    rospy.Subscriber('joy', Joy, joy_callback)
    rospy.spin()
