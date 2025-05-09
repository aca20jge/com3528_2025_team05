#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Makes a Miro look for and 'play' with another Miro.
"""

# Imports
import os
import math
import rospy

from sensor_msgs.msg import JointState, Range
from geometry_msgs.msg import TwistStamped, Pose2D

import miro2 as miro


class MiRoClient:
    """
    Script settings below
    """
    ##########################
    TICK = 0.02  # This is the update interval for the main control loop in secs
    SLOW = 0.1  # Radial speed when turning on the spot (rad/s)
    FAST = 0.4  # Linear speed when kicking the ball (m/s)
    MIN_DISTANCE = 0.18  # Minimum distance to maintain between MiRos (m)

    ##########################
    """
    End of script settings
    """

    def reset_head_pose(self, miro_agent=1):
        """
        Reset MiRo head to default position
        """
        kin_joints = JointState()
        kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
        kin_joints.position = [0.0, math.radians(34.0), 0.0, 0.0]
        t = 0
        while not rospy.core.is_shutdown():
            if miro_agent == 1:
                self.pub_kin.publish(kin_joints)
            else:
                self.pub_kin_2.publish(kin_joints)
            rospy.sleep(self.TICK)
            t += self.TICK
            if t > 1:
                break

    def drive(self, speed_l=0.1, speed_r=0.1, miro_agent=1):  # (m/sec, m/sec)
        """
        Helps Miro drive 
        Altered version of example script code to include multiple Miros
        """
        # Prepare an empty velocity command message
        msg_cmd_vel = TwistStamped()

        # Desired wheel speed (m/sec)
        wheel_speed = [speed_l, speed_r]

        # Convert wheel speed to command velocity (m/sec, Rad/sec)
        (dr, dtheta) = miro.lib.wheel_speed2cmd_vel(wheel_speed)

        # Update the message with the desired speed
        msg_cmd_vel.twist.linear.x = dr
        msg_cmd_vel.twist.angular.z = dtheta

        # Publish message to control/cmd_vel topic
        if miro_agent == 1:
            self.vel_pub.publish(msg_cmd_vel)
        else:
            self.vel_pub_2.publish(msg_cmd_vel)

    def callback_miro_pose(self, msg): # Miro pose
        self.miro_pose = msg

    def callback_miro_0_pose(self, msg): # 
        self.miro_0_pose = msg

    def callback_miro_sonar(self, msg):
        self.sonar_distance = msg.range

    def callback_miro_0_sonar(self, msg):
        self.miro_0_sonar_distance = msg.range

    def angle_between_miros(self, miro_pose_1, miro_pose_2):
        '''
        calculates angle between miro_pose_2 to miro_pose_1
        returns angle to other miro in [-pi, pi]
        '''
        x0, y0 = miro_pose_1.x, miro_pose_1.y
        x, y, theta = miro_pose_2.x, miro_pose_2.y, miro_pose_2.theta

        angle_to_other_miro = math.atan2(y0 - y, x0 - x)
        theta = (theta + math.pi) % (2 * math.pi) - math.pi # normalise to -pi, pi

        return angle_to_other_miro, theta

    def miro_in_view(self, miro_agent=1):
        """
        detect if one MiRo can 'see' anotether using coordinates
        """
        if miro_agent == 1:
            angle_to_other_miro, theta = self.angle_between_miros(self.miro_0_pose, self.miro_pose)
        else:
            angle_to_other_miro, theta = self.angle_between_miros(self.miro_pose, self.miro_0_pose)

        vision_angle = math.radians(25) # random fov "it just works"

        if abs(theta - angle_to_other_miro) < vision_angle:
            return True
        
        return False

    def look_for_miro(self):
        """
        rotates miro 1 to look at miro 0
        """
        if self.just_switched_1:
            rospy.loginfo("MiRo 1 is looking for Miro 0")
            self.just_switched_1 = False

        # find angle between miros
        angle_to_other_miro, miro_1_angle = self.angle_between_miros(self.miro_0_pose, self.miro_pose)
        angle_diff = (angle_to_other_miro - miro_1_angle + math.pi) % (2 * math.pi) - math.pi # normalise to [-pi, pi]

        # rotates if cant see other miro
        if not self.miro_in_view(miro_agent=1):
            if angle_diff < 0:
                self.drive(self.FAST, 0, miro_agent=1)
            else:
                self.drive(0, self.FAST, miro_agent=1)

        # start playing once it finds another miro
        else:
            self.status_code_1 = 2
            self.just_switched_1 = True

    def miro_0_play(self):
        """
        miro 0 moves in a curve, avoiding walls
        """
        if self.just_switched_0:
            rospy.loginfo("Miro 0 is playing...")
            self.just_switched_0 = False

        # detect wall
        if self.miro_0_sonar_distance < self.MIN_DISTANCE:
            rospy.loginfo("Miro 0 sees a wall")
            self.drive(self.SLOW, -self.SLOW, miro_agent=0)

        # run around
        else:
            rospy.loginfo("Miro 0 running")
            self.drive(self.FAST, self.FAST * 0.8, miro_agent=0)

    def miro_1_play(self):
        """
        miro 1 follows miro 2 whilst avoiding walls
        """
        if self.just_switched_1:
            rospy.loginfo("Miro 1 is chasing Miro 0")
            self.just_switched_1 = False

        # avoid walls
        if self.sonar_distance < self.MIN_DISTANCE:
            self.avoiding_wall_1 = True
            self.drive(self.SLOW, -self.SLOW, miro_agent=1)
        elif self.avoiding_wall_1 and self.sonar_distance < self.MIN_DISTANCE:
            self.drive(self.SLOW, -self.SLOW, miro_agent=1)

        # if wall avoided or other miro lost, look for other miro again
        elif self.avoiding_wall_1 or not self.miro_in_view(miro_agent=1):
            self.avoiding_wall_1 = False
            self.status_code_1 = 1
            self.just_switched_1 = True
            self.drive(0, 0, miro_agent=1)

        # follow other miro
        else:
            self.drive(self.FAST, self.FAST, miro_agent=1)


    def __init__(self):

        # ros node to communicate with miro
        rospy.init_node("play_behaviour", anonymous=True)
        
        # time to initialise
        rospy.sleep(2.0)
        
        # Robot prefixes for topics/subscribers
        miro_topic = "/miro" 
        miro_topic_2 = "/miro_0"

        # first miro 1 location data
        self.miro_pose = rospy.Subscriber(
            miro_topic + "/sensors/body_pose",
            Pose2D,
            self.callback_miro_pose
        )
        # miro 0 location
        self.miro_0_pose = rospy.Subscriber(
            miro_topic_2 + "/sensors/body_pose",
            Pose2D,
            self.callback_miro_0_pose
        )

        # miro 1 sonar
        self.sonar = rospy.Subscriber(
            miro_topic + "/sensors/sonar",
            Range,
            self.callback_miro_sonar
        )
        # miro 0 sonar
        self.sonar_2 = rospy.Subscriber(
            miro_topic_2 + "/sensors/sonar",
            Range,
            self.callback_miro_0_sonar
        )

        
        # miro 1 velocity publisher
        self.vel_pub = rospy.Publisher(
            miro_topic + "/control/cmd_vel", TwistStamped, queue_size=0
        )
        # miro 0 velocity publisher
        self.vel_pub_2 = rospy.Publisher(
            miro_topic_2 + "/control/cmd_vel", TwistStamped, queue_size=0
        )

        # miro 1 head publisher
        self.pub_kin = rospy.Publisher(
            miro_topic + "/control/kinematic_joints", JointState, queue_size=0
        )
        # miro 0 head publisher
        self.pub_kin_2 = rospy.Publisher(
            miro_topic_2 + "/control/kinematic_joints", JointState, queue_size=0
        )

        # for determining if function just changed for both miros
        self.just_switched_0 = True
        self.just_switched_1 = True
        
        # flag if currently avoiding obstacle
        self.avoiding_wall_1 = False

        # status codes for both miros (for changing funcs)
        self.status_code_0 = 1
        self.status_code_1 = 1

        self.reset_head_pose(miro_agent=1)
        self.reset_head_pose(miro_agent=0)

    def loop(self):
        """
        main control loop
        """
        print("MiRo play behavior, press CTRL+C to halt...")
        while not rospy.core.is_shutdown():
            # calls miro 0 funcs
            if self.status_code_0 == 1:
                self.miro_0_play()
            else:
                self.miro_0_play()

            # calls miro 1 funcs
            if self.status_code_1 == 1:
                self.look_for_miro()
            elif self.status_code_1 == 2:
                self.miro_1_play()
            else:
                self.status_code_1 = 1

            rospy.sleep(self.TICK)



# This condition fires when the script is called directly
if __name__ == "__main__":
    main = MiRoClient()  # Instantiate class
    main.loop()  # Run the main control loop
