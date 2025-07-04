#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Makes a Miro look for and 'play' with another Miro.
"""

# Imports
import os
import math
import random
import rospy

from sensor_msgs.msg import JointState, Range
from geometry_msgs.msg import TwistStamped, Pose2D

import miro2 as miro


class MiRoClient:

    TICK = 0.02  # update interval in seconds
    SLOW = 0.1  # m/s
    FAST = 0.4  # m/s
    MIN_DISTANCE = 0.18  # minimum distance to maintain between miros
    N_MIROS = 7 # number of miros to be used in sim

    def __init__(self):
        rospy.loginfo("starting")
        # ros node to communicate with miro
        rospy.init_node("play_behaviour", anonymous=True)
        # time to initialise
        rospy.sleep(2.0)
        
        self.miros = {}

        rospy.loginfo("getting miros...")

        # create dictionary with attributes for all miros
        for miro_id in range(self.N_MIROS):

            miro_topic =  f"/miro_{miro_id-1}" if miro_id > 0 else "/miro"
            
            rospy.loginfo(f"initialising miro {miro_topic}")

            self.miros[miro_id] = {
                "id": miro_id,
                "pose": Pose2D(), # initialise pose object
                "sonar": 100.0, # initialise sonar arbitrarily high
                "just_switched": True, # avoids excessive printing
                "status_code": 1, # controls flow
                "excitement": 0, # level of excitement--changes based on social behaviours
                "runner": False, # is going to be chased by another miro
                "is_chasing": False, # is going to chase another miro
                "partner": None, # miro id it is currently looking at
                # subscribers use lambda functions for dynamic callbacks for each miro 
                "pose_sub": rospy.Subscriber(miro_topic + "/sensors/body_pose", Pose2D, lambda msg, id=miro_id: self.callback_pose(msg, id)), 
                "sonar_sub": rospy.Subscriber(miro_topic + "/sensors/sonar", Range, lambda msg, id=miro_id: self.callback_sonar(msg, id)),
                "vel_pub": rospy.Publisher(miro_topic + "/control/cmd_vel", TwistStamped, queue_size=0),
                "kin_pub": rospy.Publisher(miro_topic + "/control/kinematic_joints", JointState, queue_size=0)
            }

        # reset head pose of all miros
        for miro_id in self.miros:
            self.reset_head_pose(miro_id)
            rospy.loginfo(f"head pose reset for miro {miro_id}")

    def callback_pose(self, msg, miro_id):
        self.miros[miro_id]["pose"] = msg

    def callback_sonar(self, msg, miro_id):
        self.miros[miro_id]["sonar"] = msg.range


    def reset_head_pose(self, miro_id):
        """
        resets miro's head to default position
        altered from example code
        """
        kin_joints = JointState()
        kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
        kin_joints.position = [0.0, math.radians(34.0), 0.0, 0.0]
        t = 0
        while not rospy.core.is_shutdown():
            self.miros[miro_id]["kin_pub"].publish(kin_joints)
            rospy.sleep(self.TICK)
            t += self.TICK
            if t > 1:
                break


    def drive(self, speed_l=0.1, speed_r=0.1, miro_id=0):  # (m/sec, m/sec)
        """
        helps miro drive 
        altered version of example script code to work with multiple miros
        """
        # empty vel message
        msg_cmd_vel = TwistStamped()
        wheel_speed = [speed_l, speed_r]

        # convert wheel speed to command velocity (m/sec, Rad/sec)
        (dr, dtheta) = miro.lib.wheel_speed2cmd_vel(wheel_speed)

        # update the message with the desired speed
        msg_cmd_vel.twist.linear.x = dr
        msg_cmd_vel.twist.angular.z = dtheta

        # publish message to control/cmd_vel topic for given miro
        self.miros[miro_id]["vel_pub"].publish(msg_cmd_vel)

    def angle_between_miros(self, other_miro=None, miro_id=None):
        '''
        calculates angle between miro_pose_2 to miro_pose_1
        returns angle to other miro in [-pi, pi] and normalised angle of current miro
        '''

        # gets pose of current miro
        first_pose = self.miros[miro_id]["pose"]
        
        # automatically finds target miro if parameter not passed
        if other_miro == None:
            second_pose = self.miros[self.miros[miro_id]["partner"]]["pose"]
        # otherwise uses parameters
        else:
            second_pose = self.miros[other_miro]["pose"]

        # gets coordinates of current miro and target miro
        x, y, theta = first_pose.x, first_pose.y, first_pose.theta
        x0, y0 = second_pose.x, second_pose.y

        # calculates an angle between the miros
        angle_to_other_miro = math.atan2(y0 - y, x0 - x)

        # normalise to -pi, pi
        theta = (theta + math.pi) % (2 * math.pi) - math.pi

        return angle_to_other_miro, theta

    def other_miro_in_view(self, miro_id):
        """
        detect if miro can 'see' another using coordinates
        """
        miro = self.miros[miro_id]
        # set an fov—this probably isnt miros actual fov
        vision_angle = math.radians(25) 

        # if not targeting a specific agent
        if miro["partner"] == None:

            # iterate through each agent
            for other_miro in self.miros:
                if other_miro != miro_id:

                    # detect if other_miro is in view
                    angle_to_other_miro, theta = self.angle_between_miros(other_miro, miro_id)
                    if abs(theta - angle_to_other_miro) < vision_angle:
                        
                        # assign as partner if in view
                        miro["partner"] = other_miro
                        return True
            
            # if no miro in view return false
            return False
        
        # detects if partner miro in view if applicable
        else:
            angle_to_other_miro, theta = self.angle_between_miros(miro["partner"], miro_id)
            if abs(theta - angle_to_other_miro) < vision_angle:
                return True
            return False


    def look_for_miro(self, miro_id):
        """
        miros wander until seeing another miro
        """
        miro = self.miros[miro_id]
        if miro["just_switched"]:
            miro["just_switched"] = False

        # wanders around whilst avoiding walls if a miro is not in view
        if not self.other_miro_in_view(miro_id):
            if miro["sonar"] > self.MIN_DISTANCE:
                self.drive(self.FAST, self.FAST*0.7, miro_id)
            else:
                self.drive(self.FAST, -self.FAST, miro_id)
        
        # if a miro is in view, play or approach
        else:
            self.miros[miro_id]["just_switched"] = True
            
            # continue playing if already chasing
            if miro["is_chasing"]:
                miro["status_code"] = 3
            
            # otherwise just approach
            else:
                miro["status_code"] = 2



    def approach_other_miro(self, miro_id):
        """
        approach another miro before playing
        """
        miro = self.miros[miro_id]
        partner = self.miros[self.miros[miro_id]["partner"]]

        if miro["just_switched"] == True:
            rospy.loginfo(f"miro {miro_id} is slowly approaching {partner['id']}")
            miro["just_switched"] = False

        # slowly approach other miro
        if self.other_miro_in_view(miro_id):

            # increase excitement while other miro in view
            miro["excitement"] = 100 if miro["excitement"] > 100 else miro["excitement"] + random.random()

            # play if excited enough
            if miro["excitement"] >= 100:
                # play if partner is excited or is currently playing
                if partner["excitement"] >=100 or partner["is_chasing"] or partner["runner"]:
                    miro["status_code"] = 3
                    miro["just_switched"] = True

                # if partner is not yet excited, continue to approach
                else:
                    # adopt runner role if partner is not runner
                    if not partner["runner"]:
                        miro["runner"] = True
                    if miro["sonar"] > self.MIN_DISTANCE:
                        self.drive(self.FAST*0.5, self.FAST*0.5, miro_id)
                    else:
                        self.drive(-self.SLOW, self.SLOW, miro_id)
                    
            # approach other miro whilst avoiding wall
            elif miro["sonar"] > self.MIN_DISTANCE:
                self.drive(self.FAST*0.5, self.FAST*0.5, miro_id)
            else:
                self.drive(-self.SLOW, self.SLOW, miro_id)
                    
        # look at other miro again if lost
        else:
            angle_to_other_miro, miro_0_angle = self.angle_between_miros(miro_id=miro_id) 

            angle_diff = (angle_to_other_miro - miro_0_angle + math.pi) % (2 * math.pi) - math.pi

            # turn in direction of partner miro
            if angle_diff < 0:
                self.drive(self.FAST, 0, miro_id)
            else:
                self.drive(0, self.FAST, miro_id)
        
    def play_run(self, miro_id):
        """
        miro 0 turns away from miro 1 and moves in a curve, avoiding walls
        """
        
        miro = self.miros[miro_id]

        if miro["just_switched"]:
            rospy.loginfo(f"miro {miro_id} is playing")
            miro["just_switched"] = False

        # decrease excitement whilst playing
        miro["excitement"] -= random.random()/5

        # get angles between miros
        angle_to_other_miro, miro_0_angle = self.angle_between_miros(miro_id=miro_id) 
        angle_diff = (angle_to_other_miro - miro_0_angle + math.pi) % (2 * math.pi) - math.pi

        if miro["excitement"] > 0:
            # turns away from miro 1 before starting to run (avoids a crash)
            if abs(angle_diff) < math.radians(90):
                self.drive(self.SLOW, -self.SLOW, miro_id)
            
            # detect wall
            elif miro["sonar"] < self.MIN_DISTANCE:
                self.drive(self.SLOW, -self.SLOW, miro_id)
            # run around
            else:
                self.drive(self.FAST, self.FAST*0.8, miro_id)

        # once excitement is depleted, reset and look for another miro
        else:
            self.drive(0, 0, miro_id)
            miro["runner"] = False
            miro["partner"] = None
            miro["status_code"] = 1
            miro["just_switched"] = True
            rospy.loginfo(f"miro {miro_id} is bored")

    def play_chase(self, miro_id):
        """
        miro 1 follows miro 0 whilst avoiding walls
        """
        miro = self.miros[miro_id]
        if miro["just_switched"]:
            rospy.loginfo(f"miro {miro_id} is chasing {miro['partner']}")
            miro["just_switched"] = False
            miro["is_chasing"] = True

        # decrease excitement whilst playing
        miro["excitement"] -= random.random()/5

        if miro["excitement"] > 0:
            # avoid collisions
            if miro["sonar"] < self.MIN_DISTANCE:
                self.drive(-self.SLOW, -self.SLOW, miro_id)
            
            # if other miro lost, look for other miro again
            elif not self.other_miro_in_view(miro_id):
                angle_to_other_miro, miro_0_angle = self.angle_between_miros(other_miro=miro["partner"], miro_id=miro_id) 
                angle_diff = (angle_to_other_miro - miro_0_angle + math.pi) % (2 * math.pi) - math.pi
                
                # turn in direction of other miro
                if angle_diff < 0:
                    self.drive(self.FAST, 0, miro_id)
                else:
                    self.drive(0, self.FAST, miro_id)

            # chase other miro whilst in view
            else:
                self.drive(self.FAST, self.FAST, miro_id)

        # once excitement depletes, reset and look for another miro
        else:
            self.drive(0, 0, miro_id)
            miro["is_chasing"] = False
            miro["partner"] = None
            miro["status_code"] = 1
            miro["just_switched"] = True
            rospy.loginfo(f"miro {miro_id} is bored")

    def loop(self):
        """
        main control loop
        """
        print("MiRo play behavior, press CTRL+C to halt...")
        while not rospy.core.is_shutdown():

            # iterates through each miro, calling a control function respectively to its status code
            for miro_id in self.miros:
                miro = self.miros[miro_id]
                if miro["status_code"] == 1:
                    self.look_for_miro(miro_id)
                elif miro["status_code"] == 2:
                    self.approach_other_miro(miro_id)
                elif miro["status_code"] == 3:
                    if miro["runner"]:
                        self.play_run(miro_id)
                    else:
                        self.play_chase(miro_id)

            rospy.sleep(self.TICK)


if __name__ == "__main__":
    main = MiRoClient()
    main.loop()
