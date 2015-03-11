#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2014, SRI International.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2014, SRI International.
# Revision $Id$

"""@package docstring
ROS node providing an actionlib action server for controling a Robotiq
C-Model gripper using the CModelTcpNode.py driver node.

The script takes no arguments.  It subscribes to CModelRobotInput to
receive gripper status and publishes to CModelRobotOutput to send
commands. It implements an actionlib action server using the
"GripperCommand" action defined in
control_msgs/action/GripperCommand.action.
"""

import roslib; roslib.load_manifest('robotiq_c_model_control')
roslib.load_manifest('robotiq_modbus_tcp')
import actionlib
import control_msgs.msg
import rospy
import threading
from robotiq_c_model_control.c_model_85_conversions import CModel85Conversions
from robotiq_c_model_control.msg import _CModel_robot_input  as inputMsg
from robotiq_c_model_control.msg import _CModel_robot_output as outputMsg

class CModelActionServer(object):
    feedback_ = control_msgs.msg.GripperCommandFeedback()
    result_   = control_msgs.msg.GripperCommandResult()
    conversions_ = CModel85Conversions()

    def __init__(self, action_name):

        self.gripper_status_ = None
        self.status_lock_ = threading.Lock()
        rospy.Subscriber('CModelRobotInput', inputMsg.CModel_robot_input, self.receiveGripperStatus)    

        self.pub_ = rospy.Publisher('CModelRobotOutput', outputMsg.CModel_robot_output)

        self.activateGripper()

        self.action_name_ = action_name
        self.action_server_ = actionlib.SimpleActionServer(self.action_name_,
                                                           control_msgs.msg.GripperCommandAction,
                                                           execute_cb = self.execute,
                                                           auto_start = False)
        self.action_server_.start()
        rospy.loginfo("Gripper action server started.")

    def getStatus(self):
        self.status_lock_.acquire()
        status = self.gripper_status_
        self.status_lock_.release()
        return status

    def receiveGripperStatus(self, gripper_status_msg):
        self.status_lock_.acquire()
        self.gripper_status_ = gripper_status_msg
        self.status_lock_.release()

    def resetGripper(self):
        msg_to_gripper = outputMsg.CModel_robot_output()
        msg_to_gripper.rACT = 0 # 1 = activate, 0 = reset
        msg_to_gripper.rGTO = 1 # 1 = go to position, 0 = stop
        msg_to_gripper.rATR = 0 # 1 = automatic release in case of e-stop, 0 = normal
        msg_to_gripper.rPR = 0 # all the way open
        msg_to_gripper.rSP = 255 # always use maximum speed (for now)
        msg_to_gripper.rFR = 0 # minimum force for activation

        gripper_status = self.getStatus()
        while gripper_status == None or gripper_status.gACT != 0:
            self.pub_.publish(msg_to_gripper)
            rospy.sleep(0.05)
            gripper_status = self.getStatus()
        print("Gripper reset.")

    def activateGripper(self):
        self.resetGripper()

        # When the gripper is first activated, it goes through a
        # calibration routine where it opens and closes fully.  This
        # function waits for it to finish.
        msg_to_gripper = outputMsg.CModel_robot_output()
        msg_to_gripper.rACT = 1 # 1 = activate, 0 = reset
        msg_to_gripper.rGTO = 1 # 1 = go to position, 0 = stop
        msg_to_gripper.rATR = 0 # 1 = automatic release in case of e-stop, 0 = normal
        msg_to_gripper.rPR = 0 # all the way open
        msg_to_gripper.rSP = 255 # always use maximum speed (for now)
        msg_to_gripper.rFR = 0 # minimum force for activation

        gripper_status = self.getStatus()
        while gripper_status == None or gripper_status.gSTA != 3:
            self.pub_.publish(msg_to_gripper)
            rospy.sleep(0.05)
            gripper_status = self.getStatus()
        print("Gripper activated.")

    def execute(self, goal):
        rospy.loginfo("Gripper action goal received.")

        command = goal.command

        msg_to_gripper = outputMsg.CModel_robot_output()
        msg_to_gripper.rACT = 1 # 1 = activate, 0 = reset
        msg_to_gripper.rGTO = 1 # 1 = go to position, 0 = stop
        msg_to_gripper.rATR = 0 # 1 = automatic release in case of e-stop, 0 = normal
        msg_to_gripper.rPR = int(self.conversions_.dist_to_command(command.position))
        msg_to_gripper.rSP = 255 # always use maximum speed (for now)
        msg_to_gripper.rFR = int(self.conversions_.force_to_command(command.max_effort))

        self.pub_.publish(msg_to_gripper)

        gripper_status = self.getStatus()

        while (not rospy.is_shutdown() and
               (gripper_status == None or # keep looping until we get a real status
                (gripper_status.gFLT == 0 and # no fault
                 not (gripper_status.gOBJ == 1 or gripper_status.gOBJ == 2 or # haven't stopped due to object - OR
                      (#gripper_status.gPO == msg_to_gripper.rPR and # got to the target position and 
                       gripper_status.gOBJ == 3))))): # arrived-at-goal status is set.

            rospy.loginfo("Gripper action execute loop.")
            rospy.sleep(0.05)

            gripper_status = self.getStatus()

        rospy.loginfo("Gripper action execute loop complete.")
        if rospy.is_shutdown():
            return
               
        # print "status at end of loop:"
        # print repr(gripper_status)

        self.result_.position = self.conversions_.command_to_dist(gripper_status.gPO)
        self.result_.effort = command.max_effort

        self.result_.stalled = (gripper_status.gOBJ == 2)
        self.result_.reached_goal = (gripper_status.gOBJ == 3)

        if gripper_status.gOBJ == 3:
            rospy.loginfo("Gripper action execute succeeding.")
            self.action_server_.set_succeeded(self.result_)
        else:
            rospy.loginfo("Gripper action execute aborting.")
            self.action_server_.set_aborted(self.result_)

if __name__ == '__main__':
    try:
        rospy.init_node('CModelActionServer')
        CModelActionServer(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
