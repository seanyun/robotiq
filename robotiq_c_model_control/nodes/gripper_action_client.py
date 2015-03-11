#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2014, SRI, Inc.
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
# Copyright (c) 2014, SRI, Inc.

import roslib; roslib.load_manifest('robotiq_c_model_control')
import rospy

# Brings in the SimpleActionClient
import actionlib
import control_msgs.msg
import sys

# This is a simple GripperCommand action client.  It is not specific
# to Robotiq grippers, should work with anything implementing the ROS
# control_msgs/GripperCommand action.  I didn't see a generic Python
# one lying around, so I wrote one and am leaving it here for now.
#   - Dave Hershberger

def gripper_client(command):
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('CModelActionServer', control_msgs.msg.GripperCommandAction)

    print "waiting for server..."
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    print "done waiting for server."

    # Creates a goal to send to the action server.
    goal = control_msgs.msg.GripperCommandGoal()
    if(command == "open"):
        goal.command.position = 85
    if(command == "close"):
        goal.command.position = 0
    if(command == "halfway"):
        goal.command.position = 85 / 2
    goal.command.max_effort = 1

    real_pos = goal.command.position

    goal.command.position = real_pos
    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A GripperCommandResult

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("USAGE: gripper_action_client.py <open|close|halfway>")
        exit(1)
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('gripper_action_client_py')
        result = gripper_client(sys.argv[1])
        print "Result:"
        print repr(result)
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
