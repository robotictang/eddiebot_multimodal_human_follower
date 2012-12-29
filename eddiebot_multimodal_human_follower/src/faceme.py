#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Tang Tiong Yew
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
import roslib; roslib.load_manifest('eddiebot_human_follower')
import rospy

#import message types:
#from mapping_msgs.msg import PolygonalMap
from pi_tracker.msg import Skeleton
from geometry_msgs.msg import Twist

from math import  *


START_ACC = 0.2

class Acc:
    rate = 0.0
    def get(self):
        return self.rate
    def set(self,rate_input):
        self.rate = rate_input

global acc_instance

acc_instance = Acc()

def getangle(scan,index):
    return scan.angle_min+scan.angle_increment*float(index)


def cmdstop():
    print "stopping"
    cmd = Twist()
    pub.publish(cmd)
    acc_instance.set(START_ACC)
hadcallback=1    


    
def myskelCallback(skeleton):
    
    
    # Smoothing
    if(acc_instance.get() <= 1):
        acc_instance.set(acc_instance.get() + 0.002)
    else:
        print "acc_instance is one"
    
    global hadcallback
    hadcallback=0
    avex = 0.0
    avez = 0.0
    for p in skeleton.position:
        avex+=p.x
        avez+=p.z
    if len(skeleton.position) == 0:
        cmdstop()
        return
    avex/=len(skeleton.position)
    avez/=len(skeleton.position)
    
    cmd = Twist()
    
    #if the object is within .8 meters, turn to face
#    if abs(avex) > .01:
#       cmd.angular.z = avex *2.0
#    
#    if abs(avez - 1.50) > .1:
#       cmd.linear.x = (avez-1.50)*1.0 
    
    #if the object is within .8 meters, turn to face
    if abs(avex) > .01:
       cmd.angular.z = avex * 0.25 #* acc_instance.get()
    else:
        acc_instance.set(START_ACC)
    
    if abs(avez - 1.35) > .1:
       cmd.linear.x = (avez-1.35) * 2.5 * acc_instance.get()
    else:
        acc_instance.set(START_ACC)
       
    
       
    #regardless of whether we set things, publish the command:
    pub.publish(cmd)   

    
#this command registers this process with the ros master
rospy.init_node('faceme')

#register a publisher, to topic '/base_controller/command', of type Twist
pub = rospy.Publisher('/cmd_vel', Twist)
#register a callback for messages of type LaserScan, on the topic "/base_scan"
sub = rospy.Subscriber("/skeleton", Skeleton, myskelCallback)
cmdstop()

#this line is equivalent to: 
# while(everything is ok)
#   check for messages, call callbacks
#   sleep
#rospy.spin()

r = rospy.Rate(5) # 5hz

while not rospy.is_shutdown():
    hadcallback+=1
    if hadcallback > 1:
        print "Timeout exceded"
        cmdstop()
    r.sleep()




