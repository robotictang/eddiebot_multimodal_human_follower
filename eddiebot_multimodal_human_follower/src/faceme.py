#!/usr/bin/env python
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




