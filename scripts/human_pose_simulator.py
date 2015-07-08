#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from people_msgs.msg import Person,People
from decimal import *
import tf
from tf import TransformListener
import json

class PoseTransformer:

    def set_visible(self,data):
        d=json.loads(data.data)
        if len(d)>0:
            self.visible=True
        else:
            self.visible=False


    def callback(self,data):
        rospy.logdebug(rospy.get_caller_id()+"I heard something from frame %s",data.header.frame_id)
        if self.visible:
            t = self.tf.getLatestCommonTime(self.target_tf, data.header.frame_id)
            data.header.stamp = t
            new_pose=self.tf.transformPose(self.target_tf,data)
            self.pub.publish(new_pose)

            p = Person()
            p.position.x=new_pose.pose.position.x
            p.position.y=new_pose.pose.position.y
            p.position.z=new_pose.pose.position.z
            
            newTime = data.header.stamp.secs + (data.header.stamp.nsecs*0.000000001)
            p.velocity.x = Decimal((new_pose.pose.position.x-self.oldX)/(newTime-self.oldT))
            p.velocity.y = Decimal((new_pose.pose.position.y-self.oldY)/(newTime-self.oldT))
            p.reliability = 1.0

            p1 = People()
            p1.header = new_pose.header
            p1.people.append(p)
            self.pubpeople.publish(p1)

            self.oldX = new_pose.pose.position.x
            self.oldY = new_pose.pose.position.y
            self.oldT = newTime
        else:
            rospy.logdebug("human not visible at the moment")

    def __init__(self):
        self.oldX = 0
        self.oldY = 0
        self.oldT = 0
        rospy.init_node('posetransformer_node', anonymous=True)
        self.in_topic = rospy.get_param('~in', '/human/pose')
        self.out_topic_trans = rospy.get_param('~out', '/human/transformed')
        self.out_topic = rospy.get_param('~out', '/people')
        self.target_tf = rospy.get_param('~target', '/robot')
        self.sem_cam = rospy.get_param('~sem_cam', '/humancam')
        self.tf = TransformListener()

        self.sub = rospy.Subscriber(self.in_topic, PoseStamped, self.callback)
        self.sub = rospy.Subscriber(self.sem_cam, String, self.set_visible)
        self.pub = rospy.Publisher(self.out_topic_trans, PoseStamped)
        self.pubpeople = rospy.Publisher(self.out_topic, People)
        
        self.visible = True

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    PoseTransformer()