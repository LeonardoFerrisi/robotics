#!/usr/bin/env python

#NJW DO NOT delete the line above
#AUTHOR: Leonardo Ferrisi - No lab partner because am quarantined :(
#DATE: January 14, 2021

import rospy
from std_msgs.msg import String
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

import math

class Robot:

    def __init__(self):
        self.old_x = 0
        self.old_y = 0
        self.distance = 0
        self.heading = 0
        self.init = False
        self.startHeading = 0

        # Extra Credit instance variables
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.__extraCredit = True # Toggle to enable extra credit

    def update_distance(self, data_x, data_y):
        """
        Updates the distance of the Robot iff the new position data is different from older position data
        """
        if (data_x!=self.old_x or data_y!=self.old_y or (data_x!=self.old_x and data_y!=self.old_y)):
            self.distance += (math.sqrt((data_x - self.old_x)**2 + (data_y - self.old_y)**2))

    def extraCredit(self):
        """
        If desired, extra credit can be run which will move the turle 4 units forward then halt it
        """
        if (self.distance < 4):
            twist = Twist()
            twist.linear.x = 1
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
        else:
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            self.__extraCredit = False
            # self.init=True
        self.pub.publish(twist)


    def record_data(self,data):
        """
        Prints out the data releating to the Robot's Distance Traveled and Heading
        """
        degrees = math.degrees(data.theta)#%360

        if (self.init==False):
            self.old_x = data.x
            self.old_y = data.y
            self.startHeading = degrees
            self.init=True

        if (self.__extraCredit):
            self.extraCredit()

        # 3a: Distance updater
        self.update_distance(data.x, data.y)

        # 3b: (written as 4b previously): calculate the current heading (direction) in degrees
        self.heading = (degrees - self.startHeading)%360
        print("Distance Traveled:{}, Current Heading:{}".format(self.distance, self.heading))

        # update old data as current data
        self.old_x = data.x
        self.old_y = data.y


def deadReckon():

    rospy.init_node('deadReckon', anonymous=True)

    # 3b - MAKE CHANGES HERE
    rospy.Subscriber("/turtle1/pose",Pose, myRobot.record_data)
    
    # spin() simply keeps python from exiting until this node is stopped
    # We'll talk more about this later
    rospy.spin()

if __name__ == '__main__':
    try:
        myRobot = Robot()
        deadReckon()
    except rospy.ROSInterruptException:
        pass
"""
Answers to Lab Questions:
--------------------------
(1):
> a. No question asked, screenshots of process available at: https://docs.google.com/document/d/1n60u8sM5bjXTqv9FawC8QCY3kaslRj6pnTLua7ipsNo/edit?usp=sharing
> b. No questions asked, see screenshots for details available at: https://docs.google.com/document/d/1n60u8sM5bjXTqv9FawC8QCY3kaslRj6pnTLua7ipsNo/edit?usp=sharing
> c. No questions asked, see screenshots for details available at: https://docs.google.com/document/d/1n60u8sM5bjXTqv9FawC8QCY3kaslRj6pnTLua7ipsNo/edit?usp=sharing

> **d. When I first attempt to run the node; (teleop_twist_keyboard.py), 
     I get the error message: 'Waiting for subscriber to connect to /cmd_vel' 

> e. No questions asked, see screenshots for details available at: https://docs.google.com/document/d/1n60u8sM5bjXTqv9FawC8QCY3kaslRj6pnTLua7ipsNo/edit?usp=sharing

> **f. Unlike the turtle_teleop_key which connects (publishes) directly to the topic '/turtle1/cmd_vel', 
       teleop_twist_keyboard by default connects (publishes) to the topic '/cmd_vel' using the Twist message type

> g. No questions asked, see screenshots for details available at: https://docs.google.com/document/d/1n60u8sM5bjXTqv9FawC8QCY3kaslRj6pnTLua7ipsNo/edit?usp=sharing

(2):
> a. No question asked. See code above: (Code did not work because there was no topic or msg type specified to subscribe to)
> b. No question asked, See code above: (Code did not work because there was no topic or msg type specified to subscribe to)

(3):
> a. See 3a section in Robot.record_data()
> b. See 3b section in Robot.record_data()

(Extra Credit):
> See Robot.extraCredit()

"""
