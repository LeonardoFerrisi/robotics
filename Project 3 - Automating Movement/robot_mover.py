#!/usr/bin/env python3
import rospy,math
from std_msgs.msg import String
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import pow, atan2, sqrt

# Notice the message types that have been imported. Look up the descriptions
# of these message types on the ros wiki for more information
# http://docs.ros.org/kinetic/api/turtlesim/html/msg/Pose.html
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html

class Robot:

    def __init__(self):

        self.init = False
        self.turtlesim_pose = Pose()

        # Extra Credit
        self.originalTheta = 0
        self.distanceTurned = 0

    def poseCallback(self,data):

        self.turtlesim_pose.x = data.x
        self.turtlesim_pose.y = data.y
        self.turtlesim_pose.theta = data.theta

    def setDesiredOrientation(self,desired_angle_radians):
        print("Setting orientation to:",desired_angle_radians)
        print("Current angle is:",self.turtlesim_pose.theta)


        rotate_clockwise = True
        clockwise_factor = 1
        if desired_angle_radians < 0:
            rotate_clockwise = False
            clockwise_factor = -1

        relative_angle = desired_angle_radians*clockwise_factor - self.turtlesim_pose.theta

        rotate(desired_angle_radians*clockwise_factor, rotate_clockwise)

    def moveProportionally(self):
        pass

    def getSteeringAngle(self, goal_pose):
        return atan2(goal_pose.y - self.turtlesim_pose.y, goal_pose.x - self.turtlesim_pose.x)


    def steerTo(self, goal_pose, publisher, twist_msg):

        steeringAngle = self.getSteeringAngle(goal_pose)
        currentAngle = self.turtlesim_pose.theta
        angularDistance = steeringAngle - currentAngle
        speedConstant = 4
        direction_mod = 1
        if (angularDistance) < 0:
            direction_mod = -1
        angularVel = ((angularDistance)*speedConstant*direction_mod)

        print("steeringAngle is: ", steeringAngle, "currentAngle is: ", currentAngle, "angularDistance is: ", angularDistance)
        print("Setting angular velocity to: ", angularVel)
        angle_tolerance = 0.001

        rate = rospy.Rate(10)

        while (abs(angularDistance) > angle_tolerance):
            twist_msg.angular.z = radians2degrees(angularVel)
            steeringAngle = self.getSteeringAngle(goal_pose)
            currentAngle = self.turtlesim_pose.theta
            angularDistance = steeringAngle - currentAngle
            angularVel = (angularDistance) * speedConstant*direction_mod
            publisher.publish(twist_msg)
            rate.sleep()

        twist_msg.angular.z = 0
        publisher.publish(twist_msg)

        print("Finished Steering")

    def moveGoal(self, goal_pose, distance_tolerance):

        self.originalTheta = self.turtlesim_pose.theta
        print("Current Theta: ", self.originalTheta)
        pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        rate = rospy.Rate(10)
        vel_msg = Twist()

        current_distance = findDistance(goal_pose.x, self.turtlesim_pose.x, goal_pose.y, self.turtlesim_pose.y)

        print("Now attempting to steering towards: ", goal_pose.x, ", ", goal_pose.y)
        self.steerTo(goal_pose=goal_pose, publisher=pub, twist_msg=vel_msg)
        rate.sleep()
        # Move towards goal
        speedConstant = 1.5
        linearVel = current_distance*speedConstant

        while ( current_distance > distance_tolerance ):
            vel_msg.linear.x = linearVel
            pub.publish(vel_msg)
            current_distance = findDistance(goal_pose.x, self.turtlesim_pose.x, goal_pose.y, self.turtlesim_pose.y)
            linearVel = current_distance * speedConstant
            print("Current X:", self.turtlesim_pose.x, " Current Y:", self.turtlesim_pose.y)
            rate.sleep()

        vel_msg.linear.x = 0
        pub.publish(vel_msg)
        rate.sleep()

def findDistance(x1,x0,y1,y0):
    newDistance = math.sqrt((x1-x0)**2 + (y1-y0)**2)
    return newDistance


def degrees2radians(angle):
	return angle * (math.pi/180.0)

def radians2degrees(angle):
    return math.degrees(angle)

def move(distance, isForward):

    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    outData = Twist()

    speed = 1.0
    backward_speed = -1.0
    t0 = rospy.get_rostime().secs

    while t0 == 0:
        t0 = rospy.get_rostime().secs

    current_distance = 0
    rate = rospy.Rate(10)
    if (isForward):
        ## Create a twist message to make this go linear 1.0 value
        outData.linear.x = speed
    else:
        outData.linear.x = backward_speed


    while (current_distance < distance):
        pub.publish(outData)

        t1 = rospy.get_rostime().secs

        current_distance = outData.linear.x * (t1 - t0)
        rate.sleep() # sleeps such that this loop runs ten times a second

def rotate(relative_angle, isClockwise):
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    outData = Twist()
    angular_vel = degrees2radians(30.0)
    # Internal Clock ###########
    t0 = rospy.get_rostime().secs
    while t0 == 0:
        t0 = rospy.get_rostime().secs
    ####################
    current_angle = degrees2radians(0)
    rate = rospy.Rate(1)

    if (isClockwise==True):
        outData.angular.z = -abs(angular_vel)
        clockwise_factor = -1
    else:
        outData.angular.z = abs(angular_vel)
        clockwise_factor = 1

    while (abs(current_angle) < abs(relative_angle)):
        pub.publish(outData)
        t1 = rospy.get_rostime().secs
        current_angle = abs(outData.angular.z * (t1 - t0))*clockwise_factor
        rate.sleep()

def moveRobot():

    rospy.init_node('robotMover', anonymous=True)
    rospy.Subscriber("/turtle1/pose", Pose, myRobot.poseCallback)
    rate = rospy.Rate(100)


    # Below are a sequence of tests. Uncomment each, along with the rate.sleep()
    # command following it.
    #(1)
    #(1) 
    # move(3.0, True)
    # rate.sleep()

    #(2)
    # rotate(degrees2radians(90.0),True)
    #(2)
    # rate.sleep()

    #(3)
    # myRobot.setDesiredOrientation(degrees2radians(-270))
    #(3)
    # rate.sleep()

    goal_pose = Pose()
    goal_pose.x = 1.0
    goal_pose.y = 1.0
    goal_pose.theta = 0.0
    myRobot.moveGoal(goal_pose,0.1)
    rate.sleep()

if __name__ == '__main__':
    myRobot = Robot()
    moveRobot()
