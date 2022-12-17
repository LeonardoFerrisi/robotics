#!/usr/bin/env python3
import rospy,math
from std_msgs.msg import String
from turtlesim.msg import Pose
from sensor_msgs.msg import LaserScan
from math import pow, atan2, sqrt


class Robot:

    def __init__(self):
        self.fwd = 0
        self.left = 0
        self.right = 0
        self.bkwd = 0
        # self.scanData = [self.fwd, self.right,self.bkwd, self.left]
        self.scanData = {"Front":self.fwd, "Right":self.right, "Rear":self.bkwd, "Left":self.left}

    def laserCallback(self, inData):

        self.fwd = inData.ranges[0]
        self.right = inData.ranges[89]
        self.bkwd = inData.ranges[179]
        self.left = inData.ranges[269]
        
        self.updateScanData()
        self.printScanData()

    def updateScanData(self):
        self.scanData["Front"] = self.fwd
        self.scanData["Right"] = self.right
        self.scanData["Rear"] = self.bkwd
        self.scanData["Left"] = self.left
    
    def printScanData(self):
        print(str(self.scanData))
        print("Front: ", str(type(self.scanData["Front"])))

def degrees2radians(angle):
	return angle * (math.pi/180.0)

def radians2degrees(angle):
    return math.degrees(angle)

def main():
    
    myRobot = Robot()
    
    rospy.init_node('lidarScan', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, myRobot.laserCallback)
    rate = rospy.Rate(100)

    # print(myRobot.scanData)

    rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    main()
    


