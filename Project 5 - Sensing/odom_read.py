#!/usr/bin/env python3
import rospy,math
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Pose, PoseWithCovariance
from tf.transformations import euler_from_quaternion

class Robot:

    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.odomData = {"roll":self.x, "pitch":self.y, "yaw":self.z}
    
    def getConvertedQData(self, oData : Odometry):
        qData = oData.pose.pose.orientation
        qDataList = [0,1,2,3]
        qDataList[0] = qData.x
        qDataList[1] = qData.y
        qDataList[2] = qData.z
        qDataList[3] = qData.w

        # print("qDataList: ", qDataList)

        eData = euler_from_quaternion(qDataList)
        # print("eData: ", eData)

        return eData


    def odomCallback(self, oData : Odometry):
        # print("Here is some Q-Data: ", qData)
        odomAsEuler = self.getConvertedQData(oData)
        # print(str(odomAsEuler))
        
        self.x = odomAsEuler[0]
        self.y = odomAsEuler[1]
        self.z = odomAsEuler[2]
        
        self.updateOdomData()
        self.printOdomData()

    def updateOdomData(self):
        self.odomData["roll"] = self.x
        self.odomData["pitch"] = self.y
        self.odomData["yaw"] = self.z
    
    def printOdomData(self):
        print("Yaw: ", str(radians2degrees(self.odomData["yaw"])))

def degrees2radians(angle):
	return angle * (math.pi/180.0)

def radians2degrees(angle):
    return math.degrees(angle)

def main():
    
    myRobot = Robot()
    
    rospy.init_node('odomReader', anonymous=True)

    rospy.Subscriber("/odom", Odometry, myRobot.odomCallback)

    
    rate = rospy.Rate(100)

    rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    main()
