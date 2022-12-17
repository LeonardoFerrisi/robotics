#!/usr/bin/env python3
from dis import dis
from logging import exception

import rospy,math
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Pose, PoseWithCovariance, Twist
from tf.transformations import euler_from_quaternion

from turtlesim.msg import Pose
from sensor_msgs.msg import LaserScan
from math import pow, atan2, sqrt


class Robot:

    def __init__(self):
        self.postition = Pose()
        self.scanData = {"Front":0, "fRight":0, "Right":0, "rRear":0, "Rear":0, "lRear":0, "Left":0, "fLeft":0}
        self.directions = []
        self.yaw = 0
        self.__assignDirections()

        self.keep_alive = False
        self.halted = True
        self.debug = True

    def __assignDirections(self):
        # Format for list of directions is [left, leftFront, Front, rightFront, right]
        self.directions.append("Left")
        self.directions.append("fLeft")
        self.directions.append("Front")
        self.directions.append("fRight")
        self.directions.append("Right")


    def poseCallback(self, data):

        self.postition.x = data.x
        self.postition.y = data.y
        self.postition.theta = data.theta

    # AVOIDANCE LOGIC #############################

    def getBestDirection(self):

        goodDirections = []

        for direction in self.directions:

            currentDirectionDistance = float(self.scanData[direction])

            if currentDirectionDistance < 0.6 and currentDirectionDistance != float('inf') and currentDirectionDistance!=float(0.0):
                continue
            else: 
                goodDirections.append(direction)

        # print("Good directions: ", goodDirections)

        greatestDistance = 0.0
        bestDirection = ""
        toReturn = ""

        if "Front" not in goodDirections:

            for direction in goodDirections:
                if float(self.scanData[direction]) > greatestDistance:
                    greatestDistance = self.scanData[direction]
                    bestDirection = direction

            if bestDirection == "Left" or bestDirection == "fLeft":
                toReturn = "left"
            elif bestDirection == "Right" or bestDirection == "fRight":
                toReturn = 'right'
            else:
                if self.debug:
                    print("NO GOOD DIRECTIONS")

            return toReturn

        else:
            return "fwd"


    def steerToAvoidThing(self):
        '''
        Steer the robot to avoid a thing
        '''
        directionToTurn = self.getBestDirection()

        if (directionToTurn == "left"):
            if self.debug:
                print('turning left')
            while self.obstacleAhead():
                self.rotate45(1)

        elif directionToTurn == "right":
            if self.debug:
                print('turning right')
            while self.obstacleAhead():
                self.rotate45(-1)

        else:
            pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            movCmd = Twist()

            if directionToTurn == "fwd":
                movCmd.linear.x = 0.2
                pub.publish(movCmd)
            else:
                movCmd.linear.x = -0.2
                pub.publish(movCmd)

    def obstacleAhead(self):
        obstaclesFront = float(self.scanData["Front"]) < float(0.6) and float(self.scanData["Front"]) != float('inf') and float(self.scanData["Front"]) != float(0.0)
        obstaclesToLeft = float(self.scanData["fLeft"]) < float(0.6) and float(self.scanData["fLeft"]) != float('inf') and float(self.scanData["fLeft"]) != float(0.0)
        obstaclesToRight = float(self.scanData["fRight"]) < float(0.6) and float(self.scanData["fRight"]) != float('inf') and float(self.scanData["fRight"]) != float(0.0)
        
        if obstaclesFront or obstaclesFront and obstaclesToLeft or obstaclesToRight or obstaclesToLeft and obstaclesToRight:
            if self.debug:
                print("obstacle ahead")
            return True
        else:
            if self.debug:
                print("No obstacle ahead")
            return False

    ###############################################


    # LASER SCAN ####################################

    def laserCallback(self, laserData : LaserScan):

        self.scanData["Front"] = laserData.ranges[0]
        self.scanData["fLeft"] = laserData.ranges[44]
        self.scanData["Left"] = laserData.ranges[89]
        self.scanData["lRear"] = laserData.ranges[134]
        self.scanData["Rear"] = laserData.ranges[179]
        self.scanData["rRear"] = laserData.ranges[224]
        self.scanData["Right"] = laserData.ranges[269]
        self.scanData["fRight"] = laserData.ranges[314]
    
    ##################################################

    # ODOMETRY #########################################

    def getConvertedQData(self, oData : Odometry):

        qData = oData.pose.pose.orientation
        qDataList = [0,1,2,3]
        qDataList[0] = qData.x
        qDataList[1] = qData.y
        qDataList[2] = qData.z
        qDataList[3] = qData.w

        eData = euler_from_quaternion(qDataList)

        return eData

    def odomCallback(self, oData : Odometry):
        
        odomAsEuler = self.getConvertedQData(oData)
        self.yaw = radians2degrees(odomAsEuler[2])

    #### ROTATION CODE ########################################
    def __haltRotate(self):
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        outData = Twist()
        outData.angular.z = 0 # set angular speed to 0
        pub.publish(outData)

    def rotate45(self, direction):
        '''
        Rotates 45 degrees in the negative or positive direction
        '''
        if direction > 0:
            if self.debug:
                print("Attempting to rotate 45 degrees")
            self.rotate(45) # rotate left
        else:
            if self.debug:
                print("Attempting to rotate -45 degrees")
            self.rotate(-45) # rotate right

    def _convertOrientationTo360(self):
        '''
        Converts the Orientation from 180 , -180 to a full 360 degrees for readability
        '''
        toReturn = self.yaw

        if self.yaw < 0:
            toReturn = 360 + self.yaw
        return toReturn


    def rotate(self, desiredAngle):
        '''
        Rewritten rotate function that uses the ODOMETRY data
        @param desired angle: must be a value of no more than 180 degrees or -180 degrees
        '''
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        outData = Twist()
        angular_vel = degrees2radians(30.0)

        rate = rospy.Rate(10)
        
        # convert all values to 360 format ###

        currentOrientation = self._convertOrientationTo360()

        ######################################
        if desiredAngle == -45: # Turn right 
        
            current = self._convertOrientationTo360()
            distanceTraveled = 0

            turnTo = -45

            if self.debug:
                print("Turning right >> YAW: ", currentOrientation, "distanceTraveled: ", distanceTraveled)

            while distanceTraveled > turnTo:

                if currentOrientation == turnTo:
                    self.__haltRotate()

                currentOrientation = self._convertOrientationTo360()
                
                if self.debug:
                    print("Turning right >> YAW: ", current, "distanceTraveled: ", distanceTraveled)
                
                outData.angular.z = -abs(angular_vel) # for moving clockwise
                pub.publish(outData)
                rate.sleep()
                
                if self._convertOrientationTo360() > current:
                    # in event that YAW in 360 degrees is greater than current... from 0 to 360 ...
                    distanceTraveled += (360 - self._convertOrientationTo360()) - current

                else:
                    distanceTraveled += self._convertOrientationTo360() - current

                current = self._convertOrientationTo360()
            self.__haltRotate()      

        elif desiredAngle == 45: # turn left
            current = self._convertOrientationTo360()
            distanceTraveled = 0
            
            turnTo = 45

            if self.debug:
                print("Turning left >> YAW: ", current, "distanceTraveled: ", distanceTraveled)

            while distanceTraveled < turnTo:

                if currentOrientation == turnTo:
                    self.__haltRotate()

                currentOrientation = self._convertOrientationTo360()

                if self.debug:
                    print("Turning left >> YAW: ", current, "distanceTraveled: ", distanceTraveled)
                
                outData.angular.z = abs(angular_vel)
                pub.publish(outData)
                rate.sleep()

                if self._convertOrientationTo360() < current:
                    # crossed from 360 to 0
                    distanceTraveled += abs(360 + (self._convertOrientationTo360()) - current)
                    current = self._convertOrientationTo360()

                else:
                    distanceTraveled += abs(self._convertOrientationTo360() - current)
                    current = self._convertOrientationTo360()

            self.__haltRotate() 


    ##########################################################


    def runAvoidanceMode(self):
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rate = rospy.Rate(10)

        movCmd = Twist()
        speed = 0.2 

        while True:
            
            if self.obstacleAhead():
                
                ##########################
                if self.debug:
                    print("entered obstacle ahead mode")
                ##########################
                
                if self.halted == False:
                    movCmd.linear.x = 0
                    pub.publish(movCmd)

                while self.obstacleAhead():
                    # print("steering to saftey")
                    self.steerToAvoidThing()
            else:
                movCmd.linear.x = speed
                self.halted = False
                pub.publish(movCmd)

            rate.sleep()
        # rospy.spin()

##################################################################################

def degrees2radians(angle):
	return angle * (math.pi/180.0)

def radians2degrees(angle):
    return math.degrees(angle)

def run_obstacle_avoider():

    myRobot = Robot()

    rospy.init_node('avoid_obstacles', anonymous=True)


    # subscribe to known for topics
    rospy.Subscriber("/scan", LaserScan, myRobot.laserCallback)
    rospy.Subscriber("/pose", Pose, myRobot.poseCallback)
    rospy.Subscriber("/odom", Odometry, myRobot.odomCallback)

    myRobot.runAvoidanceMode()


if __name__ == '__main__':
    
    run_obstacle_avoider()