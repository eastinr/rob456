#!/usr/bin/env python

import rospy
import math
import tf
import time
from tf.transformations import euler_from_quaternion 
from tf.transformations import quaternion_from_euler
import message_filters

# The laser scan message
from sensor_msgs.msg import LaserScan

# The odometry message
from nav_msgs.msg import Odometry
    

# the velocity command message
from geometry_msgs.msg import Twist

command = Twist()

#class to hold tuple values
class Point:
    
    def __init__(self, x, y):
        self.x = x
        self.y = y
        
    def dist(self, goal):
        return math.sqrt((goal.x - self.x)**2 + (goal.y - self.y)**2)
        
    def angle(self, goal):
        return math.atan2(goal.y-self.y, goal.x-self.x)

#class to hold robot functions
class Robot:
    #safety bubble around robot
    radius = 1
    
    scanAngleIncrement = None
    ranges = None
    minScanAngle = None
    maxScanAngle = None
    
    #refresh the robot variables    
    def refresh(self):
       
        globalOdom = Odometry()
        xOr = globalOdom.pose.pose.position.x
        yOr = globalOdom.pose.pose.position.y
        zOr = globalOdom.pose.pose.orientation.z
        wOr = globalOdom.pose.pose.orientation.w
        (roll, pitch, yaw) = euler_from_quaternion([xOr, yOr, zOr, wOr])
        self.x = xOr
        self.y = yOr
        self.location = Point(self.x, self.y)
        self.yaw = yaw
    
    #rotate robot pose to point in the direction of theta    
    def rotate(self, theta):
        
        #difference between pose and goal angle
        dtheta = self.yaw - theta
        
        #angular speed
        speed = 0.1
        
        #stop turning if pointing towards theta
        if abs(dtheta) < .01:
            command.angular.z = 0
        #turn right 
        elif dtheta > 0:
            command.angular.z = -abs(speed)
        #turn left
        elif dtheta < 0:
            command.angular.z = abs(speed)
        #publish command
        if abs(dtheta) > 0.1:
            pub.publish(command)
        else:
            command.angular.z = 0
            pub.publish(command)
        #ensure loop runs at 10 hz
        rospy.Rate(10)
    
    #seach path for point closest to goal.    
    def searchProfile(self, centerIndex, goal):
        goalAngle = self.location.angle(goal)
        goalIndex = self.getScanIndex(goalAngle)
        goalDist = self.location.dist(goal)
        centerAngle = self.getScanAngle(centerIndex)
        startAngle = centerAngle - math.radians(90)
        startIndex = max(0,self.getScanIndex(startAngle))
        endAngle = centerAngle + math.radians(90)
        endIndex = min(len(self.ranges), self.getScanIndex(endAngle))
        #index of the closest obstacle        
        minIndex = None
        #scan the profile for obstacles        
        for searchIndex in range(startIndex, endIndex):
            searchAngle = (searchIndex - startIndex) * self.scanAngleIncrement
            searchDist = abs(self.radius / math.cos(searchAngle))
            # robot is facing goal            
            if centerIndex == goalIndex:
                searchDist = min(goalDist + self.radius, searchDist)
            #obstacle is in path
            if self.ranges[searchIndex] < searchDist:
                if minIndex == None:
                    minIndex = searchIndex
                elif self.ranges[searchIndex] < self.ranges[minIndex]:
                    minIndex = searchIndex
        if minIndex != None:
            obstacleDist = self.ranges[minIndex]                   
            maxTravelDist = obstacleDist - self.radius
            maxTravelPoint = self.getScanPoint(maxTravelDist, centerAngle)

            return maxTravelPoint
        else:
            return goal

    #find path to goal around obstacle
    def findPath(self, goal):
        goalAngle = self.location.angle(goal)
        goalIndex = self.getScanIndex(goalAngle) 
        startIndex = self.getScanIndex(self.yaw - math.radians(90))
        endIndex = self.getScanIndex(self.yaw + math.radians(90))
        if startIndex < goalIndex < endIndex:
            nearestPoint = None
            nearestDist = None
            for index in range(startIndex, endIndex):
                tempPoint = self.searchProfile(index, goal)
                tempDist = tempPoint.dist(goal)
                if tempPoint == goal:
                    self.goto(goal)
                    return
                elif nearestPoint == None:
                    nearestPoint = tempPoint
                    nearestDist = tempDist
                elif tempDist < nearestDist:
                    nearestPoint = tempPoint
                    nearestDist = tempDist
            self.goto(nearestPoint)
        else:
            self.rotate(goalAngle)
    
    #head towards point    
    def goto(self, goal):
        command.linear.x = 0.0
        command.angular.z = 0.0
        goalAngle = self.location.angle(goal)
        goalDist = self.location.dist(goal)        
        
        if abs(goalDist) < 0.1:
            command.linear.x = 0
            command.angular.z = 0
            pub.publish(command)
        
        elif abs(self.yaw - goalAngle) <= 0.1 and abs(goalDist) > 1:
            command.linear.x = 2.0
            pub.publish(command)
        else:
            self.rotate(goalAngle)
            
    def getScanAngle(self, index):
        return self.yaw + self.minScanAngle + index * self.scanAngleIncrement        
       
    def getScanIndex(self, angle):
        return int((angle - self.yaw - self.minScanAngle) / self.scanAngleIncrement)
    
    def __init__(self):
        self.refresh()
        
    def getScanPoint(self, radius, angle):
        return Point(self.x + radius * math.cos(angle), self.y + radius * math.sin(angle))
    
# instantiate global variables "globalOdom"
globalOdom = Odometry()

# global pi - this may come in handy
pi = math.pi

# method to control the robot
def callback(scan, odom):
    # the odometry parameter should be global
    global globalOdom
    globalOdom = odom

    # make a new twist message
    command = Twist()

    # Fill in the fields.  Field values are unspecified 
    # until they are actually assigned. The Twist message 
    # holds linear and angular velocities.
    command.linear.x = 0.0
    command.linear.y = 0.0
    command.linear.z = 0.0
    command.angular.x = 0.0
    command.angular.y = 0.0
    command.angular.z = 0.0

    # get goal x and y locations from the launch file
    goal = Point(0,0)
    goal.x = rospy.get_param('hw2/goalX',0.0)
    goal.y = rospy.get_param('hw2/goalY',0.0)
    
    # find current (x,y) position of robot based on odometry
    robot = Robot()
    #robot.ranges = scan.ranges
    robot.x = globalOdom.pose.pose.position.x
    robot.y = globalOdom.pose.pose.position.y
    robot.location = Point(robot.x, robot.y)

    # find current orientation of robot based on odometry (quaternion coordinates)
    xOr = globalOdom.pose.pose.orientation.x
    yOr = globalOdom.pose.pose.orientation.y
    zOr = globalOdom.pose.pose.orientation.z
    wOr = globalOdom.pose.pose.orientation.w

    # find orientation of robot (Euler coordinates)
    (roll, pitch, yaw) = euler_from_quaternion([xOr, yOr, zOr, wOr])

    # find currentAngle of robot (equivalent to yaw)
    # now that you have yaw, the robot's pose is completely defined by (currentX, currentY, currentAngle)
    robot.yaw = yaw

    # find laser scanner properties (min scan angle, max scan angle, scan angle increment)
    robot.maxScanAngle = scan.angle_max
    robot.minScanAngle = scan.angle_min
    robot.scanAngleIncrement = scan.angle_increment

    # find current laser angle, max scan length, distance array for all scans, and number of laser scans
    robot.ranges = scan.ranges
    
    # start making path towards goal point    
    robot.findPath(goal)
    
# main function call
if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('lab2', log_level=rospy.DEBUG, anonymous=True)
    
    # subscribe to laser scan message
    sub = message_filters.Subscriber('base_scan', LaserScan)
    rospy.Subscriber('odom', Odometry)
    # subscribe to odometry message  
    sub2 = message_filters.Subscriber('odom', Odometry)
    
    # synchronize laser scan and odometry data
    ts = message_filters.TimeSynchronizer([sub, sub2], 10)
    ts.registerCallback(callback)
    
    # publish twist message
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    # Turn control over to ROS
    rospy.spin()

