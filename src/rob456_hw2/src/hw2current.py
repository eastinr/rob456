#!/usr/bin/env python

import rospy
import numpy as np
import math
from tf.transformations import euler_from_quaternion 
import message_filters

# The laser scan message
from sensor_msgs.msg import LaserScan

# The odometry message
from nav_msgs.msg import Odometry
    

# the velocity command message
from geometry_msgs.msg import Twist

command = Twist()

class Point:
    
    def __init__(self, x, y):
        self.x = x
        self.y = y
        
    def dist(self, goal):
        return math.sqrt((goal.x - self.x)**2 + (goal.y - self.y)**2)
        
    def angle(self, goal):
        return math.atan2(goal.y-self.y, goal.x-self.x)

class Robot:

    radius = 1
    points = []
    
    def __init__(self, scan, odom):
        # find current (x,y) position of robot based on odometry        
        xloc = odom.pose.pose.position.x
        yloc = odom.pose.pose.position.y
        self.location = Point(xloc, yloc)
        
        # find current orientation of robot based on odometry (quaternion coordinates)
        xOr = globalOdom.pose.pose.orientation.x
        yOr = globalOdom.pose.pose.orientation.y
        zOr = globalOdom.pose.pose.orientation.z
        wOr = globalOdom.pose.pose.orientation.w
        
        # find orientation of robot (Euler coordinates)
        (roll, pitch, yaw) = euler_from_quaternion([xOr, yOr, zOr, wOr])
        self.yaw = yaw
        
        # find laser scanner properties (min scan angle, max scan angle, scan angle increment)
        self.maxScanAngle = scan.angle_max
        self.minScanAngle = scan.angle_min
        self.scanAngleIncrement = scan.angle_increment
        
        # find current laser distance array for all scans
        self.ranges = scan.ranges
        self.scanLength = len(self.ranges)
        
        # populate arrays corresponding to scan angles and global points
        self.angles = np.zeros(self.scanLength)
        for index in range(0, self.scanLength):
            self.angles[index] = self.getScanAngle(index)
            tempx = self.location.x + self.ranges[index] * math.cos(self.angles[index]) 
            tempy = self.location.y + self.ranges[index] * math.sin(self.angles[index])            
            self.points.append(Point(tempx, tempy))   
    
        
    def getGoalProps(self, goal):
        Angle = self.location.angle(goal)
        Dist = self.location.dist(goal)
        Index = self.getScanIndex(Angle)
        DistArray = []
        for index in range(0, self.scanLength):
            DistArray.append(self.points[index].dist(goal))
        return [Angle, Dist, Index]
        
        
    
    def rotate(self, theta):
        
        dtheta = self.yaw - theta
        command.linear.x = 0.0
        speed = 0.1
        if dtheta/5 > speed:
            speed = dtheta/5
        print 'robot angle: {0}'.format(math.degrees(self.yaw))
        #print 'theta: {0}'.format(math.degrees(theta))
        #print 'angle diff: {0}'.format(math.degrees(self.yaw - theta))    
        if abs(dtheta) < 0.01:
            command.angular.z = 0
        elif dtheta > 0:
            command.angular.z = -abs(speed)
        elif dtheta < 0:
            command.angular.z = abs(speed)
        if abs(dtheta) > 0.01:
            pub.publish(command)
        else:
            command.angular.z = 0
            pub.publish(command)        
    def searchProfile(self, centerIndex, goal):
        [goalAngle, goalDist, goalIndex] = self.getGoalProps(goal)
        centerAngle = self.getScanAngle(centerIndex)
        startAngle = centerAngle - math.radians(90)
        startIndex = max(0,self.getScanIndex(startAngle))
        endAngle = centerAngle + math.radians(90)
        endIndex = min(len(self.ranges), self.getScanIndex(endAngle))
        minIndex = None

        for searchIndex in range(startIndex, endIndex):
            searchAngle = (searchIndex - startIndex) * self.scanAngleIncrement
            searchDist = abs(self.radius / math.cos(searchAngle))

            if centerIndex == goalIndex:
                searchDist = min(goalDist + self.radius, searchDist)
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
            
    
    def findPath(self, goal):
        [goalAngle, goalDist, goalIndex] = self.getGoalProps(goal) 
        startIndex = self.getScanIndex(self.yaw - math.radians(90))
        endIndex = self.getScanIndex(self.yaw + math.radians(90))
        
#        if goal == self.searchProfile(goalIndex, goal):
#            self.goto(goal)
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
        
    def goto(self, goal):
        [goalAngle, goalDist, goalIndex] = self.getGoalProps(goal)         
        command.linear.x = 0.0
        command.angular.z = 0.0
        print 'goalAngle: {0}'.format(math.degrees(goalAngle))
        print 'robotAngle: {0}'.format(math.degrees(self.yaw))
        if abs(goalDist) < 0.1:
            command.linear.x = 0
            command.angular.z = 0
            pub.publish(command)
        elif abs(self.yaw - goalAngle) <= 0.1:
            command.linear.x = max(goalDist, 1.0)
            pub.publish(command)
        else:
            self.rotate(goalAngle)
            
        
    def getScanAngle(self, index):
        return self.yaw + self.minScanAngle + index * self.scanAngleIncrement        
       
    def getScanIndex(self, angle):
        return int((angle - self.yaw - self.minScanAngle) / self.scanAngleIncrement)
    
        
    def getScanPoint(self, radius, angle):
        return Point(self.location.x + radius * math.cos(angle), self.location.y + radius * math.sin(angle))
    
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
    
    # Initialize the Robot Class
    robot = Robot(scan, odom)
    
    robot.findPath(goal) 
    oldyaw = robot.yaw
# main function call
if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('lab2', log_level=rospy.DEBUG, anonymous=True)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        
        # subscribe to laser scan message
        sub = message_filters.Subscriber('base_scan', LaserScan)
    
        # subscribe to odometry message    
        sub2 = message_filters.Subscriber('odom', Odometry)
        
        #rospy.Subscriber()
    
        # synchronize laser scan and odometry data
        ts = message_filters.TimeSynchronizer([sub, sub2], 10)
        ts.registerCallback(callback)
        
        
    
        # publish twist message
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rate.sleep()
    
    # Turn control over to ROS
    rospy.spin()

