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
    scanAngleIncrement = None
    ranges = None
    minScanAngle = None
    maxScanAngle = None
    
        
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
        
    def rotate(self, theta):
        
        
        speed = (self.yaw - theta)/2
        speed = 0.1
            
        #print 'robot angle: {0}'.format(math.degrees(self.yaw))
        #print 'angle diff: {0}'.format(math.degrees(self.yaw - theta))
        
        if abs(self.yaw - theta) < .01:
            command.angular.z = 0
        elif self.yaw - theta > 0:
            command.angular.z = -abs(speed)
        elif self.yaw - theta < 0:
            command.angular.z = abs(speed)
        pub.publish(command)
        #rospy.spin()
        #time.sleep(1)
    
        
    def searchProfile(self, centerIndex, goal):
        goalAngle = self.location.angle(goal)
        goalIndex = self.getScanIndex(goalAngle)
        goalDist = self.location.dist(goal)
        centerAngle = self.getScanAngle(centerIndex)
        startAngle = centerAngle - math.radians(90)
        startIndex = max(0,self.getScanIndex(startAngle))
        endAngle = centerAngle + math.radians(90)
        endIndex = min(len(self.ranges), self.getScanIndex(endAngle))
        minIndex = None
        #print ': {0}'.format()
        #print 'pose angle: {0}'.format(self.yaw*180/pi)
        #print 'center angle: {0}'.format(self.getScanAngle(centerIndex)*180/pi)
        #print 'start angle: {0}'.format(math.degrees(startAngle))
        #print 'end angle: {0}'.format(math.degrees(endAngle))
        #search linear path in the direction of index for the closest obstacle
        for searchIndex in range(startIndex, endIndex):
            searchAngle = (searchIndex - startIndex) * self.scanAngleIncrement
            searchDist = abs(self.radius / math.cos(searchAngle))
            #print 'searchIndex: {0}'.format(searchIndex)
            #print 'searchdist: {0}'.format(searchDist)
            #print 'scan range: {0}'.format(self.ranges[searchIndex])
            if centerIndex == goalIndex:
                searchDist = min(goalDist + self.radius, searchDist)
            if self.ranges[searchIndex] < searchDist:
                if minIndex == None:
                    minIndex = searchIndex
                elif self.ranges[searchIndex] < self.ranges[minIndex]:
                    minIndex = searchIndex
                    #print 'minIndex: {0}'.format(minIndex)
        if minIndex != None:
            obstacleDist = self.ranges[minIndex]        
            theta = (centerIndex - minIndex) * self.scanAngleIncrement
            #while theta < 0:
            #    theta += 2 * pi
            #while theta > 2 * pi:
            #    theta -= 2 * pi
            #print 'obstacle range: {0}'.format(obstacleDist)
            #print 'theta: {0}'.format(theta)
            #if theta > math.radians(90):
            #    rad = self.radius
            #else:
            #    rad = -self.radius
#            rad = self.radius            
#            if theta != 0:
#                maxTravelDist = rad * math.sin(pi - theta - math.asin(obstacleDist * math.sin(theta) / rad)) / math.sin(theta)       
#            else:
#                maxTravelDist = obstacleDist - rad
            maxTravelDist = obstacleDist - self.radius
            maxTravelPoint = self.getScanPoint(maxTravelDist, centerAngle)
            
            #print 'obstacle index: {0}'.format(minIndex)
            #print 'obstacle range: {0}'.format(obstacleDist)
            #print 'travel distance: {0}'.format(maxTravelDist)
            return maxTravelPoint
        else:
            return goal
        #input()

    
    def findPath(self, goal):
        goalAngle = self.location.angle(goal)
        goalIndex = self.getScanIndex(goalAngle) 
        startIndex = self.getScanIndex(self.yaw - math.radians(90))
        endIndex = self.getScanIndex(self.yaw + math.radians(90))
        goalDist = self.location.dist(goal)
        print 'goalAngle: {0}'.format(math.degrees(goalAngle))
        print 'goalDist: {0}'.format(goalDist)
        #print 'yaw: {0}'.format(math.degrees(self.yaw))
        print 'robot x: {0}'.format(self.location.x)
        print 'robot y: {0}'.format(self.location.y)
        print 'goal x: {0}'.format(goal.x)
        print 'goal y: {0}'.format(goal.y)
        #raw_input()        
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
        command.linear.x = 0.0
        command.angular.z = 0.0
        goalAngle = self.location.angle(goal)
        goalDist = self.location.dist(goal)        
        
        #input()
        
        
        
        if abs(self.yaw - goalAngle) <= 0.1 and abs(goalDist) > 1:
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
def callback(scan,odom):
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
#    print 'xOr: {0}'.format(xOr)
#    print 'yOr: {0}'.format(yOr)
#    print 'zOr: {0}'.format(zOr)
#    print 'wOr: {0}'.format(wOr)
    print 'yaw: {0}'.format(math.degrees(yaw))
    raw_input()
    # find laser scanner properties (min scan angle, max scan angle, scan angle increment)
    robot.maxScanAngle = scan.angle_max
    robot.minScanAngle = scan.angle_min
    robot.scanAngleIncrement = scan.angle_increment

    # find current laser angle, max scan length, distance array for all scans, and number of laser scans
    currentLaserTheta = robot.minScanAngle
    maxScanLength = scan.range_max 
    robot.ranges = scan.ranges
    numScans = len(robot.ranges)
   
	
    
    # based on the motion you want (found using goal location,
    # current location, and obstacle info), set the robot
    # motion, e.g.:
    # command.linear.x = 0.0
    # command.angular.z = 0.0

    rospy.wait_for_message('/odom', Odometry)    
    robot.findPath(goal)
    #print 'angle: {0}'.format(robot.yaw * 180 / pi)
        
    

# main function call
if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('lab2', log_level=rospy.DEBUG, anonymous=True)
        
    # subscribe to laser scan message
    sub = message_filters.Subscriber('base_scan', LaserScan)
    
    # subscribe to odometry message
    rospy.Subscriber('odom', Odometry)    
    sub2 = message_filters.Subscriber('odom', Odometry)
    
    #rospy.Subscriber()

    # synchronize laser scan and odometry data
    ts = message_filters.TimeSynchronizer([sub, sub2], 1)
    ts.registerCallback(callback)

    # publish twist message
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    
    # Turn control over to ROS
    rospy.spin()

