#!/usr/bin/env python

import rospy
import math
import tf
from tf.transformations import euler_from_quaternion

# The laser scan message
from sensor_msgs.msg import LaserScan

#the odometry message
from nav_msgs.msg import Odometry

# The velocity command message
from geometry_msgs.msg import Twist

class GuidanceControl:
    #global pi
    pi = math.pi

    #Class constructor
    def __init__(self):
        print "Creating GuidanceConcrol object..."
        rospy.init_node('guidance_control') # ros node housekeeping

        # Subscribers and publishers
        self.scan_sub_ = rospy.Subscriber('base_scan', LaserScan, self.scan_callback) #Subscribe to base_scan topic
        self.odom_sub_ = rospy.Subscriber('odom', Odometry, self.odom_callback) #Subscribe to odom topic
        self.cmd_vel_pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=10) #Publish to cmd_vel topic

        # Initialize member Variables
        self.scan_received = False
        self.odom_received = False
        #Robot control variables
        self.distThreshold = 2.0 #obstacle avoidance threshold
        self.scaleVel = 1.0       #magnitude of obstacle avoidance and goal seeking velocities

        #Get goal x and y locations from the launch file
        self.goalX = rospy.get_param('robot_control/goalX', 0.0)
        self.goalY = rospy.get_param('robot_control/goalY', 0.0)

    # Callback function triggered whenever a base_scan message is recieved
    def scan_callback(self, msg):
        #find laser scanner properties (min/max scan angles, scna angle increment)
        self.maxAngle = msg.angle_max
        self.minAngle = msg.angle_min
        self.angleIncrement = msg.angle_increment

        # Current laser propertiesue
        self.currentLaserTheta = self.minAngle
        self.maxScanLength = msg.range_max
        self.distanceArray = msg.ranges
        self.numScans = len(self.distanceArray)

        #acknowledge that a scan has been received and attempt to compute a new control command
        self.scan_received = True
        self.compute_cmd_vel()

    # Calback function triggered whenever an odom message is received
    def odom_callback(self, msg):
        # find current (x,y) position of robot based on odometry
        self.currentX = msg.pose.pose.position.x
        self.currentY = msg.pose.pose.position.y

        #find current orientation of robot base on odometry
        xOr = msg.pose.pose.orientation.x
        yOr = msg.pose.pose.orientation.y
        zOr = msg.pose.pose.orientation.z
        wOr = msg.pose.pose.orientation.w

        # find orientation of robot (Euler coordinates)
        (roll, pitch, yaw) = euler_from_quaternion([xOr, yOr, zOr, wOr])

        #find currentAngle of robot (equivalent to yaw), now that you have yaw, the robots pose is completely defined by (currentX, currentY, currentAngle)
        self.currentAngle = yaw

        # acknowledge that an odometry message has been eceived and attempt to comput a new control mannamd
        self.odom_received = True
        self.compute_cmd_vel()

    #member function to compute a new control command given the latest sensor scan and dometry
    def compute_cmd_vel(self):
        #only compute a new control if bother base_scan and odom have been recieved
        if self.scan_received and self.odom_received:
            #make a new twist message
            command = Twist()

            #Fill in the fields. Field calues are inspecigied until they are actualy assigned
            command.linear.x = 0.0
            command.linear.y = 0.0
            command.linear.z = 0.0
            command.angular.x = 0.0
            command.angular.y = 0.0
            command.angular.z = 0.0

            #for each laser scan
            turnLeft = False        #boolean check for left turn
            turnRight = False       #boolean check for right turn
            obsAvoidBearing = 0.0   #heading change to acoid obstacles
            obsAvoidVel = 0.0       #Velocity change to avoid obstacles

            for curScan in range(0, self.numScans):
                if self.distanceArray[curScan] < self.distThreshold:
                    if -self.pi/2.0 <= self.currentLaserTheta <= 0.0:
                        # obstacle detected on the right quadrant
                        if not turnLeft:
                            obsAvoidBearing = 1.0
                            print 'Left turn manoeuvre applied'
                            turnLeft = True

                    elif 0.0 <= self.currentLaserTheta <= self.pi/2.0:
                        # obstacle detected on the right quadrant
                        if not turnRight:
                            obsAvoidBearing += -1.0
                            print 'Right turn manoeuvre applied'
                            turnRight = True

                    if -self.pi/6.0 <= self.currentLaserTheta <= self.pi/6.0:
                        # obstacle detected in front 60 degree cone
                        if self.distanceArray[curScan]/self.distThreshold < 1.0 - obsAvoidVel:
                            obsAvoidVel= self.scaleVel*(1.0 - (self.distanceArray[curScan])/self.distThreshold)
                            print 'Slowing down'

                self.currentLaserTheta += self.angleIncrement
                
            #based on the motion you want (found using goal location, current laction and obstacle info), set the robot motion
            headingToGoal = math.atan2(self.goalY - self.currentY, self.goalX - self.currentX)
            bearing = headingToGoal - self.currentAngle
            
            #compute distance to goal position
            distToGoal = math.sqrt((self.goalY - self.currentY)**2 + (self.goalX - self.currentX)**2)
            vel = self.scaleVel
            
            if distToGoal < 5.0:
                vel *= distToGoal/5.0
                if distToGoal < 1.0:
                    #stop if you are within 1 unit of goal
                    vel = 0.0
                    print 'arrived at goal!'
                    
            # commanded velocities
            command.linear.x = 2.5*(vel - obsAvoidVel)
            command.angular.z = 1.0*(bearing + obsAvoidBearing)
            self.cmd_vel_pub_.publish(command)
            
            # reset flags so that we only comput based on new data
            self.scan_received = False
            self.odom_received = False
            
        else:
            if not self.scan_received:
                print "No scan received yet"
            if not self.odom_received:
                print "No odometry received yet"

if __name__ == '__main__':
    robot_control = GuidanceControl()
    rospy.spin()