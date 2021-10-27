#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt, pi
import rosbag
import std_msgs.msg
from nav_msgs.msg import Odometry
import tf

class TurtleBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        # Publisher which will publish to the topic '/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/cmd_vel',
                                                  Twist, queue_size=10)

        # Publisher which will publish to the topic '/turtle1/error'.
        self.error_publisher = rospy.Publisher('/error',
                                                  Pose, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose.x = round(data.pose.pose.position.x, 4)
        self.pose.y = round(data.pose.pose.position.y, 4)
        (r, p, y) = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x,
         data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
        self.pose.theta = y

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        vel = constant * self.euclidean_distance(goal_pose)
        if vel > 1:
            vel = 1
        return vel

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def turnShortestWay(self, goal_pose, vel_msg, constant=6):
        qerr = goal_pose.theta-self.pose.theta
        qvel_z = vel_msg.angular.z
        if(qerr > pi):
            qvel_z = constant*(qerr-2*pi)
        elif(qerr < -pi):
            qvel_z = constant*(2*pi+qerr)
        return qvel_z

    def doCircle(self, radius=1, linear_vel=1):
        # duration 1 sec
        vel_msg = Twist()
        goal_pose = Pose()
        goal_pose = self.pose
        atStart = True
        startAngle = self.pose.theta
        while abs(startAngle-self.pose.theta) > 0.05 or atStart:
            # Linear velocity in the x-axis.
            vel_msg.linear.x = linear_vel
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = linear_vel/radius

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            # Publish at the desired rate.
            self.rate.sleep()
            if(atStart and abs(startAngle-self.pose.theta) > 0.05):
                atStart = False
            print(self.pose.theta)

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # Display info message
        rospy.loginfo("Circle complete")

        # If we press control + C, the node will stop.
        #rospy.spin()

    # Function to follow a predefined path (from rosbag)
    def followpath(self):

        # Initialize message types
        goal_pose = Pose()
        vel_msg = Twist()
        error_msg = Pose()

        # Read rosbag and go through all the points one by one
        bag = rosbag.Bag('posegaze.bag')
        counter = 0

        for topic, msg, t in bag.read_messages(topics=['/odom']):

            if counter < 50:
                counter += 1
                continue
            else:
                counter = 0

            goal_pose.x = msg.pose.pose.position.x
            goal_pose.y = msg.pose.pose.position.y
            (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
            goal_pose.theta = y

            # Display info message
            rospy.loginfo("Received target: " + str(goal_pose.x) + ", " + str(goal_pose.y))

            # Main loop, move turtle to point
            while self.euclidean_distance(goal_pose) >= 0.15:

                # Linear velocity in the x-axis.
                vel_msg.linear.x = self.linear_vel(goal_pose)
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0

                # Angular velocity in the z-axis.
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = self.angular_vel(goal_pose)
                vel_msg.angular.z = self.turnShortestWay(goal_pose,vel_msg,1)
                # Publishing our vel_msg
                self.velocity_publisher.publish(vel_msg)

                # Publish at the desired rate.
                rospy.Rate(200).sleep()


            self.velocity_publisher.publish(vel_msg)

            d_distance = self.euclidean_distance(goal_pose)
            d_theta = goal_pose.theta - self.pose.theta
            #print("Error in distance: ", d_distance, ", error in angle: ", d_theta)
            #print(goal_pose.theta, self.pose.theta)

            # Publishing error
            error_msg.x = goal_pose.x - self.pose.x
            error_msg.y = goal_pose.y - self.pose.y
            error_msg.theta = goal_pose.theta - self.pose.theta
            self.error_publisher.publish(error_msg)

            # Display info message
            rospy.loginfo("Reached goal: " + str(self.pose.x) + ", " + str(self.pose.y))

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0

        bag.close()

    def move2goal(self):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        # Get the input from the user.
        goal_pose.x = float(input("Set your x goal: "))
        goal_pose.y = float(input("Set your y goal: "))
        goal_pose.theta = float(input("Set your theta goal: "))

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = float(input("Set your tolerance: "))

        vel_msg = Twist()

        # Display info message
        rospy.loginfo("Received target: " + str(goal_pose.x) + ", " + str(goal_pose.y))

        while self.euclidean_distance(goal_pose) >= distance_tolerance:

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            print("linear "+str(vel_msg.linear.x))

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)
            print("angular "+str(vel_msg.angular.z))

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        while abs(self.pose.theta - goal_pose.theta) >= distance_tolerance:

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = (goal_pose.theta - self.pose.theta)
            vel_msg.angular.z = self.turnShortestWay(goal_pose,vel_msg,1)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()
        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # Display info message
        rospy.loginfo("Reached goal: " + str(self.pose.x) + ", " + str(self.pose.y))

        # If we press control + C, the node will stop.
        #rospy.spin()

if __name__ == '__main__':
    try:
        x = TurtleBot()
        mode = ""
        while mode != "e":
            print(" 'p' for predefined path\n 'g' for moving to the certain goal\n 'c' for doing a circle\n 'e' for exit")
            mode = input("command: ")
            if(mode == "p"):
                x.followpath()
            elif(mode == "g"):
                x.move2goal()
            elif(mode == "c"):
                x.doCircle(1,0.5)
    except rospy.ROSInterruptException:
        pass
