#!/usr/bin/env python3

import rospy
import time
import actionlib
import cv2
from cv_bridge import CvBridge, CvBridgeError

from math import pi
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from franka_gripper.msg import MoveAction, MoveGoal
from franka_gripper.msg import GraspAction, GraspGoal
from sensor_msgs.msg import Image

class arm_control():
 
    def __init__(self):
        self.time_to_wait = 5.0

        self.bridge = CvBridge()
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback);

        self.pub = rospy.Publisher("/cartesian_impedance_example_controller/equilibrium_pose", PoseStamped, queue_size=50)
        self.action_client_open = actionlib.SimpleActionClient("/franka_gripper/move", MoveAction)
        self.action_client_close = actionlib.SimpleActionClient("/franka_gripper/grasp", GraspAction)
        self.action_client_open.wait_for_server()
        self.action_client_close.wait_for_server()

        self.base  = [-0.1000,  0.0, 0.615];
        self.home  = [ 0.1000,  0.0, 1.000];
        self.blue  = [-0.4652, -0.5, 0.800];
        self.green = [-0.4652,  0.5, 0.800];
        self.auxblue  = [-0.1000, -0.5, 0.800];
        self.auxgreen = [-0.1000,  0.5, 0.800];

        destinations = [
            ['bBottle1' ,'blue' , 'bottle' ,'blue', -0.1700, -0.4600, 0.6135],
            # ['rCan2'    ,'red'  , 'can'    ,'blue', -0.0774, -0.6800, 0.6901],
            # ['gCan3'    ,'green', 'can'    ,'blue', -0.0771, -0.6800, 0.5734],
            ['gCan1'    ,'green', 'can'    ,'blue', -0.1067,  0.3634, 0.6900],
            # ['rCan1'    ,'red'  , 'can'    ,'blue',  0.2950,  0.5031, 0.5857],
        ]

        rospy.loginfo(f'Going to home.')
        self.goto(self.home[0],self.home[1],self.home[2])       
        self.wait(self.time_to_wait)  

        for d in destinations:

            shift1 = 0.15 # em cima do objeto

            if(d[2]=='bottle'):
                shift2 = 0.06  # no ponto de grasp
            if(d[2]=='can'):
                shift2 = 0.02  # no ponto de grasp


            rospy.loginfo(f'Going to {d[0]} head.')
            self.goto(d[4],d[5],d[6]+shift1)       
            self.wait(self.time_to_wait)  

            rospy.loginfo(f'Open gripper')
            self.gripper_open()     
            self.wait(self.time_to_wait)  

            rospy.loginfo(f'Going to {d[0]} pose.')
            self.goto(d[4],d[5],d[6]+shift2)       
            self.wait(self.time_to_wait)  

            rospy.loginfo(f'Close gripper')
            self.gripper_close()
            self.wait(self.time_to_wait)  

            rospy.loginfo(f'Going to {d[0]} head.')
            self.goto(d[4],d[5],d[6]+shift1)       
            self.wait(self.time_to_wait)  

            if(d[3] == 'blue'):
                rospy.loginfo(f'Going to blue.')
                self.goto(self.blue[0],self.blue[1],self.blue[2])       
                self.wait(self.time_to_wait)  
            if(d[3] == 'green'):
                rospy.loginfo(f'Going to green.')
                self.goto(self.green[0],self.green[1],self.green[2])       
                self.wait(self.time_to_wait)  

            rospy.loginfo(f'Open gripper')
            self.gripper_open()
            self.wait(self.time_to_wait)  

            if(d[3] == 'blue'):
                rospy.loginfo(f'Going to aux blue.')
                self.goto(self.auxblue[0],self.auxblue[1],self.auxblue[2])       
                self.wait(self.time_to_wait)  
            if(d[3] == 'green'):
                rospy.loginfo(f'Going to aux green.')
                self.goto(self.auxgreen[0],self.auxgreen[1],self.auxgreen[2])       
                self.wait(self.time_to_wait)  

            rospy.loginfo(f'Going to home.')
            self.goto(self.home[0],self.home[1],self.home[2])       
            self.wait(self.time_to_wait)  


    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            cv2.circle(cv_image, (50,50), 10, 255)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

    def goto(self, x, y, z):
        rospy.loginfo(f'goto')

        quaternion = quaternion_from_euler(-pi,0,0)
        ps = PoseStamped()
        ps.pose.position.x = x - self.base[0]
        ps.pose.position.y = y - self.base[1]
        ps.pose.position.z = z - self.base[2]
        ps.pose.orientation.x = quaternion[0]
        ps.pose.orientation.y = quaternion[1]
        ps.pose.orientation.z = quaternion[2]
        ps.pose.orientation.w = quaternion[3]
        self.pub.publish(ps)

    def gripper_open(self):
        goal = MoveGoal()
        goal.width = 0.08
        goal.speed = 0.2
        self.action_client_open.send_goal(goal)

    def gripper_close(self):
        goal = GraspGoal()
        goal.width = 0.04
        goal.speed = 0.01
        goal.force = 20
        goal.epsilon.inner = 0.05
        goal.epsilon.outer = 0.05
        self.action_client_close.send_goal(goal)
        self.action_client_close.wait_for_result()

    def wait(self, max_seconds):
        # rospy.sleep(max_seconds)

        start = time.time()
        count = 0
        while count < max_seconds:
            count = time.time() - start            

if __name__ == '__main__':
    rospy.init_node('arm_control', log_level=rospy.INFO)
    rospy.loginfo('Starting arm_control node')
    arm_control()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass