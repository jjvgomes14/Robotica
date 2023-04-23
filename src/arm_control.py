#!/usr/bin/env python3

import rospy
import time
import actionlib

from math import pi
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from franka_gripper.msg import MoveAction, MoveGoal
from franka_gripper.msg import GraspAction, GraspGoal

class arm_control():
 
    def __init__(self):
        self.time_to_wait = 5.0
        self.pub = rospy.Publisher("/cartesian_impedance_example_controller/equilibrium_pose", PoseStamped, queue_size=50)
        self.action_client_open = actionlib.SimpleActionClient("/franka_gripper/move", MoveAction)
        self.action_client_close = actionlib.SimpleActionClient("/franka_gripper/grasp", GraspAction)
        self.action_client_open.wait_for_server()
        self.action_client_close.wait_for_server()

        self.base  = [-0.1000,  0.0, 0.615];
        self.home  = [ 0.4000,  0.0, 0.615];
        self.blue  = [-0.4652, -0.5, 0.615];
        self.half  = [-0.4652,  0.0, 0.615];
        self.green = [-0.4652,  0.5, 0.615];

# gCan1 greenCan  -0.1067 0.3634 0.69  0.00  -0.01  0.00
# gCan2 greenCan  0.2191 -0.0220 0.5548  1.57  -1.55  0.01
# gCan3 greenCan  -0.0809 -0.6600 0.5858  0.00  -0.00  -1.73
# gCan4 greenCan  0.3690 -0.5296 0.6768  0.00  -0.00  -2.99

# rCan1 redCan  0.295 0.5031 0.5857  3.14  -0.00  3.00
# rCan2 redCan  -0.0809 -0.6600 0.740  3.14  -0.00  0.00
# rCan3 redCan  0.70 0.0322 0.5857  3.14  -0.00  0.66

# yCan1 yellowCan  0.60 0.1722 0.5856  3.14  -0.00  1.30
# yCan2 yellowCan  0.55 -0.4821 0.5718  -1.57  0.10  -0.83
# yCan3 yellowCan  0.36 -0.45 0.61  0.00  -0.00  -2.95
# yCan4 yellowCan  0.26 -0.42 0.61  -1.57  -1.09  2.18

# rBottle1 redBottle  0.22 0.62 0.556  1.57  -0.42  1.53
# rBottle2 redBottle  0.820 0.00 0.5558  1.57  -0.10  0.00

# bBottle1 blueBottle  -0.17 -0.46 0.6339  0.00  -0.00  -0.00
# bBottle2 blueBottle  0.0734 0.2300 0.6338  0.00  -0.00  0.00
# bBottle3 blueBottle  0.2630 -0.2251 0.5554  1.61  1.56  1.61

# yBottle1 yellowBottle  -0.1430 0.5264 0.5558  1.57  1.47  1.58
# yBottle2 yellowBottle  0.680 -0.1600 0.6093  0.00  -0.00  0.00
# yBottle3 yellowBottle  0.3064 0.1509 0.6339  0.00  0.00  -0.01
# yBottle4 yellowBottle  0.2430 -0.6251 0.7558  1.57  1.47  1.58

        destinations = [
            ['bBottle1' ,'blueBottle' ,'blue', -0.1700, -0.4600, 0.6339],
            ['rCan2'    ,'redCan'     ,'blue', -0.0809, -0.6600, 0.7400],
            ['gCan1'    ,'greenCan'   ,'blue', -0.1067,  0.3634, 0.6900],
            ['rCan1'    ,'redCan'     ,'blue',  0.2950,  0.5031, 0.5857],
        ]

        rospy.loginfo(f'Going to home.')
        self.goto(self.home[0],self.home[1],self.home[2])       
        self.wait(self.time_to_wait)  

        for d in destinations:

            rospy.loginfo(f'Going to {d[0]} head.')
            self.goto(d[3],d[4],d[5]+0.1)       
            self.wait(self.time_to_wait)  

            rospy.loginfo(f'Open gripper')
            self.gripper_open()     
            self.wait(self.time_to_wait)  

            rospy.loginfo(f'Going to {d[0]} pose.')
            self.goto(d[3],d[4],d[5])       
            self.wait(self.time_to_wait)  

            rospy.loginfo(f'Close gripper')
            self.gripper_close()
            self.wait(self.time_to_wait)  

            rospy.loginfo(f'Going to half.')
            self.goto(self.half[0],self.half[1],self.half[2])       
            self.wait(self.time_to_wait)  

            if(d[2] == 'blue'):
                rospy.loginfo(f'Going to blue.')
                self.goto(self.blue[0],self.blue[1],self.blue[2])       
                self.wait(self.time_to_wait)  
            if(d[2] == 'green'):
                rospy.loginfo(f'Going to green.')
                self.goto(self.green[0],self.green[1],self.green[2])       
                self.wait(self.time_to_wait)  

            rospy.loginfo(f'Open gripper')
            self.gripper_open()
            self.wait(self.time_to_wait)  

            rospy.loginfo(f'Going to half.')
            self.goto(self.half[0],self.half[1],self.half[2])       
            self.wait(self.time_to_wait)  

        rospy.loginfo(f'Going to home.')
        self.goto(self.home[0],self.home[1],self.home[2])       
        self.wait(self.time_to_wait)  

    def goto(self, x, y, z):

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
        goal = MoveGoal
        goal.width = 0.08
        goal.speed = 0.2
        self.action_client_open.send_goal(goal)

    def gripper_close(self):
        goal = GraspGoal
        goal.width = 0.04
        goal.speed = 0.2
        goal.force = 0.5
        goal.epsilon.inner = 0.02
        goal.epsilon.outer = 0.02
        self.action_client_close.send_goal(goal)

    def wait(self, max_seconds):
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