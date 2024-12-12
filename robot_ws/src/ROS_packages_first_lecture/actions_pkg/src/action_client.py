#! /usr/bin/env python3
import sys
import rospy
import actionlib
from actionlib_tutorials.msg import FibonacciGoal, FibonacciFeedback, FibonacciResult, FibonacciAction #Imports the action msgs

def feedback_callback(feedback):
	#This function is called when the server sends feedback
	print("Feedback: " + str(feedback.sequence[-1]))
	
rospy.init_node('fibonacci_action_client')
rate=rospy.Rate(1) #1Hz
client=actionlib.SimpleActionClient('/fibonacci_as', FibonacciAction) #Stablishes the connection with the server
client.wait_for_server() #Waits until the action server is available

goal=FibonacciGoal()
goal.order=15
client.send_goal(goal, feedback_cb=feedback_callback) #Calls the action action sending the goal and defining the feedback callback function

state_result = client.get_state() #0 --> PENDING, 1 --> ACTIVE, >=2 --> FINISHED
while state_result < 2:
	#The node keeps executing other stuff while the action is executed.
	rospy.loginfo("Doing some stuff while waiting for the server to give a result...")
	rate.sleep()
	state_result = client.get_state()

print("Action finished")
