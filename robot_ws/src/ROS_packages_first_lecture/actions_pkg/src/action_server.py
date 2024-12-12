#! /usr/bin/env python3
import sys
import rospy
import actionlib
from actionlib_tutorials.msg import FibonacciFeedback, FibonacciResult, FibonacciAction #Imports the action msgs

class FibonacciClass(object):
	_feedback=FibonacciFeedback() #Class attributes
	_result=FibonacciResult()

	def __init__(self):
		self._as=actionlib.SimpleActionServer("fibonacci_as", FibonacciAction, self.goal_callback, False) #Defines the action server: Name of the server, type and callback function
		self._as.start() #Starts the action server, so it can be called from the client

	def goal_callback(self, goal):
		#This is the function called when the server receives a call from the client, and it receives the info of the goal topic, sent from the client
		r=rospy.Rate(1) #1Hz
		success=True
		self._feedback.sequence=[] #FibonacciFeedback() msg is composed by the field 'sequence', that is an array of integers, and it is initialized with its two first values
		self._feedback.sequence.append(0)
		self._feedback.sequence.append(1)
		rospy.loginfo('"Fibonacci_as": Executing, creating Fibonacci sequence of order ' + str(goal.order) + ' with seeds ' + str(self._feedback.sequence[0]) + ', ' + str(self._feedback.sequence[1]))
		fibonacciOrder=goal.order #Reads the goal sent by the client, it contains the order required for the fibonacci sequence
		for i in range(1, fibonacciOrder):
			if self._as.is_preempt_requested(): #If the goal is cancelled or preempted from the client
				rospy.loginfo('The goal has been cancelled/preempted')
				self._as.set_preemted() #Cancels the action
				success=False
				break #Exits the for loop and the action finishes
			self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
			self._as.publish_feedback(self._feedback) #Publishes the feedback. Everytime this happens the client calls its callback_feedback function
			r.sleep()
		if success:
			self._result.sequence= self._feedback.sequence
			rospy.loginfo('Succeeded calculating the Fibonacci of order %i' % fibonacciOrder)
			self._as.set_succeeded(self._result) #Set the action as succeded and sent the result to the client

if __name__ == '__main__':
	#Initializes the node and the action server. Then the node is kept alive waiting for calls from the clients
	rospy.init_node('fibonacci_action_server')
	FibonacciClass()
	rospy.spin()
