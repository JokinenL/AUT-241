#! /usr/bin/env python3
import sys
import rospy
from services_pkg.srv import *

rospy.init_node('example_service_server_node', anonymous=True)
service_name = '/exercises_RMCP/example_service'

def service_callback(req): 
	"""
	Service that tells if a number is even or odd
	"""
	print("Received a request for number: " + str(req.number))
	resp = ServiceDataResponse()
	try:
		if req.number % 2 == 0:
			resp.info = 'even'
		else:
			resp.info = 'odd'
		resp.success = True
		return resp
	except:
		resp.success = False
		return resp	


rospy.Service(service_name, ServiceData, service_callback)


# Keep this process running until Enter is pressed
print ("Press Enter to quit...")
try:
	sys.stdin.readline()
except KeyboardInterrupt:
	print("End of test")


