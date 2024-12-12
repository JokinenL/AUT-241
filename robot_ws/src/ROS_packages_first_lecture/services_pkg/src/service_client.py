#! /usr/bin/env python3
import sys
import rospy
from services_pkg.srv import *

rospy.init_node('example_service_client_node', anonymous=True)
service_name = '/exercises_RMCP/example_service'
req_number = rospy.get_param('~number', "")

rospy.wait_for_service(service_name)
my_service = rospy.ServiceProxy(service_name, ServiceData)
servReq = ServiceDataRequest()
servReq.number = req_number
servResult = my_service(servReq)
if servResult.success:
	print(str(servReq.number) + " is an " + servResult.info + " number")
else:
	print("ERROR")


