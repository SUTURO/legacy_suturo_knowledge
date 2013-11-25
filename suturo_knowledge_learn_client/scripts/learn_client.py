#!/usr/bin/env python
import roslib; roslib.load_manifest('suturo_knowledge_learn_client')
import rospy
from std_msgs.msg import String
import json_prolog
from suturo_perception_msgs.srv import *
import sys 

def get_object_from_perception():
	print("Im waiting for the service now")
	rospy.wait_for_service('GetClusters')
	print("I'm configurating the service method now.")
	get_value = rospy.ServiceProxy('GetClusters', GetClusters)
	print("I'm sending the 'get' to the Perception.")	
	percepted = get_value("get")
	return percepted

if __name__ == "__main__":
	inp = raw_input("Press Enter to get new Object from Perception: ")
	print("Im getting stuff from the Perception")
	try:
		percepted = get_object_from_perception()
		print()
		if len(percepted.perceivedObjs) > 1:
			print(percepted)
			index = ""
			while not (index.isdigit() and int(index) < len(percepted.perceivedObjs)):  
				index = raw_input("Choose the Index of the object to add: \n")
			index = int(index)
		volume = percepted.perceivedObjs[index].c_volume
		id = percepted.perceivedObjs[index].c_id
		centroid = {}
		centroid['x'] = percepted.perceivedObjs[index].c_centroid.x
		centroid['y'] = percepted.perceivedObjs[index].c_centroid.y
		centroid['z'] = percepted.perceivedObjs[index].c_centroid.z
	except KeyboardInterrupt:
		print("You interrupted waiting for the perception")
		sys.exit()
	label = raw_input("Enter a label for the choosen object: \n")
	edible = raw_input("Is this object edible? Enter yes or no: \n")
	while edible != "yes" and edible != "no":
		print("You have to enter 'yes' or 'no'")
		edible = raw_input("Is this object edible? Enter yes or no: \n")
	raw_input("Do you really want to add the object with the label %s,\n\
	 the volume: %f \n\
	 which is edible: %s \n\
	 and has the centroid: %s \n\
	 with the id: %d \n\
	 (Enter 'yes' or 'no'): \n" %(label, volume, edible, str(centroid), id))
