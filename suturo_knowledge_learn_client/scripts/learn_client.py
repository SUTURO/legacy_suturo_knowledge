#!/usr/bin/env python
import roslib; roslib.load_manifest('suturo_knowledge_learn_client')
import rospy
from std_msgs.msg import String
import json_prolog
from suturo_perception_msgs.srv import *
import sys 
###############################################
# This Client provides learning new Objects in an   
# easy to use way.
# You yust has to start it and follow the instructions
# which are given.
###############################################
#shape is a List of shapes that are given by the perception this list ist only for readability in this script.
shape = ["None", "Box", "Cylinder", "Sphere "]

##############################################
# get_object_from_perception() is a method wich will return the actual percepted object
# Therefore its not important if it is an bagfile or live-data. All to do is to start the perception service in the wished way.7
# @ return percepted PerceivedObject defined by the msg-file of the Perception module
###############################################
def get_object_from_perception():
    print("Im waiting for the service now")
    rospy.wait_for_service('GetClusters')
    print("I'm configurating the service method now.")
    get_value = rospy.ServiceProxy('GetClusters', GetClusters)
    print("I'm sending the 'get' to the Perception.")   
    percepted = get_value("get")
    return percepted

###############################################
# send_object_to_prolog(id, label, volume, variance, shape, centroid, edible) is a method that will take all important data
# which are given so far by the perception. It will take these parameters and give it to the json_prolog module wich adds the object to the 
# reasoning database
# @ param id The id which is given by the perception
# @ param label A Label, or name which can be choosen and will be saved
# @ param volume The Volume of the Object calculated by the Perception-Module
# @ param variance The Variance in Percent, which can be choosen individually
# @ param shape An Integer which is defined by the Perception msg PerceivedObject
# @ param centroid A Dictionary with a x, y and z coordinate in it
# @ param edible A boolean wether the Object is edible or not
###############################################
def send_object_to_prolog(id, label, volume, variance, shape, centroid, edible):
    rospy.init_node('json_prolog')
    prolog = json_prolog.Prolog()
    query = prolog.query("set_food(id, label, volume, variance, shape, edible)")
    for solution in query.solutions():
        print 'Found solution. %s' % (solution)
    query.finish()

###############################################
# the Main Procedure which will be started by calling the script
# It will guide you through the operation of learning
###############################################
if __name__ == "__main__":
    inp = raw_input("Press Enter to get new Object from Perception: ")
    print("Im getting stuff from the Perception")
    # Trying to get a perceived Object and saving important parameters.
    try:
        percepted = get_object_from_perception()
        print(percepted)
        if len(percepted.perceivedObjs) > 1:
            #print(percepted)
            index = ""
            while not (index.isdigit() and int(index) < len(percepted.perceivedObjs)):  
                index = raw_input("Choose the Index of the object to add: \n")
            index = int(index)
        else: 
            index = 0
        volume = percepted.perceivedObjs[index].c_volume
        id = percepted.perceivedObjs[index].c_id
        centroid = {}
        centroid['x'] = percepted.perceivedObjs[index].c_centroid.x
        centroid['y'] = percepted.perceivedObjs[index].c_centroid.y
        centroid['z'] = percepted.perceivedObjs[index].c_centroid.z
        shape_num = percepted.perceivedObjs[index].c_shape
    except KeyboardInterrupt:
        print("You interrupted waiting for the perception\n")
        sys.exit()
    # Ask and save the different missing parameters
    label = raw_input("Enter a label for the choosen object: \n")
    edible = raw_input("Is this object edible? Enter yes or no: \n")
    # Proof wether the Object is edible.
    while edible != "yes" and edible != "no":
        print("You have to enter 'yes' or 'no'")
        edible = raw_input("Is this object edible? Enter yes or no: \n")
    if edible == "yes":
        edible = True
    else: edible = False7
    # Ask for the variance and save the given
    variance = ""
    while not variance.isdigit():
        variance = raw_input("Type in the Variance in percent the volume is %f : \n" % volume)
    # Last chance to abort 
    while edible != "yes" and edible != "no":
        rly = raw_input("Do you really want to add the object with the label %s,\n\
     the volume: %f \n\
     which is edible: %s \n\
     and has the centroid: %s \n\
     with the id: %d \n\
     shape: %s \n\
     (Enter 'yes' or 'no'): \n" %(label, volume, edible, str(centroid), id, shape[shape_num]))
    if rly = 'yes':
        send_object_to_prolog(id, label, volume, variance, shape, centroid, edible)
