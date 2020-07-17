#!/usr/bin/env python2

# sets up the code to use python commands
import rospy
# uses the msgs from ROS to communicate between nodes
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from geometry_msgs.msg import Twist


# python libraries
import av
import cv2
import numpy
import threading
import traceback
import tellopy
# USE A TELLO DRIVER THAT HAS SETUP A CV2 BRIDGE BETWEEN THE TELLO AND ROS

class humanpose_driver():
        
    def __init__(self):
        # # creates the humanpose_driver node on the ROS network
        # rospy.init_node('humanpose_driver_node', anonymous=False)

        # creates the ROS publishers for the information to be pushed out onto the ROS network
        self._cmd_vel = rospy.Publisher('cmd_vel', Twist)
        rospy.init_node("cmd_vel", anonymous=True)
    
        self._takeoff = rospy.Publisher('takeoff', Empty)
        rospy.init_node("takeoff", anonymous=True)
    
        self._land = rospy.Publisher('land', bool)
        rospy.init_node("land", anonymous=True)
    
        self._flip = rospy.Publisher('flip', Flip)
        rospy.init_node("flip", anonymous=True)

        # # links up to the ROS subscriptions for information to be pulled out from the ROS network
        # rospy.Subscriber('flight_data', FlightData, queue_size=10)
        # rospy.Subscriber('image_raw', Image, queue_size=10)
        # # opens a container for the av stream
        # container = av.open(stream)
        # # verifies through ROS that the stream is opened
        # rospy.loginfo('main: opened')

        # #for every frame within the container, decode to separate images
        # for frame in container.decode(video=0):
        #     image = cv2.cvtColor(numpy.array(
        #         frame.to_image()), cv2.COLOR_RGB2BGR)
        #     cv2.imshow('Frame', image)
        #     cv2.waitKey(1)
    
    def update_msgs(self, pitch, roll, throttle, yaw):
        
        rospy.loginfo()
        self._cmd_vel.publish(coords)

    ## 
    # The ROS msgs to the Tello would need the following
    # Yaw       Int
    # Roll      Int
    # Pitch     Int
    # Throttle  Int
    # IsFlying  Boolean 
    ##
    


def main():
   
   # opens a container for the av stream
   container = av.open(drone.stream)
   # verifies through ROS that the stream is opened
   rospy.loginfo('main: opened')

   #for every frame within the container, decode to separate images
   for frame in container.decode(video=0):
       image = cv2.cvtColor(numpy.array(
           frame.to_image()), cv2.COLOR_RGB2BGR)
       cv2.imshow('Frame', image)
       cv2.waitKey(1)


