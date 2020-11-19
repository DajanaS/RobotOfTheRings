#!/usr/bin/env python
import roslib
roslib.load_manifest('localizer')
import rospy
import sys, select, termios, tty
from std_msgs.msg import String, Bool, ColorRGBA
import sensor_msgs.msg
import message_filters
import collections
from detection_msgs.msg import Detection
from localizer.srv import Localize
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import Point, Vector3
import math
from tf import TransformListener
import geometry_msgs.msg
import move_base_msgs.msg

numFaces = 1
counter = 0
maxDistance = 0.5
alreadyDetected = [] # an array containing n points, the first n detected that were considered the same detection
n = 3 # number of points required in order to determine that the detection is a ring
detected = [] #an array with the already passed rings
brojach = 0
avgx = 0
avgy = 0
avgr = 0
minHeight = 0.1
maxHeight =1.0
maxDistRing = 2.0

flag = 1
class DetectionMapper():
        
    def distance(self, d1, d2):
		p1 = d1.pose.position
		p2 = d2.pose.position
		xd = p1.x - p2.x
		yd = p1.y - p2.y
		dd = xd ** 2 + yd ** 2
		res = math.sqrt(dd)
		return res 

    def changeFlag_callback(self, msg):
        global flag
        if flag is 1:
			flag = 0
        if flag is 0:
			flag = 1
        print("Changing flag: ", flag)
	
    def approach_callback(self, msg):
        print("Set do map to false")
        self.doMap = False
	
    def start_callback(self, msg):
        print("Set do map to true")
        self.doMap = True


    def detections_callback(self, detection):
    
        transformedPoint = move_base_msgs.msg.MoveBaseGoal()
        transformedPoint.target_pose.header.frame_id = detection.target_pose.header.frame_id
        transformedPoint.target_pose.pose = detection.target_pose.pose
        transformedPoint.target_pose.header.stamp = rospy.Time.now()

#		now2 = rospy.Time.now()
#		self.tf2.waitForTransform("base_link", "map", now2, rospy.Duration(5.0))

        print("Got cylinder, start checking")
        print("Detected zylinder: ", len(detected))
        if len(detected) == 0:
            beenDetected = False
        else:
            beenDetected = False
            for p in detected:
                if self.distance(p, transformedPoint.target_pose) <= maxDistance:
                    print("Already detected ring!")
                    beenDetected = True
                    break
			
        if(beenDetected == False):
              self.pub.publish(transformedPoint)
              detected.append(transformedPoint.target_pose)
              print("Publishing new ring", transformedPoint.target_pose.pose.position)
              #self.pub_ring.publish(transformedPoint)
              marker = Marker()
              marker.header.stamp = rospy.Time.now()
              marker.header.frame_id = "map"
              marker.pose.position.x = transformedPoint.target_pose.pose.position.x
              marker.pose.position.y = transformedPoint.target_pose.pose.position.y
              marker.pose.position.z = transformedPoint.target_pose.pose.position.z
              marker.type = Marker.CUBE
              marker.action = Marker.ADD
              marker.frame_locked = False
              marker.lifetime = rospy.Duration.from_sec(1)
              marker.id = self.marker_id_counter
              marker.scale = Vector3(0.1, 0.1, 0.1)
              marker.color = ColorRGBA(0, 1, 0, 1)
              self.markers.markers.append(marker)
              self.marker_id_counter += 1
              print("Number of detected rings: ", len(detected))							
			#if len(detected) == numFaces:
			#	print("Sending shutdown")


    def flush(self):
        if not self.markers.markers:
            self.markers = MarkerArray()
            return
        self.markers_pub.publish(self.markers)
        #self.markers = MarkerArray()

    def __init__(self):
		self.tf = TransformListener()
		self.tf2 = TransformListener()
		self.markers = MarkerArray()
		self.marker_id_counter = 0 
		self.pub = rospy.Publisher('cylinder_new', move_base_msgs.msg.MoveBaseGoal)
		self.detections_sub = message_filters.Subscriber('cylinder', move_base_msgs.msg.MoveBaseGoal)
		self.detections_sub.registerCallback(self.detections_callback)

		#self.detections_sub = message_filters.Subscriber('changeFlag', String)
		#self.detections_sub.registerCallback(self.changeFlag_callback)	
		
		self.markers_pub = rospy.Publisher('zmarkers', MarkerArray)
   
if __name__ == '__main__':

	rospy.init_node('zylindermapper')
	try:
		mapper = DetectionMapper()
		print("Mapper created")
		r = rospy.Rate(30)
		while not rospy.is_shutdown():
			mapper.flush()
			r.sleep()
	except rospy.ROSInterruptException: pass




