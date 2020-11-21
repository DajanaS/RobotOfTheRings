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

class DetectionMapper():

'''
    global maxDistance #the maximal distance between two points required for them to be considered the same detection	
    maxDistance = 0.5			
    global alreadyDetected 
    alreadyDetected = [3] # an array containing n points, the first n detected that were considered the same detection
    global n 
    n = 3 # number of points required in order to determine that the detection is a face
    global lastAdded	
    global counter 
    counter = 0
	
'''
    def camera_callback(self, camera_info):
        self.camera_infos.append(camera_info)
'''
    def distance(p1, p2):
	xd = p1.x - p2.x
	yd = p1.y - p2.y
	dd = xd ** 2 + yd ** 2
	res = math.sqrt(dd)
	return res 
'''		

    def detections_callback(self, detection):


#coordinates	
        u = detection.x + detection.width / 2
        v = detection.y + detection.height / 2

        camera_info = None
        best_time = 100
        for ci in self.camera_infos:
            time = abs(ci.header.stamp.to_sec() - detection.header.stamp.to_sec())
            if time < best_time:
                camera_info = ci
                best_time = time

        if not camera_info or best_time > 1:
            return

        camera_model = PinholeCameraModel()
        camera_model.fromCameraInfo(camera_info)

        point = Point(((u - camera_model.cx()) - camera_model.Tx()) / camera_model.fx(),
             ((v - camera_model.cy()) - camera_model.Ty()) / camera_model.fy(), 1)

        localization = self.localize(detection.header, point, self.region_scope)

        if not localization:
            return

#adds the marker here
#so here we need to check if it is the same spot

	#check the distance between the lastAdded and the new point
	#if the array is shorter than 3, add the new point and make it lastAdded
	#else if the discard the new point
	#if the distance is bigger, just discard the new point

'''
	print("Found a face.")
	rospy.loginfo("Found a face.")

	if counter == 0:
		lastAdded = point
		rospy.loginfo("Adding the point to the array.")
		alreadyDetected[0] = point
		counter+=1
		
	else:
		dist = self.distance(point, lastAdded)
		if dist <= maxDistance:
		     if counter < 3:
			rospy.loginfo("Adding the point to the array.")
			alreadyDetected[counter] = point
			lastAdded = point
			counter += 1
		     elif counter == 3:
			counter = 0
			rospy.loginfo("Adding marker.")
			

'''

	#add the marker here
	marker = Marker()
	marker.header.stamp = detection.header.stamp
	marker.header.frame_id = detection.header.frame_id
	marker.pose = localization.pose
		
	marker.type = Marker.CUBE
	marker.action = Marker.ADD
	marker.frame_locked = False
	marker.lifetime = rospy.Duration.from_sec(1)
	marker.id = self.marker_id_counter
	marker.scale = Vector3(0.1, 0.1, 0.1)
	marker.color = ColorRGBA(1, 0, 0, 1)
	self.markers.markers.append(marker)
	self.marker_id_counter += 1


    def flush(self):
        if not self.markers.markers:
            self.markers = MarkerArray()
            return
        self.markers_pub.publish(self.markers)
        #self.markers = MarkerArray()

    def __init__(self):
	   self.markers = MarkerArray()
        self.marker_id_counter = 0
        self.region_scope = rospy.get_param('~region', 3)
        self.buffer_size = rospy.get_param('~camera_buffer_size', 50)
        rospy.wait_for_service('localizer/localize')

        self.camera_infos = collections.deque(maxlen = self.buffer_size)
        self.detections_sub = message_filters.Subscriber('detections', Detection)
	   self.detections_sub.registerCallback(self.detections_callback)

        self.camera_sub = message_filters.Subscriber('camera_info', CameraInfo)
	   self.camera_sub.registerCallback(self.camera_callback)

        self.localize = rospy.ServiceProxy('localizer/localize', Localize)
        self.markers_pub = rospy.Publisher('markers', MarkerArray)
   
if __name__ == '__main__':

        rospy.init_node('mapper')

        try:
            mapper = DetectionMapper()
            r = rospy.Rate(30)
            while not rospy.is_shutdown():
                mapper.flush()
                r.sleep()
        except rospy.ROSInterruptException: pass


    












