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
flag = 1
class DetectionMapper():

    def camera_callback(self, camera_info):
        self.camera_infos.append(camera_info)
        
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
        flag = 0
	print("Changing flag")


    def detections_callback(self, detection):
		global counter
		global maxDistance #the maximal distance between two points required for them to be considered the same detection	
		global alreadyDetected 
		global n 
		global lastAdded	
		global detected
		global numFaces
		global brojach
		global avgx
		global avgy
		global avgr
		global flag
		if flag == 0:
			if brojach < 10:
				avgx = avgx + detection.pose.position.x 
				avgy = avgy + detection.pose.position.y
				avgr = avgr + detection.pose.position.z
				brojach = brojach + 1
				return
			if brojach == 10:
				tp = detection
				avgx = avgx / 10
				avgy = avgy / 10
				avgr = avgr / 10
				camera_info = None
				best_time = 100
				for ci in self.camera_infos:
					time = abs(ci.header.stamp.to_sec() - detection.header.stamp.to_sec())
					if time < best_time:
						camera_info = ci
						best_time = time

				if not camera_info or best_time > 1:
					print("Best time is higher than 1")
					tp.pose.position.z = -1
					self.pub_avg_ring.publish(tp)
					flag = 1
					return

				camera_model = PinholeCameraModel()
				camera_model.fromCameraInfo(camera_info)

				p = Point(((avgx - camera_model.cx()) - camera_model.Tx()) / camera_model.fx(),((avgy - camera_model.cy()) - camera_model.Ty()) / camera_model.fy(), avgr)
				lp = self.localize(detection.header, p, self.region_scope)
				self.tf.waitForTransform(detection.header.frame_id, "map", rospy.Time.now(), rospy.Duration(5.0))
				mp = geometry_msgs.msg.PoseStamped()
				mp.header.frame_id = detection.header.frame_id
				mp.pose = lp.pose
				mp.header.stamp = detection.header.stamp
				tp = self.tf.transformPose("map", mp)

				now2 = rospy.Time.now()
				self.tf2.waitForTransform("base_link", "map", now2, rospy.Duration(5.0))

				robot = geometry_msgs.msg.PoseStamped()
				robot.header.frame_id = "base_link"
				robot.pose.position.x = 0
				robot.pose.position.y = 0
				#robot.pose = localization1.pose
				robot.header.stamp = now2
				robotmap = self.tf2.transformPose("map", robot)

				if lp.pose.position.x != 0:
					dp = self.distance(robotmap, tp)
					if dp > 1.5:
						print("Distance too big: ",dp)
						tp.pose.position.z = -1
					#if ((point1.y - point2.y)/2 > 0.7 or (point1.y - point2.y)/2 < 0.5):
					#	tp.pose.position.z = -1
					#if (point1.z > 0.5 or point1.z < 0.3):
					#	tp.pose.position.z = -1
					#if (point2.z > 0.5 or point2.z < 0.3): # visina
					#	tp.pose.position.z = -1
				else:
					print("Localizer failed")
					tp.pose.position.z = -1
				self.pub_avg_ring.publish(tp)
				flag = 1
				return
			
		if flag == 1:
			u = detection.pose.position.x
			#u = avgx+avgr
			v = detection.pose.position.y+detection.pose.position.z
			#v = avgy

			w = detection.pose.position.x
			#w = avgx-avgr
			q = detection.pose.position.y-detection.pose.position.z

			g = detection.pose.position.x
			#w = avgx-avgr
			h = detection.pose.position.y+detection.pose.position.z + 4

			r = detection.pose.position.x
			#w = avgx-avgr
			t = detection.pose.position.y+detection.pose.position.z - 4
			#q = avgy
		
			camera_info = None
			best_time = 100
			for ci in self.camera_infos:
				time = abs(ci.header.stamp.to_sec() - detection.header.stamp.to_sec())
				if time < best_time:
					camera_info = ci
					best_time = time

			if not camera_info or best_time > 1:
				print("Best time is higher than 1")
				return

			camera_model = PinholeCameraModel()
			camera_model.fromCameraInfo(camera_info)

			point1 = Point(((u - camera_model.cx()) - camera_model.Tx()) / camera_model.fx(),
		     ((v - camera_model.cy()) - camera_model.Ty()) / camera_model.fy(), 1)


			point2 = Point(((w - camera_model.cx()) - camera_model.Tx()) / camera_model.fx(),
		     ((q - camera_model.cy()) - camera_model.Ty()) / camera_model.fy(), 1)

			point3 = Point(((g - camera_model.cx()) - camera_model.Tx()) / camera_model.fx(),
		     ((h - camera_model.cy()) - camera_model.Ty()) / camera_model.fy(), 1)


			point4 = Point(((r - camera_model.cx()) - camera_model.Tx()) / camera_model.fx(),
		     ((t - camera_model.cy()) - camera_model.Ty()) / camera_model.fy(), 1)
	
			localization1 = self.localize(detection.header, point1, self.region_scope)
			localization2 = self.localize(detection.header, point2, self.region_scope)
			localization3 = self.localize(detection.header, point3, self.region_scope)
			localization4 = self.localize(detection.header, point4, self.region_scope)
			if not localization1:
				return

			# calculate center
			center = Point(localization1.pose.position.x, localization1.pose.position.y, localization1.pose.position.z)
			now = detection.header.stamp
			self.tf.waitForTransform(detection.header.frame_id, "map", now, rospy.Duration(5.0))

			m = geometry_msgs.msg.PoseStamped()
			m.header.frame_id = detection.header.frame_id
			#m.pose.position.x = center.x
			#m.pose.position.y = center.y
			m.pose = localization1.pose
			m.header.stamp = detection.header.stamp
			transformedPoint1 = self.tf.transformPose("map", m)




			m2 = geometry_msgs.msg.PoseStamped()
			m2.header.frame_id = detection.header.frame_id
			m2.pose = localization2.pose
			m2.header.stamp = detection.header.stamp
			transformedPoint2 = self.tf.transformPose("map", m2)




			m3 = geometry_msgs.msg.PoseStamped()
			m3.header.frame_id = detection.header.frame_id
			m3.pose = localization3.pose
			m3.header.stamp = detection.header.stamp
			transformedPoint3 = self.tf.transformPose("map", m3)



			m4 = geometry_msgs.msg.PoseStamped()
			m4.header.frame_id = detection.header.frame_id
			m4.pose = localization4.pose
			m4.header.stamp = detection.header.stamp
			transformedPoint4 = self.tf.transformPose("map", m4)


			
			now2 = rospy.Time.now()
			self.tf2.waitForTransform("base_link", "map", now2, rospy.Duration(5.0))

			robot = geometry_msgs.msg.PoseStamped()
			robot.header.frame_id = "base_link"
			robot.pose.position.x = 0
			robot.pose.position.y = 0
			#robot.pose = localization1.pose
			robot.header.stamp = now2
			robotmap = self.tf2.transformPose("map", robot)

			#dist = self.distance(robotmap,transformedPoint)
			transformedPoint = transformedPoint1
			if localization1.pose.position.x != 0:
				dist1 = self.distance(robotmap, transformedPoint1)
			else:
				dist1 = 100000
			if localization2.pose.position.x != 0:
				dist2 = self.distance(robotmap, transformedPoint2)
			else:
				dist2 = 100000
			if localization3.pose.position.x != 0:
				dist3 = self.distance(robotmap, transformedPoint3)
			else:
				dist3 = 100000
			if localization4.pose.position.x != 0:
				dist4 = self.distance(robotmap, transformedPoint4)
			else:
				dist4 = 100000
			# find smallest distance to a point.
			dist = dist1
			if dist2 < dist:
				dist = dist2
				transformedPoint = transformedPoint2
			if dist3 < dist:
				dist = dist3
				transformedPoint = transformedPoint3
			if dist4 < dist:
				dist = dist4
				transformedPoint = transformedPoint4
			
			print("distance: ", dist)	
			if( dist < 2 ):
				radius = abs((transformedPoint1.pose.position.y - transformedPoint2.pose.position.y)/2)
				print("radius: ", radius)
				print("height: ", transformedPoint.pose.position.z)
				#print("height p2: ", transformedPoint2.pose.position.z)
				if((dist1 < 2 and dist2 < 2)):# or (dist1 >2 and dist2 > 2)):
					print("Checking radius")
					if (radius > 0.7 or radius < 0.3):	
						print("Wrong radius")
						return
				else: 
					print("Cant use radius")
					#return
				if (transformedPoint.pose.position.z > 0.56 or transformedPoint.pose.position.z < 0.48):
					print("Wrong height")
					return 
				#if (transformedPoint2.pose.position.z > 0.5 or transformedPoint2.pose.position.z < 0.3): # visina
				#	return 
				#self.pub.publish(transformedPoint)
				print("start checking")
				print(len(detected))
				if len(detected) == 0:
					beenDetected = False
				else:
					beenDetected = False
					for p in detected:
						if self.distance(p, transformedPoint) <= maxDistance:
							print("Already detected ring!")
							beenDetected = True
							break
					
				if(beenDetected == False):
					detected.append(transformedPoint)
					print("Publishing new ring")
					self.pub_ring.publish(transformedPoint)
					marker = Marker()
					marker.header.stamp = detection.header.stamp
					marker.header.frame_id = transformedPoint.header.frame_id
					marker.pose.position.x = transformedPoint.pose.position.x
					marker.pose.position.y = transformedPoint.pose.position.y
					marker.pose.position.z = transformedPoint.pose.position.z
					marker.type = Marker.CUBE
					marker.action = Marker.ADD
					marker.frame_locked = False
					marker.lifetime = rospy.Duration.from_sec(1)
					marker.id = self.marker_id_counter
					marker.scale = Vector3(0.1, 0.1, 0.1)
					marker.color = ColorRGBA(1, 0, 0, 1)
					self.markers.markers.append(marker)
					self.marker_id_counter += 1
					print("Number of detected rings: ", len(detected))							
					#if len(detected) == numFaces:
					#	print("Sending shutdown")	
			else:
				print("Just zeros")


    def flush(self):
        if not self.markers.markers:
            self.markers = MarkerArray()
            return
        self.markers_pub.publish(self.markers)
        #self.markers = MarkerArray()

    def __init__(self):
		numFaces = 1
		counter = 0
		maxDistance = 0.9
		alreadyDetected = [] # an array containing n points, the first n detected that were considered the same detection
		n = 3 # number of points required in order to determine that the detection is a face
		detected = []
		self.tf = TransformListener()
		self.tf2 = TransformListener()
		self.markers = MarkerArray()
		self.marker_id_counter = 0 
		self.region_scope = rospy.get_param('~region', 3)
		self.buffer_size = rospy.get_param('~camera_buffer_size', 50)
		rospy.wait_for_service('localizer/localize')
		self.pub = rospy.Publisher('ring_position', geometry_msgs.msg.PoseStamped)
		self.pub_ring = rospy.Publisher('new_ring', geometry_msgs.msg.PoseStamped)
		self.pub_avg_ring = rospy.Publisher('avg_ring', geometry_msgs.msg.PoseStamped)
		self.camera_infos = collections.deque(maxlen = self.buffer_size)
		self.detections_sub = message_filters.Subscriber('blob_topic', geometry_msgs.msg.PoseStamped)
		self.detections_sub.registerCallback(self.detections_callback)

		self.detections_sub = message_filters.Subscriber('changeFlag', String)
		self.detections_sub.registerCallback(self.changeFlag_callback)	

		self.camera_sub = message_filters.Subscriber('/camera/depth_registered/camera_info', CameraInfo)
		self.camera_sub.registerCallback(self.camera_callback)


		self.localize = rospy.ServiceProxy('localizer/localize', Localize)
		self.markers_pub = rospy.Publisher('markers', MarkerArray)
   
if __name__ == '__main__':

	rospy.init_node('mapper')
	try:
		mapper = DetectionMapper()
		print("Mapper created")
		r = rospy.Rate(30)
		while not rospy.is_shutdown():
			mapper.flush()
			r.sleep()
	except rospy.ROSInterruptException: pass


