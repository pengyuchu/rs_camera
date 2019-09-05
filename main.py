#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('rs_camera')
import sys
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import pyrealsense2 as rs
import numpy as np
import cv2


# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
MAX_WIDTH = 640
MAX_HEIGHT = 480
config.enable_stream(rs.stream.depth, MAX_WIDTH, MAX_HEIGHT, rs.format.z16, 30)
config.enable_stream(rs.stream.color, MAX_WIDTH, MAX_HEIGHT, rs.format.bgr8, 30)

profile = pipeline.start(config)
bridge = CvBridge()
next_iteration = True

def init_publishers():
  global image_pub
  global depth_pub
  global pos_pub
  image_pub = rospy.Publisher("color_image",Image, queue_size = 1)
  depth_pub = rospy.Publisher("depth_image",Image, queue_size = 1)
  pos_pub = rospy.Publisher("positions",PoseArray, queue_size = 1)


def init_subcribers():
  global detection_sub
  # image_sub = rospy.Subscriber("color_image",Image,view_callback)
  detection_sub = rospy.Subscriber("Detection",Int16MultiArray ,bbox_callback)

def meter2inch(data):
	return data * 39.3701

def get_images_from_rs_camera():
  global color_image
  global depth_image
  global depth_scale
  global depth_intrinsics
  global depth_colormap

  # Wait for a coherent pair of frames: depth and color
  frames = pipeline.wait_for_frames()
  depth_frame = frames.get_depth_frame()
  color_frame = frames.get_color_frame()
  depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

  depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

  if not depth_frame or not color_frame:
      return

  # Convert images to numpy arrays
  depth_image = np.asanyarray(depth_frame.get_data())
  color_image = np.asanyarray(color_frame.get_data())

  # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
  depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)


def stream_images():
	# capture images 
	global next_iteration
	r = rospy.Rate(4) # 10hz
	try:
		while True:
			if next_iteration:
				get_images_from_rs_camera()
				image_pub.publish(bridge.cv2_to_imgmsg(color_image, "bgr8"))
				depth_pub.publish(bridge.cv2_to_imgmsg(depth_image, "mono16"))

				print('Send Msg')

			r.sleep()
	finally:
		# Stop streaming
		pipeline.stop()


def view_callback(data):
  try:
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    print('Receive Msg')
  except CvBridgeError as e:
    print(e)

  cv2.imshow("Image window", cv_image)
  key = cv2.waitKey(3)
  if key == 'q':
    cv2.destroyAllWindows()

def sortZ(point):
	return point.position.z

def cal_coodinates(bboxes):
  points = PoseArray()

  i = 0
  while i < bboxes.shape[0]:
    y1 = bboxes[i][0]
    x1 = bboxes[i][1]
    y2 = bboxes[i][2]
    x2 = bboxes[i][3]
    i += 1
    cv2.rectangle(color_image,(x1,y1),(x2,y2),(0,255,255),1)
    width, height = x2-x1, y2-y1
    distance_loss_x = width/4
    distance_loss_y = height/4

    # calculate average depth in the specific area
    depth = depth_image[x1+distance_loss_x:x2-distance_loss_x, y1+distance_loss_y:y2-distance_loss_y].astype(float)
    distance,_,_,_ = cv2.mean(depth)
    # calcuate the depth in the central pixel
    distance = depth_image[(y2+y1)/2, (x2+x1)/2]

    # Tune distance based on the intrincs
    distance *= depth_scale

    # Projection
    point_location = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [(x2+x1)/2, (y2+y1)/2], distance)
    point_location = [meter2inch(p) for p in point_location]
    print('The location is:',  point_location)

    point = Pose()
    point.position.x = point_location[0]
    point.position.y = point_location[1]
    point.position.z = point_location[2]
    points.poses.append(point)
    # print('The Distance is ', distance)

    # Display (x, y, z) on the bounding boxes
    font                   = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (x1,y1-5)
    fontScale              = 0.3
    fontColor              = (0,215,215)
    lineType               = 2
    text = '%.1f, %.1f, %.1f' % (point_location[0],point_location[1],point_location[2])

    cv2.putText(color_image,text,
        bottomLeftCornerOfText,
        font,
        fontScale,
        fontColor,
        lineType)

  final = cv2.hconcat([color_image, depth_colormap])
  cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
  cv2.imshow('RealSense', color_image)
  key = cv2.waitKey(1)
  if key & 0xFF == ord('q') or key == 27:
    cv2.destroyAllWindows()
	
  points.poses.sort(key=sortZ)
  pos_pub.publish(points)
      


def bbox_callback(data):
	global next_iteration

	bboxes = np.array(data.data).reshape(len(data.data)/4, 4)
	cal_coodinates(bboxes)
	next_iteration = True


if __name__ == '__main__':
  rospy.init_node('rs_camera', anonymous=True)
  init_publishers()
  init_subcribers()
  stream_images()
