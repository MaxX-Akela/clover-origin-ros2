# This example makes the drone find and follow the red circle.
# To test in the simulator, place 'Red Circle' model on the floor.
# More information: https://clover.coex.tech/red_circle

# Input topic: main_camera/image_raw (camera image)
# Output topics:
#   cv/mask (red color mask)
#   cv/red_circle (position of the center of the red circle in 3D space)

import threading
import rclpy
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.wait_for_message import wait_for_message
import cv2
import numpy as np
from math import nan
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
from cv_bridge import CvBridge
from clover import long_callback, srv, service_proxy
import tf2_ros
import tf2_geometry_msgs
import image_geometry

rclpy.init()
node = rclpy.create_node('cv')

get_telemetry = service_proxy('get_telemetry', srv.GetTelemetry)
set_position = service_proxy('set_position', srv.SetPosition)

bridge = CvBridge()

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer, node)

mask_pub = node.create_publisher(Image, '~/mask', 1)
point_pub = node.create_publisher(PointStamped, '~/red_circle', 1)

# read camera info
camera_model = image_geometry.PinholeCameraModel()
camera_model.from_camera_info(wait_for_message(CameraInfo, node, 'main_camera/camera_info',
                                               qos_profile=qos_profile_sensor_data)[1])


def img_xy_to_point(xy, dist):
    xy_rect = camera_model.rectify_point(xy)
    ray = camera_model.project_pixel_to_3d_ray(xy_rect)
    return Point(x=ray[0] * dist, y=ray[1] * dist, z=dist)

def get_center_of_mass(mask):
    M = cv2.moments(mask)
    if M['m00'] == 0:
        return None
    return M['m10'] // M['m00'], M['m01'] // M['m00']

follow_red_circle = False

@long_callback
def image_callback(msg):
    img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # we need to use two ranges for red color
    mask1 = cv2.inRange(img_hsv, (0, 150, 150), (15, 255, 255))
    mask2 = cv2.inRange(img_hsv, (160, 150, 150), (180, 255, 255))

    # combine two masks using bitwise OR
    mask = cv2.bitwise_or(mask1, mask2)

    # publish the mask
    if mask_pub.get_subscription_count() > 0:
        mask_pub.publish(bridge.cv2_to_imgmsg(mask, 'mono8'))

    # calculate x and y of the circle
    xy = get_center_of_mass(mask)
    if xy is None:
        return

    # calculate and publish the position of the circle in 3D space
    altitude = get_telemetry('terrain').z
    xy3d = img_xy_to_point(xy, altitude)
    target = PointStamped(header=msg.header, point=xy3d)
    point_pub.publish(target)

    if follow_red_circle:
        # follow the target
        setpoint = tf_buffer.transform(target, 'map', timeout=Duration(seconds=0.2))
        set_position(x=setpoint.point.x, y=setpoint.point.y, z=nan, yaw=nan, frame_id=setpoint.header.frame_id)

# process each camera frame:
image_sub = node.create_subscription(Image, 'main_camera/image_raw', image_callback, qos_profile_sensor_data)

# process the callbacks in a background thread, so the main thread can wait for the input
threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

node.get_logger().info('Hit enter to follow the red circle')
input()
follow_red_circle = True
threading.Event().wait()
