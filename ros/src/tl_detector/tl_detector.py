#!/usr/bin/env python
from __future__ import division
from __future__ import print_function

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from styx_msgs.msg import TrafficLightArray, TrafficLight, Lane
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel
from cv_bridge import CvBridge
# from light_classification.tl_classifier import TLClassifier
from tl_detection.tl_detector import TrafficLightDetector
import tf
import yaml
import math
import numpy as np
import cv2
from utils import benchmark
from utils import Waypoints
from utils import loop_at_rate


DEBUG_TLD = False
STATE_COUNT_THRESHOLD = 3


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector', log_level=rospy.DEBUG)

        self.pose_msg = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        # Camera info and stop positions
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.bridge = CvBridge()
        self.tl_detector = TrafficLightDetector()

        # Not used
        # self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        # FIXME. These parameters will need tuning for the test site
        self.max_light_dist = 50  # meters
        self.min_light_dist = 5  # meters

        # NOTE. We don't handle stereo cameras (rectification and projection matrix)
        self.camera = PinholeCameraModel()

        # Initialize camera model with the info from config file
        camera_info = self.config['camera_info']
        if 'focal_length_x' in camera_info:
            camera_info_msg = CameraInfo()
            camera_info_msg.width = camera_info["image_width"]
            camera_info_msg.height = camera_info["image_height"]
            camera_info_msg.R = np.eye(3).reshape(9, 1)
            camera_info_msg.K = self.get_camera_matrix(camera_info).reshape(9, 1)
            camera_info_msg.P = self.get_camera_projection(camera_info).reshape(12, 1)
            self.camera.fromCameraInfo(camera_info_msg)

        self.upcoming_tl_pub = \
            rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        if DEBUG_TLD:
            self.tl_image_pub = \
                rospy.Publisher('/traffic_image', Image, queue_size=1)
            self.bridge = CvBridge()

        self.pose_subscriber = \
            rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.waypoints_subscriber = \
            rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        # helps you acquire an accurate ground truth data source for the traffic light
        # classifier by sending the current color state of all traffic lights in the
        # simulator. When testing on the vehicle, the color state will not be available. You'll need to
        # rely on the position of the light and the camera image to predict it.
        self.traffic_lights_subscriber = \
            rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size=1)
        self.camera_info_subscriber = \
            rospy.Subscriber('/camera_info', CameraInfo, self.camera_info_cb, queue_size=1)
        # delayed subscription
        self.image_color_subscriber = None

        self.loop()

    def set_image_subscription(self, on):
        if on:
            if self.image_color_subscriber:
                return
            rospy.loginfo("tl_detector: subscribing to /image_color")
            self.image_color_subscriber = \
                rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1, buff_size=1441024)
        elif self.image_color_subscriber:
            rospy.loginfo("tl_detector: unsubscribing from /image_color")
            self.image_color_subscriber.unregister()
            self.image_color_subscriber = None


    def camera_info_cb(self, camera_info):
        self.camera.fromCameraInfo(camera_info)
        # NOTE. We don't plan to change the camera while driving.
        self.camera_info_subscriber.unregister()

    def pose_cb(self, msg):
        self.pose_msg = msg

    def waypoints_cb(self, lane):
        if self.waypoints is None:
            self.waypoints = Waypoints(lane.waypoints)
        else:
            self.waypoints.update(lane.waypoints)
        # NOTE. For this project the waypoints are always constant,
        # we can unsubscribe the topic to save cpu
        self.waypoints_subscriber.unregister()

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera
        """
        self.camera_image = msg

    def loop(self):
        self.state = TrafficLight.UNKNOWN
        self.last_wp = -1

        publish_frequency = rospy.get_param("~publish_frequency", 50)

        for _ in loop_at_rate(50):
            # Wait for pose, waypoints and lights...
            if self.pose_msg and self.waypoints and self.lights:
                break

        for _ in loop_at_rate(publish_frequency):
            # Find next stop point
            stop_point, stop_distance = self.next_stop_line(self.pose_msg)

            if stop_point is None:
                # Stop looking for TL
                self.set_image_subscription(False)
                light_wp, state = -1, TrafficLight.UNKNOWN
            else:
                self.set_image_subscription(True)
                light_wp, state = self.last_wp, self.last_state

                if self.camera_image:
                    camera_image = self.camera_image
                    self.camera_image = None

                    with benchmark("process_traffic_lights %d" % camera_image.header.seq):
                        light_wp, state = self.process_traffic_lights(camera_image,
                                                                      stop_point,
                                                                      stop_distance,
                                                                      self.pose_msg,
                                                                      self.waypoints,
                                                                      self.lights)

            # Publish upcoming red lights at constant rate.
            # Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
            # of times till we start using it. Otherwise the previous stable state
            # is used.
            if self.state != state:
                self.state_count = 0
                self.state = state
            elif self.state_count >= STATE_COUNT_THRESHOLD:
                self.last_state = state
                # NOTE. Warn on red and yellow lights
                if (state == TrafficLight.RED or state == TrafficLight.YELLOW):
                    self.last_wp = light_wp
                else:
                    self.last_wp = -1

            self.upcoming_tl_pub.publish(Int32(self.last_wp))
            self.state_count += 1

    def next_stop_line(self, pose_msg):
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']

        # car position
        ego_x = pose_msg.pose.position.x
        ego_y = pose_msg.pose.position.y
        ego_yaw = self.yaw_from_quaternion(pose_msg.pose.orientation)
        cos_yaw = math.cos(-ego_yaw)
        sin_yaw = math.sin(-ego_yaw)

        # Find next stop position
        stop_point = None
        stop_distance = None
        for stop_x, stop_y in stop_line_positions:
            dx = stop_x - ego_x
            dy = stop_y - ego_y
            # Give a margin in case we are at the stop position
            if dx * cos_yaw - dy * sin_yaw < -2:
                continue

            stop_distance = math.sqrt(dx*dx + dy*dy)
            if stop_distance <= self.max_light_dist:
                stop_point = stop_x, stop_y
                break

        return stop_point, stop_distance

    def process_traffic_lights(self, image_msg, stop_point, stop_distance, pose_msg, waypoints, lights):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closest to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        # Check traffic light
        light_index = self.get_closest_waypoint(lights, stop_point)
        # Ground truth:
        # light_state = lights[light_index].state
        light_state = self.get_light_state(image_msg, lights[light_index])

        light_wp = waypoints.find(stop_point[0], stop_point[1])

        ego_x = pose_msg.pose.position.x
        ego_y = pose_msg.pose.position.y

        if lights[light_index].state < 3:
            validation = " [{}]".format("OK" if light_state == lights[light_index].state else "FAIL")
            rospy.loginfo("Traffic light: %s (%s) at %d waypoints (%.2f m)%s",
                          self.light_state_string(light_state),
                          self.light_state_string(lights[light_index].state),
                          light_wp - waypoints.find(ego_x, ego_y),
                          stop_distance,
                          validation)
        else:
            rospy.loginfo("Traffic light: %s at %d waypoints (%.2f m)",
                          self.light_state_string(light_state),
                          light_wp - waypoints.find(ego_x, ego_y),
                          stop_distance)


        return light_wp, light_state

    def get_closest_waypoint(self, waypoints, position, start_at=0):
        """Identifies the closest waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            position (x, y): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in waypoints
        """
        x, y = position
        # minimum squared distance to pose in waypoints
        return min(xrange(start_at, len(waypoints)),
                   key=lambda i: ((waypoints[i].pose.pose.position.x - x)**2 +
                                  (waypoints[i].pose.pose.position.y - y)**2))

    def get_light_state(self, camera_image, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

        # image = self.bridge.imgmsg_to_cv2(camera_image, "rgb8")
        image = (np.frombuffer(camera_image.data, dtype=np.uint8)
                 .reshape(camera_image.height, camera_image.width, 3))

        # FIXME: Disabled to save time
        # Correct image for distortions
        # if self.camera:
        #     tmp = np.zeros_like(image)
        #     self.camera.rectifyImage(image, tmp)
        #     image = tmp

        # Detect traffic lights in image.
        # In the simulator the ground truth is in light.state
        result = self.tl_detector.analyze(image)

        if result:
            # Just use the first one
            state, score, bbox = result[0]
            rospy.logdebug("tl_detector: %s (%.2f). /traffic_light state = %s",
                           self.light_state_string(state),
                           score,
                           self.light_state_string(light.state))

            if DEBUG_TLD:

                y0, x0, y1, x1 = bbox
                x0 = int(x0 * image.shape[1])
                x1 = int(x1 * image.shape[1])
                y0 = int(y0 * image.shape[0])
                y1 = int(y1 * image.shape[0])
                img_box = image[y0:y1, x0:x1]
                cv2.putText(img_box, self.light_state_string(state), (0, y1-y0), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255))
                img_msg = self.bridge.cv2_to_imgmsg(img_box)
                img_msg.encoding = "bgr8"
                self.tl_image_pub.publish(img_msg)

        else:
            state = TrafficLight.UNKNOWN
            rospy.logdebug("tl_detector: no TL present. /traffic_light state = %s",
                           self.light_state_string(light.state))

        return state

    def light_state_string(self, state):
        if state == TrafficLight.RED:
            return "RED"
        elif state == TrafficLight.GREEN:
            return "GREEN"
        elif state == TrafficLight.YELLOW:
            return "YELLOW"
        elif state == TrafficLight.UNKNOWN:
            return "UNKNOWN"
        else:
            return "UNKNOWN(%d)" % (state)

    def quaternion_rotate(self, q, v):
        # q conjugate
        q_conjugate = (-q[0], -q[1], -q[2], q[3])
        q_v = (v[0], v[1], v[2], 1.)
        q_v_rotated = self.quaternion_multiply(
            self.quaternion_multiply(q_conjugate, q_v), q)
        v_rotated = q_v_rotated[:3]
        return v_rotated

    def quaternion_multiply(self, q1, q0):
        x0, y0, z0, w0 = q0
        x1, y1, z1, w1 = q1
        return ( x1*w0 + y1*z0 - z1*y0 + w1*x0,
                -x1*z0 + y1*w0 + z1*x0 + w1*y0,
                 x1*y0 - y1*x0 + z1*w0 + w1*z0,
                -x1*x0 - y1*y0 - z1*z0 + w1*w0 )

    def yaw_from_quaternion(self, q):
        return math.atan2(2.0 * (q.z * q.w + q.x * q.y),
                          - 1.0 + 2.0 * (q.w * q.w + q.x * q.x))

    def get_camera_matrix(self, camera_info):
        fx = camera_info['focal_length_x']
        fy = camera_info['focal_length_y']
        cx = camera_info['image_width'] / 2.0
        cy = camera_info['image_height'] / 2.0
        return np.array([[fx,  0, cx],
                         [ 0, fy, cy],
                         [ 0,  0,  1]])

    def get_camera_projection(self, camera_info):
        K = self.get_camera_matrix(camera_info)
        return np.column_stack((K, [0., 0., 0.]))



if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
