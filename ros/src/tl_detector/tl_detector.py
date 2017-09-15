#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from styx_msgs.msg import TrafficLightArray, TrafficLight, Lane
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import numpy as np


STATE_COUNT_THRESHOLD = 3


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector', log_level=rospy.DEBUG)

        self.pose_msg = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # /vehicle/traffic_lights helps you acquire an accurate ground truth data source for the traffic light
        # classifier, providing the location and current color state of all traffic lights in the
        # simulator. This state can be used to generate classified images or subbed into your solution to
        # help you work on another single component of the node.
        #
        # After [9afd2a] This topic will be available in Carla but without the light state.
        #
        # /vehicle/traffic_lights topic will publish the exact position of the light
        # both, in the simulator and in Carla.
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        rospy.Subscriber('/image_color', Image, self.image_cb)
        rospy.Subscriber('/camera_info', CameraInfo, self.camera_info_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        # FIXME. These parameters will need tuning for the test site
        self.max_light_dist = 150  # meters
        self.min_light_dist = 5  # meters

        # NOTE. We don't handle stereo cameras (rectification and projection matrix)
        self.camera = PinholeCameraModel()

        # Initialize camera model from config file
        camera_info = self.config['camera_info']
        camera_info_msg = CameraInfo()
        camera_info_msg.width = camera_info["image_width"]
        camera_info_msg.height = camera_info["image_height"]
        camera_info_msg.R = np.eye(3).reshape(9, 1)
        camera_info_msg.K = self.get_camera_matrix(camera_info).reshape(9, 1)
        camera_info_msg.P = self.get_camera_projection(camera_info).reshape(12, 1)
        self.camera.fromCameraInfo(camera_info_msg)

        rospy.spin()

    def camera_info_cb(self, camera_info):
        self.camera.fromCameraInfo(camera_info)

    def pose_cb(self, msg):
        self.pose_msg = msg

    def waypoints_cb(self, lane):
        # Just take the waypoints
        self.waypoints = lane.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera
        """
        # Wait for pose and waypoints...
        if not self.pose_msg or not self.waypoints:
            self.state = TrafficLight.UNKNOWN
            self.last_wp = -1
            return

        # The msg contains now the right encoding
        self.camera_image = msg

        # Save current pose assuming it's the closest to
        # the moment the picture was taken
        # TODO: We should check the msg timestamps...
        self.camera_pose = self.pose_msg.pose

        light_wp, state = self.process_traffic_lights()

        # Publish upcoming red lights at camera frequency.
        # Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        # of times till we start using it. Otherwise the previous stable state
        # is used.
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closest to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        light_wp = -1
        light_state = TrafficLight.UNKNOWN

        # Light positions
        lights = self.lights
        # Positions where to stop at
        stop_positions = self.config['light_positions']
        # Base waypoints
        waypoints = self.waypoints
        # Car pose at the time of the picture
        car_pose = self.camera_pose

        # car position
        ego_x = car_pose.position.x
        ego_y = car_pose.position.y
        ego_yaw = self.yaw_from_quaternion(car_pose.orientation)
        cos_yaw = math.cos(-ego_yaw)
        sin_yaw = math.sin(-ego_yaw)

        # Find next stop position
        stop_point = None
        stop_distance = None
        for stop_x, stop_y in stop_positions:
            dx = stop_x - ego_x
            dy = stop_y - ego_y
            if dx * cos_yaw + dy * sin_yaw < 0:
                continue

            stop_distance = math.sqrt(dx*dx + dy*dy)
            if stop_distance <= self.max_light_dist:
                stop_point = stop_x, stop_y
                break

        if stop_point:
            # Check traffic light
            light_index = self.get_closest_waypoint(lights, stop_point)
            light_state = self.get_light_state(lights[light_index])

            if light_state == TrafficLight.RED:
                light_wp = self.get_closest_waypoint(waypoints, stop_point)

            start_at = max((light_index - 200), 0)
            ego_wp = self.get_closest_waypoint(waypoints, (ego_x, ego_y), start_at)
            rospy.loginfo("Traffic light: %s at %d (%.2f m) (car at %d)",
                          self.light_state_string(light_state),
                          light_wp, stop_distance, ego_wp)

        return light_wp, light_state

    def get_closest_waypoint(self, waypoints, position, start_at=0):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            position (x, y): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        x, y = position
        # minimum squared distance to pose in waypoints
        return min(xrange(start_at, len(waypoints)),
                   key=lambda i: ((waypoints[i].pose.pose.position.x - x)**2 +
                                  (waypoints[i].pose.pose.position.y - y)**2))

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

        image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # Correct image for distortions
        if True or self.camera.distortion_model:
            tmp = np.zeros_like(image)
            self.camera.rectifyImage(image, tmp)
            image = tmp

        # Get classification
        # FIXME: implement the detector
        return light.state
        # return self.light_classifier.get_classification(image)

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
