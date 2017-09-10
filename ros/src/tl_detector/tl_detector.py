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

        # This parameters will need tuning for the test site
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
        # DEBUG: log the position of the lights reported by the simulator
        # pos = ["(%.2f, %.2f)" % (light.pose.pose.position.x,  light.pose.pose.position.y)
        #         for light in self.lights]
        # rospy.logdebug("Got lights: %s", pos)

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

        self.camera_image = msg
        self.camera_image.encoding = "rgb8"

        # Save current pose assuming it's the closest to
        # the moment the picture was taken
        # TODO: We should check the msg timestamps...
        self.camera_pose = self.pose_msg.pose

        light_wp, state = self.process_traffic_lights()

        # Publish upcoming red lights at camera frequency.
        # Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        # of times till we start using it. Otherwise the previous stable state is
        # used.
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

        light = None
        light_wp = -1
        light_state = TrafficLight.UNKNOWN

        # NOTE. With the simulator, the positions reported here are different
        # from the positions reported in /traffic_lights.
        # Now the we will have the /traffic_lights topic available when testing
        # with the real car, we can ignore these positions and uses the
        # reported lights.
        # light_positions = self.config['light_positions']

        # Light positions
        lights = self.lights
        # Base waypoints
        waypoints = self.waypoints
        # Car pose at the time of the picture
        car_pose = self.camera_pose

        # Car waypoint
        ego_position = self.get_closest_waypoint(waypoints, car_pose)
        ego_x = waypoints[ego_position].pose.pose.position.x
        ego_y = waypoints[ego_position].pose.pose.position.y
        ego_yaw = self.yaw_from_quaternion(car_pose.orientation)
        cos_yaw = math.cos(-ego_yaw)
        sin_yaw = math.sin(-ego_yaw)

        light_distance = self.max_light_dist

        # Find the closest visible traffic light (if one exists)
        for tl in lights:
            dx = tl.pose.pose.position.x - ego_x
            dy = tl.pose.pose.position.y - ego_y

            # Check that the light is in front of car
            if dx * cos_yaw + dy * sin_yaw < 0:
                continue

            # And that it's in range
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < self.min_light_dist or dist > light_distance:
                continue

            light = tl
            light_distance = dist

        if light:
            light_state = self.get_light_state(light)

            if light_state == TrafficLight.RED:
                light_wp = self.get_closest_waypoint(waypoints, light.pose.pose)

            rospy.loginfo("Traffic light: %s at %d (%.2f m) (car at %d)",
                            self.light_state_string(light_state),
                            light_wp, light_distance, ego_position)

        # TODO: Check the impact of this optimization
        self.waypoints = None

        return light_wp, light_state

    def get_closest_waypoint(self, waypoints, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        x = pose.position.x
        y = pose.position.y
        # minimum squared distance to pose in waypoints
        return min(xrange(len(waypoints)),
                    key=lambda i: ((waypoints[i].pose.pose.position.x - x)**2
                                 + (waypoints[i].pose.pose.position.y - y)**2))

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        # FIXME: implement the detector
        return light.state

        image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # Correct image for distortions
        if True or self.camera.distortion_model:
            tmp = np.zeros_like(image)
            self.camera.rectifyImage(image, tmp)
            image = tmp

        # FIXME: this code is not working...
        # x, y = self.project_to_image_plane(light.pose.pose.position)
        # if ( x < 0 or y < 0 ):
        #     rospy.logwarn("Failed to project light to image plane")
        #     return TrafficLight.UNKNOWN
        # rospy.logdebug("TL x=%d y=%d", x, y)
        #
        # # Use light location to zoom in on traffic light in image
        # h, w = image.shape[:2]
        #
        # x0 = max(0, x - 200)
        # x1 = min(w, x + 200)
        # y0 = max(0, y - 100)
        # y1 = min(h, y + 100)
        #
        # roi = image[y0:y1, x0:x1]

        # basename = "/tmp/tl_detector/traffic_light_{}_{}_{}_{}".format(
        #     rospy.get_time(), self.light_state_string(light.state),
        #     x, y)
        # image_filename = basename + ".jpg"
        # cv2.imwrite(image_filename, image)


        # Get classification
        return self.light_classifier.get_classification(zoomed)

    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image
        """

        # BUG. base_link seems to be way off from the actual position of the car.
        # We will use instead the current pose as an approximation.
        if False:
            # get transform between pose of camera and world frame
            trans = None
            try:
                now = rospy.Time.now()
                self.listener.waitForTransform("/base_link",
                      "/world", now, rospy.Duration(1.0))
                (trans, rot) = self.listener.lookupTransform("/base_link",
                      "/world", now)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                rospy.logerr("Failed to find camera to map transform")
                return -1, -1
        else:
            trans = self.camera_pose.position
            trans = (trans.x, trans.y, trans.z)

            rot = self.camera_pose.orientation
            rot = (rot.x, rot.y, rot.z, rot.w)

        # Use tranform and rotation to calculate 2D position of light in image

        # Convert point to camera coordinates
        point = self.quaternion_rotate(rot,
                    (point_in_world.x - trans[0],
                     point_in_world.y - trans[1],
                     point_in_world.z - trans[2]))
        # The camera axis is rotated
        px, py, pz = point
        point = (-py, -pz, px)

        # Project into image plane
        x, y = self.camera.project3dToPixel(point)
        if math.isnan(x) or math.isnan(y):
            # Failed to project
            return -1, -1

        # x = self.camera.width - x
        # y = self.camera.height - y

        x = max(0, min(int(round(x)), self.camera.width - 1))
        y = max(0, min(int(round(y)), self.camera.height - 1))

        return x, y

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
        q_v = ( v[0],  v[1],  v[2],   1.)
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
        return np.column_stack((K, [0.,0.,0.]))



if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
