#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Header
from geometry_msgs.msg import Quaternion,PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import os
import math
import cv2
import yaml


STATE_COUNT_THRESHOLD = 3


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
    
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.light_positions = self.config['stop_line_positions']

        # stop_line_positions
        
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
         
        self.bridge = CvBridge()

        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        # waypoints_2d = [[w_p.pose.pose.position.x, w_p.pose.pose.position.y] for w_p in waypoints.waypoints]
        # self.kdtree_waypoints = KDTree(waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        """
        if self.state != light_state:
            self.state_count = 0
            self.state = light_state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_waypoint = light_waypoint if light_state == TrafficLight.RED else -1
            self.last_wp = light_waypoint
            self.upcoming_red_light_pub.publish(Int32(light_waypoint))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1
        """
# --------------------------------------------------------------------------------------------
    def create_light_site(self, x, y, z, yaw, state):
        light = TrafficLight()

        light.header = Header()
        light.header.stamp = rospy.Time.now()
        light.header.frame_id = 'world'

        # Create a Pose object to place inside the TrafficLight object
        light.pose = PoseStamped()

        light.pose.header = Header()
        light.pose.header.stamp = rospy.Time.now()
        light.pose.header.frame_id = 'world'

        light.pose.pose.position.x = x
        light.pose.pose.position.y = y
        light.pose.pose.position.z = z

        # For reference: https://answers.ros.org/question/69754/quaternion-transformations-in-python/
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, math.pi * yaw / 180.0)
        light.pose.pose.orientation = Quaternion(*q)

        light.state = state

        return light

    def dist2d(self, x1, y1, x2, y2):
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)
# ------------------------------------------------------------------------------------------

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement by using kd tree(scipy~, see line 61~62)
        dist = float('inf')
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        wp = 0
        for i in range(len(self.waypoints.waypoints)):
            new_dist = dl(pose.position, self.waypoints.waypoints[i].pose.pose.position)
            if new_dist < dist:
                dist = new_dist
                wp = i
        return wp

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # for testing
        # return light.state

        if(not self.has_image):
            self.prev_light_loc = None
            return False
        
        self.camera_image.encoding = "rgb8"
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
        # cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        # Get classification
        state = self.light_classifier.get_classification(cv_image)
        if state == TrafficLight.UNKNOWN and self.last_state:
            state = self.last_state

        return state


        
    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #closest_light = None
        #line_wp_idx = None
        light = None
        light_waypoint = None
        if self.waypoints == None or self.lights == None:
            return -1, TrafficLight.UNKNOWN
        # List of positions that correspond to the line to stop in front of for a given intersection
        if self.pose and self.waypoints:
            # car_position
            car_wp = self.get_closest_waypoint(self.pose.pose)

            light_positions = self.light_positions
            min_dist = float('inf')
            for i, light_position in enumerate(self.light_positions):
                
                light_candidate = self.create_light_site(light_position[0], light_position[1], 
                                                         0.0, 0.0, TrafficLight.UNKNOWN)
                light_wp = self.get_closest_waypoint(light_candidate.pose.pose)
                
                light_dist = self.dist2d(self.waypoints.waypoints[car_wp].pose.pose.position.x,
                                             self.waypoints.waypoints[car_wp].pose.pose.position.y,
                                             self.waypoints.waypoints[light_wp].pose.pose.position.x,
                                             self.waypoints.waypoints[light_wp].pose.pose.position.y)
                
                if (light_wp % len(self.waypoints.waypoints)) > (car_wp % len(self.waypoints.waypoints)) and (light_dist < 100) and (light_dist < min_dist):
                    light = light_candidate
                    closest_light_wp = light_wp
                    min_dist = light_dist
            """
            uint8 UNKNOWN=4
            uint8 GREEN=2
            uint8 YELLOW=1
            uint8 RED=0
            """

            if light:
                state = self.get_light_state(light)
                light_wp = closest_light_wp
                rospy.logwarn("Traffic light id: {}, and its color state: {}".format(closest_light_wp, state))
            else:
                state = TrafficLight.UNKNOWN
                light_wp = -1
            
        else:
            state = TrafficLight.RED
            light_wp = -1
        
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            if state not in [TrafficLight.RED, TrafficLight.YELLOW]:
                light_wp = -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        
        self.state_count += 1

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
