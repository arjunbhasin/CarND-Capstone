#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml

from scipy.spatial import KDTree
import math
import numpy as np

from darknet_ros_msgs.msg import BoundingBox
from darknet_ros_msgs.msg import BoundingBoxes


STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        # Waypoint KD Tree
        self.waypoints_2d = None
        self.waypoint_tree = None

        self.camera_image = None
        self.lights = []

        # Subscribe Current pose and base waypoints 
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
        sub4 = rospy.Subscriber('/image_color', Image, self.image_cb)
        
        # darknet_ros message
        sub5 = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.detected_bb_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        # Get simulator_mode parameter (1== ON, 0==OFF)
        self.simulator_mode = rospy.get_param("/simulator_mode")

        # Publish the index of the waypoint where we have to stop
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        # WARNING! Only use during testing in site mode (for diagnostics)
        if int(self.simulator_mode) == 0:
            self.cropped_tl_bb_pub = rospy.Publisher('/cropped_bb', Image, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.TL_BB_list = None

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    # Note: waypoints remain static, this runs only once
    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        # Setup the Kd Tree which has log(n) complexity
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)


    # WARNING: lights state will not be available in real life, use only for diagnostics
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
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
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

    # # WARNING! ROSBAG testing version - Use only in site mode
    # # WARNING! If using this function, comment out function of same name below
    # def detected_bb_cb(self, msg):
    #     # Clear the list
    #     self.TL_BB_list = []
    #     for bb in msg.bounding_boxes:
    #         # Bounding Box class should be 'traffic light' with probability >= 40%
    #         if str(bb.Class) == 'traffic light' and bb.probability >= 0.50:
    #             # Get the camera image
    #             cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
    #             # Crop image
    #             bb_image = cv_image[bb.ymin:bb.ymax, bb.xmin:bb.xmax]
    #             # Get height and width
    #             height, width, channels = bb_image.shape

    #             # Partition into Red, Yellow and Green Areas
    #             red_area = bb_image[0:height//3, 0:width]
    #             yellow_area = bb_image[height//3: 2*height//3, 0:width]
    #             green_area = bb_image[2*height//3: height, 0:width]

    #             #bb_image = cv2.cvtColor(bb_image, cv2.COLOR_BGR2GRAY)
    #             # Standard Gray conversion, coefficients = [0.114, 0.587, 0.299] (bgr)
    #             coefficients_red = [0.1, 0.1, 0.8]
    #             coefficients_yellow = [0.114, 0.587, 0.299]
    #             coefficients_green = [0.1, 0.8, 0.1]

    #             red_area = cv2.transform(red_area, np.array(coefficients_red).reshape((1,3)))
    #             yellow_area = cv2.transform(yellow_area, np.array(coefficients_yellow).reshape((1,3)))
    #             green_area = cv2.transform(green_area, np.array(coefficients_green).reshape((1,3)))
                
    #             # Patch the image back
    #             bb_image = np.concatenate((red_area,yellow_area,green_area),axis=0)
    #             # Get height and width values again (just to be sure interger division didn't eat up some values)
    #             height, width = bb_image.shape

    #             # Create mask 
    #             mask = np.zeros((height, width), np.uint8)
    #             width_offset = 3
    #             height_offset = 4
    #             cv2.ellipse(mask, (width//2, 1*height//6), (width//2 - width_offset, height//6 - height_offset), 0, 0, 360, 1, -1)
    #             cv2.ellipse(mask, (width//2, 3*height//6), (width//2 - width_offset, height//6 - height_offset), 0, 0, 360, 1, -1)
    #             cv2.ellipse(mask, (width//2, 5*height//6), (width//2 - width_offset, height//6 - height_offset), 0, 0, 360, 1, -1)

    #             # Apply mask
    #             bb_image = np.multiply(bb_image, mask)

    #             # Threhold the grayscale image
    #             bb_image = cv2.inRange(bb_image, 210, 255)

    #             # Partition into Red, Yellow and Green Areas
    #             red_area = bb_image[0:height//3, 0:width]
    #             yellow_area = bb_image[height//3: 2*height//3, 0:width]
    #             green_area = bb_image[2*height//3: height, 0:width]

    #             red_count = cv2.countNonZero(red_area)
    #             yellow_count = cv2.countNonZero(yellow_area)
    #             green_count = cv2.countNonZero(green_area)

    #             # Publish the image
    #             self.cropped_tl_bb_pub.publish(self.bridge.cv2_to_imgmsg(bb_image, "mono8"))

    #             #print (red_count, yellow_count, green_count)
    #             if red_count > yellow_count and red_count > green_count:
    #                 print ('Red Light Detected!')
    #             elif yellow_count > red_count and yellow_count > green_count:
    #                 print ('Yellow Light Detected!')
    #             elif green_count > red_count and green_count > yellow_count:
    #                 print ('Green Light Detected!')
    #             else:
    #                 print ('Warning! Unable to determine Light state')


    # Simulator Version
    # Important function
    def detected_bb_cb(self, msg):
        # Clear the list
        self.TL_BB_list = []

        # Parameters: Diagnonal size thresholds
        simulator_bb_size_threshold = 85 #px
        site_bb_size_threshold = 40 #px
        # min probability of detection
        simulator_bb_probability = 0.85
        site_bb_probability = 0.50

        if int(self.simulator_mode) == 1:
            prob_thresh = simulator_bb_probability
            size_thresh = simulator_bb_size_threshold
        else:
            prob_thresh = site_bb_probability
            size_thresh = site_bb_size_threshold

        for bb in msg.bounding_boxes:
            # Simulator mode: Bounding Box class should be 'traffic light' with probability >= 85%
            # Site Mode: Bounding Box class should be 'traffic light' with probability >= 65%
            if str(bb.Class) == 'traffic light' and bb.probability >= prob_thresh:
                # Simulator mode: If diagonal size of bounding box is more than 85px
                # Site mode: If diagonal size of bounding box is more than 80px
                if math.sqrt((bb.xmin - bb.xmax)**2 + (bb.ymin - bb.ymax)**2) >= size_thresh:
                    self.TL_BB_list.append(bb)

                    # if running in site mode/ROS bag mode
                    if int(self.simulator_mode) == 0:

                        '''The ROS bag version only has video data. Hence no waypoints are loaded and get light function is not called.
                            So to check detection in ROS bag video, we do TL state classification here itself.
                        '''
                        # Get the camera image
                        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
                        # Crop image
                        bb_image = cv_image[bb.ymin:bb.ymax, bb.xmin:bb.xmax]
                        # Get height and width
                        height, width, channels = bb_image.shape

                        # Partition into Red, Yellow and Green Areas
                        red_area = bb_image[0:height//3, 0:width]
                        yellow_area = bb_image[height//3: 2*height//3, 0:width]
                        green_area = bb_image[2*height//3: height, 0:width]

                        # We boost individual channel values, i.e., Red is boosted in Red Area, Green is boosted in Green Area
                        # Standard Gray conversion, coefficients = [0.114, 0.587, 0.299] (bgr)
                        coefficients_red = [0.1, 0.1, 0.8]
                        coefficients_yellow = [0.114, 0.587, 0.299]
                        coefficients_green = [0.1, 0.8, 0.1]

                        red_area = cv2.transform(red_area, np.array(coefficients_red).reshape((1,3)))
                        yellow_area = cv2.transform(yellow_area, np.array(coefficients_yellow).reshape((1,3)))
                        green_area = cv2.transform(green_area, np.array(coefficients_green).reshape((1,3)))
                        
                        # Patch the image back
                        bb_image = np.concatenate((red_area,yellow_area,green_area),axis=0)
                        # Get height and width values again (just to be sure interger division didn't eat up some values)
                        height, width = bb_image.shape

                        # Create mask 
                        mask = np.zeros((height, width), np.uint8)
                        width_offset = 3
                        height_offset = 4
                        cv2.ellipse(mask, (width//2, 1*height//6), (width//2 - width_offset, height//6 - height_offset), 0, 0, 360, 1, -1)
                        cv2.ellipse(mask, (width//2, 3*height//6), (width//2 - width_offset, height//6 - height_offset), 0, 0, 360, 1, -1)
                        cv2.ellipse(mask, (width//2, 5*height//6), (width//2 - width_offset, height//6 - height_offset), 0, 0, 360, 1, -1)

                        # Apply mask
                        bb_image = np.multiply(bb_image, mask)

                        # Threhold the grayscale image
                        bb_image = cv2.inRange(bb_image, 210, 255)

                        # Partition into Red, Yellow and Green Areas
                        red_area = bb_image[0:height//3, 0:width]
                        yellow_area = bb_image[height//3: 2*height//3, 0:width]
                        green_area = bb_image[2*height//3: height, 0:width]

                        red_count = cv2.countNonZero(red_area)
                        yellow_count = cv2.countNonZero(yellow_area)
                        green_count = cv2.countNonZero(green_area)

                        # Publish the image
                        self.cropped_tl_bb_pub.publish(self.bridge.cv2_to_imgmsg(bb_image, "mono8"))

                        #print (red_count, yellow_count, green_count)
                        if red_count > yellow_count and red_count > green_count:
                            print ('Red Light Detected!')
                        elif yellow_count > red_count and yellow_count > green_count:
                            print ('Yellow Light Detected!')
                        elif green_count > red_count and green_count > yellow_count:
                            print ('Green Light Detected!')
                        else:
                            print ('Warning! Unable to determine Light state')


    # Similar to function in waypoint_update.py
    def get_closest_waypoint(self, x, y):
        closest_idx = self.waypoint_tree.query([x,y], 1)[1]
        return closest_idx


    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # WARNING!! Use only while testing
        # return light.state

        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image, self.TL_BB_list, self.simulator_mode)

        


    # Important function
    """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
    def process_traffic_lights(self):
        closest_light = None
        line_wp_idx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']

        if(self.pose):
            # waypoint closest to current car pose
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

            diff = len(self.waypoints.waypoints)

            for i, light in enumerate(self.lights):
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])

                d = temp_wp_idx - car_wp_idx

                if d >=0 and d < diff:
                    diff = d
                    closest_light = light
                    line_wp_idx = temp_wp_idx

        if closest_light:
            state = self.get_light_state(closest_light)
            return line_wp_idx, state
        
        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
