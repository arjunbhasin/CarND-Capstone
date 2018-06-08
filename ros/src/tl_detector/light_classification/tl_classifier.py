import cv2
import numpy as np
from styx_msgs.msg import TrafficLight
from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge


class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        
        # For Diagnostics
        # Publish cropped TL bounding box
        self.cropped_tl_bb_pub = rospy.Publisher('/cropped_bb', Image, queue_size=1)
        self.bridge = CvBridge()

    # Return state of light in BB image
    #*****************************Works well on site video...Okayish on Simulator**********************************
    def detect_light_state(self, bb_image):
        # Get height and width
        height, width, channels = bb_image.shape
        # Draw partition lines
        #cv2.line(bb_image, (0, height//3), (width, height//3), (0,0,0), 1)
        #cv2.line(bb_image, (0, 2*height//3), (width, 2*height//3), (0,0,0), 1)

        # Partition into Red, Yellow and Green Areas
        red_area = bb_image[0:height//3, 0:width]
        yellow_area = bb_image[height//3: 2*height//3, 0:width]
        green_area = bb_image[2*height//3: height, 0:width]

        # We boost individual channel values, i.e., Red is boosted in Red Area, Green is boosted in Green Area
        # Standard Gray conversion, coefficients = [0.114, 0.587, 0.299] (bgr)
        coefficients_red = [0.1, 0.1, 0.8]
        coefficients_yellow = [0.114, 0.587, 0.299]
        coefficients_green = [0.1, 0.8, 0.1]

        # Apply co-efficients to get grayscale image
        red_area = cv2.transform(red_area, np.array(coefficients_red).reshape((1,3)))
        yellow_area = cv2.transform(yellow_area, np.array(coefficients_yellow).reshape((1,3)))
        green_area = cv2.transform(green_area, np.array(coefficients_green).reshape((1,3)))
        
        # Patch the image back (Note image is now in grayscale)
        bb_image = np.concatenate((red_area,yellow_area,green_area),axis=0)

        # Get height and width values again (just to be sure interger division didn't eat up some values)
        height, width = bb_image.shape

        # Create mask 
        mask = np.zeros((height, width), np.uint8)
        # Can play around with offset
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

        # Publish the image for diagnostics
        self.cropped_tl_bb_pub.publish(self.bridge.cv2_to_imgmsg(bb_image, "mono8"))

        # Default state is unknown
        state = TrafficLight.UNKNOWN

        if red_count > yellow_count and red_count > green_count:
            print ('Red Light Detected!')
            state = TrafficLight.RED
        elif yellow_count > red_count and yellow_count > green_count:
            print ('Yellow Light Detected!')
            state = TrafficLight.YELLOW
        elif green_count > red_count and green_count > yellow_count:
            print ('Green Light Detected!')
            state = TrafficLight.GREEN
        else:
            print ('Warning! Unable to determine Light state')
        
        return state


    def get_classification(self, image, TL_BB_list, simulator_mode):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light
            TL_BB_list (List): List containing bounding boxe(s) of Traffic Lights

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction

        # if list is empty, return UNKNOWN
        if not TL_BB_list:
            return TrafficLight.UNKNOWN

        else:
            # We consider the BB with highest probability (at index 0)
            xmin = TL_BB_list[0].xmin
            xmax = TL_BB_list[0].xmax
            ymin = TL_BB_list[0].ymin
            ymax = TL_BB_list[0].ymax

            # cropped image
            bb_image = image[ymin:ymax, xmin:xmax]

            # Check if running in simulator mode
            if int(simulator_mode) == 1:
                #**********************************************Only works in Simulator, Not on site********************************
                # Convert to HSV
                hsv_bb_img = cv2.cvtColor(bb_image, cv2.COLOR_BGR2HSV)

                # Red Color ranges (Red has two ranges)
                frame_threshed_red1 = cv2.inRange(hsv_bb_img, (0, 70, 50), (10, 255, 255)) 
                frame_threshed_red2 = cv2.inRange(hsv_bb_img, (170, 70, 50), (180, 255, 255)) 

                # Yellow Color range
                frame_threshed_yellow = cv2.inRange(hsv_bb_img, (40.0/360*255, 100, 100), (66.0/360*255, 255, 255)) 
                # Green color range
                frame_threshed_green = cv2.inRange(hsv_bb_img, (90.0/360*255, 100, 100), (140.0/360*255, 255, 255)) 

                # Publish the HSV image (for diagnostics)
                self.cropped_tl_bb_pub.publish(self.bridge.cv2_to_imgmsg(hsv_bb_img, "bgr8"))
               
                # If more than a certain number of pixels are red
                if cv2.countNonZero(frame_threshed_red1) + cv2.countNonZero(frame_threshed_red2) > 40:
                    print('Red Light Detected!')
                    return TrafficLight.RED
                
                elif cv2.countNonZero(frame_threshed_yellow) > 20 :
                    print('Yellow Light Detected!')
                    return TrafficLight.YELLOW
                elif cv2.countNonZero(frame_threshed_green) > 20 :
                    print('Green Light Detected!')
                    return TrafficLight.GREEN
                else:
                    print ('Warning! Could not determine color of light!')
                    return TrafficLight.UNKNOWN
                #**************************************************************************************************************

            # Running in site mode
            else:
                return self.detect_light_state(bb_image)

                