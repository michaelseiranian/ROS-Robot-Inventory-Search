#!/usr/bin/env python3
# coding=utf-8
import rospy
import smach
from cv_bridge import CvBridge
import cv2
from yolov4 import Detector
from sensor_msgs.msg import Image
from second_coursework.msg import SearchFeedback


class Yolo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['cake_found', 'image_processed'], output_keys=['feedback'])
        self.yolo = YOLOv4ROSITR()

    def execute(self, userdata):
        result = self.yolo.main_loop()
        userdata.feedback = self.yolo.get_feedback()
        if result == "cake_found":
            return "cake_found"
        else:
            return "image_processed"


class YOLOv4ROSITR:
    def __init__(self):
        self.bridge = CvBridge()
        self.cv_image = None
        self.frame = 0
        self.skip = 20
        self.objects = []
        self.amounts = []
        self.cam_subs = rospy.Subscriber('/camera/image', Image, self.img_callback)
        self.detector = Detector(gpu_id=0, config_path='/opt/darknet/cfg/yolov4.cfg',
                                 weights_path='/opt/darknet/yolov4.weights',
                                 lib_darknet_path='/opt/darknet/libdarknet.so',
                                 meta_path='/home/ubuntu/ros_ws/src/second_coursework/cfg/coco.data')

    def main_loop(self):
        while not rospy.is_shutdown():
            feedback = SearchFeedback()
            if self.cv_image is not None:
                cv_copy = self.cv_image.copy()
                img_arr = cv2.resize(cv_copy, (self.detector.network_width(), self.detector.network_height()))
                detections = self.detector.perform_detect(image_path_or_buf=img_arr, show_image=True)
                for detection in detections:
                    if detection.class_name in self.objects:
                        index = self.objects.index(detection.class_name)
                        self.amounts[index] += 1
                    else:
                        self.objects.append(detection.class_name)
                        self.amounts.append(1)

                    feedback.object_feedback = self.objects
                    feedback.amount_feedback = self.amounts
                    if "cake" in detection.class_name:
                        return "cake_found"
            return "image_processed"

    def img_callback(self, msg):
        if self.frame % self.skip == 0:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        else:
            self.cv_image = None
        self.frame = (self.frame + 1) % self.skip

    def get_feedback(self):
        feedback = SearchFeedback()
        feedback.object_feedback = self.objects
        feedback.amount_feedback = self.amounts
        return feedback
