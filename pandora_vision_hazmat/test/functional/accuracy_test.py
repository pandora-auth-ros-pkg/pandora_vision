#!/usr/bin/env python
import sys
import cv2
import os
import unittest
from cv_bridge import CvBridge, CvBridgeError

PKG = "pandora_vision_hazmat"
NAME = "accuracy_test"

import roslib
roslib.load_manifest(PKG)
import rostest
import rospy


from pandora_testing_tools.testing_interface import test_base
from sensor_msgs.msg import Image


class AccuracyTester(test_base.TestBase):
    def readImages(self, imagePath):
        self.images = []
        imageTypes = [".png", ".jpg", ".bmp"]
        bridge = CvBridge()
        if not os.path.isdir(imagePath):
            print("ERROR : Incorrect Path")

        for fileName in os.listdir(imagePath):
            isImage = False
            for type in imageTypes:
                if (type in fileName):
                    isImage = True
                    break
            if (not isImage):
                print fileName, "is not an image"
                continue
            # Read the next image.
            currentImg = cv2.imread(os.path.join(imagePath,
                                    fileName), -1)

            # If the image was not read succesfully continue to the
            # next file.
            if (currentImg is None):
                print "Error reading image", fileName
                print "The process will read the next image!"
                continue
            # Store the image.
            self.images.append(bridge.cv2_to_imgmsg(currentImg))

    def test_accuracy(self):

        rospy.loginfo("Reading Images")
        self.readImages("/home/vchoutas/Programming/pandora_ws/images")
        rospy.logdebug("Now test publishing...")
        truePos = len(self.images)
        count = 0
        for image in self.images:
            rospy.loginfo("Sending Image \n")
            self.mockPublish("/camera/image_raw", "/alert/hazmat", image)

            if ((self.repliedList["/alert/hazmat"]) and
               (len(self.messageList["/alert/hazmat"]) == 1)):
                count += 1

        rospy.loginfo("Accuracy %f \n", count/float(truePos))

if __name__ == "__main__":

    subscriber_topics = [
        ("/alert/hazmat", "std_msgs", "Bool")]
    publisher_topics = [
        ("/camera/image_raw", "sensor_msgs", "Image")]

    rospy.init_node(NAME, anonymous=True, log_level=rospy.DEBUG)
    rospy.loginfo("Test is Starting!")

    AccuracyTester.connect(subscriber_topics, publisher_topics, 2, False)
    rostest.rosrun(PKG, NAME, AccuracyTester, sys.argv)
    AccuracyTester.disconnect()
