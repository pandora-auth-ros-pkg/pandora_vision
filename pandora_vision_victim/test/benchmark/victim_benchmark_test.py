#!/usr/bin/env python
PKG = "pandora_vision_victim"
NAME = "victim_benchmark_test"

import os
import sys
import roslib; roslib.load_manifest(PKG)
import rostest
import rospy
import rospkg
import math

from pandora_vision_support.pandora_vision_testing_interface import vision_benchmark_test_base

PKG_PATH = rospkg.RosPack().get_path(PKG)


class BenchmarkTester(vision_benchmark_test_base.VisionBenchmarkTestBase):

    def test_benchmark(self):
        self.datasetCamera = rospy.get_param("dataset_camera")
        self.imageHFOV = rospy.get_param("/kinect_rgb_optical_frame/hfov")
        self.imageVFOV = rospy.get_param("/kinect_rgb_optical_frame/vfov")

        self.imageHFOV *= math.pi / 180
        self.imageVFOV *= math.pi / 180

        self.algorithm = rospy.get_param("algorithm")
        if rospy.has_param("benchmarkFlag"):
            benchmarkFlag = rospy.get_param("benchmarkFlag")
        else:
            benchmarkFlag = True

        imagePath = rospy.get_param("dataset_path")
        self.benchmarkTest(PKG_PATH + imagePath,
                           publisherTopic, subscriberTopic, benchmarkFlag)

    @classmethod
    def mockPublish(cls, input_topic, output_topic, data):

        cls.block.clear()
        # cls.repliedList[output_topic] = False
        for key in cls.repliedList.iterkeys():
            cls.repliedList[key] = False
        cls.messageList[output_topic] = list()
        if not isinstance(data, cls.publishedTypes[input_topic]):
            rospy.logerr("[mockPublish] Publishes wrong message type.")
        cls.publishers[input_topic].publish(data)
        if not cls.benchmarking:
            rospy.sleep(cls.publish_wait_duration)
        cls.block.wait()

    @classmethod
    def mockCallback(cls, data, output_topic):
        rospy.logdebug("Got message from topic : " + str(output_topic))
        rospy.logdebug(data)
        cls.messageList[output_topic] = []
        cls.messageList[output_topic].append(data)
        cls.repliedList[output_topic] = True
        # Set the processor block to notify the program that the processor
        # answered
        if "processor" in output_topic:
            if "hole" in output_topic:
                if not data.success:
                    cls.block.set()
                    return None
            # elif "victim" in output_topic:
                # cls.block.set()
        counter = 0
        for key, val in cls.repliedList.iteritems():
            if "processor" in key and val:
                counter += 1
        if counter == 2:
            cls.block.set()
        # Notify the program that an alert has been received.
        if "alert" in output_topic:
            cls.alertEvent.set()
        print "Exit callback with topic", str(output_topic)

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node(NAME, anonymous=True, log_level=rospy.DEBUG)

    publisherTopic = None
    publisherMessagePackage = None
    publisherMessageType = None
    if rospy.has_param("publisherTopic"):
        publisherTopic = rospy.get_param("publisherTopic")
    else:
        rospy.logfatal("Could not find the Point Cloud Topic name parameter!")
    if rospy.has_param("publisherMessagePackage"):
        publisherMessagePackage = rospy.get_param("publisherMessagePackage")
    else:
        rospy.logfatal("Could not find the Message Package parameter!")
    if rospy.has_param("publisherMessageType"):
        publisherMessageType = rospy.get_param("publisherMessageType")
    else:
        rospy.logfatal("Could not find the Message Type parameter!")

    subscriberTopic = None
    subscriberMessagePackage = None
    subscriberMessageType = None

    if rospy.has_param("syncSubscriberTopic"):
        subscriberTopic = rospy.get_param("syncSubscriberTopic")
    else:
        rospy.logfatal("Could not find the Synchronizer Topic name" +
                       " parameter!")

    if rospy.has_param("subscriberMessagePackage"):
        subscriberMessagePackage = rospy.get_param("subscriberMessagePackage")
    else:
        rospy.logfatal("Could not find the Processor Message Package" +
                       " parameter!")
    if rospy.has_param("subscriberMessageType"):
        subscriberMessageType = rospy.get_param("subscriberMessageType")
    else:
        rospy.logfatal("Could not find the Processor Message Type parameter!")

    subscriberAlertTopic = None
    subscriberAlertMessagePackage = None
    subscriberAlertMessageType = None

    if rospy.has_param("subscriberAlertTopic"):
        subscriberAlertTopic = rospy.get_param("subscriberAlertTopic")
    else:
        rospy.logfatal("Could not find the Alert Image Topic name" +
                       " parameter!")

    if rospy.has_param("subscriberAlertMessagePackage"):
        subscriberAlertMessagePackage = rospy.get_param("subscriberAlert" +
                                                        "MessagePackage")
    else:
        rospy.logfatal("Could not find the Alert Message Package" +
                       " parameter!")
    if rospy.has_param("subscriberAlertMessageType"):
        subscriberAlertMessageType = rospy.get_param("subscriberAlert" +
                                                     "MessageType")
    else:
        rospy.logfatal("Could not find the Alert Message Type parameter!")

    holeSubscriberTopic = None
    holeSubscriberMessagePackage = None
    holeSubscriberMessageType = None

    if rospy.has_param("holeSubscriberTopic"):
        holeSubscriberTopic = rospy.get_param("holeSubscriberTopic")
    else:
        rospy.logfatal("Could not find the hole Topic name" +
                       " parameter!")

    if rospy.has_param("holeSubscriberMessagePackage"):
        holeSubscriberMessagePackage = rospy.get_param("holeSubscriber" +
                                                        "MessagePackage")
    else:
        rospy.logfatal("Could not find the hole Message Package" +
                       " parameter!")
    if rospy.has_param("holeSubscriberMessageType"):
        holeSubscriberMessageType = rospy.get_param("holeSubscriber" +
                                                     "MessageType")
    else:
        rospy.logfatal("Could not find the hole Message Type parameter!")

    if (subscriberTopic and subscriberMessagePackage and
       subscriberMessageType and subscriberAlertTopic and
       subscriberAlertMessagePackage and subscriberAlertMessageType and
       holeSubscriberTopic and holeSubscriberMessagePackage and
       holeSubscriberMessageType and publisherTopic and
       publisherMessageType and
       publisherMessagePackage):

        subscriber_topics = [(subscriberTopic, subscriberMessagePackage,
                              subscriberMessageType),
                             (subscriberAlertTopic,
                              subscriberAlertMessagePackage,
                              subscriberAlertMessageType),
                             (holeSubscriberTopic,
                              holeSubscriberMessagePackage,
                              holeSubscriberMessageType)]

        subscriberTopic = [subscriberTopic, subscriberAlertTopic,
                           holeSubscriberTopic]
        publisher_topics = [(publisherTopic, publisherMessagePackage,
                            publisherMessageType)]
        rospy.loginfo("Test is Starting!")
        BenchmarkTester.connect(subscriber_topics, publisher_topics, 3, False)
        rostest.rosrun(PKG, NAME, BenchmarkTester, sys.argv)
        BenchmarkTester.disconnect()

    else:
        rospy.logerr("Could not execute the benchmark test for the package :" +
                     " %s !", PKG)
