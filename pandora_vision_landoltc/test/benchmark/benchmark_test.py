#!/usr/bin/env python
PKG = "pandora_vision_landoltc"
NAME = "benchmark_test"
import sys
import roslib; roslib.load_manifest(PKG)
import rostest
import rospy
import rospkg

from pandora_vision_support.pandora_vision_testing_interface import vision_benchmark_test_base

class BenchmarkTester(vision_benchmark_test_base.VisionBenchmarkTestBase):
    def test_benchmark(self):
        self.algorithm = rospy.get_param("algorithm")
        
        imagePath = "/home/miltos/Documents/pandora_supplementary_material/vision/benchmark_dataset/images_720p"
        self.benchmarkTest(imagePath,
                           publisherTopic, subscriberTopic)
if __name__ == "__main__":
    publisherTopic = rospy.get_param("kinect/topic_name")
    publisherMessagePackage = rospy.get_param("publisherMessagePackage")
    publisherMessageType = rospy.get_param("publisherMessageType")
    
    subscriberTopic = rospy.get_param("subscriberTopic")
    subscriberMessagePackage = rospy.get_param("subscriberMessagePackage")
    subscriberMessageType = rospy.get_param("subscriberMessageType")
    
    subscriber_topics = [
        (subscriberTopic, subscriberMessagePackage, subscriberMessageType)]
    publisher_topics = [
        (publisherTopic, publisherMessagePackage, publisherMessageType)]
    rospy.init_node(NAME, anonymous=True, log_level=rospy.DEBUG)
    rospy.loginfo("Test is Starting!")
    BenchmarkTester.connect(subscriber_topics, publisher_topics, 2, False)
    rostest.rosrun(PKG, NAME, BenchmarkTester, sys.argv)
    BenchmarkTester.disconnect()
