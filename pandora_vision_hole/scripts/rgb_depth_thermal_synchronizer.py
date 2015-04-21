#!/usr/bin/env python

import rospy
import message_filters
from camera_calibration import approxsync
from sensor_msgs.msg import Image, PointCloud2

def callback(pointCloud, image2):
  # Solve all of perception here...
  rospy.loginfo("login")

if __name__=='__main__':
   
    rospy.loginfo("loged in synch rgbdt")
    ns = rospy.get_namespace()

    if (rospy.has_param(ns + "/rgb_depth_thermal_synchronizer_node/subscribed_topics/input_topic")):    
        kinect_topic = rospy.get_param(ns + "/rgb_depth_thermal_synchronizer_node/subscribed_topics/input_topic")
    else:
        print "No point cloud topic found"
        rospy.signal_shutdown("shutdown RGBDT synchronizer")


    if (rospy.has_param(ns + "/rgb_depth_thermal_synchronizer_node/subscribed_topics/input_thermal_topic")):    
        flir_topic = rospy.get_param(ns + "/rgb_depth_thermal_synchronizer_node/subscribed_topics/input_thermal_topic")
    else:    
        print "No flir topic found"
        rospy.signal_shutdown("shutdown RGBDT synchronizer")

    rospy.loginfo("subscribers")
    # subscribers of flir
    kinect_subscriber = message_filters.Subscriber(kinect_topic, PointCloud2)
    flir_subscriber = message_filters.Subscriber(flir_topic, Image)


    rospy.loginfo("synch the subscribers")
    sync = approxsync.ApproximateSynchronizer(0.02,[kinect_subscriber, flir_subscriber], 10)
    sync.registerCallback(callback)

    rospy.init_node('RGBDT synchronizer')
    rospy.spin()
