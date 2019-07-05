#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Extract images from a rosbag.
"""

import os
import argparse

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    """Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("output_dir", help="Output directory.")
    parser.add_argument("image_topic", help="Image topic.")
    parser.add_argument("one_every", help="Only save one image every n.")

    args = parser.parse_args()

    print "Extract images from %s on topic %s into %s" % (args.bag_file,
                                                          args.image_topic, args.output_dir)

    bag = rosbag.Bag(args.bag_file, "r")
    bridge = CvBridge()
    count = 0
    i = 0
    one_every = int(args.one_every)
    for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
        
        if i % one_every == 0:
        
            try:
                cv_img = bridge.imgmsg_to_cv2(msg, "rgb8")
                cv_img = cv_img[:, :, ::-1]
            except CvBridgeError as e:
                rospy.logerr(e)
            
            filename = os.path.join(args.output_dir, "frame%06i.png" % count)
            cv2.imwrite(filename, cv_img)
            print "Saved image %s (t=%f)" % (filename, t.to_sec())

            count += 1
        
        i += 1

    bag.close()

    return

if __name__ == '__main__':
    main()
