#!/usr/bin/env python

from __future__ import print_function

import imageio
import os
import yaml

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tf


class ImageSaver:
    def __init__(self):
        
        if rospy.get_param("/use_sim_time", False):
            print("\n***\nRunning in simulation time.\n***\n")
        else:
            print("Running in real time")
        
        self.camera_topic = '~images_in'
        self.save_period = rospy.Duration(rospy.get_param("~save_period", 2))
        self.image_folder_output = os.path.expanduser(rospy.get_param("~image_folder_output", '~/image_saver_output'))
        self.fixed_frame = rospy.get_param("~fixed_frame", "map")

        self.only_one_info_file = rospy.get_param("~only_one_info_file", False)
        self.use_timestamps_file = rospy.has_param("~timestamps")
        ts = rospy.get_param("~timestamps", list())
        self.timestamps = dict(map(lambda t: (rospy.Time(*t[0:2]), t[2]), ts))

        self.save_reference_points = rospy.get_param("~save_reference_points", False)

        self.reference_points_dict = dict()
        self.reference_up_points_dict = dict()
        if self.save_reference_points:
            
            if not rospy.has_param("~reference_points"):
                rospy.logfatal("Param save_reference_points set to true, but reference_points not defined!")
                rospy.signal_shutdown("")
            
            reference_points_params_dict = rospy.get_param("~reference_points")
            assert(isinstance(reference_points_params_dict, dict))

            print('reference points: ')
            print('\n'.join(reference_points_params_dict.keys()))

            for reference_point_name, reference_point_new in reference_points_params_dict.iteritems():
                x, y, z = reference_point_new['point']['x'], reference_point_new['point']['y'], reference_point_new['point']['z']

                self.reference_points_dict[reference_point_name] = PointStamped(
                    header=Header(frame_id=self.fixed_frame),
                    point=Point(x, y, z)
                )

                self.reference_up_points_dict[reference_point_name] = PointStamped(
                    header=Header(frame_id=self.fixed_frame),
                    point=Point(x, y, z + 1)
                )

        self.image_file_format_string = os.path.join(self.image_folder_output, '%05i.png')
        self.info_file_format_string = os.path.join(self.image_folder_output, '%05i.yaml')
        self.info_file = os.path.join(self.image_folder_output, 'images_info.yaml')
        self.bridge = CvBridge()
        self.listener = tf.TransformListener()
        self.camera_sub_ = None
        self.image = None
        self.i = 0

        if not os.path.exists(self.image_folder_output):
            rospy.logwarn("mkdir %s" % self.image_folder_output)
            os.mkdir(self.image_folder_output)

    def image_filename(self, index):
            return self.image_file_format_string % index

    def info_filename(self, index):
            return self.info_file_format_string % index

    def create_subscriber(self):
        self.camera_sub_ = rospy.Subscriber(self.camera_topic, Image, self.image_callback)

    def image_callback(self, msg):
        if self.use_timestamps_file:
            if msg.header.stamp in self.timestamps:
                self.process_image(msg, self.timestamps[msg.header.stamp])
        else:
            self.image = msg

    def loop(self):

        self.create_subscriber()

        if self.use_timestamps_file:
            rospy.spin()
        else:
            while not rospy.is_shutdown():
                rospy.sleep(self.save_period)
                self.process_image(self.image)

    def process_image(self, image, image_index=None):

        if image is None:
            rospy.loginfo("No new image to save")
            return

        points_camera = dict()
        reference_up_points_camera = dict()
        camera_transform = None

        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "rgb8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        if self.save_reference_points:
            try:
                for reference_point_name in self.reference_points_dict.keys():
                    reference_point_new = self.reference_points_dict[reference_point_name]
                    reference_up_point_new = self.reference_up_points_dict[reference_point_name]

                    reference_point_new.header.stamp = image.header.stamp
                    self.listener.waitForTransform(reference_point_new.header.frame_id, image.header.frame_id,
                                                   image.header.stamp, rospy.Duration(0.1))
                    points_camera[reference_point_name] = self.listener.transformPoint(image.header.frame_id,
                                                                                       reference_point_new)

                    reference_up_point_new.header.stamp = image.header.stamp
                    self.listener.waitForTransform(reference_up_point_new.header.frame_id, image.header.frame_id,
                                                   image.header.stamp, rospy.Duration(0.1))
                    reference_up_points_camera[reference_point_name] = self.listener.transformPoint(image.header.frame_id, reference_up_point_new)
            except tf.Exception:
                pass

        try:
            self.listener.waitForTransform(image.header.frame_id, self.fixed_frame,
                                           image.header.stamp, rospy.Duration(0.1))
            camera_transform = self.listener.lookupTransform(image.header.frame_id, self.fixed_frame,
                                                             image.header.stamp)
        except tf.Exception as e:
            rospy.loginfo(e)
            pass

        if image_index is not None:
            image_filename = self.image_filename(image_index)
        else:
            while os.path.exists(self.image_filename(self.i)):
                self.i += 1
            image_filename = self.image_filename(self.i)

        imageio.imwrite(image_filename, cv_image)
        rospy.loginfo("saved image %s" % image_filename)

        info_filename = self.info_file if self.only_one_info_file else self.info_filename(
            self.i if image_index is None else image_index)

        if not os.path.exists(info_filename):
            with open(info_filename, "w") as f:
                yaml.dump({}, f)
                rospy.loginfo("saved info file %s" % info_filename)
        else:
            rospy.logwarn("info file %s already exists. Content will be overwritten." % info_filename)

        with open(info_filename) as f:
            info_yaml = yaml.load(f)

        info_yaml[os.path.basename(self.image_filename(self.i))] = {
            'reference_up_points': reference_up_points_camera,
            'reference_points': points_camera,
            'image_header': image.header,
            'camera_transform': camera_transform,
            'fixed_frame': self.fixed_frame
        }

        with open(info_filename, "w") as f:
            yaml.dump(info_yaml, f)

        self.i += 1


if __name__ == '__main__':
    # Initiating ros node
    rospy.init_node('mbot_image_saver', anonymous=True)

    # Instantiating ImageSaver object
    image_saver = ImageSaver()

    image_saver.loop()
