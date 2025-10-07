#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from pypylon import pylon
#___Import Modules:
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


# inspiration from https://github.com/ANI717/ros2_camera_publish/tree/main
# and https://stackoverflow.com/a/72692747
TOPIC = 'pypylon/image'
QUEUE_SIZE = 1
from cv_bridge import CvBridge
bridge = CvBridge()

#__Classes:
class CameraPublisher(Node):    
    def __init__(self, capture, topic=TOPIC, queue=QUEUE_SIZE):

        super().__init__('pypylon_camera_publisher')
        
        # initialize publisher
        self.publisher_ = self.create_publisher(Image, topic, queue)
        self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())

        # Grabing Continusely (video) with minimal delay
        self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly) 
        self.converter = pylon.ImageFormatConverter()

        # converting to opencv bgr format
        self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

        while self.camera.IsGrabbing():
            # grabResult = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            grabResult = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

            if grabResult.GrabSucceeded():
                # Access the image data
                image = self.converter.Convert(grabResult)
                img = image.GetArray()
                msg = bridge.cv2_to_imgmsg(img, encoding="passthrough")
                # publishes message
                self.publisher_.publish(msg)
                # self.get_logger().info('%d Images Published' % self.i)

            grabResult.Release()

            # image counter increment
            rclpy.spin_once(self, timeout_sec=0.0)
            
        # Releasing the resource    
        self.camera.StopGrabbing()

#___Main Method:
def main(args=None):
    """This is the Main Method.
    
    """
    rclpy.init(args=args)
    camera_publisher = CameraPublisher(capture=None) # init is just spinning

    # shuts down node and releases everything
    camera_publisher.destroy_node()
    rclpy.shutdown()
    
    return None


#___Driver Program:
if __name__ == '__main__':
    main()

